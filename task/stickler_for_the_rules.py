import rospy
import time
import math
import sys
import cv2
import numpy as np
import mediapipe as mp

from module.CLIP.clip_detection import CLIPDetector, CLIPDetectorConfig
from utils.marker_maker import MarkerMaker
from utils.axis_transform import Axis_transform
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

sys.path.append('.')


# Stickler for the rules SHOES CLIP detection config
SHOES_CONFIG = CLIPDetectorConfig(
    name="shoes",
    labels=["negative", "positive"],
    # positive_texts=[
    #     "a photo of a person wearing socks",
    #     "a photo of a person barefoot",
    # ],
    # negative_texts=[
    #     "a photo of a person wearing shoes",
    #     "a photo of a person wearing sandals",
    # ],
    positive_texts=[
        "a photo of a person wearing socks or being barefoot, without shoes or sandals",
    ],
    negative_texts=[
        "a photo of a person wearing shoes or sandals",
    ],
    threshold=0
)

# Stickler for the rules DRINK CLIP detection config
DRINK_CONFIG = CLIPDetectorConfig(
    name="drink",
    labels=["negative", "positive"],
    positive_texts=[
        "a photo of a person standing with a cup in their hand",
        "a photo of a person standing with a bottle in their hand",
        "a photo of a person standing with a soda can in their hand",
        "a photo of a person standing with a beverage in their hand",
    ],
    negative_texts=[
        "a photo of a person standing without any objects in their hand",
        "a photo of a person standing without any objects in their hand with a drink in the background",
    ],
    threshold=0
)


class ShoeDetection:
    def __init__(self, agent, axis_transform):
        self.agent = agent
        self.axis_transform = axis_transform
        self.detector = CLIPDetector(config=SHOES_CONFIG, mode="HSR")
        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray,
                         self._knee_pose_callback)
        self.knee_list = None


    def _knee_pose_callback(self, data):
        self.knee_list = np.reshape(data.data, (-1, 2))
        visualize_mode = False
        if visualize_mode:
            for x, y in self.knee_list:
                cv2.circle(self.agent.rgb_img, (x, y), 2,
                           (255, 0, 0), -1, cv2.LINE_AA)
            cv2.imshow('hsr_vision', self.agent.rgb_img)
            cv2.waitKey(1)  # 1 millisecond


    def find_shoes(self):
        image = self.agent.rgb_img
        return self.detector.detect(images=image)


    def detect(self):
        rospy.sleep(1)
        head_tilt_list = [-20, -40]

        for tilt in head_tilt_list:
            self.agent.pose.head_tilt(tilt)
            if self.find_shoes():
                return True
            rospy.sleep(1)

        return False


    def clarify_violated_rule(self):
        if len(self.knee_list) > 0:
            for x, y in self.knee_list:
                move_human_infront(
                    self.agent, self.axis_transform, y, x, coord=False)

            # clarify what rule is being broken
            self.agent.pose.head_tilt(20)
            self.agent.say('Hello!', show_display=True)
            rospy.sleep(1)
            self.agent.say(
                'Sorry but\n all guests should take off\n their shoes at the entrance.', show_display=True)
            rospy.sleep(5)
                

    def ask_to_action(self, entrance):
        rospy.sleep(1)
        # take the offender to the entrance
        self.agent.say('Allow me to guide you\nto the entrance.',
                       show_display=True)
        rospy.sleep(3)
        self.agent.say('Please follow me!', show_display=True)
        rospy.sleep(2)
        self.agent.move_abs_safe(entrance)

        # ask to take off their shoes
        self.agent.pose.head_tilt(20)
        self.agent.say('Please take off your shoes\n here at the enterance', show_display=True)
        rospy.sleep(3)

        self.agent.say('I will wait ten seconds\nfor you to take off your shoes', show_display=True)
        rospy.sleep(13)

        rebelion_count = 0
        while rebelion_count < 3:
            self.agent.pose.head_tilt(-40)
            self.agent.say('Are you finished?\nLet me see your feet', show_display=True)
            rospy.sleep(4)

            if self.find_shoes():
                self.agent.pose.head_tilt(20)
                self.agent.say('You are still wearing shoes!', show_display=True)
                rospy.sleep(2)
                self.agent.say(f'I will wait five more seconds\nfor you to take off your shoes', show_display=True)
                rospy.sleep(8)
                rebelion_count += 1
            else:
                self.agent.pose.head_tilt(20)
                self.agent.say('Thank you!\nEnjoy your party', show_display=True)
                rospy.sleep(2.5)

        self.agent.pose.head_tilt(20)
        self.agent.say('I give up.\nEnjoy your party', show_display=True)
        rospy.sleep(2.5)


class ForbiddenRoom:
    def __init__(self, agent, axis_transform, min_points, max_points):
        self.agent = agent
        self.mark_pub = rospy.Publisher(
            '/snu/forbidden_3d', Marker, queue_size=100)
        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray,
                         self._knee_pose_callback)
        self.marker_maker = MarkerMaker('/snu/human_location')
        self.min_points = min_points
        self.max_points = max_points
        self.axis_transform = axis_transform
        self.draw_forbidden_room_marker(self.min_points, self.max_points)
        self.offender_pos = None

    def _knee_pose_callback(self, data):
        self.knee_list = np.reshape(data.data, (-1, 2))
        visualize_mode = False
        if visualize_mode:
            for x, y in self.knee_list:
                cv2.circle(self.agent.rgb_img, (x, y), 2,
                           (255, 0, 0), -1, cv2.LINE_AA)
            cv2.imshow('hsr_vision', self.agent.rgb_img)
            cv2.waitKey(1)  # 1 millisecond

    def detect_forbidden_room(self):
        self.draw_forbidden_room_marker(self.min_points, self.max_points)
        _pc = self.agent.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        for human_coord in self.knee_list:
            human_pc = pc_np[human_coord[1], human_coord[0]]
            human_coord_in_map = self.axis_transform.transform_coordinate(
                'head_rgbd_sensor_rgb_frame', 'map', human_pc)
            self.marker_maker.pub_marker(
                [human_coord_in_map[0], human_coord_in_map[1], 1], 'map')
            print('[RULE 2] human_coord_in_map', human_coord_in_map)
            if self.min_points[0] < human_coord_in_map[0] < self.max_points[0] and \
                    self.min_points[1] < human_coord_in_map[1] < self.max_points[1] and \
                    self.min_points[2] < human_coord_in_map[2] < self.max_points[2]:
                print('[RULE 2] human detected in front of forbidden room')
                self.offender_pos = [
                    human_coord_in_map[0], human_coord_in_map[1]]
                return True

    def clarify_violated_rule(self):
        # go to the offender and clarify what rule is being broken
        move_human_infront(self.agent, self.axis_transform,
                           self.offender_pos[1], self.offender_pos[0], coord=True)
        self.agent.pose.head_tilt(20)
        self.agent.say('Hello!', show_display=True)
        rospy.sleep(1)
        # self.agent.say('I apologize for\nany inconvenience,\nbut unfortunately,', show_display=True)
        # rospy.sleep(4.5)
        self.agent.say(
            'Sorry but\n this room is not\naccessible to guests.', show_display=True)
        rospy.sleep(5)

    def ask_to_action(self, destination):
        # take the offender to the other party guests
        # self.agent.say('Allow me to assist you\nin finding other guests.', show_display=True)
        # self.agent.say('Let me guide you \nto another guest.',show_display=True)
        # rospy.sleep(3)
        self.agent.say('Please follow me!', show_display=True)
        rospy.sleep(1)
        self.agent.move_abs_safe('bedroom_search_reverse')

        self.agent.move_abs_safe(destination)

        self.agent.pose.head_tilt(20)
        self.agent.say('Thank you!', show_display=True)
        rospy.sleep(1)
        self.agent.say('Now you are in\nan appropriate room!',
                       show_display=True)
        rospy.sleep(2.5)
        self.agent.say('Enjoy your party', show_display=True)
        rospy.sleep(2)

    def draw_forbidden_room_marker(self, min_points, max_points):
        cube_points = [[min_points[0], min_points[1], min_points[2]],  # 1
                       [min_points[0], min_points[1], max_points[2]],  # 2
                       [min_points[0], max_points[1], max_points[2]],  # 3
                       [min_points[0], max_points[1], min_points[2]],  # 4
                       [min_points[0], min_points[1], min_points[2]],  # 5
                       [max_points[0], min_points[1], min_points[2]],  # 6
                       [max_points[0], min_points[1], max_points[2]],  # 7
                       [max_points[0], min_points[1], max_points[2]],  # 7
                       [max_points[0], max_points[1], max_points[2]],  # 8
                       [max_points[0], max_points[1], min_points[2]],  # 9
                       [max_points[0], min_points[1], min_points[2]],  # 10
                       [max_points[0], max_points[1], min_points[2]],  # 11
                       [min_points[0], max_points[1], min_points[2]],  # 12
                       [min_points[0], max_points[1], min_points[2]],  # 13
                       [min_points[0], max_points[1], max_points[2]],  # 14
                       [max_points[0], max_points[1], max_points[2]],  # 15
                       [max_points[0], min_points[1], max_points[2]],  # 16
                       [min_points[0], min_points[1], max_points[2]]]  # 17
        marker = self.make_point_marker()
        for p in cube_points:
            marker.points.append(Point(p[0], p[1], p[2]))
        self.mark_pub.publish(marker)

    def make_point_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "unit_vector"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.color = ColorRGBA(1, 1, 0, 1)
        marker.scale.x = 0.01
        marker.points = []
        marker.id = 1

        return marker


class NoLittering:
    def __init__(self, agent, axis_transform):
        self.agent = agent
        self.axis_transform = axis_transform
        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray,
                         self._knee_pose_callback)
        self.garbage_id = None
        self.garbage_pos = None

    def _knee_pose_callback(self, data):
        self.knee_list = np.reshape(data.data, (-1, 2))
        self.min_knee = np.inf
        for x, y in self.knee_list:
            if self.agent.depth_image[y, x] < self.min_knee:
                self.min_knee = self.agent.depth_image[y, x]

        visualize_mode = False
        if visualize_mode:
            for x, y in self.knee_list:
                cv2.circle(self.agent.rgb_img, (x, y), 2,
                           (255, 0, 0), -1, cv2.LINE_AA)
            cv2.imshow('hsr_vision', self.agent.rgb_img)
            cv2.waitKey(1)  # 1 millisecond

    def detect_garbage(self):
        import copy
        self.agent.pose.head_tilt(-40)
        rospy.sleep(1)
        if len(self.agent.yolo_module.yolo_bbox) != 0:

            _pc = self.agent.pc.reshape(480, 640)
            pc_np = np.array(_pc.tolist())[:, :, :3]
            yolo_bbox = copy.deepcopy(self.agent.yolo_module.yolo_bbox)
            for bbox in yolo_bbox:
                garbage_pc = pc_np[bbox[1], bbox[0]]
                print('a')
                garbage_coord = self.axis_transform.transform_coordinate('head_rgbd_sensor_rgb_frame',
                                                                         'map',
                                                                        garbage_pc)

                if garbage_coord[2] < 0.28: # TODO : Check garbage height
                    self.garbage_pos = garbage_coord[0:2]
                    self.agent.say("Litter found.")
                    rospy.sleep(1)
                    return True

                print('4')
        return False

    def find_closest_offender(self):
        # find closest offender in terms of pan degree
        for pan_degree in [60, 0, -60, -120, -180, -220]:
            self.agent.pose.head_pan_tilt(pan_degree, -25)
            rospy.sleep(3)
            print('self.knee_list', self.knee_list)
            for x, y in self.knee_list:
                if pan_degree != 0:
                    move_human_infront(
                        self.agent, self.axis_transform, y, x, coord=False)
                    return True

                else:  # pan_degree == 0 # see infront of litter # yjyoo added
                    return True

        return False

    def clarify_violated_rule(self):
        # go in front of the garbage
        move_human_infront(self.agent, self.axis_transform,
                           self.garbage_pos[1], self.garbage_pos[0], coord=True)
        rospy.sleep(1)

        # find closest offender in terms of pan degree
        # try:
        found_offender = False
        for i in range(2):
            human_detected = self.find_closest_offender()

            if human_detected:
                found_offender = True
                break
            else:
                self.agent.say("Please come closer to me")

        # see the closest offender and clarify what rule is being broken
        # self.agent.pose.head_pan_tilt(robot_to_human_pan, 0)
        # except:
        #     print('[error] in littering, line 314')
        if found_offender is False:
            self.agent.say("Please stand in front of me")
            rospy.sleep(5)

        # clarify the rule
        self.agent.pose.head_tilt(20)
        self.agent.say('Hello!', show_display=True)
        rospy.sleep(1)
        # self.agent.say('I apologize for\nany inconvenience,\nbut unfortunately,', show_display=True)
        # rospy.sleep(4.5)
        self.agent.say(
            'Sorry but\nyou cannot leave\ngarbage on the floor', show_display=True)
        rospy.sleep(5)

    def ask_to_action(self, bin_location):
        self.agent.say(
            "Please pick up\nthe litter in front of me", show_display=True)
        rospy.sleep(8)
        # ask the offender to throw the garbage into the bin
        # self.agent.say('Allow me to assist you\nto throw it into the bin.', show_display=True)
        # rospy.sleep(5)
        self.agent.say('Please follow me\nto the bin', show_display=True)
        rospy.sleep(2)
        self.agent.move_abs_safe(bin_location)
        # rospy.sleep(2)
        self.agent.pose.head_tilt(-60)
        self.agent.say("Please throw\nthe garbage\ninto the bin",
                       show_display=True)

        # confirm_start_time = time.time()
        # while len(self.agent.yolo_module.yolo_bbox) == 0:
        #     if time.time() - confirm_start_time > 10:
        #         break
        #     elif time.time() - confirm_start_time > 5:
        #         self.agent.say("Please trash\n the garbage")
        #         rospy.sleep(2)
        self.agent.pose.head_pan_tilt(90, 20)
        self.agent.say('Thank you!\nEnjoy your party', show_display=True)
        rospy.sleep(3.5)


class DrinkDetection:
    def __init__(self, agent, axis_transform, hand_drink_pixel_dist_threshold):
        self.agent = agent
        rospy.Subscriber('snu/openpose/hand',
                         Int16MultiArray, self._openpose_cb)
        self.thre = hand_drink_pixel_dist_threshold
        self.axis_transform = axis_transform
        self.drink_check = False
        self.drink_list = [0, 1, 2, 3, 4, 5]
        self.marker_maker = MarkerMaker('/snu/human_location')
        self.no_drink_human_coord = None

    def _openpose_cb(self, data):
        data_list = data.data
        self.human_hand_poses = np.reshape(data_list, (-1, 2, 2))

    def show_image(self, l_hand_x, l_hand_y, r_hand_x, r_hand_y):
        img = self.agent.rgb_img
        l_hand_box = tuple(
            map(int, (l_hand_x - self.thre, l_hand_y - self.thre, l_hand_x + self.thre, l_hand_y + self.thre)))
        img = cv2.rectangle(
            img, l_hand_box[0:2], l_hand_box[2:4], color='r', thickness=2)
        r_hand_box = tuple(
            map(int, (r_hand_x - self.thre, r_hand_y - self.thre, r_hand_x + self.thre, r_hand_y + self.thre)))
        img = cv2.rectangle(
            img, r_hand_box[0:2], r_hand_box[2:4], color='r', thickness=2)
        cv2.imshow('img', img)
        cv2.waitKey(1)

    def detect_no_drink_hand(self):
        self.agent.pose.head_tilt(0)
        rospy.sleep(0.5)
        try:
            for human_hand in self.human_hand_poses:  # 사람별
                print("human found")
                l_hand_x, l_hand_y = human_hand[0]
                r_hand_x, r_hand_y = human_hand[1]

                human_coord = [(l_hand_x + r_hand_x) // 2,
                               (l_hand_y + r_hand_y) // 2]

                _pc = self.agent.pc.reshape(480, 640)
                pc_np = np.array(_pc.tolist())[:, :, :3]
                human_pc = pc_np[human_coord[1], human_coord[0]]
                human_coord_in_map = self.axis_transform.transform_coordinate('head_rgbd_sensor_rgb_frame', 'map',
                                                                              human_pc)
                print('test' + str(human_coord_in_map))
                if not self.agent.arena_check.is_in_arena([human_coord_in_map[0], human_coord_in_map[1]]):
                    print("[RULE 4] people out of arena")
                    continue

                drink_check = False

                for _ in range(0, 5):
                    if drink_check:
                        break
                    for bbox in self.agent.yolo_module.yolo_bbox:
                        # drink
                        if self.agent.yolo_module.find_type_by_id(bbox[4]) in self.drink_list:
                            cent_x, cent_y = bbox[0], bbox[1]
                            # print(l_hand_x, l_hand_y, r_hand_x, r_hand_y, cent_x, cent_y)
                            # left hand check
                            if (l_hand_x - self.thre <= cent_x <= l_hand_x + self.thre) and (
                                    l_hand_y - self.thre <= cent_y <= l_hand_y + self.thre):
                                drink_check = True
                                print('check')
                                break
                            # right hand check
                            if (r_hand_x - self.thre <= cent_x <= r_hand_x + self.thre) and (
                                    r_hand_y - self.thre <= cent_y <= r_hand_y + self.thre):
                                drink_check = True
                                print('check')
                                break
                    rospy.sleep(0.2)

                # detect no drink
                if not drink_check:
                    print('Found someone not holding a drink!')
                    # show image (if no drinks)
                    # self.show_image(l_hand_x, l_hand_y, r_hand_x, r_hand_y)

                    # self.no_drink_human_coord = [(l_hand_x + r_hand_x)//2, (l_hand_y + r_hand_y)//2]
                    # modified by lsh... 뎁스로 할때 가끔씩 이상한 위치로 가는 버그가 있어서 베이스링크 기준으로 고쳐봅니다.
                    self.no_drink_human_coord = human_coord_in_map

                    return True
        except:
            print('error in detect_no_drink_hand')
            return False

        return False

    def clarify_violated_rule(self):
        # go to the offender
        move_human_infront(self.agent, self.axis_transform,
                           self.no_drink_human_coord[1], self.no_drink_human_coord[0], coord=True)

        # clarify what rule is being broken
        self.agent.pose.head_tilt(20)
        self.agent.say('Hello!', show_display=True)
        rospy.sleep(1)
        # self.agent.say('I apologize for\nany inconvenience,\nbut unfortunately,', show_display=True)
        # rospy.sleep(4.5)
        self.agent.say(
            'Sorry but\nall guests should\nhave a drink.', show_display=True)
        rospy.sleep(5)

    def ask_to_action(self, bar_location):
        self.agent.say('We prepare some drinks.', show_display=True)
        rospy.sleep(2)
        self.agent.say('Please follow me!', show_display=True)
        rospy.sleep(4)
        self.agent.move_abs_safe(bar_location)
        rospy.sleep(2)
        self.agent.pose.head_tilt(20)
        self.agent.say('Please hold drink\non the bar.', show_display=True)
        rospy.sleep(5)

        self.agent.pose.head_tilt(5)
        self.agent.say("Hold the drink \ninfront of me to double check", show_display=True)
        rospy.sleep(5)
        if self.detect_no_drink_hand():
            self.agent.pose.head_tilt(20)
            self.agent.say("You did not pick up a drink", show_display=True)
            rospy.sleep(2)
            self.agent.say("Please hold your drink at the bar",
                           show_display=True)
            rospy.sleep(5)
        self.agent.pose.head_tilt(5)
        rospy.sleep(2)

        self.agent.pose.head_tilt(20)
        self.agent.say('Thank you!\nEnjoy your party', show_display=True)
        rospy.sleep(3)
        self.agent.move_abs_safe('study_search_reverse')


def stickler_for_the_rules(agent):
    agent.say('start stickler for the rules')
    stickler_start_time = time.time()
    ### task params #################
    # start location: kitchen (search 1st)
    # when marked forbidden room, replace forbidden location to n-2 index search location

    forbidden_search_location = 'bedroom_search'

    break_rule_check_list = {'shoes': False,
                             'room': False,
                             'garbage': False,
                             'drink': False}

    ## params for rule 1. No shoes ##
    entrance = 'shoe_warning'
    ## params for rule 2. forbidden room ##
    # If needed, mark min & max points of all 4 rooms !
    # forbidden_room_min_points = {'bedroom_search': [5.354, -6.1233, 0.03]}
    # forbidden_room_max_points = {'bedroom_search': [9.2773, -3.1132, 2.0]}
    forbidden_room_min_points = {'bedroom_search': [-2.5636, 0.7627, 0.03]}
    forbidden_room_max_points = {'bedroom_search': [-1.0000, 3.0000, 2.0]}
    ## params for rule 3. No littering ##
    bin_location = 'bin_littering'
    ## params for rule 4. Compulsory hydration ####
    hand_drink_pixel_dist_threshold = 40
    compulsory_hydration_bar_location = 'bar_drink'

    ##################################

    #### class instantiation ########
    axis_transform = Axis_transform()
    forbidden_room = ForbiddenRoom(agent, axis_transform,
                                   forbidden_room_min_points[forbidden_search_location],
                                   forbidden_room_max_points[forbidden_search_location])
    shoe_detection = ShoeDetection(agent, axis_transform)
    no_littering = NoLittering(agent, axis_transform)
    drink_detection = DrinkDetection(
        agent, axis_transform, hand_drink_pixel_dist_threshold)
    #################################

    forbidden_room_name = 'bedroom_search'
    search_location_list = ['living_room_search', 'study_search']
    # search_location_list = ['bedroom_search', 'kitchen_search', 'living_room_search', 'study_search',
    #                         'bedroom_search', 'kitchen_search', 'living_room_search', 'study_search',
    #                         'kitchen_search', 'living_room_search', 'study_search',
    #                         'kitchen_search', 'living_room_search', 'study_search']

    agent.pose.head_pan_tilt(0, 0)
    forbidden_search_start = False

    agent.pose.move_pose()

    # while True:
    for search_location in search_location_list:
        # pan_degree_list = [-60, -30, 0, 30, 60]  # default for kitchen
        pan_degree_list = [-30, 0, 30]
        # pan_degree_list = [0]
        # if search_location == "living_room_search":
        #     pan_degree_list = [60, 0, -60, -120, -180, -220]
        # elif search_location == "study_search":
        #     pan_degree_list = [-60, -30, 30, 60]
        # move to the search location


        agent.say(f"I'm moving to\n{search_location}.", show_display=True)
        rospy.sleep(2)
        
        agent.pose.head_tilt(0)
        agent.move_abs_safe(search_location)
        agent.say('I am checking \n the rules', show_display=True)
        rospy.sleep(2)


        # [RULE 2] Forbidden room
        if search_location == 'bedroom_search':
            agent.pose.head_tilt(0)
            for pan_degree in pan_degree_list:
                agent.pose.head_pan(pan_degree)
                rospy.sleep(1)

                if forbidden_room.detect_forbidden_room():
                    # go to the offender and clarify what rule is being broken
                    forbidden_room.clarify_violated_rule()

                    agent.say('Please leave this room empty',
                                show_display=True)
                    rospy.sleep(2)
                    agent.say('After you leave, \nI will guide you \nto other guests', show_display=True)
                    rospy.sleep(7)
                    agent.move_abs_safe('bedroom_doublecheck')
                    agent.say('Checking the room if empty', show_display=True)
                    agent.pose.head_tilt(-15)
                    for pan_degree in [45, 0, -45]:
                        # marking forbidden room violation detection
                        break_rule_check_list['room'] = True

                        agent.pose.head_pan(pan_degree)
                        rospy.sleep(1.5)

                        if forbidden_room.detect_forbidden_room(): # TODO : check knee outside the bedroom
                            agent.say('Oh my god. \n You are still here',
                                        show_display=True)
                            rospy.sleep(3.5)
                            agent.say('Lets leave with me')
                            break

                    # take the offender to the other party guests
                    forbidden_room.ask_to_action('kitchen_search')
                    break

        # If not forbidden scan location >> check RULE 1, 3, 4
        # TODO: adjust living room pan degree & rotation position
        else:
            for pan_degree in pan_degree_list:
                agent.pose.head_pan(pan_degree)

                # # [RULE 4] Compulsory hydration : tilt 0
                # if break_rule_check_list['drink'] is False and drink_detection.detect_no_drink_hand():
                #     # marking no drink violation detection
                #     break_rule_check_list['drink'] = True

                #     # go to the offender and clarify what rule is being broken
                #     drink_detection.clarify_violated_rule()
                #     # ask offender to grab a drink
                #     drink_detection.ask_to_action(
                #         compulsory_hydration_bar_location)
                #     break

                # [RULE 1] No shoes : tilt -20, -40
                if break_rule_check_list['shoes'] is False and shoe_detection.detect():
                    # marking whether wearing shoes violation is detected
                    break_rule_check_list['shoes'] = True

                    # go to the offender and clarify what rule is being broken
                    shoe_detection.clarify_violated_rule()
                    # take the offender to the entrance & ask to take off their shoes
                    shoe_detection.ask_to_action(entrance)
                    break

                # # [RULE 3] No littering : tilt -40
                # if break_rule_check_list['garbage'] is False and no_littering.detect_garbage():
                #     # marking no littering violation detection
                #     break_rule_check_list['garbage'] = True

                #     # go to the offender and clarify what rule is being broken
                #     no_littering.clarify_violated_rule()
                #     # ask the offender to pick up and trash the garbage
                #     no_littering.ask_to_action(bin_location)
                #     break

        if sum(break_rule_check_list.values())==4:
            break

        # Move to another room
        agent.pose.head_pan_tilt(0, 0)
        agent.say("Now I'm going to\nmove to another room.",
                    show_display=True)
        rospy.sleep(3)
        agent.say(
            "If you are in my path,\nplease move to the side.", show_display=True)
        rospy.sleep(6)


    #
    # while True:
    #     for idx, search_location in enumerate(search_location_list):
    #         # case 1 : 8 min over
    #         if time.time() - stickler_start_time > 480:
    #             forbidden_search_start = True
    #             break
    #         # case 2 : 3 rule already detected
    #         if break_rule_check_list['shoes'] and break_rule_check_list['garbage'] and break_rule_check_list['drink']:
    #             forbidden_search_start = True
    #             break
    #
    #         # move to the search location
    #         agent.move_abs_safe(search_location)
    #
    #         # agent.say('I want to see you all.')
    #         # rospy.sleep(2)
    #
    #         # If not forbidden scan location >> check RULE 1, 3, 4
    #         # TODO: adjust living room pan degree & rotation position
    #         # -> depend on search location which is dependent to Arena map
    #         pan_degree_list = [-60, -30, 0, 30, 60]
    #         if search_location == "living_room_search":
    #             pan_degree_list = [90, 45, 0, -45, -90, -135, -180, -225]
    #         elif search_location == "study_search":
    #             pan_degree_list = [-60, -30, 30, 60]
    #
    #         for pan_degree in pan_degree_list:
    #             agent.pose.head_pan(pan_degree)
    #
    #             # [RULE 4] Compulsory hydration : tilt 0
    #             if break_rule_check_list['drink'] is False and drink_detection.detect_no_drink_hand():
    #                 # marking no drink violation detection
    #                 break_rule_check_list['drink'] = True
    #
    #                 # go to the offender and clarify what rule is being broken
    #                 drink_detection.clarify_violated_rule()
    #                 # ask offender to grab a drink
    #                 drink_detection.ask_to_action(
    #                     compulsory_hydration_bar_location)
    #                 break
    #
    #             # [RULE 1] No shoes : tilt -20, -40
    #             if break_rule_check_list['shoes'] is False and shoe_detection.run():
    #                 # marking whether wearing shoes violation is detected
    #                 break_rule_check_list['shoes'] = True
    #
    #                 # go to the offender and clarify what rule is being broken
    #                 shoe_detection.clarify_violated_rule()
    #                 # take the offender to the entrance & ask to take off their shoes
    #                 shoe_detection.ask_to_action(entrance)
    #                 break
    #
    #             # [RULE 3] No littering : tilt -40
    #             if break_rule_check_list['garbage'] is False and no_littering.detect_garbage():
    #                 # marking no littering violation detection
    #                 break_rule_check_list['garbage'] = True
    #
    #                 # go to the offender and clarify what rule is being broken
    #                 no_littering.clarify_violated_rule()
    #                 # ask the offender to pick up and trash the garbage
    #                 no_littering.ask_to_action(bin_location)
    #                 break
    #
    #
    #         # Move to another room
    #         agent.pose.head_pan_tilt(0, 0)
    #         agent.say("Now I'm going to\nmove to another room.",
    #                   show_display=True)
    #         rospy.sleep(3)
    #         agent.say(
    #             "If you are in my path,\nplease move to the side.", show_display=True)
    #         rospy.sleep(4)
    #     if forbidden_search_start:
    #         break
    #
    # # go to the forbidden room
    # agent.move_abs_safe(forbidden_room_name)
    # # [RULE 2] Forbidden room
    # while not break_rule_check_list['room']:
    #     agent.pose.head_tilt(0)
    #     rospy.sleep(1)
    #     for pan_degree in [-60, -30, 0, 30, 60]:
    #         agent.pose.head_pan(pan_degree)
    #         rospy.sleep(2)
    #
    #         if forbidden_room.detect_forbidden_room():
    #             # marking forbidden room violation detection
    #             # go to the offender and clarify what rule is being broken
    #             forbidden_room.clarify_violated_rule()
    #
    #             agent.say('Please leave this room empty',
    #                       show_display=True)
    #             rospy.sleep(2)
    #             agent.say('After you leave, \nI will guide you \nto other guests', show_display=True)
    #
    #             rospy.sleep(7)
    #
    #             agent.say('Checking the room if empty', show_display=True)
    #
    #             for pan_degree in [-60, -30, 0, 30, 60]:
    #                 break_rule_check_list['room'] = True
    #
    #                 agent.pose.head_pan(pan_degree)
    #                 rospy.sleep(2)
    #
    #                 if forbidden_room.detect_forbidden_room():
    #                     agent.say('Oh my god',
    #                               show_display=True)
    #                     rospy.sleep(2)
    #
    #                     agent.say('You are still here',
    #                               show_display=True)
    #
    #                     rospy.sleep(2)
    #                     agent.say('Lets leave with me')
    #                     break
    #
    #             # take the offender to the other party guests
    #             forbidden_room.ask_to_action('kitchen_search')
    #             break

# FIXME : need to handle when human_infront_coord_in_map overlaps with any furniture


def move_human_infront(agent, axis_transform, y, x, coord=False):
    print("move to human")
    robot_coord_in_map = axis_transform.transform_coordinate(
        'head_rgbd_sensor_rgb_frame', 'map', [0, 0, 0])
    if not coord:
        _pc = agent.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        print('x, y', x, y)
        human_pc = pc_np[y, x]
        print("coord_y coord_x", y, x)
        print("human_pc", human_pc)
        human_coord_in_map = axis_transform.transform_coordinate(
            'head_rgbd_sensor_rgb_frame', 'map', human_pc)
    else:
        human_coord_in_map = [x, y, 1.0]

    print("human coord in map", human_coord_in_map)
    if np.any(np.isnan(np.array(human_coord_in_map[:2]))):
        return

    human_to_robot = robot_coord_in_map - human_coord_in_map
    human_to_robot_normal = human_to_robot / np.linalg.norm(human_to_robot)

    human_infront_coord_in_map = human_coord_in_map + human_to_robot_normal * 1.0
    human_infront_coord_in_map[2] = math.atan2(
        human_to_robot_normal[1], human_to_robot_normal[0]) + math.pi
    # print(human_infront_coord_in_map[2])
    # self.marker_maker.pub_marker([human_infront_coord_in_map[0], human_infront_coord_in_map[1], 1], 'map')

    print(human_infront_coord_in_map)
    agent.move_abs_coordinate(human_infront_coord_in_map)


if __name__ == '__main__':
    from hsr_agent.agent import Agent

    rospy.init_node('stickler_rule_test')
    agent = Agent()
    axis_transform = Axis_transform()

    # forbidden_room_min_points = [-2.01, 3.74, 0]
    # forbidden_room_max_points = [-0.66, 4.97, 2.0]
    # forbidden_room = ForbiddenRoom(agent, axis_transform,
    #                                forbidden_room_min_points,
    #                                forbidden_room_max_points)

    shoe_detection = ShoeDetection(agent)

    hand_drink_pixel_dist_threshold = 50
    # drink_detection = DrinkDetection(agent, axis_transform, hand_drink_pixel_dist_threshold)

    agent.pose.head_tilt(-20)
    agent.pose.head_pan(0)
    shoe_detection.find_shoes()
