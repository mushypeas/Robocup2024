import rospy
from std_msgs.msg import Int16MultiArray, String
from cv_bridge import CvBridge
import sys
sys.path.append('.')
from utils.depth_to_pc import Depth2PC
from utils.axis_transform import Axis_transform
from hsr_agent.agent import Agent
from utils.marker_maker import MarkerMaker
from module.person_following_bot.follow import HumanReidAndFollower
import time
from std_srvs.srv import Empty, EmptyRequest
from sensor_msgs.msg import Image
import copy
import dynamic_reconfigure.client
import cv2
import mediapipe as mp
import numpy as np
from collections import deque
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import time
from sklearn.preprocessing import StandardScaler

class HumanFollowing:
    def __init__(self, agent, human_reid_and_follower, start_location, goal_radius, stop_rotate_velocity, tilt_angle, stt_option):
        self.agent = agent
        rospy.sleep(2)
        # self.omni_client = dynamic_reconfigure.client.Client("/omni_path_follower", config_callback=self.omni_cb)
        # self.omni_client.update_configuration({"base_max_linear_velocity": .2,
        #                                   "base_max_angular_velocity": 0.5})
        self.human_reid_and_follower = human_reid_and_follower
        self.start_location = start_location
        self.goal_radius = goal_radius
        self.stop_rotate_velocity = stop_rotate_velocity
        if self.stop_rotate_velocity < 0:
            self.stop_rotate_velocity = - self.stop_rotate_velocity
        self.d2pc = Depth2PC() # to depth
        self.axis_transform = Axis_transform()
        self.marker_maker = MarkerMaker('/snu/robot_path_visu')
        self.human_box_list = [None] # for human_reid_and_follower
        self.bridge = CvBridge()
        self.show_byte_track_image = False
        self.track_queue = deque()
        self.obstacle_offset = 0.0
        self.last_say = time.time()
        self.tilt_angle = tilt_angle
        self.stt_option = stt_option
        self.save_one_time = False
        bytetrack_topic = '/snu/bytetrack_img'
        rospy.Subscriber(bytetrack_topic, Image, self._byte_cb)
        rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_cb)

        rospy.loginfo("LOAD HUMAN FOLLOWING")

    def freeze_for_humanfollowing(self):
        self.show_byte_track_image = True
        self.agent.pose.head_pan_tilt(0, -self.tilt_angle)
        rospy.sleep(1)
        head_map_client = dynamic_reconfigure.client.Client("/tmc_map_merger/inputs/head_rgbd_sensor",
                                                                 config_callback=self.head_map_cb)
        head_map_client.update_configuration({"enable": True})
        # head_obstacle_client = dynamic_reconfigure.client.Client(
        #     '/tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle', config_callback=self.head_circle_cb)
        # head_obstacle_client.update_configuration({"forbid_radius": 0.05,
        #                                                 "obstacle_radius": 0.25,
        #                                                 "obstacle_occupancy": 80
        #                                                 })
        # base_obstacle_client = dynamic_reconfigure.client.Client(
        #     "/tmc_map_merger/inputs/base_scan/obstacle_circle", config_callback=self.base_circle_cb)
        # base_obstacle_client.update_configuration({
        #     "forbid_radius": 0.05,
        #     "obstacle_radius": 0.25,
        #     "obstacle_occupancy": 80
        # })

        yolo_map_client = dynamic_reconfigure.client.Client("/tmc_map_merger/inputs/custom_point_cloud",
                                                                 config_callback=self.head_map_cb)
        yolo_map_client.update_configuration({"enable": True})
        # yolo_obstacle_client = dynamic_reconfigure.client.Client(
        #     '/tmc_map_merger/inputs/custom_point_cloud/obstacle_circle', config_callback=self.head_circle_cb)
        # yolo_obstacle_client.update_configuration({"forbid_radius": 0.05,
        #                                                 "obstacle_radius": 0.25,
        #                                                 "obstacle_occupancy": 80
        #                                                 })
        stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_client.call(EmptyRequest())

    def _byte_cb(self, data):
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        if self.show_byte_track_image:
            self.agent.head_display_image_pubish(img)

    def head_circle_cb(self, config):
        rospy.loginfo(config)

    def head_map_cb(self, config):
        rospy.loginfo(config)

    def base_circle_cb(self, config):
        rospy.loginfo(config)

    def _human_yolo_cb(self, data):
        '''
        snu/yolo will be depreciated
        '''
        data_list = data.data
        if data_list[0] == -1:
            self.human_box_list = [None]
        else:
            self.human_box_list = [data_list[0], #[human_id, target_tlwh, target_score]
                                  np.asarray([data_list[1], data_list[2], data_list[3], data_list[4]], dtype=np.int64),
                                  data_list[5]]

    def check_human_pos(self, human_box_list, location=False):
        x = human_box_list[1][0]
        y = human_box_list[1][1]
        w = human_box_list[1][2]
        h = human_box_list[1][3]
        center = [y + int(h/2), x + int(w/2)] # (y,x)
        # print("center", center)
        if location:
            if center[1] < 140:
                return 'l'
            if center[1] > 500:
                return 'r'

        if center[1] < 0 or center[1] > 640:
            return True # stop

        return False # pass

    def stt_destination(self, stt_option):
        cur_pose = self.agent.get_pose(print_option=False)
        # print("in area", [cur_pose[0], cur_pose[1]], "last moved time", time.time() - self.agent.last_moved_time)
        if (time.time() - self.agent.last_moved_time > 11.0) and not (self.start_location[0] - self.goal_radius < cur_pose[0] < self.start_location[0] + self.goal_radius and \
            self.start_location[1] - self.goal_radius < cur_pose[1] < self.start_location[1] + self.goal_radius):

            print("lmt_if", self.agent.last_moved_time)

            self.show_byte_track_image = False
            self.agent.move_base.base_action_client.cancel_all_goals()
            if stt_option:
                self.agent.say('Is this your destination?\nsay yes or no after a ding sound', show_display=True)
                rospy.sleep(5)
                answer, _ = self.agent.stt(3.)
                question_num = 0
                while 'yes' not in answer and 'no' not in answer and question_num < 3:
                    self.agent.say('Answer only by \nyes or no', show_display=True)
                    print('Answer only by \nyes or no')
                    rospy.sleep(2.5)
                    answer, _ = self.agent.stt(3.)
                    question_num += 1
                if 'yes' not in answer and 'no' not in answer:
                    answer = 'yes'
                if 'yes' in answer:
                    return True
                elif 'no' in answer:
                    self.agent.say('Okay! I will follow you', show_display=True)
                    print('Okay! I will follow you')
                    rospy.sleep(1)
                    self.show_byte_track_image = True
                    self.agent.last_moved_time = time.time()
                    return False
            else:
                # '''
                # updated 0706
                # '''
                #
                # self.agent.say('Is this your destination?\n Then, do not move for five seconds', show_display=True)
                # rospy.sleep(5)
                # for i in range(5):
                #     self.agent.say(str(5-i))
                #     print("lmt", self.agent.last_moved_time)
                #     if time.time() - self.agent.last_moved_time < 2.0:
                #         self.agent.say('Okay I will follow you', show_display=True)
                #
                #         self.show_byte_track_image = True
                #         self.agent.last_moved_time = time.time()
                #         return False
                #     rospy.sleep(1.5)
                # '''
                # done
                # '''
                return True
        else:
            # if time.time() - self.agent.last_moved_time > 7.0 and time.time() - self.last_say > 4.0:
            #     self.agent.say('Is this your destination?\n Then do not move', show_display=False)
            #     self.last_say = time.time()
            #     rospy.sleep(1)
            #     print("seven seconds")
            if time.time() - self.agent.last_moved_time > 3.0 and time.time() - self.last_say > 4.0:
                self.agent.say('Please come closer to me', show_display=True)
                print("Please come closer to me")
                self.last_say = time.time()
                rospy.sleep(1)
            return False

    def follow_human(self, start_time=time.time(), pose_save_time_period=10):
        if self.human_box_list[0] is None: # no human detected
            if self.stt_destination(self.stt_option):
                return True
            return False
        human_info_ary = copy.deepcopy(self.human_box_list)

        if self.check_human_pos(human_info_ary):  # If human is on the edge of the screen
            print("2.1 go to center!")
            if time.time()-self.last_say > 3:
                self.agent.say('Please, go to center!')
                self.last_say = time.time()
            rospy.sleep(2)
            if self.stt_destination(self.stt_option):
                return True
            return False
        else:
            # we move "depth" to the front
            depth = np.asarray(self.d2pc.depth)
            twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)
            if calc_z > 2000.0 and time.time()-self.last_say > 5:
                self.agent.say('Your so far')
                rospy.sleep(0.5)
                self.agent.say('Please move slowly!')
                self.last_say = time.time()


            # print("2.2 linear x", twist.linear.x, "angular", twist.angular.z)
            if twist.linear.x == 0 and twist.angular.z == 0:
                # change angular.z
                loc = self.check_human_pos(human_info_ary, location=True)
                if loc == 'l':
                    print("left")
                    # twist.angular.z = -self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, self.stop_rotate_velocity, wait=False)
                    rospy.sleep(.5)
                if loc == 'r':
                    print("right")
                    # twist.angular.z = +self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, -self.stop_rotate_velocity, wait=False)
                    rospy.sleep(.5)

                if self.stt_destination(self.stt_option):
                    return True

                return False

            target_xyyaw = self.calculate_twist_to_human(twist, calc_z)
            self.marker_maker.pub_marker([target_xyyaw[0], target_xyyaw[1], 1], 'base_link')
            # 2.4 move to human
            self.agent.move_rel(target_xyyaw[0], target_xyyaw[1], target_xyyaw[2], wait=False)
            rospy.sleep(.5)
            cur_pos = self.agent.get_pose(print_option=False)
            if round((time.time() - start_time) % pose_save_time_period) == 0:
                if self.save_one_time:
                    self.track_queue.append((cur_pos[0], cur_pos[1], cur_pos[2]+np.pi))
                    self.save_one_time = False
                    print('queue_size', len(self.track_queue))
            else:
                self.save_one_time = True

            # check arrive at destination zone
            if self.stt_destination(self.stt_option):
                return True

        return False

    def follow_human_queue(self, try_bytetrack):
        if try_bytetrack:
            if self.human_box_list[0] is None: # no human detected
                self.agent.move_abs_coordinate(self.start_location, wait=False)
                print('No human detected, go start location')
                rospy.sleep(0.5)
                return False
            human_info_ary = copy.deepcopy(self.human_box_list)
            depth = np.asarray(self.d2pc.depth)
            twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)

            if calc_z < 1200.0:
                if twist.linear.x == 0 and twist.angular.z == 0:
                    self.agent.move_base.base_action_client.cancel_all_goals()
                    self.agent.move_rel(0, 0, 0, wait=False)
                    rospy.sleep(0.5)
                    return False

                target_xyyaw = self.calculate_twist_to_human(twist, calc_z)
                self.marker_maker.pub_marker([target_xyyaw[0], target_xyyaw[1], 1], 'base_link')
                # 2.4 move to human
                self.agent.move_base.base_action_client.cancel_all_goals()
                self.agent.move_rel(target_xyyaw[0], target_xyyaw[1], target_xyyaw[2], wait=False)
                rospy.sleep(1)
                print("Human is close to me... Move towards human")
            else:
                self.agent.move_abs_coordinate_safe(self.start_location, angle=90)
                print("Go back to start location since human is too far")
                rospy.sleep(1)
            
        else:
            self.agent.move_abs_coordinate_safe(self.start_location, thresh=0.5, timeout=8, giveup_timeout=10)
            
        cur_pos = self.agent.get_pose(print_option=False)

        # check arrive at destination zone
        if self.start_location[0] - self.goal_radius < cur_pos[0] < self.start_location[0] + self.goal_radius and \
            self.start_location[1] - self.goal_radius < cur_pos[1] < self.start_location[1] + self.goal_radius:
            self.agent.move_rel(0, 0, 0)
            # arrive at start location zone
            return True
        return False

    def calculate_twist_to_human(self, twist, calc_z):
        ######TODO#################
        #calc_z is mm
        calc_z /= 1000.0
        cur_linear = twist.linear.x
        # cur_linear = calc_z * .8
        if twist.linear.x == 0 and twist.angular.z == 0:
            return [0, 0, 0]

        target_z_rgbd_frame = cur_linear * np.cos(twist.angular.z)
        target_x_rgbd_frame = - cur_linear * np.sin(twist.angular.z)
        target_xyz_base_link = self.axis_transform.transform_coordinate( \
            'head_rgbd_sensor_rgb_frame', 'base_link', [target_x_rgbd_frame, 0, target_z_rgbd_frame])
        target_yaw_base_link = np.arctan2(target_xyz_base_link[1], target_xyz_base_link[0])
        while cur_linear < calc_z * .8:
            target_z_rgbd_frame = cur_linear * np.cos(twist.angular.z)
            target_x_rgbd_frame = - cur_linear * np.sin(twist.angular.z)
            target_xyz_base_link = self.axis_transform.transform_coordinate( \
                'head_rgbd_sensor_rgb_frame', 'base_link', [target_x_rgbd_frame, 0, target_z_rgbd_frame])
            h, w = self.agent.dynamic_obstacles_with_static.shape
            target_x_in_pixel =max(min(h - 1, h // 2 - int(target_xyz_base_link[0] / 0.05)), 0)
            target_y_in_pixel = max(min(w - 1, w // 2 - int(target_xyz_base_link[1] / 0.05)), 0)
            target_yaw_base_link = np.arctan2(target_xyz_base_link[1], target_xyz_base_link[0])
            if self.agent.dynamic_obstacles_with_static[target_y_in_pixel, target_x_in_pixel] > 30:
                cur_linear += .1
            else:
                # print("cur_linear", cur_linear, calc_z, self.agent.dynamic_obstacles_with_static[target_y_in_pixel, target_x_in_pixel])
                return (target_xyz_base_link[0], target_xyz_base_link[1], target_yaw_base_link)

        return (target_xyz_base_link[0], target_xyz_base_link[1], target_yaw_base_link)

class BagInspection:
    def __init__(self, agent):
        self.agent = agent
        self.pointing_dir = -1 # left : 0, right : 1
        self.bridge = CvBridge()
        self.marker_maker = MarkerMaker('/snu/robot_path_visu')
        #Subscribe to take a bag
        self.yolo_bag_list = [43]
        self.d2pc = Depth2PC()

        yolo_topic = '/snu/yolo_img'
        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'  # use data in agent
        rospy.Subscriber('/snu/yolo_conf', Int16MultiArray, self._bag_yolo_cb)
        rospy.Subscriber(yolo_topic, Image, self._yolo_img_cb)
        rospy.Subscriber(rgb_topic, Image, self._rgb_cb)

        self.axis_transform = Axis_transform()

        self.bag_yolo_data = []
        self.conf_threshold = 75 # write in percent

    def _bag_yolo_cb(self, data):
        self.bag_yolo_data = data.data


    def detect_bag_3d(self, bag_height, timeout=5.0):
        start_time = time.time()
        while time.time() - start_time <= timeout:
            _pc = self.agent.pc.reshape(480, 640)
            pc_np = np.array(_pc.tolist())[:, :, :3]
            bag_yolo_data = self.bag_yolo_data

            self.agent.head_display_image_pubish(self.yolo_img)
            for idx in range(len(bag_yolo_data) // 6):
                item = bag_yolo_data[6 * idx: 6 * (idx + 1)]
                cent_x, cent_y, width, height, class_id, conf_percent = item
                
                # if class_id is not paper_bag
                if class_id not in self.yolo_bag_list and conf_percent < .65:
                    continue
                '''
                BAG PICKING 0706
                '''
                if class_id == 42:
                    bag_height = 0.115
                elif class_id == 43:
                    bag_height = 0.225
                else:
                    bag_height = 0.37

                '''
                '''
                
                start_x = cent_x - (width // 2)
                start_y = cent_y - (height // 2)
                object_pc = pc_np[start_y:start_y+height, start_x:start_x+width]
                object_pc = object_pc.reshape(-1, 3)
                points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

                # floor_height = 0.05 #lower bound for picking..., in pnu
                floor_height = 0.03 # in snu
                floor_to_base = .0 #in pnu
                # floor_to_base = .2 # in snu
                bag_height -= floor_to_base

                height_threshold = [floor_height, bag_height]
                # print(np.nanmin(points_by_base_link), np.nanmax(points_by_base_link))

                # 1. hard-constraint thresholding by fixed parameters
                points_by_base_link = points_by_base_link[np.where((points_by_base_link[:, 2] > height_threshold[0])
                                                                 & (points_by_base_link[:, 2] < height_threshold[1]))]

                print('points_by_base_link', points_by_base_link.shape)
                try:
                    scaler = StandardScaler()
                    points_by_base_link = points_by_base_link[np.abs(scaler.fit_transform(points_by_base_link))[:, 2] < 1]
                    # 2. soft-constraint thresholding by objects depth
                    # object_depth = 0.25 # in pnu
                    object_depth = 0.05 # in snu
                    point_min_x = round(np.nanmin(points_by_base_link[:, 0]), 4)
                    points_by_base_link = points_by_base_link[
                        np.where((points_by_base_link[:, 0] < point_min_x + object_depth))]



                    # 3. select min_max points for draw 3d box
                    min_points = [round(np.nanmin(points_by_base_link[:, 0]), 4),
                                  round(np.nanmin(points_by_base_link[:, 1]), 4),
                                  round(np.nanmin(points_by_base_link[:, 2]), 4)]
                    max_points = [round(np.nanmax(points_by_base_link[:, 0]), 4),
                                  round(np.nanmax(points_by_base_link[:, 1]), 4),
                                  round(np.nanmax(points_by_base_link[:, 2]), 4)]
                except ValueError:
                    print('[YOLO] zero-size array for ')
                    continue

                target_base_xyz = [min_points[0],     # x
                                   (min_points[1] + max_points[1]) / 2,     # y
                                   max_points[2],                           # z
                                   class_id]                                # class_id
                object_size = [max_points[0] - min_points[0],
                               max_points[1] - min_points[1],
                               max_points[2] - min_points[2]]
                cube_points = [[min_points[0], min_points[1], min_points[2]],  #1
                               [min_points[0], min_points[1], max_points[2]],  #2
                               [min_points[0], max_points[1], max_points[2]],  #3
                               [min_points[0], max_points[1], min_points[2]],  #4
                               [min_points[0], min_points[1], min_points[2]],  #5
                               [max_points[0], min_points[1], min_points[2]],  #6
                               [max_points[0], min_points[1], max_points[2]],  #7
                               [max_points[0], max_points[1], max_points[2]],  #8
                               [max_points[0], max_points[1], min_points[2]],  #9
                               [max_points[0], min_points[1], min_points[2]],  #10
                               [max_points[0], max_points[1], min_points[2]],  #11
                               [min_points[0], max_points[1], min_points[2]],  #12
                               [min_points[0], max_points[1], min_points[2]],  #13
                               [min_points[0], max_points[1], max_points[2]],  #14
                               [max_points[0], max_points[1], max_points[2]],  #15
                               [max_points[0], min_points[1], max_points[2]],  #16
                               [min_points[0], min_points[1], max_points[2]]]  #17

                self.agent.yolo_module.draw_line_marker(cube_points, idx)
                return target_base_xyz, object_size, bag_height
        return None, None, None

    def bag_grasping_check(self, thres=0.6):
        _depth = self.agent.depth_image
        _depth_bottom = _depth[378:456, 528:612]
        #(528, 217), (612, 272)
        _depth_top = _depth[217:272, 528:612]
        # cv2.rectangle(im0, (528, 378), (612, 456), (255, 0, 0), thickness=1, lineType=cv2.LINE_AA)
        print("bot",np.nanmean(_depth_bottom) / 1000)
        print("top", np.nanmean(_depth_top) / 1000)
        if (np.nanmean(_depth_bottom) / 1000 < thres) or (np.nanmean(_depth_top) / 1000 < thres):
            return True
        else:
            return False


    def _rgb_cb(self, data):
        # add mediapipe module and find pointing direction
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')


    def _yolo_img_cb(self, data):
        self.yolo_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')


    def pointing_position(self):
        image = self.agent.rgb_img
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing_styles = mp.solutions.drawing_styles
        mp_pose = mp.solutions.pose

        # For webcam input:
        try:
            with mp_pose.Pose(
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5) as pose:

                image.flags.writeable = False
                image_hight, image_width, _ = image.shape
                results = pose.process(image)

                left_elbow = [results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x * image_width,
                              results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y * image_hight]
                left_wrist = [results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * image_width,
                              results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * image_hight]

                right_elbow = [results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x * image_width,
                               results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y * image_hight]
                right_wrist = [results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * image_width,
                               results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * image_hight]

                right_grad = (right_elbow[0] - right_wrist[0]) / (right_elbow[1] - right_wrist[1])
                left_grad = (left_elbow[0] - left_wrist[0]) / (left_elbow[1] - left_wrist[1])

                # Draw the pose annotation on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                mp_drawing.draw_landmarks(
                        image,
                        results.pose_landmarks,
                        mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                point_thre = 0.4

                if right_grad > point_thre or left_grad > point_thre:
                    direction = 'right'  # right
                elif right_grad < -point_thre or left_grad < -point_thre:
                    direction = 'left' # left
                else:
                    direction = None
            rospy.loginfo("direction: {0}, right_grad: {1}, left_grad: {2}".format(direction\
                                                                 , right_grad, left_grad))
            image = cv2.flip(image, 1)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image, direction
        except Exception as e:
            print('[MediaPipe]', e)
            return None, None

    def run_bag_inspection(self, pointing_dir, bag_height):
        before_pick_pose = self.agent.get_pose(print_option=False)
        target_base_xyz, object_size, bag_height = self.rotate_to_find_bag(pointing_dir, bag_height)
        if target_base_xyz is None:
            print("1.4 We cannot find a bag. Please hand me a bag!")
            self.agent.say("We cannot find a bag.", show_display=True)
            self.agent.open_gripper()
            rospy.sleep(1)
            self.agent.say("Please Hand me bag", show_display=True)
            self.agent.pose.neutral_pose()
            rospy.sleep(1)
            for i in range(5, 0, -1):
                self.agent.say(str(i))
                rospy.sleep(1)
            self.agent.grasp()
            self.agent.pose.move_to_go()
        else:
            # Todo: grasp towards the target point
            print("1.5 bag_coord_baselink_final", target_base_xyz)
            self.agent.open_gripper()
            self.agent.say("I will pick up")
            print("1.6 let's pick up the bag")
            self.agent.pose.head_pan_tilt(0, 0)

            hand_dist_xyz = self.agent.yolo_module.calculate_dist_to_pick(target_base_xyz, 5)
            mov_x, mov_y = hand_dist_xyz[0], hand_dist_xyz[1]
            print("1.7 go to the direction", (mov_x, mov_y))
            self.marker_maker.pub_marker([mov_x, mov_y, 1], 'base_link')
            self.agent.move_rel(mov_x, mov_y, wait=True)

            if object_size[0] >= object_size[1]:  # x > y
                bag_orientation = 0
            else:
                bag_orientation = -1.57

            isSuccc = False
            isSecond = False
            for i in range(2):
                if isSecond:
                    if bag_orientation == 0:
                        bag_orientation = -1.57
                    else:
                        bag_orientation = 0
                isSecond = True
                self.agent.pose.pick_up_bag(bag_height, bag_orientation)
                self.agent.pose.arm_lift_top_down(bag_height)
                self.agent.grasp()
                self.agent.pose.bag_inspection_pose()
                self.agent.say("Let's check.")
                rospy.sleep(3)
                # 잡아서 눈앞에 들었을 때 있는지 확인
                # TODO:  Bag check아직 완벽하지 않은 알고리즘
                if self.bag_grasping_check():
                    # 있으면 Human following
                    print("Robot grasp bag!")
                    self.agent.say("Grasp Successful")
                    rospy.sleep(1)
                    isSuccc = True
                    break
                else:
                    self.agent.open_gripper()
            if not isSuccc:
                self.agent.pose.hand_me_bag()
                self.agent.open_gripper()
                self.agent.say("Please Hand me bag", show_display=True) # TODO:tts
                self.agent.pose.neutral_pose()
                for i in range(5, 0, -1):
                    self.agent.say(str(i))
                    rospy.sleep(1)
                self.agent.grasp()
            self.agent.pose.move_to_go()
            # self.agent.move_rel(-mov_x, -mov_y, 0, wait=True)
            self.agent.move_abs_coordinate(before_pick_pose, wait=True)
        self.agent.say("I am ready to follow you", show_display=True)
        rospy.sleep(2)

    def rotate_to_find_bag(self, pointing_dir, bag_height): # leftside:1, rightside:-1
        if pointing_dir is None:
            direction = 0
        elif pointing_dir == 'left':
            direction = 1
        else:               #'right'
            direction = -1
        target_base_xyz, object_size, bag_height = self.detect_bag_3d(bag_height, timeout=1)

        #0도일때
        for tilt_idx in range(2):
            if tilt_idx == 0:
                self.agent.pose.head_pan_tilt(0, -45)
            else:    
                self.agent.pose.head_pan_tilt(0, -30)
            print("Searching")
            rospy.sleep(2)
            target_base_xyz, object_size, bag_height = self.detect_bag_3d(bag_height, timeout=1)
            if target_base_xyz is not None:
                print("We find bag!")
                return target_base_xyz, object_size, bag_height

        if target_base_xyz is None: # box is not detected
            if direction == 0:
                range_num = 3
                direction = 1
            else:
                range_num = 3

            for _ in range(range_num):
                self.agent.move_rel(0, 0, 60 * direction / 180 * np.pi, wait=True)
                for tilt_idx in range(2):
                    if tilt_idx == 0:
                        self.agent.pose.head_pan_tilt(0, -45)
                    else:    
                        self.agent.pose.head_pan_tilt(0, -30)
                    print("Searching")
                    rospy.sleep(2)
                    target_base_xyz, object_size, bag_height = self.detect_bag_3d(bag_height, timeout=1)
                    if target_base_xyz is not None:
                        print("We find bag!")
                        return target_base_xyz, object_size, bag_height
            print("We cannot find bag.. i'll just rotate back")
            # if pointing_dir is not None:
            #     self.agent.move_rel(0, 0, -np.pi * direction, wait=True)
            self.agent.move_rel(0, 0, -np.pi * direction, wait=True)
            return None, None, None
        else:
            print("We find bag!")
            return target_base_xyz, object_size, bag_height

def head_map_cb(config):
    rospy.loginfo(config)

def carry_my_luggage(agent):
    # task params
    bag_search_limit_time = 15
    goal_radius = 1.0
    pose_save_time_period = 7
    start_location = agent.get_pose(print_option=False)
    bag_height = 0.25
    stop_rotate_velocity = 1.2
    try_bag_picking = True
    try_bytetrack = False
    map_mode = False
    stt_option = False
    tilt_angle = 20
    

    # Capture target
    demotrack_pub = rospy.Publisher('/snu/demotrack', String, queue_size=10)

    ## class instantiation
    bag_inspection = BagInspection(agent)

    human_reid_and_follower = HumanReidAndFollower(init_bbox=[320 - 100, 240 - 50, 320 + 100, 240 + 50],
                                                   frame_shape=(480, 640),
                                                   stop_thres=.7,
                                                   linear_max=.3,
                                                   angular_max=.2,
                                                   tilt_angle=tilt_angle)
    human_following = HumanFollowing(agent, human_reid_and_follower, start_location, goal_radius, stop_rotate_velocity, tilt_angle, stt_option)
    #####################
    # 0. start
    agent.say('start carry my luggage!')
    agent.pose.move_pose()
    rospy.sleep(3)
    agent.pose.head_pan_tilt(0, 0)

    ####################
    if try_bag_picking:
        # [Bonus] 1. bag detection
        print("1-1. Hello, Please pointing where the bag is")
        agent.say("Please point ")
        agent.say("where the bag is")
        #########
        # 1.1 check human pointing direction
        bag_searching_start_time = time.time()
        while time.time() - bag_searching_start_time <= bag_search_limit_time:
            start_searching_time = time.time()
            while time.time() - start_searching_time <= 5:  # check bag during 5 seconds
                visualized_image, pointing_dir = bag_inspection.pointing_position()
                if visualized_image is not None:
                    agent.head_display_image_pubish(visualized_image)
            # check human direction
            if pointing_dir is None:
                print("1-2. Please point again")
                agent.say("Please point again")
            else:
                break
            ########
        print("1-3.pointing direction:", pointing_dir)
        # 1.3 check shopping bag
        agent.pose.head_pan_tilt(0, -30)
        bag_inspection.run_bag_inspection(pointing_dir, bag_height)

    else:   # no try bag picking
        agent.open_gripper()
        rospy.sleep(1)
        agent.say("Please Hand me bag", show_display=True)
        agent.pose.neutral_pose()
        rospy.sleep(1)
        for i in range(5, 0, -1):
            agent.say(str(i))
            rospy.sleep(1)
        agent.grasp()
        agent.pose.move_to_go()

    ######################
    # 2. human following

    demotrack_pub.publish(String('target'))
    agent.pose.head_pan_tilt(0, 0)
    agent.say("If you are arrived\n at the destination\nPlease stand still", show_display=True)
    rospy.sleep(3)
    # agent.say("Please do not move for ten seconds when you arrived at the destination", show_display=True)
    agent.say("Please keep the one meter between us!", show_display=True)
    rospy.sleep(3)
    human_following.freeze_for_humanfollowing()
    rospy.loginfo("start human following")
    agent.say("Come in front of me\nThen I will follow you", show_display=True)
    rospy.sleep(1.5)
    start_time = time.time()
    agent.last_moved_time = time.time() # reset the last_moved_time
    while not rospy.is_shutdown():
        end_following = human_following.follow_human(start_time, pose_save_time_period)
        if end_following:
            human_following.show_byte_track_image = False
            stop_client = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
            print("VIEWPOINT CONTROLLER ON")
            stop_client.call(EmptyRequest())
            break

    # 3. reach destination
    print('3. Please take your bag')
    agent.say("I arrived at the destination!", show_display=True)
    print("I arrived at the destination!")
    rospy.sleep(3)
    # stop feature extraction
    demotrack_pub.publish(String('off_feature'))

    agent.say("Take your bag", show_display=True)
    agent.pose.hand_me_bag()
    rospy.sleep(3)
    for i in range(5, 0, -1):
        agent.say(str(i))
        rospy.sleep(1)
    agent.open_gripper()
    rospy.sleep(1)
    agent.pose.move_pose()
    agent.grasp()

    # 4. back to the start location
    agent.move_rel(0, 0, np.pi, wait=True)
    rospy.loginfo("go back to original location")
    
    # version 2 : track queue
    if not map_mode:
        track_queue = human_following.track_queue  # get trace of the robot

        while len(track_queue):
            coordinate = track_queue.pop()
            agent.move_abs_coordinate_safe(coordinate)
            # rospy.sleep(0.5)
            print('go to arena')

    # version 3 : ByteTrack
    else:
        human_following.show_byte_track_image = True

        # disable head rgbd camera while going back to the starting poing : deprecated
        # head_map_client = dynamic_reconfigure.client.Client("/tmc_map_merger/inputs/head_rgbd_sensor",
        #                                                     config_callback=head_map_cb)
        # head_map_client.update_configuration({"enable": False})

        while not rospy.is_shutdown():
            end_following = human_following.follow_human_queue(try_bytetrack)
            if end_following:
                human_following.show_byte_track_image = False
                break

    agent.say("Finish carry my luggage", show_display=True)

if __name__ == '__main__':
    rospy.init_node('carry_my_luggage_test')
    agent = Agent()
    bag_inspection = BagInspection(agent)

    # for _ in range(10):
    #     print(np.asarray(bag_inspection.d2pc.depth))
    # input()

    ########
    # check_bag = True
    # if check_bag:
    #     print(bag_inspection.bag_grasping_check())
    #     sys.exit()
    ########
    agent.pose.move_pose()
    bag_height = 0.25
    bag_inspection = BagInspection(agent)
    agent.pose.head_pan_tilt(0, -30)
    # while True:
    #     bag_inspection.detect_bag_3d(bag_height, timeout=5.0)

    target_base_xyz, object_size, bag_height = bag_inspection.detect_bag_3d(bag_height, timeout=5.0)
    if object_size[0] >= object_size[1]: # x > y
        bag_orientation = 0
    else:
        bag_orientation = -1.57
    hand_dist_xyz = agent.yolo_module.calculate_dist_to_pick(target_base_xyz, 5)
    print('object_size', object_size)
    print('hand_dist_xyz', hand_dist_xyz)
    agent.move_rel(0, hand_dist_xyz[1], wait=True)
    rospy.sleep(1)
    agent.move_rel(hand_dist_xyz[0], 0, wait=True)

    agent.pose.pick_up_bag(bag_height, bag_orientation)
    agent.pose.arm_lift_top_down(bag_height)
    agent.grasp()
    agent.pose.pick_up_bag(bag_height, bag_orientation)
    agent.open_gripper()
    agent.pose.move_pose()

