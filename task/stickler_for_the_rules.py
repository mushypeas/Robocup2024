from utils.marker_maker import MarkerMaker
from utils.axis_transform import Axis_transform
import rospy
import time
import math
import sys
import cv2
import numpy as np
import mediapipe as mp
import open_clip
from PIL import Image
import torch
# from module.CLIP.clip_detection import CLIPDetector, CLIPDetectorConfig
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import cv2
import numpy as np
import mediapipe as mp
import sys

sys.path.append('.')

class CLIP:
    def __init__(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')
        self.clip_model.eval()  # model in train mode by default, impacts some models with BatchNorm or stochastic depth active
        self.tokenizer = open_clip.get_tokenizer('ViT-B-32')
        self.clip_model = self.clip_model.eval().requires_grad_(False).to(self.device)

class ShoeDetection:
    def __init__(self, agent, clip):
        self.agent = agent
        self.axis_transform = Axis_transform()
        # self.shoe_detected = False
        self.shoe_position = None
        # FIXME: return shoe_human_pos when detect guest wearing shoe
        # self.shoe_human_pos = None
        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray,
                         self._knee_pose_callback)
        rospy.Subscriber('/snu/openpose/ankle', Int16MultiArray,
                         self._ankle_pose_callback)
        self.knee_list = None
        self.ankle_list = None
        self.image_save_index = 0

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms('ViT-B/32')
        state_dict = torch.load('module/CLIP/openfashionclip.pt', map_location=self.device)
        self.clip_model.load_state_dict(state_dict['CLIP'])
        self.clip_model = self.clip_model.eval().requires_grad_(False).to(self.device)
        self.tokenizer = open_clip.get_tokenizer('ViT-B-32')

        # self.clip_model = clip.clip_model
        # self.clip_preprocess = clip.clip_preprocess
        # self.tokenizer = clip.tokenizer

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')
        # self.clip_model.eval()  # model in train mode by default, impacts some models with BatchNorm or stochastic depth active
        # self.tokenizer = open_clip.get_tokenizer('ViT-B-32')
        # self.clip_model = self.clip_model.eval().requires_grad_(False).to(self.device)

    def _knee_pose_callback(self, data):
        self.knee_list = np.reshape(data.data, (-1, 2))
        self.min_knee = np.inf
        for x, y in self.knee_list:
            if self.agent.depth_image[y, x] < self.min_knee:
                self.min_knee = self.agent.depth_image[y, x]

    def _ankle_pose_callback(self, data):
        self.ankle_list = np.reshape(data.data, (-1, 2))
        # sort by x
        self.ankle_list = self.ankle_list[self.ankle_list[:, 0].argsort()]

    def find_shoes_clip(self, double_check=False):

        # Prompt for CLIP model
        prompt = "a photo of a"
        # text_inputs = ["shoes", "socks", "barefoot"]
        text_inputs = ["shoes", "socks"] 
        text_inputs = [prompt + " " + t for t in text_inputs]
        tokenized_prompt = self.tokenizer(text_inputs).to(self.device)

        # double check mode
        if double_check:
            text_inputs = text_inputs[:2]
            count = 0
            while count < 5:
                image = self.agent.rgb_img
                crop_img = image[:,80:560]

                cv2.imshow('crop_img', crop_img)
                cv2.waitKey(1)
                cv2.imwrite(f'/home/tidy/Robocup2024/module/CLIP/shoe_crop_img_double_check_{count}.jpg', crop_img)

                # Preprocess image
                crop_img = Image.fromarray(cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB))
                crop_img = self.clip_preprocess(crop_img)
                crop_img = crop_img.unsqueeze(0).to(self.device)

                # Encode image and text features
                with torch.no_grad():
                    image_features = self.clip_model.encode_image(crop_img)
                    text_features = self.clip_model.encode_text(tokenized_prompt)
                    image_features /= image_features.norm(dim=-1, keepdim=True)
                    text_features /= text_features.norm(dim=-1, keepdim=True)

                    # Calculate text probabilities
                    text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)

                # Convert probabilities to percentages
                text_probs_percent = text_probs * 100
                text_probs_percent_np = text_probs_percent.cpu().numpy()
                formatted_probs = ["{:.2f}%".format(value) for value in text_probs_percent_np[0]]

                print("Labels probabilities in percentage:", formatted_probs)
                if text_probs_percent_np[0][0] > 85:
                    return True
                count += 1

            return False
            

        try:
            # hsr 카메라 시야각: 58 (horizontal), 45 (vertical) -> 대회때 필요할지도
            # horizontal을 60도라고 생각했을 때, horizontal 45도 시야각 = 480 pixel (== 세로 pixel) 30도 시야각 = 320 pixel (좌우 160 pixel 제외)

            # 고개 -10도로 설정
            self.agent.pose.head_tilt(-10)
            rospy.sleep(3)
            # 사람 없으면 return
            print('knee_list:', self.knee_list)
            if len(self.knee_list) == 0:
                print('shoe detection no person')
                return True
            # 일단 openpose 기준으로 사람 리스트 생성
            no_shoes_person = [False for _ in range(len(self.knee_list)//2)]
            # 가로 시야각 30도 내 사람만 체크, 시야각 밖의 사람은 True로 설정
            for human_idx in range(len(self.knee_list)//2):
                left_x, left_y = self.knee_list[human_idx*2]
                right_x, right_y = self.knee_list[human_idx*2+1]
                if left_x >= 640-120 or right_x <= 120:
                    no_shoes_person[human_idx] = True
            print(f'no_shoes_person: {no_shoes_person}')
            no_shoes_prob = [0 for _ in range(no_shoes_person.count(False))]
            coord_map_list = [None for _ in range(no_shoes_person.count(False))]

            for head_tilt_angle in [-20]:
            
                self.agent.pose.head_tilt(head_tilt_angle)
                rospy.sleep(3)

                count = 0
                while count < 5:
                    
                    image = self.agent.rgb_img

                    for human_idx in range(len(self.ankle_list)//2):
                        if human_idx*2+1>=len(self.ankle_list):
                            continue
                        left_x, left_y = self.ankle_list[human_idx*2]
                        right_x, right_y = self.ankle_list[human_idx*2+1]

                        if left_x >= 640-120 or right_x <= 120:
                            continue
                        if human_idx >= len(no_shoes_person):
                            continue
                        if no_shoes_person[human_idx]:
                            continue

                        print(f'shoe idx: {human_idx}, ankle: {left_x, left_y, right_x, right_y}')

                        y_min = min(left_y, right_y)

                        crop_img = image[
                            max(0,(left_y+right_y-224)//2):min(480-1,(left_y+right_y+224)//2), 
                            max(0,(left_x+right_x-224)//2):min(640-1,(left_x+right_x+224)//2)
                        ]
                        
                        human_coord = [(left_x+right_x)//2, max((left_y+right_y)//2+50, 0)]
                        _pc = self.agent.pc.reshape(480, 640)
                        pc_np = np.array(_pc.tolist())[:, :, :3]
                        human_pc = pc_np[human_coord[1], human_coord[0]]
                        human_coord_in_map = self.axis_transform.transform_coordinate('head_rgbd_sensor_rgb_frame', 'map', human_pc)
                        coord_map_list[human_idx] = human_coord_in_map

                        cv2.imshow('crop_img', crop_img)
                        cv2.waitKey(1)
                        cv2.imwrite(f'/home/tidy/Robocup2024/module/CLIP/shoe_crop_img_{self.image_save_index}.jpg', crop_img)
                        self.image_save_index += 1

                        # Preprocess image
                        crop_img = Image.fromarray(cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB))
                        crop_img = self.clip_preprocess(crop_img)
                        crop_img = crop_img.unsqueeze(0).to(self.device)
                        # print(crop_img.size())

                        # Encode image and text features
                        with torch.no_grad():
                            image_features = self.clip_model.encode_image(crop_img)
                            text_features = self.clip_model.encode_text(tokenized_prompt)
                            image_features /= image_features.norm(dim=-1, keepdim=True)
                            text_features /= text_features.norm(dim=-1, keepdim=True)

                            # Calculate text probabilities
                            # text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)
                            text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)
                            # text_probs = (100.0 * image_features @ text_features.T)
                            # print('probs before softmax', text_probs)
                            # text_probs = text_probs.softmax(dim=-1)
                            # print('probs after softmax', text_probs)

                        # Convert probabilities to percentages
                        text_probs_percent = text_probs * 100
                        text_probs_percent_np = text_probs_percent.cpu().numpy()
                        formatted_probs = ["{:.2f}%".format(value) for value in text_probs_percent_np[0]]

                        print("Labels probabilities in percentage:", formatted_probs)
                        # if text_probs_percent_np[0][0] > 60:
                        #     # no_shoes_person[human_idx] = False
                        #     self.shoe_position = human_coord_in_map
                        # else:
                        #     # self.shoe_position = [left_x, left_y]
                        #     no_shoes_person[human_idx] = True
                        # if text_probs_percent_np[0][0] < 60:
                        no_shoes_prob[human_idx] += text_probs_percent_np[0][0]
                        # if text_probs_percent_np[0][0] < 90:
                        #     no_shoes_person[human_idx] = True
                        # else:
                        #     self.shoe_position = human_coord_in_map
                    
                            
                    count += 1
            no_shoes_prob = np.array(no_shoes_prob)
            no_shoes_prob = no_shoes_prob / 5
            print(f'no_shoes_prob: {no_shoes_prob}')
            for human_idx in range(len(no_shoes_person)):
                if no_shoes_prob[human_idx] < 95:
                    no_shoes_person[human_idx] = True
                else:
                    self.shoe_position = coord_map_list[human_idx]

            # if text_probs_percent_np[0][0] < 90:
            #     no_shoes_person[human_idx] = True
            # else:
            #     self.shoe_position = human_coord_in_map

            if no_shoes_person.count(False) > 0:
                for person_idx in range(len(no_shoes_person)):
                    print(f'person {person_idx}: {no_shoes_person[person_idx]}')
                return False
            else:
                return True
        except:
            return True

    # def find_shoes(self):
    #     mp_drawing = mp.solutions.drawing_utils
    #     mp_objectron = mp.solutions.objectron

    #     # For webcam input:
    #     with mp_objectron.Objectron(static_image_mode=False,
    #                                 max_num_objects=5,
    #                                 min_detection_confidence=0.65,
    #                                 min_tracking_confidence=0.99,
    #                                 model_name='Shoe') as objectron:
    #         image = self.agent.rgb_img
    #         # To improve performance, optionally mark the image as not writeable to
    #         # pass by reference.
    #         image.flags.writeable = False
    #         image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #         results = objectron.process(image)

    #         # Draw the box landmarks on the image.
    #         image.flags.writeable = True
    #         image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    #         if results.detected_objects is not None:
    #             for detected_object in results.detected_objects:
    #                 print(detected_object)
    #                 mp_drawing.draw_landmarks(
    #                     image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
    #                 mp_drawing.draw_axis(image, detected_object.rotation,
    #                                      detected_object.translation)

    #                 shoe_x = 0
    #                 shoe_y = 0
    #                 for i in detected_object.landmarks_2d.landmark:
    #                     shoe_x += i.x / \
    #                         len(detected_object.landmarks_2d.landmark)
    #                     shoe_y += i.y / \
    #                         len(detected_object.landmarks_2d.landmark)

    #                 self.shoe_position = [shoe_x, shoe_y]

    #                 print("shoes found")
    #                 cv2.imshow('MediaPipe Objectron', cv2.flip(image, 1))
    #                 cv2.waitKey(1)
    #             return True
    #     return False

    # def run(self):
    #     rospy.sleep(1)
    #     # head_tilt_list = [-30]
    #     head_tilt_list = [-20, -40]

    #     for tilt in head_tilt_list:
    #         self.agent.pose.head_tilt(tilt)
    #         if self.find_shoes():
    #             return True
    #         rospy.sleep(1)

    #     return False

    def clarify_violated_rule(self):
        # _pc = self.agent.pc.reshape(480, 640)
        # pc_np = np.array(_pc.tolist())[:, :, :3]
        # shoe_x = self.shoe_position[0]
        # shoe_y = self.shoe_position[1]
        # human_pc = pc_np[max(0, min(480-1, int(shoe_y*480))),
        #                  max(0, min(640-1, int(shoe_x*640)))]  # Shoe Position!
        # closest_ppos = self.axis_transform.transform_coordinate(
        #     'head_rgbd_sensor_rgb_frame', 'map', human_pc)

        # if len(closest_ppos) != 0:
            # go to the offender
            # print("CLOSEST PPOS SHOES", closest_ppos)
            # move_human_infront(self.agent, self.axis_transform,
            #                    closest_ppos[1], closest_ppos[0], coord=True)

        move_human_infront(self.agent, self.axis_transform, self.shoe_position[1], self.shoe_position[0], coord=True)

        # clarify what rule is being broken
        self.agent.pose.head_tilt(20)
        self.agent.say('Hello!', show_display=True)
        rospy.sleep(1)
        # self.agent.say('I apologize for\nany inconvenience,\nbut unfortunately,', show_display=True)
        # rospy.sleep(4.5)
        self.agent.say(
            'Sorry but\n all guests should take off\n their shoes outside the entrance.', show_display=True)
        rospy.sleep(4)

        # else:
            # self.agent.say("Please come closer to me")

                

    def ask_to_action(self, entrance, current_location='kitchen_search'):
        rospy.sleep(1)
        # take the offender to the entrance
        # self.agent.say('I will guide you\nto the entrance.',
        #                show_display=True)
        # rospy.sleep(3)
        self.agent.say('Please follow me! \n If you are in my path, \n please move out!', show_display=True)
        rospy.sleep(1)

        if current_location=='kitchen_search2':
            self.agent.move_abs_safe('kitchen_living_middle')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('livingroom_leave')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('hallway_enter')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location=='kitchen_search':
            self.agent.move_abs_safe('office_leave2')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('office_leave1')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('hallway_enter')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location=='livingroom_search':
            self.agent.move_abs_safe('livingroom_leave')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('hallway_enter')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)

        self.agent.move_abs_safe(entrance)

        # ask to take off their shoes
        self.agent.pose.head_tilt(20)
        self.agent.say('Please put your shoes\n outside the entrance', show_display=True)
        rospy.sleep(3)

        self.agent.say('I will wait a few seconds\nfor you to take off your shoes', show_display=True)
        rospy.sleep(10)

        rebelion_count = 0
        while rebelion_count < 3:
            self.agent.pose.head_tilt(-40)
            self.agent.say('Are you finished?\nCome in front of me.\nLet me see your feet', show_display=True)
            rospy.sleep(6)

            # if self.find_shoes():
            if not self.find_shoes_clip(double_check=True):
                self.agent.pose.head_tilt(20)
                self.agent.say('You are still wearing shoes!', show_display=True)
                rospy.sleep(2)
                # 0609
                # self.agent.say(f'I will wait five more seconds\nfor you to take off your shoes', show_display=True)
                # rospy.sleep(8)
                # rebelion_count += 1
                # self.agent.pose.head_tilt(20)
                self.agent.say('Please take off your shoes.\nEnjoy your party', show_display=True)
                rospy.sleep(4)
                break
            else:
                self.agent.pose.head_tilt(20)
                self.agent.say('Thank you!\nEnjoy your party', show_display=True)
                rospy.sleep(3)
                return

        # self.agent.pose.head_tilt(20)
        # self.agent.say('I give up.\nEnjoy your party', show_display=True)
        # rospy.sleep(2.5)


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

    def clarify_violated_rule(self, double=False):
        # go to the offender and clarify what rule is being broken
        move_human_infront(self.agent, self.axis_transform,
                           self.offender_pos[1], self.offender_pos[0], coord=True)
        self.agent.pose.head_tilt(20)
        if not double:
            self.agent.say('Hello!', show_display=True)
            rospy.sleep(1)
        self.agent.say('This room is not\naccessible to guests.', show_display=True)
        rospy.sleep(3)

    # def ask_to_action(self, destination):
    #     # take the offender to the other party guests
    #     # self.agent.say('Allow me to assist you\nin finding other guests.', show_display=True)
    #     # self.agent.say('Let me guide you \nto another guest.',show_display=True)
    #     # rospy.sleep(3)
    #     self.agent.say('Please follow me!', show_display=True)
    #     rospy.sleep(1)
    #     self.agent.pose.move_pose() # 0609
    #     self.agent.move_abs_safe('bedroom_search_reverse')

    #     self.agent.move_abs_safe(destination)

    #     self.agent.pose.head_tilt(20)
    #     # self.agent.pose.head_pan(135) # 0609
    #     self.agent.say('Thank you!', show_display=True)
    #     rospy.sleep(1)
    #     self.agent.say('Now you are in\nan appropriate room!',
    #                    show_display=True)
    #     rospy.sleep(2.5)
    #     self.agent.say('Enjoy your party', show_display=True)
    #     rospy.sleep(2)
    #     self.agent.pose.head_pan(0) # 0609

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
        rospy.sleep(4)
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
        # for pan_degree in [60, 0, -60, -120, -180, -220]:
        for pan_degree in [90, 60, 30, 0, -30, -60, -90, -120, -150, -180]:
            self.agent.pose.head_pan_tilt(pan_degree, -10)
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
        rospy.sleep(4)

    def ask_to_action(self, bin_location, current_location='kitchen_search'):
        self.agent.say(
            "Please pick up\nthe litter in front of me", show_display=True)
        rospy.sleep(5)
        # rospy.sleep() # 0609
        # ask the offender to throw the garbage into the bin
        # self.agent.say('Allow me to assist you\nto throw it into the bin.', show_display=True)
        # rospy.sleep(5)
        self.agent.say('Please follow me\nto the bin! \nIf you are in my path, \n please move out!', show_display=True)
        rospy.sleep(2)
        self.agent.pose.head_pan(0) # 0609

        if current_location=='kitchen_search':
            self.agent.move_abs_safe('kitchen_search')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location=='kitchen_search2':
            self.agent.move_abs_safe('kitchen_search2')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location=='livingroom_search':
            self.agent.move_abs_safe('kitchen_living_middle')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location=='hallway_search':
            self.agent.move_abs_safe('hallway_enter')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('livingroom_leave')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('kitchen_living_middle')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)



        self.agent.move_abs_safe(bin_location)
        # rospy.sleep(2)
        # self.agent.pose.head_tilt(-60) # 0609
        self.agent.say("Please throw\nthe garbage\ninto the bin",
                       show_display=True)
        rospy.sleep(4) # 0609

        # confirm_start_time = time.time()
        # while len(self.agent.yolo_module.yolo_bbox) == 0:
        #     if time.time() - confirm_start_time > 10:
        #         break
        #     elif time.time() - confirm_start_time > 5:
        #         self.agent.say("Please trash\n the garbage")
        #         rospy.sleep(2)
        # self.agent.pose.head_pan_tilt(90, 20) # 0609
        self.agent.pose.head_tilt(20)
        self.agent.say('Thank you!\nEnjoy your party', show_display=True)
        rospy.sleep(3)


class DrinkDetection:
    def __init__(self, agent, axis_transform, hand_drink_pixel_dist_threshold, clip):
        self.agent = agent
        # rospy.Subscriber('snu/openpose/hand',
        #                  Int16MultiArray, self._openpose_cb)
        # rospy.Subscriber('snu/openpose/bbox',
        #                  Int16MultiArray, self._openpose_bbox_cb)
        rospy.Subscriber('/snu/openpose/human_bbox_with_pub',
                          Int16MultiArray, self._openpose_cb)
        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray,
                         self._knee_pose_callback)
        # self.thre = hand_drink_pixel_dist_threshold
        self.axis_transform = axis_transform
        # self.drink_check = False
        # self.marker_maker = MarkerMaker('/snu/human_location') # ?? 
        self.no_drink_human_coord = None # 이건 필요
        # self.detector = CLIPDetector(config=DRINK_CONFIG, mode="HSR")
        self.image_save_index = 0

        self.clip_model = clip.clip_model
        self.clip_preprocess = clip.clip_preprocess
        self.tokenizer = clip.tokenizer

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')
        # self.clip_model.eval()  # model in train mode by default, impacts some models with BatchNorm or stochastic depth active
        # self.tokenizer = open_clip.get_tokenizer('ViT-B-32')
        # self.clip_model = self.clip_model.eval().requires_grad_(False).to(self.device)
        # print(self.clip_preprocess)

    def _openpose_cb(self, data):
        data_list = data.data
        # self.human_hand_poses = np.reshape(data_list, (-1, 2, 2))
        # self.human_hand_poses = np.reshape(data_list, (-1, 2))
        self.human_bbox_with_hand = np.reshape(data_list, (-1, 4, 2))
        # sort by top left x
        self.human_bbox_with_hand = self.human_bbox_with_hand[self.human_bbox_with_hand[:, 0, 0].argsort()]

    def _knee_pose_callback(self, data):
        self.knee_list = np.reshape(data.data, (-1, 2))
        # sort by x
        self.knee_list = self.knee_list[self.knee_list[:, 0].argsort()]

    # def show_image(self, l_hand_x, l_hand_y, r_hand_x, r_hand_y):
    #     img = self.agent.rgb_img
    #     l_hand_box = tuple(
    #         map(int, (l_hand_x - self.thre, l_hand_y - self.thre, l_hand_x + self.thre, l_hand_y + self.thre)))
    #     img = cv2.rectangle(
    #         img, l_hand_box[0:2], l_hand_box[2:4], color='r', thickness=2)
    #     r_hand_box = tuple(
    #         map(int, (r_hand_x - self.thre, r_hand_y - self.thre, r_hand_x + self.thre, r_hand_y + self.thre)))
    #     img = cv2.rectangle(
    #         img, r_hand_box[0:2], r_hand_box[2:4], color='r', thickness=2)
    #     cv2.imshow('img', img)
    #     cv2.waitKey(1)

    # def find_drink_yolo(self):
    #     self.agent.pose.head_tilt(20)
    #     rospy.sleep(4)
    #     try:
    #         for _ in range(5):
    #             if len(self.agent.yolo_module.yolo_bbox) != 0:
    #                 return True
    #         return False
    #     except:
    #         return False

    def find_drink(self, double_check=False):

        # self.agent.pose.head_tilt(10)
        # rospy.sleep(0.5)

        # Prompt for CLIP model
        prompt = "a photo of a"
        text_inputs = ["human who is holding drink", "human with empty hand", "human whose hand is not visible"]
        text_inputs = [prompt + " " + t for t in text_inputs]
        tokenized_prompt = self.tokenizer(text_inputs).to(self.device)

        # double check mode
        if double_check:
            self.agent.pose.head_tilt(20)
            text_inputs = text_inputs[:2]
            count = 0
            while count < 5:
                image = self.agent.rgb_img
                crop_img = image[:,80:560]

                cv2.imshow('crop_img', crop_img)
                cv2.waitKey(1)
                cv2.imwrite(f'/home/tidy/Robocup2024/module/CLIP/crop_img_double_check_{count}.jpg', crop_img)

                # Preprocess image
                crop_img = Image.fromarray(cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB))
                crop_img = self.clip_preprocess(crop_img)
                crop_img = crop_img.unsqueeze(0).to(self.device)

                # Encode image and text features
                with torch.no_grad():
                    image_features = self.clip_model.encode_image(crop_img)
                    text_features = self.clip_model.encode_text(tokenized_prompt)
                    image_features /= image_features.norm(dim=-1, keepdim=True)
                    text_features /= text_features.norm(dim=-1, keepdim=True)

                    # Calculate text probabilities
                    text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)

                # Convert probabilities to percentages
                text_probs_percent = text_probs * 100
                text_probs_percent_np = text_probs_percent.cpu().numpy()
                formatted_probs = ["{:.2f}%".format(value) for value in text_probs_percent_np[0]]

                print("Labels probabilities in percentage:", formatted_probs)
                if text_probs_percent_np[0][0] > 90:
                    return True
                count += 1

            return False
            

        try:
            # hsr 카메라 시야각: 58 (horizontal), 45 (vertical) -> 대회때 필요할지도
            # horizontal을 60도라고 생각했을 때, horizontal 45도 시야각 = 480 pixel (== 세로 pixel) 30도 시야각 = 320 pixel (좌우 160 pixel 제외)

            # 고개 0도로 설정
            # openpose 성능 이슈 -> 움직인 뒤 충분히 sleep 해줘야 함... 안그러면 움직이는 도중의 blurry한 사진이 들어가고, openpose에서는 아무것도 뱉지 않음
            self.agent.pose.head_tilt(0)
            rospy.sleep(3)
            # 사람 없으면 return
            print('knee_list:', self.knee_list)
            if len(self.knee_list) == 0:
                return True
            # 일단 openpose 기준으로 사람 리스트 생성
            print('human_bbox_with_hand', self.human_bbox_with_hand)
            drink_person = [False for _ in range(len(self.knee_list)//2)]
            # 가로 시야각 30도 내 사람만 체크, 시야각 밖의 사람은 True로 설정
            for human_idx in range(len(self.knee_list)//2):
                left_x, _ = self.knee_list[human_idx*2]
                right_x, _ = self.knee_list[human_idx*2+1]
                if left_x >= 640-120 or right_x <= 120:
                    drink_person[human_idx] = True
            print(f'drink_person: {drink_person}')
            if drink_person.count(False) == 0:
                return True
            drink_prob = [0 for _ in range(len(drink_person))]
            coord_map_list = [None for _ in range(len(drink_prob))]

            hand_thres_person = 50

            for head_tilt_angle in [0]:
            
                self.agent.pose.head_tilt(head_tilt_angle)
                rospy.sleep(2)

                count = 0
                while count < 5:
                    
                    image = self.agent.rgb_img

                    for _, human_bbox_with_hand in enumerate(self.human_bbox_with_hand):
                        top_left_x, top_left_y = human_bbox_with_hand[0]
                        bottom_right_x, bottom_right_y = human_bbox_with_hand[1]

                        human_idx = -1
                        for knee_idx, knee in enumerate(self.knee_list):
                            if (knee[0] >= top_left_x-50 and knee[0] <= bottom_right_x+50) and (knee[1] >= top_left_y-50 and knee[1] <= bottom_right_y+50):
                                human_idx = knee_idx // 2
                                break
                        if human_idx == -1:
                            continue

                        print(f'human idx: {human_idx}, bbox: {top_left_x, top_left_y, bottom_right_x, bottom_right_y}, hand: {human_bbox_with_hand[2:]}')

                        if top_left_x >= 640-120 or bottom_right_x <= 120:
                            continue
                        if human_idx>=len(drink_person):
                            continue
                        if drink_person[human_idx]:
                            continue

                        l_hand = None
                        r_hand = None
                        for hand_coord in human_bbox_with_hand[2:]:
                            if hand_coord[0]!=-1:
                                if l_hand is None:
                                    l_hand = hand_coord
                                else:
                                    r_hand = hand_coord

                        # 1. both hands visible
                        if l_hand is not None and r_hand is not None:
                            if l_hand[0] > r_hand[0]:
                                l_hand, r_hand = r_hand, l_hand

                            crop_y_coord_0 = 0
                            crop_y_coord_1 = 480-1
                            crop_x_coord_0 = 0
                            crop_x_coord_1 = 640-1

                            l_hand_x, l_hand_y = l_hand
                            r_hand_x, r_hand_y = r_hand
                            center_hand_x = (l_hand_x + r_hand_x) // 2
                            center_hand_y = (l_hand_y + r_hand_y) // 2
                            square_len = max(abs(r_hand_x - l_hand_x), abs(r_hand_y - l_hand_y)) + hand_thres_person
                            if square_len < 224:
                                square_len = 224
                            crop_y_coord_0 = max(0, center_hand_y - int(square_len // 2))
                            crop_y_coord_1 = min(480-1, center_hand_y + int(square_len // 2))
                            crop_x_coord_0 = max(0, center_hand_x - int(square_len // 2))
                            crop_x_coord_1 = min(640-1, center_hand_x + int(square_len // 2))
                            square_xy_diff = (crop_y_coord_1 - crop_y_coord_0) - (crop_x_coord_1 - crop_x_coord_0)
                            calib_cnt = 2
                            while square_xy_diff != 0 and calib_cnt > 0:
                                # print('square_xy_diff', square_xy_diff)
                                if square_xy_diff < 0:
                                    if crop_y_coord_0 == 0:
                                        crop_y_coord_1 = min(480-1, crop_y_coord_1 + int(abs(square_xy_diff)))
                                    else:
                                        crop_y_coord_0 = max(0, crop_y_coord_0 - int(abs(square_xy_diff)))
                                elif square_xy_diff > 0:
                                    if crop_x_coord_0 == 0:
                                        crop_x_coord_1 = min(640-1, crop_x_coord_1 + int(abs(square_xy_diff)))
                                    else:
                                        crop_x_coord_0 = max(0, crop_x_coord_0 - int(abs(square_xy_diff)))
                                else:
                                    break
                                square_xy_diff = (crop_y_coord_1 - crop_y_coord_0) - (crop_x_coord_1 - crop_x_coord_0)
                                calib_cnt -= 1
                            
                            # print(f'square length x: {crop_x_coord_1 - crop_x_coord_0}, y: {crop_y_coord_1 - crop_y_coord_0}')
                            crop_img = image[crop_y_coord_0:crop_y_coord_1, crop_x_coord_0:crop_x_coord_1]

                        # 2. only one hand visible
                        elif l_hand is not None:
                            l_hand_x, l_hand_y = l_hand
                            crop_y_coord_0 = 0
                            crop_y_coord_1 = 480-1
                            crop_x_coord_0 = 0
                            crop_x_coord_1 = 640-1
                            square_len = 224
                            crop_y_coord_0 = max(0, l_hand_y - int(square_len // 2))
                            crop_y_coord_1 = min(480-1, l_hand_y + int(square_len // 2))
                            crop_x_coord_0 = max(0, l_hand_x - int(square_len // 2))
                            crop_x_coord_1 = min(640-1, l_hand_x + int(square_len // 2))
                            square_xy_diff = (crop_y_coord_1 - crop_y_coord_0) - (crop_x_coord_1 - crop_x_coord_0)
                            calib_cnt = 2
                            while square_xy_diff != 0 and calib_cnt > 0:
                                # print('square_xy_diff', square_xy_diff)
                                if square_xy_diff < 0:
                                    if crop_y_coord_0 == 0:
                                        crop_y_coord_1 = min(480-1, crop_y_coord_1 + int(abs(square_xy_diff)))
                                    else:
                                        crop_y_coord_0 = max(0, crop_y_coord_0 - int(abs(square_xy_diff)))
                                elif square_xy_diff > 0:
                                    if crop_x_coord_0 == 0:
                                        crop_x_coord_1 = min(640-1, crop_x_coord_1 + int(abs(square_xy_diff)))
                                    else:
                                        crop_x_coord_0 = max(0, crop_x_coord_0 - int(abs(square_xy_diff)))
                                else:
                                    break
                                square_xy_diff = (crop_y_coord_1 - crop_y_coord_0) - (crop_x_coord_1 - crop_x_coord_0)
                                calib_cnt -= 1
                            # print(f'square length x: {crop_x_coord_1 - crop_x_coord_0}, y: {crop_y_coord_1 - crop_y_coord_0}')
                            crop_img = image[crop_y_coord_0:crop_y_coord_1, crop_x_coord_0:crop_x_coord_1]

                        # 3. no hand visible
                        else:
                            # crop_img = image[
                            #     max(0,top_left_y-self.thre):min(480,bottom_right_y+self.thre), 
                            #     max(0,top_left_x-self.thre):min(640,bottom_right_x+self.thre)
                            # ]
                            crop_img = image[
                                max(0,top_left_y-hand_thres_person):min(480-1,bottom_right_y+hand_thres_person), 
                                max(0,top_left_x-hand_thres_person):min(640-1,bottom_right_x+hand_thres_person)
                            ]
                        
                        human_coord = [(top_left_x + bottom_right_x) // 2,
                                    (top_left_y + bottom_right_y) // 2]
                        _pc = self.agent.pc.reshape(480, 640)
                        pc_np = np.array(_pc.tolist())[:, :, :3]
                        human_pc = pc_np[human_coord[1], human_coord[0]]
                        human_coord_in_map = self.axis_transform.transform_coordinate('head_rgbd_sensor_rgb_frame', 'map', human_pc)
                        coord_map_list[human_idx] = human_coord_in_map

                        cv2.imshow('crop_img', crop_img)
                        cv2.waitKey(1)
                        cv2.imwrite(f'/home/tidy/Robocup2024/module/CLIP/crop_img_{self.image_save_index}.jpg', crop_img)
                        self.image_save_index += 1

                        # Preprocess image
                        crop_img = Image.fromarray(cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB))
                        crop_img = self.clip_preprocess(crop_img)
                        crop_img = crop_img.unsqueeze(0).to(self.device)

                        # Encode image and text features
                        with torch.no_grad():
                            image_features = self.clip_model.encode_image(crop_img)
                            text_features = self.clip_model.encode_text(tokenized_prompt)
                            image_features /= image_features.norm(dim=-1, keepdim=True)
                            text_features /= text_features.norm(dim=-1, keepdim=True)

                            # Calculate text probabilities
                            text_probs = (100.0 * image_features @ text_features.T)
                            print('probs before softmax', text_probs)
                            text_probs = text_probs.softmax(dim=-1)
                            print('probs after softmax', text_probs)
                            # text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)

                        # Convert probabilities to percentages
                        text_probs_percent = text_probs * 100
                        text_probs_percent_np = text_probs_percent.cpu().numpy()
                        formatted_probs = ["{:.2f}%".format(value) for value in text_probs_percent_np[0]]

                        drink_prob[human_idx] += text_probs_percent_np[0][0]

                        print("Labels probabilities in percentage:", formatted_probs)
                        # if text_probs_percent_np[0][0] > 95:
                        #     drink_person[human_idx] = True
                        #     # return True
                        # else:
                        #     self.no_drink_human_coord = human_coord_in_map
                    count += 1

                drink_prob = np.array(drink_prob)
                drink_prob = drink_prob / 5
                
                for human_idx in range(len(drink_person)):
                    if drink_prob[human_idx] > 99:
                        drink_person[human_idx] = True
                    else:
                        self.no_drink_human_coord = coord_map_list[human_idx]
                
                if drink_person.count(False) > 0:
                    for person_idx in range(len(drink_person)):
                        print(f'person {person_idx}: {drink_person[person_idx]}')
                    return False
                else:
                    return True
        except:

            return True
    
    # def detect(self):
    #     self.agent.pose.head_tilt(0)
    #     rospy.sleep(0.5)

    #     if self.find_drink():
    #         return True
    #     rospy.sleep(1)

    #     return False

    
    # def detect_no_drink_hand(self):
    #     self.agent.pose.head_tilt(0)
    #     rospy.sleep(0.5)
    #     try:
    #         for human_hand in self.human_hand_poses:  # 사람별
    #             print("human found")
    #             l_hand_x, l_hand_y = human_hand[0]
    #             r_hand_x, r_hand_y = human_hand[1]

    #             human_coord = [(l_hand_x + r_hand_x) // 2,
    #                            (l_hand_y + r_hand_y) // 2]

    #             _pc = self.agent.pc.reshape(480, 640)
    #             pc_np = np.array(_pc.tolist())[:, :, :3]
    #             human_pc = pc_np[human_coord[1], human_coord[0]]
    #             human_coord_in_map = self.axis_transform.transform_coordinate('head_rgbd_sensor_rgb_frame', 'map',
    #                                                                           human_pc)
    #             print('test' + str(human_coord_in_map))
    #             if not self.agent.arena_check.is_in_arena([human_coord_in_map[0], human_coord_in_map[1]]):
    #                 print("[RULE 4] people out of arena")
    #                 continue

    #             drink_check = False

    #             for _ in range(0, 5):
    #                 if drink_check:
    #                     break
    #                 for bbox in self.agent.yolo_module.yolo_bbox:
    #                     # drink
    #                     if self.agent.yolo_module.find_type_by_id(bbox[4]) in self.drink_list:
    #                         cent_x, cent_y = bbox[0], bbox[1]
    #                         # print(l_hand_x, l_hand_y, r_hand_x, r_hand_y, cent_x, cent_y)
    #                         # left hand check
    #                         if (l_hand_x - self.thre <= cent_x <= l_hand_x + self.thre) and (
    #                                 l_hand_y - self.thre <= cent_y <= l_hand_y + self.thre):
    #                             drink_check = True
    #                             print('check')
    #                             break
    #                         # right hand check
    #                         if (r_hand_x - self.thre <= cent_x <= r_hand_x + self.thre) and (
    #                                 r_hand_y - self.thre <= cent_y <= r_hand_y + self.thre):
    #                             drink_check = True
    #                             print('check')
    #                             break
    #                 rospy.sleep(0.2)

    #             # detect no drink
    #             if not drink_check:
    #                 print('Found someone not holding a drink!')
    #                 # show image (if no drinks)
    #                 # self.show_image(l_hand_x, l_hand_y, r_hand_x, r_hand_y)

    #                 # self.no_drink_human_coord = [(l_hand_x + r_hand_x)//2, (l_hand_y + r_hand_y)//2]
    #                 # modified by lsh... 뎁스로 할때 가끔씩 이상한 위치로 가는 버그가 있어서 베이스링크 기준으로 고쳐봅니다.
    #                 self.no_drink_human_coord = human_coord_in_map

    #                 return True
    #     except:
    #         print('error in detect_no_drink_hand')
    #         return False

    #     return False

    def clarify_violated_rule(self):
        # go to the offender
        try:
            move_human_infront(self.agent, self.axis_transform,
                            self.no_drink_human_coord[1], self.no_drink_human_coord[0], coord=True)
        except:
            return False

        # clarify what rule is being broken
        self.agent.pose.head_tilt(20)
        self.agent.say('Hello!', show_display=True)
        rospy.sleep(1)

        # self.agent.say('If you are holding a drink,\n please show it to me.', show_display=True)
        # rospy.sleep(4)
        # if self.find_drink(double_check=True):
        #     self.agent.say('It seems you are \n holding a drink.', show_display=True)
        #     rospy.sleep(2)
        #     self.agent.say('I apologize for \n the inconvenience.', show_display=True)
        #     rospy.sleep(2)
        #     return False
        self.agent.say('Sorry but\n all guests should\n have a drink.', show_display=True)
        rospy.sleep(4)
        return True

    def ask_to_action(self, bar_location, current_location='kitchen_search'):
        # self.agent.say('We prepare some drinks.', show_display=True)
        # rospy.sleep(2)
        self.agent.say('Please follow me\n to the kitchen cabinet!\n If you are in my path, \n please move out!', show_display=True)
        rospy.sleep(1)

        if current_location == 'kitchen_search':
            pass
            # self.agent.move_abs_safe('office_leave2')
            # self.agent.say('Follow me! If you are in my path, \n please move out!', show_display=True)
        elif current_location == 'kitchen_search2':
            self.agent.move_abs_safe('kitchen_search2')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('office_leave2')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location == 'livingroom_search':
            self.agent.move_abs_safe('kitchen_living_middle')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('office_leave2')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
        elif current_location == 'hallway_search':
            self.agent.move_abs_safe('hallway_enter')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            # self.agent.move_abs_safe('office_search')
            # self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('office_leave1')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)
            self.agent.move_abs_safe('office_leave2')
            self.agent.say('Follow me! \n If you are in my path, \n please move out!', show_display=True)

        self.agent.move_abs_safe(bar_location)
        rospy.sleep(2)
        self.agent.pose.head_tilt(20)
        self.agent.say('Please hold drink\n on the cabinet.', show_display=True)
        rospy.sleep(4)

        self.agent.pose.head_tilt(5)
        self.agent.say("Hold the drink \nin front of me \nto double check", show_display=True)
        rospy.sleep(4)
        # if self.detect_no_drink_hand():
        if not self.find_drink(double_check=True):
            self.agent.pose.head_tilt(20)
            self.agent.say("You did not pick up a drink", show_display=True)
            rospy.sleep(2)
            self.agent.say("Please hold your drink at the bar", show_display=True)
            rospy.sleep(3)
        # self.agent.pose.head_tilt(5)
        # rospy.sleep(2)

        self.agent.pose.head_tilt(20)
        self.agent.say('Thank you!\nEnjoy your party', show_display=True)
        rospy.sleep(3)



def stickler_for_the_rules(agent):
    # agent.say('start stickler for the rules')
    # stickler_start_time = time.time()
    ### task params #################
    # start location: kitchen (search 1st)
    # when marked forbidden room, replace forbidden location to n-2 index search location

    # forbidden_search_location = 'bedroom_search'
    forbidden_search_location = 'office_search'

    break_rule_check_list = {'shoes': 0,
                             'room': 0,
                             'garbage': 0,
                             'drink': 0}

    ## params for rule 1. No shoes ##
    entrance = 'shoe_warning'
    ## params for rule 2. forbidden room ##
    # If needed, mark min & max points of all 4 rooms !
    # forbidden_room_min_points = {'bedroom_search': [4.2073, -7.9801, 0.03]} # PNU
    # forbidden_room_max_points = {'bedroom_search': [7.7033, -4.1148, 2.0]} # PNU
    forbidden_room_min_points = {'office_search': [1.0616, 1.2905, 0.03]} #
    forbidden_room_max_points = {'office_search': [4.0949, 5.6822, 2.0]} #

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
    
    clip = CLIP()

    shoe_detection = ShoeDetection(agent, clip=clip)
    # no_littering = NoLittering(agent, axis_transform)
    drink_detection = DrinkDetection(agent, axis_transform, hand_drink_pixel_dist_threshold, clip=clip)
    #################################
    search_location_list = [
        # 'bedroom_search',
        # 'kitchen_search', 
        # 'living_room_search', 'study_search',
        # # 'bedroom_search', 
        # 'kitchen_search', 'living_room_search', 'study_search',
        # 'kitchen_search', 'living_room_search', 'study_search',
        # 'kitchen_search', 'living_room_search', 'study_search'
        
        'office_search2',
        'office_search2',
        'kitchen_search', 
        'kitchen_search2',
        'livingroom_search',
        'hallway_search',
        'livingroom_search',
        'kitchen_search2', 
        'kitchen_search',
        'livingroom_search',
        'hallway_search',
    ]

    pan_degree_dict = {
        'office_search': [30, 0, -30], # corner, 45
        'office_search2': [0, -30, -60, -90, -120, -150, -180], # wall, 0
        'kitchen_search': [45, 15, -15, -45], # wall, 90
        'kitchen_search2': [0, -30, -60], # wall, 90
        'livingroom_search': [30, 0, -30, -60, -90, -120], # wall, 45
        'hallway_search': [30, 0, -30], # corner, 45
    }

    current_violation_flag = 'forbidden'

    agent.say('start stickler for the rules')

    agent.pose.head_pan_tilt(0, 0)

    agent.pose.move_pose()

    # while True:
    for search_idx, search_location in enumerate(search_location_list):

        # # if next room is office_search2, but already found broken rule, skip
        # if search_location == 'office_search2':
        #     if break_rule_check_list['room'] > 0:
        #         continue

        if search_location=='office_search2':
            agent.say("If you are in my path,\nplease move out.", show_display=True)
            agent.move_abs_safe('hallway_enter')
            # agent.move_abs_safe('office_search') # 없어도 상관없음
        
        
        # move to the search location
        agent.say(f"I'm moving to\n{search_location.split('_')[0]}.", show_display=True)
        rospy.sleep(2)
        agent.say("If you are in my path,\nplease move out.", show_display=True)
        
        # agent.pose.head_tilt(0)
        agent.pose.head_pan_tilt(0, 0)

        if current_violation_flag == 'forbidden':
            pass
        elif current_violation_flag == 'shoes':
            if search_location=='kitchen_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('hallway_enter')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('office_leave1')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('office_leave2')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
            elif search_location=='kitchen_search2':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('hallway_enter')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('livingroom_leave')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('kitchen_living_middle')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
            elif search_location=='livingroom_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('hallway_enter')
        elif current_violation_flag == 'drink':
            if search_location=='kitchen_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                pass
            elif search_location=='kitchen_search2':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('office_leave2')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
            elif search_location=='livingroom_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('office_leave2')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('kitchen_living_middle')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
            elif search_location=='hallway_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('office_leave2')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('office_leave1')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
        elif current_violation_flag == 'garbage':
            if search_location=='kitchen_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                pass
            elif search_location=='kitchen_search2':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                pass
            elif search_location=='livingroom_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('kitchen_living_middle')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
            elif search_location=='hallway_search':
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('kitchen_living_middle')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('livingroom_leave')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                agent.move_abs_safe('hallway_enter')
                agent.say("If you are in my path,\nplease move out.", show_display=True)
                
        
        current_violation_flag = 'forbidden'

        if search_location=='kitchen_search' and search_location_list[search_idx-1]=='office_search2':
            agent.move_abs_safe('office_leave1')
            agent.move_abs_safe('office_leave2')

        agent.move_abs_safe(search_location)

        if search_location == 'kitchen_search2':
            agent.move_rel(0.7, 0,wait=True)
        agent.say('Checking the rules.\n Please stand still \n and show your hands.', show_display=True)
        rospy.sleep(2)


        # [RULE 2] Forbidden room
        # if search_location == 'bedroom_search':
        if search_location == 'office_search2':
            agent.pose.head_tilt(0)
            # for pan_degree in pan_degree_list:
            # for pan_degree in pan_degree_list_forbidden_room:
            for pan_degree in pan_degree_dict[search_location]:
                agent.pose.head_pan(pan_degree)
                rospy.sleep(3)

                if forbidden_room.detect_forbidden_room():
                    break_rule_check_list['room'] += 1
                    # go to the offender and clarify what rule is being broken
                    forbidden_room.clarify_violated_rule()

                    agent.say('Follow me!', show_display=True)
                    rospy.sleep(1)

                    agent.move_abs_safe('office_search2')
                    agent.pose.head_tilt(20)
                    agent.say('You can leave\n through this door', show_display=True)
                    rospy.sleep(4)

                    # agent.move_abs_safe('bedroom_doublecheck')
                    # agent.say('Checking the room if empty', show_display=True)
                    agent.pose.head_tilt(0)
                    for pan_degree in [0, -45, -90, -135, -180]: # DOUBLE CHECKING

                        agent.say('Checking again', show_display=True)

                        agent.pose.head_pan(pan_degree)
                        rospy.sleep(3)

                        if forbidden_room.detect_forbidden_room(): 
                            agent.say('Oh my god. \nThere is someone still here', show_display=True)
                            rospy.sleep(2)
                            forbidden_room.clarify_violated_rule(double=True)
                            agent.say('Follow me!', show_display=True)

                            agent.move_abs_safe('office_search2')
                            agent.pose.head_tilt(20)
                            agent.say('You can leave\n through this door', show_display=True)
                            rospy.sleep(4)
                            agent.say('Thank you!', show_display=True)
                            rospy.sleep(2)
                            break
                    del forbidden_room
                    break

        # If not forbidden scan location >> check RULE 1, 3, 4
        # TODO: adjust living room pan degree & rotation position
        else:
            for pan_degree in pan_degree_dict[search_location]:
                agent.pose.head_pan(pan_degree)
                rospy.sleep(1)

                ##[RULE 4] Compulsory hydration : tilt 0
                # if break_rule_check_list['drink'] is False and drink_detection.detect_no_drink_hand():
                if break_rule_check_list['drink'] < 2 and not drink_detection.find_drink() and (search_location=='livingroom_search' or search_location=='hallway_search'):
                    # marking no drink violation detection
                    # break_rule_check_list['drink'] = True
                    break_rule_check_list['drink'] += 1
                    print('drink detection')

                    # go to the offender and clarify what rule is being broken
                    double_check_before = drink_detection.clarify_violated_rule()
                    if not double_check_before:
                        break_rule_check_list['drink'] -= 1
                        break
                    
                    # ask offender to grab a drink
                    drink_detection.ask_to_action(compulsory_hydration_bar_location, current_location=search_location)
                    current_violation_flag = 'drink'
                    break

                # [RULE 1] No shoes : tilt -20, -40
                # if break_rule_check_list['shoes'] < 2 and shoe_detection.run():
                if break_rule_check_list['shoes'] < 2 and not shoe_detection.find_shoes_clip() and search_location=='kitchen_search2':
                    # marking whether wearing shoes violation is detected
                    # break_rule_check_list['shoes'] = True
                    break_rule_check_list['shoes'] += 1
                    print('shoe detection')

                    # go to the offender and clarify what rule is being broken
                    shoe_detection.clarify_violated_rule()
                    # take the offender to the entrance & ask to take off their shoes
                    shoe_detection.ask_to_action(entrance, current_location=search_location)
                    current_violation_flag = 'shoes'
                    break

            # if search_idx > 3:
            #     for pan_degree in pan_degree_list[::-1]:
            #         # [RULE 3] No littering : tilt -40
            #         agent.pose.head_pan(pan_degree)
            #         rospy.sleep(2)
                # if break_rule_check_list['garbage'] < 2 and no_littering.detect_garbage():
                #     # marking no littering violation detection
                #     # break_rule_check_list['garbage'] = True
                #     break_rule_check_list['garbage'] += 1
                #     print('garbage detection')

                #     # go to the offender and clarify what rule is being broken
                #     no_littering.clarify_violated_rule()
                #     # ask the offender to pick up and trash the garbage
                #     no_littering.ask_to_action(bin_location, current_location=search_location)
                #     current_violation_flag = 'garbage'
                #     break

        if sum(break_rule_check_list.values())==4:
            break

        # Move to another room
        agent.pose.head_pan_tilt(0, 0)
        # agent.say("Now I'm going to\nmove to another room.", show_display=True)
        # rospy.sleep(2)
        # agent.say("If you are in my path,\nplease move to the side.", show_display=True)
        # rospy.sleep(5)



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
    agent.pose.head_pan(0)
    agent.move_abs_coordinate(human_infront_coord_in_map)


if __name__ == '__main__':
    from hsr_agent.agent import Agent

    rospy.init_node('stickler_rule_test')
    agent = Agent()
    forbidden_room_min_points = [-2.01, 3.74, 0]
    forbidden_room_max_points = [-0.66, 4.97, 2.0]
    axis_transform = Axis_transform()
    # forbidden_room = ForbiddenRoom(agent, axis_transform,
    #                                forbidden_room_min_points,
    #                                forbidden_room_max_points)

    hand_drink_pixel_dist_threshold = 50
    shoe_detection = ShoeDetection(agent)
    drink_detection = DrinkDetection(agent, axis_transform, hand_drink_pixel_dist_threshold)

    agent.pose.head_tilt(-30)
    agent.pose.head_pan(0)
    while True:
        agent.say('Looking for drink')
        rospy.sleep(1)
        is_drink_detected = drink_detection.find_drink()
        if is_drink_detected is None:
            agent.say('No one in sight')
            rospy.sleep(2)
        else:
            if not is_drink_detected:
                # agent.say('Drink not detected')
                agent.say('Found someone not holding a drink')
                rospy.sleep(2)
            else:
                # agent.say('Drink detected')
                agent.say('Everyone is holding a drink')
                rospy.sleep(2)
            
