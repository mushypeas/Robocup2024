import rospy
from std_msgs.msg import Int16MultiArray, String
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer  
from message_filters import Subscriber as sub

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
import subprocess

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
        self.byte_img = None
        self.track_queue = deque()
        # self.angle_queue = deque(maxlen=20)
        self.calcz_queue = deque(maxlen=10)
        self.obstacle_offset = 0.0
        self.last_say = time.time()
        self.tilt_angle = tilt_angle
        self.stt_option = stt_option
        self.save_one_time = False
        self.last_human_pos = None
        self.image_size = None
        self.image_shape = None
        self.seg_img = None
        self.contours = None
        self.calc_z = None
        self._depth = None
        self.depth = None
        self.twist = None
        self.data_header = None
        self.last_chance = 1
        bytetrack_topic = '/snu/bytetrack_img'
        segment_topic = '/deeplab_ros_node/segmentation'
        # self.tiny_object_list = ['pink_milk', 'blue_milk', 'coke', 'banana', 'apple', 'carrot', 'strawberry', 'sweet_potato', 'lemon', \
        #                           'cheezit', 'strawberry_jello', 'chocolate_jello', 'sugar', 'mustard', 'spam', 'tomato_soup', 'fork', 'plate',\
        #                               'knife', 'bowl', 'spoon', 'blue_mug', 'tennis_ball', 'soft_scrub', 'yellow_bag', 'blue_bag', 'white_bag',\
        #                                   'plum', 'peach', 'orange']
        self.tiny_object_list = []


    # ####sync_callback_240514
    #     self.byte_sub = sub(bytetrack_topic, Image)
    #     self.segment_sub = sub(segment_topic, Image)
        
    #     self.sync = ApproximateTimeSynchronizer([self.byte_sub, self.segment_sub], queue_size=10, slop=0.1)
    #     self.sync.registerCallback(self.sync_callback)
    # ####
        rospy.Subscriber(bytetrack_topic, Image, self._byte_cb)
        rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_cb)
        self.rgb_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, self._rgb_callback)
        rospy.Subscriber('/deeplab_ros_node/segmentation', Image, self._segment_cb)
        rospy.Subscriber('/snu/yolo_conf', Int16MultiArray, self._tiny_cb)
        rospy.loginfo("LOAD HUMAN FOLLOWING")
        self.image_pub = rospy.Publisher('/human_segmentation_with_point', Image, queue_size=10)
        self.canny_pub = rospy.Publisher('/canny_result', Image, queue_size=10)
    
    ####sync_callback_240514
    # def sync_callback(self, byte_data, segment_data):
    #     self._byte_cb(byte_data)
    #     self._segment_cb(segment_data)
    #     self.human_info_ary = copy.deepcopy(self.human_box_list)
    #     self.depth = np.asarray(self.d2pc.depth)
    #     self.twist, self.calc_z = self.human_reid_and_follower.follow(self.human_info_ary, self.depth, self.human_seg_pos)
    #     self._depth = self.barrier_check()
    ####   
    def freeze_for_humanfollowing(self):
        self.show_byte_track_image = True
        # self.agent.pose.head_pan_tilt(0, -self.tilt_angle*0.5)
        self.agent.pose.head_pan_tilt(0, -self.tilt_angle) #TODO : check tilt angle
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
        self.byte_img = img
        h, w, c = img.shape
        self.image_size = h * w
        self.image_shape = (h, w)
        # print(f"byte image ")
        if self.show_byte_track_image:
            self.agent.head_display_image_pubish(img)

    def head_circle_cb(self, config):
        rospy.loginfo(config)

    def head_map_cb(self, config):
        rospy.loginfo(config)

    def base_circle_cb(self, config):
        rospy.loginfo(config)

    def _tiny_cb(self, data):
        arr = data.data
        if arr is None:
            return None
        
        grouped_data = [arr[i:i+6] for i in range(0, len(arr), 6)]
        # print("grouped data")
        # print(grouped_data)

        # grouped_data = grouped_data[:-3]
        # print("grouped data w/o last 3")

        # print(grouped_data)

        filtered_data = [group for group in grouped_data if (group[5] >= .90 )]
        # print("filtered data")
        # print(filtered_data)

        sorted_data = sorted(filtered_data, key = lambda x: x[0])



        sorted_arr = [item for sublist in sorted_data for item in sublist]

        # print("sorted_arr")
        # print(sorted_arr)
        self.tiny_object_list = sorted_arr

    def _human_yolo_cb(self, data):
        '''
        snu/yolo will be depreciated
        '''
        # data_img = self.bridge.imgmsg_to_cv2(data)

        # print(f"human_yolo_size : {(data_img.shape)}")
        data_list = data.data
        if data_list[0] == -1:
            self.human_box_list = [None]
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
        else:
            self.human_box_list = [data_list[0], #[human_id, target_tlwh, target_score]
                                  np.asarray([data_list[1], data_list[2], data_list[3], data_list[4]], dtype=np.int64),
                                  data_list[5]]
        # if not self.seg_on:    
        #     self.human_info_ary = self.human_box_list
        #     depth = np.asarray(self.agent.depth_image)
        #     self.twist, self.calc_z = self.human_reid_and_follower.follow(self.human_info_ary, depth, self.human_seg_pos)
        #     self._depth = self.barrier_check()


    def _rgb_callback(self, data): ## tiny_object_edge 검출 용
        frame = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        # cv2.imshow('original frame', frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        # cv2.imshow('gray', gray)

        edges = cv2.Canny(gray, 100, 150)

        # 푸리에 변환 적용
        rows, cols = edges.shape
        f = np.fft.fft2(edges)
        fshift = np.fft.fftshift(f)

        # Gaussian Low-Pass Filter
        crow, ccol = rows // 2, cols // 2
        mask = np.zeros((rows, cols), np.float32)
        sigma = 50  # Standard deviation for Gaussian filter
        x, y = np.ogrid[:rows, :cols]
        mask = np.exp(-((x - crow) ** 2 + (y - ccol) ** 2) / (2 * sigma ** 2))

        # Apply mask and inverse DFT
        fshift = fshift * mask
        f_ishift = np.fft.ifftshift(fshift)
        img_back = np.fft.ifft2(f_ishift)
        img_back = np.abs(img_back)


        
        img_back = (img_back - np.min(img_back)) / (np.max(img_back) - np.min(img_back)) * 255
        img_back = np.uint8(img_back)

        kernel = np.ones((1, 1), np.uint8)
        img_back = cv2.dilate(img_back, kernel, iterations=1)
        img_back = cv2.erode(img_back, kernel, iterations=1)

        morph = img_back

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tiny_exist = False
        tiny_loc = None

        self.contours = contours

        morph = cv2.bitwise_not(morph)


        morph = np.uint8(morph)
        canny_img_msg = self.bridge.cv2_to_imgmsg(morph, 'mono8')
        # canny_img_msg.header = self.data_header
        if canny_img_msg is not None:
            self.canny_pub.publish(canny_img_msg)

    def _segment_cb(self, data):

        #######################seg는 back때만 사용
        depth = np.asarray(self.agent.depth_image)

        data_img = self.bridge.imgmsg_to_cv2(data, 'mono16')
        self.seg_img = data_img



        ######################seg&byte 동시 사용
        # if self.human_box_list[0] == None:
        #     print("There is no human box")
            # self.human_seg_pos = None
            # print(data)
        # else:
        #     depth = np.asarray(self.agent.depth_image)

        #     data_img = self.bridge.imgmsg_to_cv2(data, 'mono16')
        #     self.seg_img = data_img
#             x = self.human_box_list[1][0]
#             y = self.human_box_list[1][1]
#             w = self.human_box_list[1][2]
#             h = self.human_box_list[1][3]

#             H, W = data_img.shape
#             cropped_y_max = max(min(y + h, H-1), 0)
#             cropped_x_max = max(min(x + w, W-1), 0)
#             cropped_y = max(min(y, H-1), 0)
#             cropped_x = max(min(x, W-1), 0)

#             data.encoding='mono16'
#             crop_img = data_img[cropped_y:cropped_y_max,cropped_x:cropped_x_max]



#             #####################BRANCH 2. LABEL=15면 사람임###################

#             byte_y = max(min(y + h//4, H-1), 0)
#             byte_x = max(min(x + w//2, W-1), 0)

#             human_mask = crop_img == 15
#             human_y, human_x = np.where(human_mask)

#             if human_y.size>0 and human_x.size>0:
#                 # center_x, center_y = int(np.mean(human_x)), int(np.mean(human_y))
#                 # self.human_seg_pos = (center_x + x, center_y + y)
#                 # topmost_y = np.max(human_y)
#                 # mean_y = int(np.mean(human_y))
#                 min_y = int(np.min(human_y))
#                 if min_y < 0:
#                     min_y = 0
#                 mean_x = int(np.mean(human_x))
#                 print("y", y)
#                 # x_at_topmost_y = human_x[np.argmax(human_y)]


#                 #x,y
#                 # self.human_seg_pos = (x_at_topmost_y + x, topmost_y + y - 10)
#                 if y < 0:
#                     y = 0
#                 self.human_seg_pos = (mean_x + x, min_y + y)
#                 if min_y + y > 480:
#                     self.human_seg_pos = (mean_x + x, 480-10)
#                 # print(f"human top seg_pos x,y: {self.human_seg_pos}")



#                 ###point pubish
#                 # depth_img_8bit = cv2.convertScaleAbs(data_img, alpha=(255.0/65535.0))
#                 # depth_img_3channel = cv2.cvtColor(depth_img_8bit, cv2.COLOR_GRAY2BGR)
#                 # cv2.circle(depth_img_3channel, (self.human_seg_pos[0], self.human_seg_pos[1]), 5, (0, 0, 255), -1)
#                 # cv2.circle(depth_img_3channel, (byte_x, byte_y), 5, (255, 0, 0), -1)

# # Convert to 3-channel
#                 # seg_img_msg = self.bridge.cv2_to_imgmsg(depth_img_3channel, 'bgr8')
#                 # seg_img_msg.header = data.header
#                 # self.data_header = data.header
#                 # self.image_pub.publish(seg_img_msg)
            
        
            
#             else:
#                 print("There is a human box but no segmentation anywhere")
#                 self.human_seg_pos = None

#             # # self.human_info_ary = copy.deepcopy(self.human_box_list)
#             # self.human_info_ary = self.human_box_list
#             # # print(f"self human info ary : {self.human_info_ary}")
#             # # depth = np.asarray(self.d2pc.depth)
#             # # print(f"depth : {depth}")
#             # self.twist, self.calc_z = self.human_reid_and_follower.follow(self.human_info_ary, depth, self.human_seg_pos)
#             # self._depth = self.barrier_check()
#             # #####################BRANCH 2. LABEL=15면 사람임###################


            


    def check_human_pos(self, human_box_list, location=False):
        x = human_box_list[1][0]
        y = human_box_list[1][1]
        w = human_box_list[1][2]
        h = human_box_list[1][3]
        center = [y + int(h/2), x + int(w/2)] # (y,x)
        # if self.seg_on:
        #     if self.human_seg_pos is not None:
        #         (x,y) = self.human_seg_pos
        #         center = y,x

        print("center", center)
        # human_location = 'c'
        if location:
            # if center[1] < 80:
            #     self.angle_queue.append(-1)
            #     return 'lll'
            # elif center[1] < 110:
            #     self.angle_queue.append(-1)
            #     return 'll'
            # elif center[1] < 140:
            #     self.angle_queue.append(-1)
            #     return 'l'
            # elif center[1] > 500:
            #     self.angle_queue.append(1)
            #     return 'r'
            # elif center[1] > 530:
            #     self.angle_queue.append(1)
            #     return 'rr'
            # elif center[1] > 560:
            #     self.angle_queue.append(1)
            #     return 'rrr'
            if center[1] < 70:
                # self.angle_queue.append(-1)
                return 'lll'
            elif center[1] < 130:
                # self.angle_queue.append(-1)
                return 'll'
            elif center[1] < 180:
                # self.angle_queue.append(-1)
                return 'l'
            elif center[1] > 450:
                # self.angle_queue.append(1)
                return 'r'
            elif center[1] > 500:
                # self.angle_queue.append(1)
                return 'rr'
            elif center[1] > 570:
                # self.angle_queue.append(1)
                return 'rrr'   

        if center[1] < -40 or center[1] > 680:
            print("center[1] : ", center[1])
            return True # stop

        return False # pass
    
    def barrier_check(self, looking_downside=True):
        # _depth = self.agent.depth_image[:150, 10:630]

        if (looking_downside):
            _depth = self.agent.depth_image[100:340, 200:440] / 1000 # 480, 640
        else: # no tilt
            _depth = self.agent.depth_image[200:0, 280:640] / 1000

            
        return _depth
    

    def escape_barrier(self, calc_z):
        # if self.calc_z is not None:
        #     calc_z = self.calc_z
        cur_pose = self.agent.get_pose(print_option=False)
        # thres = 1.0
        # human_box_thres = 0.5
        # if self.human_box_list[0] is not None:
        #     # print(f"human_box_list[1] : {self.human_box_list[1]}")
        #     human_box_size = self.human_box_list[1][2] * self.human_box_list[1][3] # TODO: human box size 맞는지 체크
        # else:
        #     human_box_size = 0
        # print(f"human box size thres : {self.image_size * human_box_thres}")
        # print(f"real human box size : {human_box_size}")
        _num_rotate=0
        _depth = self.barrier_check()
        print("_depth shape: ", _depth.shape)
        origin_depth = _depth
        # _depth = np.mean(_depth)
        _depth = _depth[_depth != 0]
        print("_depth shape: ", _depth.shape)
        if len(_depth) != 0:
            _depth = np.partition(_depth, 40)
            _depth = np.mean(_depth)

            # _depth = np.min(_depth)
            masked_depth = np.ma.masked_equal(origin_depth, 0)
            min_index = np.argmin(masked_depth)
            print("min_index : ", min_index)
            # 1차원 인덱스를 2차원 인덱스로 변환
            t = np.unravel_index(min_index, origin_depth.shape)
            print("unravel_index(min_index, origin_depth.shape) :", t)
            min_y, min_x = t
            # _depth = np.mean(self._depth)
            escape_radius = 0.2
            # if self.human_box_list[0] is not None:
            #     human_info_ary = copy.deepcopy(self.human_box_list)
            #     depth = np.asarray(self.d2pc.depth)
            #     twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)


            # if calc_z > _depth * 2000:
            #     calc_z = _depth * 2000
            # if calc_z > np.mean(self.calcz_queue) * 1000 + 1000:
            #     calc_z = _depth * 1000
            # rot_dir_left = 1
            # _depth_value_except_0 = _depth[_depth != 0]

            # if self.human_box_list[0] is not None:
            #     human_box_list = self.human_box_list
            #     x = human_box_list[1][0]
            # else:
            #     x = 640


            rospy.loginfo(f"rect depth min : {_depth}")
            # rospy.loginfo(f"rect depth excetp 0 min : {_depth_value_except_0.min}")
            rospy.loginfo(f"calc_z  : {calc_z / 1000.0}")
            #and np.mean(_depth)< (calc_z-100)
            #and self.image_size * human_box_thres > human_box_size
            if (calc_z!=0 and _depth < 1.0 and (_depth< ((calc_z/ 1000.0)-0.6) or self.agent.dist <((calc_z/1000.0)-0.6) )and not (self.start_location[0] - escape_radius < cur_pose[0] < self.start_location[0] + escape_radius and \
            self.start_location[1] - escape_radius < cur_pose[1] < self.start_location[1] + escape_radius)):
                _num_rotate = _num_rotate + 1
                # rospy.sleep(1)
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                # self.agent.say('Barrier checking....', show_display=True)
                print("Barrier checking....")
                # self.agent.pose.head_pan_tilt(0, -self.tilt_angle)

                # rospy.sleep(1)

                # _depth = self.barrier_check()
                # rospy.loginfo(f"rect depth : {np.mean(_depth)}")

                # while (np.mean(_depth)< ((calc_z/ 1000.0)-0.3) or self.agent.dist <((calc_z/1000.0)-0.3)): # TODO : while 하는게 좋은지? 아니면 그때그때 한번만 이동하는게 좋은지?

                ##########################METHOD1. ROTATING########################    
                #     _num_rotate = _num_rotate + 1
                #     rospy.loginfo(f"mean depth: {np.mean(self.agent.depth_image)}")

                #     # self.agent.say('Barrier verified.', show_display=True)
                #     print("Barreir verified")
                #     self.agent.move_rel(0,0,self.stop_rotate_velocity, wait=True)
                #     rospy.sleep(1)
                #     _depth = self.barrier_check()
                #     if (np.mean(_depth) > (thres+0.4)):
                #         self.agent.move_rel(0,0,self.stop_rotate_velocity*0.6, wait=True)
                #         print("it's safe now!")
                #         # self.agent.say("It's safe now!")
                #         break
                #     elif (_num_rotate > 5):
                #         _num_rotate = 0
                #         self.agent.say("spin opposite direction.")
                #         rot_dir_left = 1
                #         rospy.sleep(4)
                #         _depth = self.barrier_check()
                #         while (np.mean(_depth) < thres):
                #             _num_rotate = _num_rotate + 1
                #             self.agent.say('Barrier verified.', show_display=True)
                #             print("Barreir verified")
                #             rospy.loginfo(f"mean depth: {np.mean(self.agent.depth_image)}")
                #             self.agent.move_rel(0,0,-self.stop_rotate_velocity, wait=True)
                #             rospy.sleep(1)
                #             _depth = self.barrier_check()
                #             if (np.mean(_depth) > (thres+0.4)):
                #                 self.agent.move_rel(0,0,-self.stop_rotate_velocity*0.6, wait=True)
                #                 print("it's safe now!")
                #                 self.agent.say("It's safe now!")
                #                 break
                ################################################################################################

                # rospy.sleep(1)
                # # self.agent.say("stop rotating.")
                # # self.agent.pose.head_pan_tilt(0, -self.tilt_angle)
                # rospy.sleep(3)
                # self.agent.move_rel(0.3,0,0, wait=True)
                # rospy.sleep(.5)
                # self.agent.move_rel(0.3,0,0, wait=True)
                # rospy.sleep(.5)
                # self.agent.move_rel(0.3,0,0, wait=True)
                # rospy.sleep(.5)
                # self.agent.move_rel(0.3,0,0, wait=True)

                # if (rot_dir_left==1):
                #     self.agent.move_rel(0,0,-(_num_rotate-1) * self.stop_rotate_velocity)
                # else :
                #     self.agent.move_rel(0,0,(_num_rotate-1) * self.stop_rotate_velocity)
                # rospy.sleep(2)
                    

                ##########################METHOD2. MOVING########################    
                # # _depth = self.barrier_check()
                # # print(f"mean depth: {np.mean(_depth)}")
                # last_5_angle_average = np.mean(self.angle_queue)
                # print("self.angle_queue", self.angle_queue)
                # print("last_5_angle_average", last_5_angle_average)
                # if last_5_angle_average < 0: # HSR has rotated left
                #     self.agent.move_rel(0,0.7,0, wait=False) #then, HSR is intended to move right
                #     rospy.sleep(1)
                # else: #  HSR has rotated right
                #     self.agent.move_rel(0,o-0.7,0, wait=False)
                # # self.agent.move_rel(0,0.3,0, wait=False) ## TODO : go left
                # rospy.sleep(1)
                ########################################################################


                ########################METOD3. SEGMENT#############################

                # _depth = self.barrier_check()
                # _depth = _depth[_depth != 0]
                # print(f"depth min depth: {np.mean(_depth)}")
                # seg_img = self.seg_img
                # height, width = seg_img.shape
                # mid_x = width // 2
                # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                # # baack_y, back_x = np.where(background_mask)

                # left_background_count = np.sum(background_mask[:, :mid_x])
                # right_background_count = np.sum(background_mask[:, mid_x:])

                # _depth = self.barrier_check()
                # # _depth = np.mean(_depth)
                # _depth = _depth[_depth != 0]
                # _depth = np.mean(_depth)
                # if _depth < 1.5:
                #     if left_background_count > right_background_count:
                #         print("left side is empty")
                #         self.agent.move_rel(0,-1.2,0, wait=True) #then, HSR is intended to move left
                #     elif right_background_count >= left_background_count:
                #         print("right side is empty")
                #         self.agent.move_rel(0,1.2,0, wait=True) #then, HSR is intended to move right
                #     # rospy.sleep(1)

                ########################################################################

                #########################METHOD4. DEPTH##############################


                depth = self.agent.depth_image

                seg_img = self.seg_img
                height, width = seg_img.shape
                mid_x = width // 2
                # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                # baack_y, back_x = np.where(background_mask)
                print("min_y+100 : ", min_y+100)
                left_background_count = np.mean(depth[min_y+100-20:min_y+100+20, :mid_x])
                print("left_background_count", left_background_count)
                right_background_count = np.mean(depth[min_y+100-20:min_y+100+20, mid_x:])
                print("right_background_count", right_background_count)
                # _depth = self.barrier_check()
                # # _depth = np.mean(_depth)
                # _depth = _depth[_depth != 0]
                # _depth = np.mean(_depth)

                if left_background_count > right_background_count:
                    print("left side is empty")
                    self.agent.move_rel(0,0.8,0, wait=False) #then, HSR is intended to move left (pos)
                    rospy.sleep(2)
                    self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//8, wait=False)
                    # self.agent.move_rel(0,0,-self.stop_rotate_velocity//4, wait=False)
                    rospy.sleep(1)
                elif right_background_count >= left_background_count:
                    print("right side is empty")
                    self.agent.move_rel(0,-0.8,0, wait=False) #then, HSR is intended to move right (neg)
                    rospy.sleep(2)
                    self.agent.move_rel(0.3,0,self.stop_rotate_velocity//8, wait=False)
                    # self.agent.move_rel(0,0,self.stop_rotate_velocity//4, wait=False)

                    rospy.sleep(1)






                ########################################################################



    def escape_tiny(self):
        tiny_object_list = self.tiny_object_list
        tiny_object_exist = False
        if len(tiny_object_list) > 0:
             for idx in range(len(tiny_object_list) // 6):
                
                item = tiny_object_list[6 * idx: 6 * (idx + 1)]
                # print("item", item)
                cent_x, cent_y, width, height, class_id, conf_percent = item
                # 480 640
                if cent_y > 330 and cent_x > 200 and cent_x < 440 and conf_percent > 90:
                    tiny_object_exist = True
                    print('Tiny object.')
                    break

        if tiny_object_exist:
            print('I\'ll avoid it.')
            self.agent.move_rel(0,-0.3,0, wait=False) ## TODO : go right
            rospy.sleep(1)

    def escape_tiny_canny(self):
        center = None
        if self.human_box_list[1] is not None:
            x = self.human_box_list[1][0]
            y = self.human_box_list[1][1]
            w = self.human_box_list[1][2]
            h = self.human_box_list[1][3]
            center = [y + int(h/2), x + int(w/2)] # (y,x)
        contours = self.contours

        tiny_exist = False
        tiny_loc = None



        seg_img = self.seg_img[:, :]

        # Create a mask for depth values greater than 0 and seg_img equal to 15
        human_mask = (seg_img == 15)
        human_y, human_x = np.where(human_mask)
        if len(human_y) != 0:
            human_y_max = np.max(human_y)
        else:
            human_y_max = 440

        for contour in contours:
            area = cv2.contourArea(contour)
            if 800 < area < 3000:  # 면적 기준으로 작은 물체 필터링 (적절히 조절 가능)
                # print(area)
                x, y, w, h = cv2.boundingRect(contour)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # if y > human_y_max and y > 440 and x > 240 and x < 400:
                if y > 420 and x > 240 and x < 400:

                    tiny_exist = True
                    print('Tiny object.')
                    # cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    break


        if len(self.calcz_queue) > 1 and self.calcz_queue[-1] is not None:
            last_calc_z = self.calcz_queue[-1]
            print("canny last calc_z: ", last_calc_z)

            if tiny_exist and (center is not None and center[1] > 100 and center[1] < 540)  :




                depth = self.agent.depth_image

                seg_img = self.seg_img
                height, width = seg_img.shape
                mid_x = width // 2
                # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                # baack_y, back_x = np.where(background_mask)
                # print("min_y+100 : ", min_y+100)
                left_background_count = np.mean(depth[100-20:100+20, :mid_x])
                print("left_background_count", left_background_count)
                right_background_count = np.mean(depth[100-20:100+20, mid_x:])
                print("right_background_count", right_background_count)



                # if (self.human_box_list[0] is not None) and (center[1] < 180 or center[1] > 500):
                print('Tiny object. I\'ll avoid it.')
                self.agent.say('Tiny object. I\'ll avoid it.', show_display=False)
                if right_background_count > left_background_count:
                    # self.agent.move_rel(0.3,-0.5,0, wait=False) ## move right is neg
                    rospy.sleep(2)
                    self.agent.move_rel(0,-0.9,0, wait=False) ## move right is neg
                    rospy.sleep(2)

                    
                    # self.agent.move_rel(0.3,0,self.stop_rotate_velocity//6, wait=False)
                else:
                    # self.agent.move_rel(0.3,0.5,0, wait=False)
                    rospy.sleep(2)
                    # self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//6, wait=False)
                    self.agent.move_rel(0,0.9,0, wait=False) ## move right is neg
                    rospy.sleep(2)



    def escape_tiny_canny_back(self):
        # if self.human_box_list[1] is not None:
        #     x = self.human_box_list[1][0]
        #     y = self.human_box_list[1][1]
        #     w = self.human_box_list[1][2]
        #     h = self.human_box_list[1][3]
        #     center = [y + int(h/2), x + int(w/2)] # (y,x)
        contours = self.contours

        tiny_exist = False
        tiny_loc = None



        # seg_img = self.seg_img[:, :]

        # # Create a mask for depth values greater than 0 and seg_img equal to 15
        # human_mask = (seg_img == 15)
        # human_y, human_x = np.where(human_mask)
        # if len(human_y) != 0:
        #     human_y_max = np.max(human_y)
        # else:
        #     human_y_max = 440

        for contour in contours:
            area = cv2.contourArea(contour)
            if 800 < area < 3000:  # 면적 기준으로 작은 물체 필터링 (적절히 조절 가능)
                # print(area)
                x, y, w, h = cv2.boundingRect(contour)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if y > 420 and x > 240 and x < 400:
                    if x < 320:
                        tiny_loc == 'left'
                    else:
                        tiny_loc == 'right'
                    tiny_exist = True
                    print('Tiny object.')
                    # cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    break


        if len(self.calcz_queue) > 1 and self.calcz_queue[-1] is not None:
            last_calc_z = self.calcz_queue[-1]
            print("canny last calc_z: ", last_calc_z)

            if tiny_exist and self.agent.dist > 1.0 :
                

                depth = self.agent.depth_image

                seg_img = self.seg_img
                height, width = seg_img.shape
                mid_x = width // 2
                # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                # baack_y, back_x = np.where(background_mask)
                # print("min_y+100 : ", min_y+100)
                left_background_count = np.mean(depth[100-20:100+20, :mid_x])
                print("left_background_count", left_background_count)
                right_background_count = np.mean(depth[100-20:100+20, mid_x:])
                print("right_background_count", right_background_count)



                # if (self.human_box_list[0] is not None) and (center[1] < 180 or center[1] > 500):
                print('Tiny object. I\'ll avoid it.')
                self.agent.say('Tiny object. I\'ll avoid it.', show_display=False)
                if right_background_count > left_background_count:
                    # self.agent.move_rel(0.3,-0.5,0, wait=False) ## move right is neg
                    rospy.sleep(2)
                    self.agent.move_rel(0,-0.9,0, wait=False) ## move right is neg
                    rospy.sleep(2)

                    
                    # self.agent.move_rel(0.3,0,self.stop_rotate_velocity//6, wait=False)
                else:
                    # self.agent.move_rel(0.3,0.5,0, wait=False)
                    rospy.sleep(2)
                    # self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//6, wait=False)
                    self.agent.move_rel(0,0.9,0, wait=False) ## move right is neg
                    rospy.sleep(2)
        

    def stt_destination(self, stt_option, calc_z=0):
        cur_pose = self.agent.get_pose(print_option=False)
        # print("in area", [cur_pose[0], cur_pose[1]], "last moved time", time.time() - self.agent.last_moved_time)
        # if (time.time() - self.agent.last_moved_time > 3.0) and self.human_box_list[0] is None:


        if (time.time() - self.agent.last_moved_time > 10.0) and not (self.start_location[0] - self.goal_radius < cur_pose[0] < self.start_location[0] + self.goal_radius and \
            self.start_location[1] - self.goal_radius < cur_pose[1] < self.start_location[1] + self.goal_radius):

            print("lmt_if", self.agent.last_moved_time)

            self.show_byte_track_image = False
            self.agent.move_base.base_action_client.cancel_all_goals()
            if stt_option:
                self.agent.say('Is this your destination?\nsay yes or no after a ding sound', show_display=True)
                rospy.sleep(5)
                answer, _ = self.agent.stt(3, mode='yesno')
                question_num = 0
                while 'yes' not in answer and 'no' not in answer and question_num < 3:
                    self.agent.say('Answer only by \nyes or no', show_display=True)
                    print('Answer only by \nyes or no')
                    rospy.sleep(2.5)
                    answer, _ = self.agent.stt(3., mode='yesno')
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
            self.escape_barrier(calc_z)
            # self.escape_tiny()
            self.escape_tiny_canny()

            # if time.time() - self.agent.last_moved_time > 3.0 and time.time() - self.last_say > 4.0:
                # if (calc_z < 1.5)
                    # self.agent.say('You are so close. Please keep the two meter between us!', show_display=True)
                    # print("You are so close. Please keep the two meter between us!")
                    # self.last_say = time.time()
                    # rospy.sleep(1)
            return False
        

    def follow_human(self, start_time=time.time(), pose_save_time_period=10):

        ##########24.3.26
        _num_rotate = 0

        # self.escape_barrier(calc_z)
        # if self.human_box_list[0] is None: # no human detected

        # human_info_ary = copy.deepcopy(self.human_box_list)

        # depth = np.asarray(self.d2pc.depth)
        # twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)
        # self.escape_barrier(calc_z)
        if self.human_box_list[0] is None: # no human detected
            # rospy.loginfo("no human")
            if self.stt_destination(self.stt_option):
                return True
            print("it is not finished but i missed human...ㅜㅜ")
            if self.last_human_pos is not None and self.last_chance > 0:
                print("lets find human")

                target_xyyaw = self.last_human_pos
                if target_xyyaw[2] > 0:
                    turn = self.stop_rotate_velocity
                else:
                    turn = -self.stop_rotate_velocity
                self.agent.move_rel(0, 0, turn, wait=False)
                rospy.sleep(3)
                self.last_chance = 0
                if self.human_box_list[0] is None:
                    self.last_chance = 1
            return False
        else:
            self.last_chance = 1
        human_info_ary = copy.deepcopy(self.human_box_list)
        depth = np.asarray(self.d2pc.depth)
        twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)
        # twist, calc_z = self.twist, self.calc_z
        _depth = self.barrier_check()
        _depth = _depth[_depth != 0]
        _depth = np.mean(_depth)

        # if calc_z > _depth * 1000 + 100:
        #     calc_z = _depth * 1000 + 100
        # print("np.mean(self.calcz_queue)", np.mean(self.calcz_queue))
        print("calc_z", calc_z)
        print("calcz_queue : ", self.calcz_queue)
        if calc_z > np.mean(self.calcz_queue) + 500:
            calc_z = _depth * 1000
            self.calcz_queue.append(calc_z)

        else:
            self.calcz_queue.append(calc_z)
        # print(f"self.calc_z = {self.calc_z}")
        # print(f"calc_z = {calc_z}")
        # if calc_z > 1000:
        #     calc_z = calc_z + 1000 #TODO : calc_z 과장할 정도 결정

        if self.check_human_pos(human_info_ary):  # If human is on the edge of the screen
            print("2.1 go to center!")
            if time.time()-self.last_say > 3:
                self.agent.say('Please, go to center!')
                self.last_say = time.time()
            rospy.sleep(2)
            if self.stt_destination(self.stt_option, calc_z):
                return True
            return False
        else:
            # we move "depth" to the front

            # if calc_z > 2000.0 and time.time()-self.last_say > 5:
            #     self.agent.say('Your so far')
            #     rospy.sleep(0.5)
            #     self.agent.say('Please move slowly!')
            #     self.last_say = time.time()


        #########
        # print("2.2 linear x", twist.linear.x, "angular", twist.angular.z)
            # change angular.z
            loc = self.check_human_pos(human_info_ary, location=True)
            if loc == 'lll':
                print("left!!!!!!")
                print("left!!!!!!")
                print("left!!!!!!")
                print("left!!!!!!")
                print("left!!!!!!")
                print("left!!!!!!")
                # twist.angular.z = -self.stop_rotate_velocity
                # +가 왼쪽으로 돌림
                self.agent.move_rel(0, 0, self.stop_rotate_velocity, wait=False)
                rospy.sleep(.5)
            if loc == 'll':
                print("left")
                # twist.angular.z = -self.stop_rotate_velocity
                self.agent.move_rel(0, 0, self.stop_rotate_velocity/2, wait=False)
                rospy.sleep(.5)
            # if loc == 'l':
            #     print("left")
            #     # twist.angular.z = -self.stop_rotate_velocity
            #     self.agent.move_rel(0, 0, self.stop_rotate_velocity/4, wait=False)
            #     rospy.sleep(.5)
            # if loc == 'r':
            #     print("right")
            #     # twist.angular.z = +self.stop_rotate_velocity
            #     self.agent.move_rel(0, 0, -self.stop_rotate_velocity/4, wait=False)
            #     rospy.sleep(.5)
            if loc == 'rr':
                print("right")
                # twist.angular.z = +self.stop_rotate_velocity
                self.agent.move_rel(0, 0, -self.stop_rotate_velocity/2, wait=False)
                rospy.sleep(.5)
            if loc == 'rrr':
                print("right")
                print("right")
                print("right")
                print("right")
                print("right")
                print("right")
                print("right")
                print("right")
                # twist.angular.z = +self.stop_rotate_velocity
                self.agent.move_rel(0, 0, -self.stop_rotate_velocity, wait=False)
                rospy.sleep(.5)
            if twist.linear.x == 0 and twist.angular.z == 0:

                if self.stt_destination(self.stt_option, calc_z):
                    return True

                return False
    
            target_xyyaw = self.calculate_twist_to_human(twist, calc_z)
            self.marker_maker.pub_marker([target_xyyaw[0], target_xyyaw[1], 1], 'base_link')
                # 2.4 move to human
            self.last_human_pos = target_xyyaw
            # if calc_z > 1000:
            #     tmp_calc_z = calc_z + 1000 #TODO : calc_z 과장할 정도 결정
            #     target_xyyaw = self.calculate_twist_to_human(twist, tmp_calc_z)
            #     self.marker_maker.pub_marker([target_xyyaw[0], target_xyyaw[1], 1], 'base_link')
            #     # 2.4 move to human
            # target_yaw = target_xyyaw[2]


            ####TODO : twist 해결 됐는지 확인 
            # if target_xyyaw[2] > 0.05:
            #     target_yaw = 0.05
            # if target_xyyaw[2] < -0.05:
            #     target_yaw = -0.05
            self.agent.move_rel(target_xyyaw[0], target_xyyaw[1], target_xyyaw[2], wait=False)
            # if target_xyyaw[2] > 0.1:
            #     self.angle_queue.append(-1)
            # elif target_xyyaw[2] < -0.1:
            #     self.angle_queue.append(1)

            # rospy.sleep(.5)
            cur_pos = self.agent.get_pose(print_option=False)
            if round((time.time() - start_time) % pose_save_time_period) == 0:
                if self.save_one_time:
                    if len(self.track_queue) > 0:
                        last_loc = self.track_queue[-1]
                        if not( last_loc[0] - self.goal_radius < cur_pos[0] < last_loc[0] + self.goal_radius and \
                        last_loc[1] - self.goal_radius < cur_pos[1] < last_loc[1] + self.goal_radius):

                            self.track_queue.append((cur_pos[0], cur_pos[1], cur_pos[2]+np.pi))
                            self.save_one_time = False
                    else:
                        self.track_queue.append((cur_pos[0], cur_pos[1], cur_pos[2]+np.pi))
                        self.save_one_time = False
                    # print('queue_size', len(self.track_queue))
            else:
                self.save_one_time = True

            # check arrive at destination zone
            if self.stt_destination(self.stt_option, calc_z):
                return True

        return False

    # def follow_human_queue(self, try_bytetrack):
    #     if try_bytetrack:
    #         if self.human_box_list[0] is None: # no human detected
    #             self.agent.move_abs_coordinate(self.start_location, wait=False)
    #             print('No human detected, go start location')
    #             rospy.sleep(0.5)
    #             return False
    #         # human_info_ary = copy.deepcopy(self.human_box_list)
    #         # depth = np.asarray(self.d2pc.depth)
    #         twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)
    #         # twist, calc_z = self.twist, self.calc_z

    #         if calc_z < 200.0:
    #             if twist.linear.x == 0 and twist.angular.z == 0:
    #                 self.agent.move_base.base_action_client.cancel_all_goals()
    #                 self.agent.move_rel(0, 0, 0, wait=False)
    #                 rospy.sleep(0.5)
    #                 return False

    #             target_xyyaw = self.calculate_twist_to_human(twist, calc_z)
    #             self.marker_maker.pub_marker([target_xyyaw[0], target_xyyaw[1], 1], 'base_link')
    #             # 2.4 move to human
    #             self.agent.move_base.base_action_client.cancel_all_goals()
    #             self.agent.move_rel(target_xyyaw[0], target_xyyaw[1], target_xyyaw[2], wait=False)
    #             rospy.sleep(1)
    #             print("Human is close to me... Move towards human")
    #         else:
    #             self.agent.move_abs_coordinate_safe(self.start_location, angle=90)
    #             print("Go back to start location since human is too far")
    #             rospy.sleep(1)
            
    #     else:
    #         self.agent.move_abs_coordinate_safe(self.start_location, thresh=0.5, timeout=8, giveup_timeout=10)
            
    #     cur_pos = self.agent.get_pose(print_option=False)

    #     # check arrive at destination zone
    #     if self.start_location[0] - self.goal_radius < cur_pos[0] < self.start_location[0] + self.goal_radius and \
    #         self.start_location[1] - self.goal_radius < cur_pos[1] < self.start_location[1] + self.goal_radius:
    #         self.agent.move_rel(0, 0, 0)
    #         # arrive at start location zone
    #         return True
    #     return False

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
        while cur_linear < calc_z * .85:
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

        return (target_xyz_base_link[0], target_xyz_base_link[1], target_yaw_base_link) # TODO : expand
