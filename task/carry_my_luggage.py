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
        self.show_byte_track_image = True
        self.byte_img = None
        self.track_queue = deque()
        self.start_time = time.time()
        # self.angle_queue = deque(maxlen=20)
        self.calcz_queue = deque(maxlen=10)
        self.obstacle_offset = 0.0
        self.last_say = time.time()
        self.tilt_angle = tilt_angle
        self.stt_option = stt_option
        self.tiny_object_yolo = None
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
        self.tiny_object_list = []


        rospy.Subscriber(bytetrack_topic, Image, self._byte_cb)
        rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_cb)
        self.rgb_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, self._rgb_callback)
        rospy.Subscriber('/deeplab_ros_node/segmentation', Image, self._segment_cb)
        rospy.Subscriber('/snu/yolo_conf', Int16MultiArray, self._tiny_cb)
        rospy.loginfo("LOAD HUMAN FOLLOWING")
        self.image_pub = rospy.Publisher('/canny_result', Image, queue_size=10)
        
    def freeze_for_humanfollowing(self):
        self.show_byte_track_image = True
        # self.agent.pose.head_pan_tilt(0, -self.tilt_angle*0.5)
        self.agent.pose.head_pan_tilt(0, -self.tilt_angle) #TODO : check tilt angle
        rospy.sleep(1)
        head_map_client = dynamic_reconfigure.client.Client("/tmc_map_merger/inputs/head_rgbd_sensor",
                                                                 config_callback=self.head_map_cb)
        head_map_client.update_configuration({"enable": True})


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
        if self.show_byte_track_image:
            bar_width = w // 4 ## TODO : 지금은 왼쪽 25%, 오른쪽 25% 제거. 확인 필요
            img[:, :bar_width] = 0
            img[:, -bar_width:] = 0
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
        filtered_data = [group for group in grouped_data if (group[5] >= .40 )]
        sorted_data = sorted(filtered_data, key = lambda x: x[0])
        sorted_arr = [item for sublist in sorted_data for item in sublist]

        yolo_x_list = []
        yolo_y_list = []


        for idx in range(len(sorted_arr) // 6):
            item = sorted_arr[6 * idx: 6 * (idx + 1)]
            x, y, w, h, class_id, conf_percent = item
            yolo_x_list.append(x)
            yolo_y_list.append(y)


        if len(yolo_y_list) != 0:
            yolo_item_y_largest_idx = np.argmax(yolo_y_list)
            self.tiny_object_yolo = (yolo_x_list[yolo_item_y_largest_idx], yolo_y_list[yolo_item_y_largest_idx])



    def _human_yolo_cb(self, data):
        '''
        snu/yolo will be depreciated
        '''

        data_list = data.data
        if data_list[0] == -1:
            self.human_box_list = [None]
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")
            print("There is no human box")

        else:
            self.human_box_list = [data_list[0], #[human_id, target_tlwh, target_score]
                                  np.asarray([data_list[1], data_list[2], data_list[3], data_list[4]], dtype=np.int64),
                                  data_list[5]]


    def _rgb_callback(self, data): ## tiny_object_edge 검출 용
        frame = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 150)
        rows, cols = edges.shape
        f = np.fft.fft2(edges)
        fshift = np.fft.fftshift(f)

        crow, ccol = rows // 2, cols // 2
        mask = np.zeros((rows, cols), np.float32)
        sigma = 50  # Standard deviation for Gaussian filter
        x, y = np.ogrid[:rows, :cols]
        mask = np.exp(-((x - crow) ** 2 + (y - ccol) ** 2) / (2 * sigma ** 2))

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
        self.image_pub.publish(canny_img_msg)


    def _segment_cb(self, data):

        #######################seg는 back때만 사용
        depth = np.asarray(self.agent.depth_image)

        data_img = self.bridge.imgmsg_to_cv2(data, 'mono16')
        self.seg_img = data_img


    def check_human_pos(self, human_box_list, location=False):
        x = human_box_list[1][0]
        y = human_box_list[1][1]
        w = human_box_list[1][2]
        h = human_box_list[1][3]
        center = [y + int(h/2), x + int(w/2)] # (y,x)


        print("center", center)
        # human_location = 'c'
        if location:
          
            if 0 < center[1]and  center[1] < 70:
                # self.angle_queue.append(-1)
                return 'lll'
            elif center[1] < 130:
                # self.angle_queue.app(-1)
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
            elif center[1] > 570 and center[1] < 640:
                # self.angle_queue.append(1)
                return 'rrr'   

        if center[1] < -40 or center[1] > 680:
            print("center[1] : ", center[1])
            return True # stop

        return False # pass
    
    def barrier_check(self, looking_downside=True, check_human=True, y_top=50,x_var=30):
        # _depth = self.agent.depth_image[:150, 10:630]
        if check_human:
            # if self.human_box_list[0] is not None:
            #     x = self.human_box_list[1][0]
            #     y = self.human_box_list[1][1]
            #     w = self.human_box_list[1][2]
            #     h = self.human_box_list[1][3]
            #     center = [y + int(h/2), x + int(w/2)] # (y,x)
            #     center_x = center[1]

            if (looking_downside):
                _depth = self.agent.depth_image[y_top:240, max(320-x_var,0):min(320+x_var, 640)] / 1000 # 480, 640, [0:340, 50:590]
            else: # no tilt
                _depth = self.agent.depth_image[200:0, 280:640] / 1000

        else:
            if (looking_downside):
                _depth = self.agent.depth_image[y_top:360, 240:400] / 1000
            
        return _depth
    

    def escape_barrier(self, calc_z):
        # if self.calc_z is not None:
        #     calc_z = self.calc_z
        cur_pose = self.agent.get_pose(print_option=False)
 
        _num_rotate=0
        y_top= 0
        _depth = self.barrier_check(y_top=y_top, x_var=320)
        print("_depth shape: ", _depth.shape)
        origin_depth = _depth
        # _depth = np.mean(_depth)
        _depth = _depth[_depth != 0]
        print("_depth shape: ", _depth.shape)
        if len(_depth) != 0:
            _depth = np.partition(_depth, min(10, len(_depth)))
            # if len(_depth) < 500:
            #     _depth = np.mean(_depth[50:min(1000, len(_depth))])
            # else:
            #     _depth = np.mean(_depth[:500])
            _depth = np.mean(_depth[:min(10, len(_depth))])

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

            rospy.loginfo(f"rect depth min : {_depth}")
            # rospy.loginfo(f"rect depth excetp 0 min : {_depth_value_except_0.min}")
            rospy.loginfo(f"calc_z  : {calc_z / 1000.0}")
            #and np.mean(_depth)< (calc_z-100)
            #and self.image_size * human_box_thres > human_box_size
            #원래 var=10, _depth < 1.3 . -> var=320, _depth < 0.7하고 sleep(0.5)
            if (calc_z!=0 and _depth < 0.5 and _depth< ((calc_z/ 1000.0)-0.3) and not (self.start_location[0] - escape_radius < cur_pose[0] < self.start_location[0] + escape_radius and \
            self.start_location[1] - escape_radius < cur_pose[1] < self.start_location[1] + escape_radius)):
                _num_rotate = _num_rotate + 1
                # rospy.sleep(1)
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")

                # self.agent.pose.head_pan_tilt(0, -self.tilt_angle)
                

                # self.agent.move_rel(-1.0,0.0,0, wait=False) #then, HSR is intended to move left (pos)
                # rospy.sleep(2.0)


                depth = self.agent.depth_image

                seg_img = self.seg_img
                height, width = seg_img.shape
                mid_x = width // 2
                # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                # baack_y, back_x = np.where(background_mask)
                print("min_y+100 : ", min_y+100)
                left_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, :mid_x])
                left_edge_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, :mid_x//2])
                print("left_background_count", left_background_count)
                right_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, mid_x:])
                right_edge_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, (mid_x*3//2):])
                print("right_background_count", right_background_count)
                # _depth = self.barrier_check()
                # # _depth = np.mean(_depth)
                # _depth = _depth[_depth != 0]
                # _depth = np.mean(_depth)
                # left_edge = np.mean(self.agent.ranges[self.agent.center_idx - 90: self.agent.center_idx - 0])
                # right_edge = np.mean(self.agent.ranges[self.agent.center_idx + 0: self.agent.center_idx + 90])

                if left_edge_background_count > 2000 or right_edge_background_count > 2000:
                    self.agent.say('Barrier checking....', show_display=False)
                    print("Barrier checking....")
                    
                    if left_background_count > right_background_count + 30:
                        print("left side is empty")
                        self.agent.move_rel(0.0,0.8,0, wait=False) #then, HSR is intended to move left (pos)
                        rospy.sleep(0.5)
                        # self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//8, wait=False)
                        # self.agent.move_rel(0,0,-self.stop_rotate_velocity//4, wait=False)
                    elif left_background_count + 30 < right_background_count:
                        print("right side is empty")
                        self.agent.move_rel(0.0,-0.8,0, wait=False) #then, HSR is intended to move right (neg)
                        rospy.sleep(0.5)
                        # self.agent.move_rel(0.3,0,self.stop_rotate_velocity//8, wait=False)
                        # self.agent.move_rel(0,0,self.stop_rotate_velocity//4, wait=False)
                    else: # ambigous
                        rospy.sleep(1.0)
                        self.agent.say('ambigous....', show_display=False)
                        print("ambigous")
                        self.agent.move_rel(-1.0,0,0, wait=False)
                        rospy.sleep(1.0)



       ################################################################

                    # #barrier post processing
                    # _depth = self.barrier_check(y_top=y_top, x_var=100)
                    # print("_depth shape: ", _depth.shape)
                    # origin_depth = _depth
                    # # _depth = np.mean(_depth)
                    # _depth = _depth[_depth != 0]
                    # print("_depth shape: ", _depth.shape)
                    # if len(_depth) != 0:
                    #     _depth = np.partition(_depth, min(10, len(_depth)))
                    #     # if len(_depth) < 500:
                    #     #     _depth = np.mean(_depth[50:min(1000, len(_depth))])
                    #     # else:
                    #     #     _depth = np.mean(_depth[:500])
                    #     _depth = np.mean(_depth[:min(10, len(_depth))])
                    #     # _depth = np.mean(_depth)

                    #     # _depth = np.min(_depth)
                    #     masked_depth = np.ma.masked_equal(origin_depth, 0)
                    #     min_index = np.argmin(masked_depth)
                    #     print("min_index : ", min_index)
                    #     # 1차원 인덱스를 2차원 인덱스로 변환
                    #     t = np.unravel_index(min_index, origin_depth.shape)
                    #     print("unravel_index(min_index, origin_depth.shape) :", t)
                    #     min_y, min_x = t
                    #     # _depth = np.mean(self._depth)
                    #     escape_radius = 0.2
                    #     # if self.human_box_list[0] is not None:
                    #     #     human_info_ary = copy.deepcopy(self.human_box_list)
                    #     #     depth = np.asarray(self.d2pc.depth)
                    #     #     twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)

                    #     rospy.loginfo(f"rect depth min : {_depth}")
                    #     # rospy.loginfo(f"rect depth excetp 0 min : {_depth_value_except_0.min}")
                    #     rospy.loginfo(f"calc_z  : {calc_z / 1000.0}")
                    #     #and np.mean(_depth)< (calc_z-100)
                    #     #and self.image_size * human_box_thres > human_box_size
                    #     if (calc_z!=0 and _depth < 0.9 and (_depth< ((calc_z/ 1000.0)-0.4) or self.agent.dist <((calc_z/1000.0)-0.4) )and not (self.start_location[0] - escape_radius < cur_pose[0] < self.start_location[0] + escape_radius and \
                    #     self.start_location[1] - escape_radius < cur_pose[1] < self.start_location[1] + escape_radius)):
                    #         _num_rotate = _num_rotate + 1
                    #         # rospy.sleep(1)
                    #         print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                    #         print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                    #         print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                    #         print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")

                    #         # self.agent.pose.head_pan_tilt(0, -self.tilt_angle)



                    #         depth = self.agent.depth_image

                    #         seg_img = self.seg_img
                    #         height, width = seg_img.shape
                    #         mid_x = width // 2
                    #         # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                    #         # baack_y, back_x = np.where(background_mask)
                    #         print("min_y+100 : ", min_y+100)
                    #         left_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, :mid_x])
                    #         left_edge_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, :mid_x//2])
                    #         print("left_background_count", left_background_count)
                    #         right_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, mid_x:])
                    #         right_edge_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, (mid_x*3//2):])
                    #         print("right_background_count", right_background_count)
                    #         # _depth = self.barrier_check()
                    #         # # _depth = np.mean(_depth)
                    #         # _depth = _depth[_depth != 0]
                    #         # _depth = np.mean(_depth)
                    #         if left_edge_background_count > 2000 or right_edge_background_count > 2000:
                    #             self.agent.say('Barrier checking 2....', show_display=True)
                    #             print("Barrier checking....")
                    #             if left_background_count > right_background_count:
                    #                 print("left side is empty")
                    #                 self.agent.move_rel(0,0.6,0, wait=False) #then, HSR is intended to move left (pos)
                    #                 rospy.sleep(2)
                    #                 # self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//8, wait=False)
                    #                 # self.agent.move_rel(0,0,-self.stop_rotate_velocity//4, wait=False)
                    #             else:
                    #                 print("right side is empty")
                    #                 self.agent.move_rel(0,-0.6,0, wait=False) #then, HSR is intended to move right (neg)
                    #                 rospy.sleep(2)
                    #                 # self.agent.move_rel(0.3,0,self.stop_rotate_velocity//8, wait=False)
                    #                 # self.agent.move_rel(0,0,self.stop_rotate_velocity//4, wait=False)


                ########################################################################







    def escape_barrier_back(self, calc_z):
        # if self.calc_z is not None:
        #     calc_z = self.calc_z
        cur_pose = self.agent.get_pose(print_option=False)
 
        _num_rotate=0
        y_top= 40
        _depth = self.barrier_check(check_human=False, y_top=y_top)
        print("_depth shape: ", _depth.shape)
        origin_depth = _depth
        # _depth = np.mean(_depth)
        _depth = _depth[_depth != 0]
        print("_depth shape: ", _depth.shape)
        if len(_depth) != 0:
            _depth = np.partition(_depth, min(1000, len(_depth)))
            if len(_depth) < 500:
                _depth = np.mean(_depth[100:min(1000, len(_depth))])
            else:
                _depth = np.mean(_depth[:500])
            # _depth = np.mean(_depth)

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

            rospy.loginfo(f"rect depth min : {_depth}")
            # rospy.loginfo(f"rect depth excetp 0 min : {_depth_value_except_0.min}")
            rospy.loginfo(f"calc_z  : {calc_z / 1000.0}")
            #and np.mean(_depth)< (calc_z-100)
            #and self.image_size * human_box_thres > human_box_size
            if (calc_z!=0 and _depth < 1.9 and (_depth< ((calc_z/ 1000.0)-0.4) or self.agent.dist <((calc_z/1000.0)-0.4) )and not (self.start_location[0] - escape_radius < cur_pose[0] < self.start_location[0] + escape_radius and \
            self.start_location[1] - escape_radius < cur_pose[1] < self.start_location[1] + escape_radius)):
                _num_rotate = _num_rotate + 1
                # rospy.sleep(1)
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
                print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")

                # self.agent.pose.head_pan_tilt(0, -self.tilt_angle)



                depth = self.agent.depth_image

                seg_img = self.seg_img
                height, width = seg_img.shape
                mid_x = width // 2
                # background_mask = np.logical_or( seg_img == 0, seg_img == 15)# 배경 부분 또는 사람 부분
                # baack_y, back_x = np.where(background_mask)
                print("min_y+100 : ", min_y+100)
                left_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, :mid_x])
                # left_edge_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y +y_top+20, :mid_x//2])
                print("left_background_count", left_background_count)
                right_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, mid_x:])
                # right_edge_background_count = np.mean(depth[max(min_y+y_top-20, 0):min_y+y_top+20, (mid_x*3//2):])
                print("right_background_count", right_background_count)
                # _depth = self.barrier_check()
                # # _depth = np.mean(_depth)
                # _depth = _depth[_depth != 0]
                # _depth = np.mean(_depth)
                left_edge = np.mean(self.agent.ranges[self.agent.center_idx - 55: self.agent.center_idx - 35])
                right_edge = np.mean(self.agent.ranges[self.agent.center_idx + 35: self.agent.center_idx + 55])

                if left_edge > 1.0 or right_edge > 1.0:
                    self.agent.say('Barrier checking....', show_display=True)
                    print("Barrier checking....")
                    if left_background_count > right_background_count:
                        print("left side is empty")
                        self.agent.move_rel(0,0.6,0, wait=False) #then, HSR is intended to move left (pos)
                        rospy.sleep(3)
                        # self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//8, wait=False)
                        # self.agent.move_rel(0,0,-self.stop_rotate_velocity//4, wait=False)
                    else:
                        print("right side is empty")
                        self.agent.move_rel(0,-0.6,0, wait=False) #then, HSR is intended to move right (neg)
                        rospy.sleep(3)
                        # self.agent.move_rel(0.3,0,self.stop_rotate_velocity//8, wait=False)
                        # self.agent.move_rel(0,0,self.stop_rotate_velocity//4, wait=False)







    def escape_tiny_canny(self):
        # center = None
        # if self.human_box_list[0] is not None:
        #     x = self.human_box_list[1][0]
        #     y = self.human_box_list[1][1]
        #     w = self.human_box_list[1][2]
        #     h = self.human_box_list[1][3]
        #     center = [y + int(h/2), x + int(w/2)] # (y,x)
        contours = self.contours

        tiny_exist = False
        tiny_loc = None



        seg_img = self.seg_img[:, :]

        # Create a mask for depth values greater than 0 and seg_img equal to 15
        # human_mask = (seg_img == 15)
        # human_y, human_x = np.where(human_mask)
        # if len(human_y) != 0:
        #     human_y_max = np.max(human_y)
        # else:
        #     human_y_max = 440

        for contour in contours:
            area = cv2.contourArea(contour)
            if 200 < area < 5000:  # 면적 기준으로 작은 물체 필터링 (적절히 조절 가능)
                # print(area)
                x, y, w, h = cv2.boundingRect(contour)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # if y > human_y_max and y > 440 and x > 240 and x < 400:
                if y+h//2 > 300 and y+h//2 < 450 and x+w//2 > 230 and x+w//2 < 410:

                    tiny_exist = True
                    print('Tiny object.')
                    # cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    break


        if len(self.calcz_queue) > 1 and self.calcz_queue[-1] is not None and self.tiny_object_yolo is not None :
            last_calc_z = self.calcz_queue[-1]
            print("canny last calc_z: ", last_calc_z)
            # self.dist = np.min(self.agent.ranges[self.agent.center_idx - 90: self.agent.center_idx + 90])

            # yolo_item_y_largest_idx = self.yolo_item_y_largest_idx

            yolo_x, yolo_y = self.tiny_object_yolo
            if tiny_exist and abs(yolo_x - x) < 20 and abs(yolo_y - y) < 20 :

                depth = self.agent.depth_image

                seg_img = self.seg_img
                height, width = seg_img.shape
                mid_x = width // 2

                left_background_count = np.mean(depth[100-20:100+20, :x])
                print("left_background_count", left_background_count)
                right_background_count = np.mean(depth[100-20:100+20, x+w:])
                print("right_background_count", right_background_count)


                left_edge_background_count = np.mean(depth[100-20:100+20, :mid_x//2])
                right_edge_background_count = np.mean(depth[100-20:100+20, (mid_x*3//2):])
                # _depth = self.barrier_check()
                # # _depth = np.mean(_depth)
                # _depth = _depth[_depth != 0]
                # _depth = np.mean(_depth)
                if left_edge_background_count > 2000 or right_edge_background_count > 2000:

                    # if (self.human_box_list[0] is not None) and (center[1] < 180 or center[1] > 500):
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    print('Tiny object. I\'ll avoid it.')
                    self.agent.say('Tiny object. I\'ll avoid it.', show_display=False)
                    if right_background_count > left_background_count:
                        self.agent.move_rel(0.0,-0.8,0, wait=False) ## move right is neg
                        rospy.sleep(2)
                        self.agent.move_rel(0.3,0,self.stop_rotate_velocity//12, wait=False)
                        rospy.sleep(1)
                    else:
                        self.agent.move_rel(0.0,0.8,0, wait=False)
                        rospy.sleep(2)
                        self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//12, wait=False)
                        rospy.sleep(1)




    def escape_tiny_canny_back(self):

        contours = self.contours

        tiny_exist = False
        tiny_loc = None


        for contour in contours:
            area = cv2.contourArea(contour)
            if 200 < area < 5000:  # 면적 기준으로 작은 물체 필터링 (적절히 조절 가능)
                # print(area)
                x, y, w, h = cv2.boundingRect(contour)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # if y > human_y_max and y > 440 and x > 240 and x < 400:
                if y+h//2 > 300 and y+h//2 < 450 and x+w//2 > 230 and x+w//2 < 410:

                    tiny_exist = True
                    print('Tiny object.')
                    # cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    break


        if tiny_exist and self.agent.dist > 1.0 :
            

            depth = self.agent.depth_image

            seg_img = self.seg_img
            height, width = seg_img.shape
            mid_x = width // 2

            left_background_count = np.mean(depth[100-20:100+20, :mid_x])
            print("left_background_count", left_background_count)
            right_background_count = np.mean(depth[100-20:100+20, mid_x:])
            print("right_background_count", right_background_count)



            # if (self.human_box_list[0] is not None) and (center[1] < 180 or center[1] > 500):
            print('Tiny object. I\'ll avoid it.')              
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            print('Tiny object. I\'ll avoid it.')
            self.agent.say('Tiny object. I\'ll avoid it.', show_display=False)
            if right_background_count > left_background_count:
                self.agent.move_rel(0.0,-0.8,0, wait=False) ## move right is neg
                rospy.sleep(3)
                # self.agent.move_rel(0.3,0,self.stop_rotate_velocity//6, wait=False)
            else:
                self.agent.move_rel(0.0,0.8,0, wait=False)
                rospy.sleep(3)
                # self.agent.move_rel(0.3,0,-self.stop_rotate_velocity//6, wait=False)


                    # se
        

    def stt_destination(self, stt_option, calc_z=0):
        cur_pose = self.agent.get_pose(print_option=False)
        # print("in area", [cur_pose[0], cur_pose[1]], "last moved time", time.time() - self.agent.last_moved_time)
        # if (time.time() - self.agent.last_moved_time > 3.0) and self.human_box_list[0] is None:


        if (time.time() - self.agent.last_moved_time > 7.0) and not (self.start_location[0] - self.goal_radius < cur_pose[0] < self.start_location[0] + self.goal_radius and \
            self.start_location[1] - self.goal_radius < cur_pose[1] < self.start_location[1] + self.goal_radius):

            print("lmt_if", self.agent.last_moved_time)

            self.show_byte_track_image = True
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
        self.start_time = start_time
        # self.escape_barrier(calc_z)
        # if self.human_box_list[0] is None: # no human detected

        # human_info_ary = copy.deepcopy(self.human_box_list)

        # depth = np.asarray(self.d2pc.depth)
        # twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth)
        # self.escape_barrier(calc_z)
        if self.human_box_list[0] is None : # no human detected
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



            self.agent.move_rel(target_xyyaw[0], target_xyyaw[1], target_xyyaw[2], wait=False)


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

class BagInspection:
    def __init__(self, agent):
        self.agent = agent
        self.pointing_dir = -1 # left : -1, right : 1
        self.bridge = CvBridge()
        self.marker_maker = MarkerMaker('/snu/robot_path_visu')

        #Subscribe to take a bag
        self.yolo_bag_list = [24, 25, 26]
        self.d2pc = Depth2PC()

        yolo_topic = '/snu/yolo_img'
        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'  # use data in agent
        rospy.Subscriber('/snu/yolo_conf', Int16MultiArray, self._bag_yolo_cb)
        rospy.Subscriber(yolo_topic, Image, self._yolo_img_cb)
        rospy.Subscriber(rgb_topic, Image, self._rgb_cb)

        self.axis_transform = Axis_transform()

        self.bag_yolo_data = [24, 25, 26]
        self.conf_threshold = .75 # write in percent

    def get_bag_by_x(self, arr):
        if arr is None:
            return None
        
        grouped_data = [arr[i:i+6] for i in range(0, len(arr), 6)]

        filtered_data = [group for group in grouped_data if (group[5] >= 80 and group[4] in self.yolo_bag_list)]

        sorted_data = sorted(filtered_data, key = lambda x: x[0])

        sorted_arr = [item for sublist in sorted_data for item in sublist]

        return sorted_arr

    def _bag_yolo_cb(self, data):
        self.bag_yolo_data = self.get_bag_by_x(data.data)
    
    def _rgb_cb(self, data):
        # add mediapipe module and find pointing direction
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def _yolo_img_cb(self, data):
        self.yolo_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')


    def bag_grasping_check(self, thres=0.6):
        _depth = self.agent.depth_image
            
        _depth_arr = [_depth[255:275, 480:500], _depth[255:275, 535:555], _depth[255:275, 570:590], _depth[370:390, 480:500], _depth[370:390, 535:555], _depth[370:390, 570:590]]

        score = 0

        for i in range(6):
            print("thres", np.nanmean(_depth_arr[i])/1000)
            if np.nanmean(_depth_arr[i])/1000 < thres:      
                score += 1
            
        if score == 0:
            return False
        else:
            return True

        """
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
        """

    #############################################
    # for pointing hand
    
    #calculate pointing coeff - point of elbow and wrist
    #output: array elbow, array wrist
    def pointing_coeff(self):
        image = self.agent.rgb_img
        #by pointing hand, deciding yaw tilt by 3d detection
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing_styles = mp.solutions.drawing_styles
        mp_pose = mp.solutions.pose

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

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                mp_drawing.draw_landmarks(
                        image,
                        results.pose_landmarks,
                        mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                right_grad = (right_elbow[0] - right_wrist[0]) / (right_elbow[1] - right_wrist[1])
                left_grad = (left_elbow[0] - left_wrist[0]) / (left_elbow[1] - left_wrist[1])

                point_thre = 0.4
                hand = None
                if right_grad > point_thre or left_grad > point_thre:
                    if right_grad > point_thre:
                        hand = 'right'  # right
                    else:
                        hand = 'left'
                    self.pointing_dir = -1
                elif right_grad < -point_thre or left_grad < -point_thre:
                    if right_grad < -point_thre:
                        hand = 'right'  # right
                    else:
                        hand = 'left'
                    self.pointing_dir = 1
                print("pointing dir", self.pointing_dir)

                if hand == 'left':
                    elbow = left_elbow
                    wrist = left_wrist
                    print("[trial] pointing hand is left")
                elif hand == 'right':
                    elbow = right_elbow
                    wrist = right_wrist
                    print("[trial] pointing hand is right")
                else:
                    return None, None, None            

            image = cv2.flip(image, 1)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            print("pixel", elbow, wrist)
            return image, elbow, wrist
        except Exception as e:
            print('[MediaPipe]', e)
            return None, None, None
        
    #calculate 3d point of elbow and wrist
    #output: point and gradient of line: by baselink coordinate    
    def get_pc(self, elbow, wrist):
        _pc = self.agent.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]

        elbow_pc = pc_np[int(np.floor(elbow[1]))-1:int(np.ceil(elbow[1]))+2, int(np.floor(elbow[0]))-1:int(np.ceil(elbow[0]))+2] 
        elbow_pc = elbow_pc.reshape(-1, 3)
        elbow_pc = np.mean(elbow_pc, axis = 0)
        elbow_by_base_link = self.axis_transform.tf_camera_to_base(elbow_pc)

        print("elbow_pc", elbow_pc)

        wrist_pc = pc_np[int(np.floor(wrist[1]))-1:int(np.ceil(wrist[1]))+2, int(np.floor(wrist[0]))-1:int(np.ceil(wrist[0]))+2]
        wrist_pc = wrist_pc.reshape(-1, 3)
        wrist_pc = np.mean(wrist_pc, axis = 0)
        wrist_by_base_link = self.axis_transform.tf_camera_to_base(wrist_pc)
        
        print("wrist_pc", wrist_pc)

        #print("elbow, wrist", elbow_by_base_link, wrist_by_base_link)
        
        point = elbow_by_base_link
        gradient = wrist_by_base_link - elbow_by_base_link


        print("point, gradient", point, gradient)
        return point, gradient
    
    #calc yaw and tilt
    def cal_yaw_tilt(self):
        image, elbow, wrist = self.pointing_coeff()

        if elbow is None:
            return None, None, None

        point, gradient = self.get_pc(elbow, wrist)

        if point is None:
            return None, None, None
        mean_height = 0.28

        t = (mean_height - point[2]) / gradient[2]

        x = np.round(point[0] + gradient[0] * t, 3)
        y = np.round(point[1] + gradient[1] * t, 3)

        print("x, y", x, y, np.arctan2(y,x))

        yaw = np.arctan2(y,x)

        distance = np.sqrt(x**2 + y**2)
        if distance < 0.5:
            tilt = -60
        elif distance < 0.8:
            tilt = -50
        elif distance < 1.2:
            tilt = -40
        elif distance < 1.5:
            tilt = -30
        elif distance < 2.0:
            tilt = -20
        else: tilt = -25

        print("yaw, tilt", yaw, tilt)

        return image, yaw, tilt
    
    def detect_bag_3d(self, timeout=5.0):
        #detect bag in 3d motion
        start_time = time.time()
        while time.time() - start_time <= timeout:
            _pc = self.agent.pc.reshape(480, 640)
            pc_np = np.array(_pc.tolist())[:, :, :3]
            bag_yolo_data = self.bag_yolo_data


            self.agent.head_display_image_pubish(self.yolo_img)
            
            print(bag_yolo_data)
            if len(bag_yolo_data) // 6 == 0:
                return None, None, None
            elif len(bag_yolo_data) // 6 == 1:
                idx = 0
            else:
                distance = []
                for i in range (len(bag_yolo_data) // 6):
                    distance.append(np.sqrt((bag_yolo_data[i*6]-240)**2 + (bag_yolo_data[i*6+1]-320)**2))
                    print(bag_yolo_data[i*6+4], np.sqrt((bag_yolo_data[i*6]-240)**2 + (bag_yolo_data[i*6+1]-320)**2))
                idx = np.argmin(distance)
          
            item = bag_yolo_data[6 * idx: 6 * (idx + 1)]
            cent_x, cent_y, width, height, class_id, conf_percent = item

            '''
            BAG PICKING 0410
            '''
            if class_id == 24:
                bag_height = 0.39
                bag_breadth = 0.41
                bag_length = 0.22
            elif class_id == 25:
                bag_height = 0.33
                bag_breadth = 0.28
                bag_length = 0.1
            else:
                bag_height = 0.285
                bag_breadth = 0.25
                bag_length = 0.09
            '''
            '''

            
                
            start_x = cent_x - (width // 2)
            start_y = cent_y - (height // 2)
            object_pc = pc_np[start_y:start_y+height, start_x:start_x+width]
            object_pc = object_pc.reshape(-1, 3)
            points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

            img = self.yolo_img[start_y:start_y+height, start_x:start_x+width]
            self.agent.head_display_image_pubish(img)

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
                #points_by_base_link = points_by_base_link[
                #    np.where((points_by_base_link[:, 0] < point_min_x + object_depth))]



                # 3. select min_max points for draw 3d box
                min_points = [round(np.nanmin(points_by_base_link[:, 0]), 4),
                              round(np.nanmin(points_by_base_link[:, 1]), 4),
                              round(np.nanmin(points_by_base_link[:, 2]), 4)]
                max_points = [round(np.nanmax(points_by_base_link[:, 0]), 4),
                              round(np.nanmax(points_by_base_link[:, 1]), 4),
                              round(np.nanmax(points_by_base_link[:, 2]), 4)]
                
                if(np.abs(bag_breadth -np.abs(max_points[0] - min_points[0])) < 0.15):
                    bag_orientation = -1.57
                elif(np.abs(bag_length -np.abs(max_points[1] - min_points[1])) < 0.15):
                    bag_orientation = 0
                else: 
                    x_max_idx = np.argmax(points_by_base_link[:, 0])
                    y_min_idx = np.argmin(points_by_base_link[:, 1])

                    x_max_point = points_by_base_link[x_max_idx]
                    y_min_point = points_by_base_link[y_min_idx]

                    vector = np.abs(x_max_point - y_min_point)
                    bag_orientation = np.arctan2(np.abs(vector[0]), np.abs(vector[1]))

                    dist = np.sqrt(np.round((x_max_point[0] - y_min_point[0])**2 + (x_max_point[1] - y_min_point[1])**2, 4))
                    if np.abs(dist - bag_breadth) < np.abs(dist - bag_length):
                        dist = bag_breadth
                        bag_orientation = - bag_orientation
                    else:
                        dist = bag_length

                

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
            return target_base_xyz, object_size, bag_height, bag_orientation
        return None, None, None, None

    def rotate_to_find_bag(self, yaw, tilt):

        self.agent.pose.head_pan_tilt(0, tilt)
        if np.isnan(yaw):
            yaw = self.pointing_dir * np.pi / 12
        self.agent.move_rel(0, 0, yaw, wait = True)

        for i in range(2):
            if i == 1:  
                self.agent.pose.head_pan_tilt(0, tilt - 10)
            print("Searching")
            rospy.sleep(2)
            target_base_xyz, object_size, bag_height, bag_orientation = self.detect_bag_3d(timeout=1)
            if(np.sqrt(target_base_xyz[0]**2 + target_base_xyz[1]**2) > 2.0):
                continue
            if target_base_xyz is not None:
                print("We find bag!")
                return target_base_xyz, object_size, bag_height, bag_orientation
        
        
        rospy.sleep(1)

        direction = self.pointing_dir

        for _ in range(3):
            self.agent.move_rel(0, 0, 30 * direction / 180 * np.pi, wait=True)
            for tilt_idx in range(2):
                if tilt_idx == 0:
                    self.agent.pose.head_pan_tilt(0, -45)
                else:    
                    self.agent.pose.head_pan_tilt(0, -30)
                print("Searching")
                rospy.sleep(2)
                target_base_xyz, object_size, bag_height, bag_orientation = self.detect_bag_3d(timeout=1)
                if target_base_xyz is not None:
                    print("We find bag!")
                    return target_base_xyz, object_size, bag_height, bag_orientation
        print("We cannot find bag.. i'll just rotate back")
        # if pointing_dir is not None:
        self.agent.move_rel(0, 0, -np.pi * direction/12, wait=True)
        return None, None, None
       

        
    def run_bag_inspection(self, yaw, tilt):

        before_pick_pose = self.agent.get_pose(print_option=False)
        target_base_xyz, object_size, bag_height, bag_orientation = self.rotate_to_find_bag(yaw, tilt)

        print("object size: " + str(object_size))
        print("bag height: " + str(bag_height))
        if target_base_xyz is None:
            print("1.4 We cannot find a bag. Please hand me a bag!")
            self.agent.say("We cannot find a bag.", show_display=True)
            self.agent.open_gripper()
            rospy.sleep(1)
            self.agent.say("Please Hand me bag", show_display=True)
            self.agent.pose.neutral_pose()
            rospy.sleep(1)
            # for i in range(1, 0, -1):
            #     self.agent.say(str(i))
            #     rospy.sleep(1)
            self.agent.grasp()
            self.agent.pose.move_to_go()
        else:
            # Todo: grasp towards the target point
            print("1.5 bag_coord_baselink_final", target_base_xyz)
            self.agent.open_gripper()
            self.agent.say("I will pick up this bag")
            print("1.6 let's pick up the bag")
            self.agent.pose.head_pan_tilt(0, 0)

            hand_dist_xyz = self.agent.yolo_module.calculate_dist_to_pick(target_base_xyz, 5)
            mov_x, mov_y = hand_dist_xyz[0], hand_dist_xyz[1]
            #mov_x += 0.05
            print("1.7 go to the direction", (mov_x, mov_y))
            print("current position: ", self.agent.get_pose())
            self.marker_maker.pub_marker([mov_x, mov_y, 1], 'base_link')
            self.agent.move_rel(mov_x, mov_y, wait=True)
            print("moved position: ", self.agent.get_pose())

            #if object_size[0] >= object_size[1]:  
            #    bag_orientation = -1.57
            #else:
            #    bag_orientation = 0

            isSuccc = False
            isSecond = False
            for i in range(2):
                if isSecond:
                    bag_orientation = bag_orientation + 1.57
                    if bag_orientation > 3.14:
                        bag_orientation -= 3.14
                isSecond = True
                self.agent.pose.pick_up_bag(bag_height, bag_orientation)
                self.agent.pose.arm_lift_top_down(bag_height)
                self.agent.grasp()
                self.agent.pose.head_tilt(-30)
                self.agent.pose.bag_inspection_pose()
                # self.agent.say("Let's check.")
                # rospy.sleep(3)
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
            #self.agent.move_abs_coordinate(before_pick_pose, wait=True)
            self.agent.move_rel(0, 0, -yaw, wait=False)
        # self.agent.say("I am ready to follow you", show_display=True)
        # rospy.sleep(2)

processes = []


def head_map_cb(config):
    rospy.loginfo(config)


# def start_process(command):
#     process = subprocess.Popen(command)
#     processes.append(process)
#     return process

# # 모든 프로세스를 종료하는 함수
# def terminate_all_processes():
#     for process in processes:
#         try:
#             os.kill(process.pid, signal.SIGTERM)
#             process.wait()
#         except OSError as e:
#             print(f"Error terminating process {process.pid}: {e}")



def carry_my_luggage(agent):
    # task params
    bag_search_limit_time = 15
    goal_radius = 0.3
    pose_save_time_period = 4
    start_location = agent.get_pose(print_option=False)
    bag_height = 0.25
    stop_rotate_velocity = 1.2 #1.2
    try_bag_picking = False #True
    try_bytetrack = False
    map_mode = False
    stt_option = False #True
    yolo_success = True
    tilt_angle = 20
    

    # Capture target
    demotrack_pub = rospy.Publisher('/snu/demotrack', String, queue_size=10)

    ## class instantiation
    bag_inspection = BagInspection(agent)

    human_reid_and_follower = HumanReidAndFollower(init_bbox=[320 - 100, 240 - 50, 320 + 100, 240 + 50],
                                                   frame_shape=(480, 640),
                                                   stop_thres=.2,
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
    try:
        # yolo_process = subprocess.run(['python3', '/home/tidy/Robocup2024/module/yolov7/run_yolov7.py'])
        script_path = "/home/tidy/Robocup2024/yolo.sh"
        # yolo_process = subprocess.Popen(['bash', script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        yolo_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {script_path}; exec bash']
        yolo_process = subprocess.Popen(yolo_command)


        ####################
        # agent.say("start yolo")
        rospy.sleep(3)
        if try_bag_picking:
            # [Bonus] 1. bag detection
            print("1-1. Hello, Please pointing where the bag is")
            agent.say("Please point Where the bag is", show_display=True)
            #########
            # 1.1 check human pointing direction
            bag_searching_start_time = time.time()
            while time.time() - bag_searching_start_time <= bag_search_limit_time:
                start_searching_time = time.time()
                while time.time() - start_searching_time <=7:  # check bag during 5 seconds
                    visualized_image, yaw, tilt = bag_inspection.cal_yaw_tilt()
                    if visualized_image is not None:
                        agent.head_display_image_pubish(visualized_image)
                # check human direction
                if yaw is None:
                    print("1-2. Please point again")
                    agent.say("Please point again")
                else:
                    break
            ########
            print("1-3.pointing direction:", 'right' if bag_inspection.pointing_dir == -1 else 'left')
            # 1.3 check shopping bag
            agent.pose.head_pan_tilt(0, -30)
            bag_inspection.run_bag_inspection(yaw, tilt)
            #####    
    except Exception as e:
        print("error starting yolo")
        yolo_success = False
    finally:
        yolo_process.terminate()

    
            
    byte_path = "/home/tidy/Robocup2024/byte.sh"
    seg_path = "/home/tidy/Robocup2024/seg.sh"

    # 명령어들
    byte_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {byte_path}; exec bash']
    seg_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {seg_path}; exec bash']

    byte_process = subprocess.Popen(byte_command)
    seg_process = subprocess.Popen(seg_command)


    if not yolo_success or not try_bag_picking:
       # no try bag picking
        agent.open_gripper()
        rospy.sleep(1)
        agent.say("Please Hand me bag", show_display=True)
        agent.pose.neutral_pose()
        rospy.sleep(1)
        # for i in range(5, 0, -1):
        #     agent.say(str(i))
        #     rospy.sleep(1)
        agent.grasp()
        agent.pose.move_to_go()


    # yolo_process.terminate()

    ######################
    # 2. human following

    demotrack_pub.publish(String('target'))
    agent.pose.head_pan_tilt(0, 0)
    rospy.sleep(15)
    agent.say("If you are arrived at the destination", 
    show_display=True)
    rospy.sleep(3)
    agent.say("Please stand still.", show_display=True)
    rospy.sleep(2)
    # agent.say("Please do not move for ten seconds when you arrived at the destination", show_display=True)
    agent.say("Please Keep the one meter between us!", show_display=True)
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
            human_following.show_byte_track_image = True
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
    # for i in range(1, 0, -1): # TODO :5
    #     agent.say(str(i))
    #     rospy.sleep(1)
    agent.open_gripper()
    rospy.sleep(1)
    agent.pose.move_pose()
    agent.grasp()

    # 4. back to the start location
    agent.move_rel(0, 0, np.pi, wait=True) ######TODO : return
    rospy.loginfo("go back to original location")

    now_d2pc = Depth2PC()

    # version 2 : track queue
    if not map_mode:
        track_queue = human_following.track_queue  # get trace of the robot

        for i in range(len(track_queue)): # 돌아가는 길
            print("i:", i)
            cur_track = track_queue[len(track_queue)-i-1]
            calc_z= 2000

            depth = np.asarray(now_d2pc.depth)
            depth_slice = depth[:, :]
            seg_img = human_following.seg_img[:, :]
            valid_depth_mask = depth_slice > 0
            human_mask = (seg_img == 15) & valid_depth_mask
            human_y, human_x = np.where(human_mask)
            human_depth_values = depth_slice[human_y, human_x]
            if human_depth_values.size != 0: # 사람이 있으면
                min_index = np.argmin(human_depth_values)
                min_human_y, min_human_x = human_y[min_index], human_x[min_index]
                human_seg_pos = [min_human_x, min_human_y]
                twist, calc_z = human_following.human_reid_and_follower.back_follow(depth, human_seg_pos)
                print("seg human detected, calc_z : ", calc_z)

                while calc_z < 650.0: #0.7m내에 사람 있는 동안 일단 정지
                    print("seg human!!!!!!!!!!!!!!!!!!")
                    print("seg human!!!!!!!!!!!!!!!!!!")
                    print("seg human!!!!!!!!!!!!!!!!!!")
                    print("seg human!!!!!!!!!!!!!!!!!!")
                    rospy.sleep(3)
                    
                    human_info_ary = copy.deepcopy(human_following.human_box_list)
                    depth = np.asarray(now_d2pc.depth)

                    depth_slice = depth[:, :]
                    seg_img = human_following.seg_img[:, :]

                    valid_depth_mask = depth_slice > 0
                    human_mask = (seg_img == 15) & valid_depth_mask

                    human_y, human_x = np.where(human_mask)

                    human_depth_values = depth_slice[human_y, human_x]
                    if human_depth_values.size == 0: #사람 없으면 탈출
                        break
                    min_index = np.argmin(human_depth_values)
                    min_human_y, min_human_x = human_y[min_index], human_x[min_index]
                    human_seg_pos = [min_human_x, min_human_y]
                    twist, calc_z = human_following.human_reid_and_follower.back_follow(depth, human_seg_pos)
            print("before escape barrier, current calc_z: ", calc_z)
            human_following.escape_barrier_back(calc_z) #사람 있으면 calc_z는 사람까지 거리, 사람 없으면 2m 고정
            # human_following.escape_tiny()
            human_following.escape_tiny_canny_back()


            agent.move_abs_coordinate(cur_track, wait=False)
            
            rospy.sleep(3)
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

    seg_process.terminate()
    byte_process.terminate()



    agent.say("Finnish carry my luggage", show_display=True)

if __name__ == '__main__':
    rospy.init_node('carry_my_luggage_test')
    agent = Agent()

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

