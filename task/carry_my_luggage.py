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
        self.track_queue = deque()
        self.obstacle_offset = 0.0
        self.last_say = time.time()
        self.tilt_angle = tilt_angle
        self.stt_option = stt_option
        self.save_one_time = False
        self.last_human_pos = None
        self.human_seg_pos = None
        self.image_size = None
        self.image_shape = None
        bytetrack_topic = '/snu/bytetrack_img'
        # self.tiny_object_list = ['pink_milk', 'blue_milk', 'coke', 'banana', 'apple', 'carrot', 'strawberry', 'sweet_potato', 'lemon', \
        #                           'cheezit', 'strawberry_jello', 'chocolate_jello', 'sugar', 'mustard', 'spam', 'tomato_soup', 'fork', 'plate',\
        #                               'knife', 'bowl', 'spoon', 'blue_mug', 'tennis_ball', 'soft_scrub', 'yellow_bag', 'blue_bag', 'white_bag',\
        #                                   'plum', 'peach', 'orange']
        self.tiny_object_list = []
        rospy.Subscriber(bytetrack_topic, Image, self._byte_cb)
        rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_cb)
        rospy.Subscriber('/deeplab_ros_node/segmentation', Image, self._segment_cb)
        rospy.Subscriber('/snu/yolo_conf', Int16MultiArray, self._tiny_cb)
        rospy.loginfo("LOAD HUMAN FOLLOWING")

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
        else:
            self.human_box_list = [data_list[0], #[human_id, target_tlwh, target_score]
                                  np.asarray([data_list[1], data_list[2], data_list[3], data_list[4]], dtype=np.int64),
                                  data_list[5]]

    def _segment_cb(self, data):
        if self.human_box_list[0] == None:
            print("There is no human box")
            self.human_seg_pos = None
        else :
            # print(data)
            x = self.human_box_list[1][0]
            y = self.human_box_list[1][1]
            w = self.human_box_list[1][2]
            h = self.human_box_list[1][3]
            # data_img = data
            data.encoding='mono16'
            # print(len(data))
            data_img = self.bridge.imgmsg_to_cv2(data, 'mono16')
            # print(data_img)
            # data_img = cv2.imread(data, -1)
            # cv2.imshow("data image", data_img*10)
            # print(f"data_img shape : {data_img.shape}")
            # print(f"self.imag shape : {self.image_shape}")
            seg_size = data_img.shape
            # data_img = cv2.resize(data_img, self.image_shape)
            crop_img = data_img[y:y+h, x:x+w]
            # cv2.imshow("crop image", crop_img)
            # print(f"crop_img : {crop_img.shape}")
            # print(f"original human_box size : {self.human_box_list[1]}")
            print(f"original human box center : {self.human_box_list[1][0] + self.human_box_list[1][2]//2, self.human_box_list[1][1] + self.human_box_list[1][3]//2}")

            # gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            # # _, th = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            # cv2.imshow("binary", gray)



            #####################BRANCH 1. 여러 SEGMENT중에 사람 찾기###################

            # ret, labels = cv2.connectedComponents(crop_img)


            # centers = []
            # for label in range(1, ret):  
            #     mask = labels == label
            #     mask.resize(seg_size)
            #     y, x = np.where(mask)
            #     center_x, center_y = np.mean(x), np.mean(y)
            #     centers.append((center_x, center_y))
            #     print(f"centers : {centers}")

            # if centers:
            #     top_center = min(centers, key=lambda c: c[1])  
            #     top_center_global = (top_center[0] + x, top_center[1] + y)
            #     self.human_seg_pos = top_center_global
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print(f"human seg pos : {self.human_seg_pos}")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")
            #     print("humanhumanhumanhumanhumanhumanhumanhumanhuman")

            # else:
            #     print("There is a human box but no segmentation anywhere")


            #####################여러 SEGMENT중에 사람 찾기###################


            #####################BRANCH 2. LABEL=15면 사람임###################


            human_mask = crop_img == 15
            human_y, human_x = np.where(human_mask)

            if human_y.size>0 and human_x.size>0:
                # center_x, center_y = int(np.mean(human_x)), int(np.mean(human_y))
                # self.human_seg_pos = (center_x + x, center_y + y)
                topmost_y = np.min(human_y)
                x_at_topmost_y = human_x[np.argmin(human_y)]
                self.human_seg_pos = (x_at_topmost_y + x, topmost_y + y)
                if topmost_y + y < 1 :
                    self.human_seg_pos = (x_at_topmost_y + x, 1)
                print(f"human top seg_pos : {self.human_seg_pos}")
            else:
                print("There is a human box but no segmentation anywhere")
                self.human_seg_pos = None


            #####################BRANCH 2. LABEL=15면 사람임###################


            


    def check_human_pos(self, human_box_list, location=False):
        x = human_box_list[1][0]
        y = human_box_list[1][1]
        w = human_box_list[1][2]
        h = human_box_list[1][3]
        center = [y + int(h/2), x + int(w/2)] # (y,x)
        # print("center", center)
        # human_location = 'c'
        if location:
            if center[1] < 80:
                return 'lll'
            elif center[1] < 110:
                return 'll'
            elif center[1] < 140:
                return 'l'
            elif center[1] > 500:
                return 'r'
            elif center[1] > 530:
                return 'rr'
            elif center[1] > 560:
                return 'rrr'
            

        if center[1] < 0 or center[1] > 640:
            return True # stop

        return False # pass
    
    def barrier_check(self, looking_downside=True):
        # _depth = self.agent.depth_image[:150, 10:630]
        if (looking_downside):
            _depth = self.agent.depth_image[200:280, 280:640] / 1000 # 480, 640
        else: # no tilt
            _depth = self.agent.depth_image[200:0, 280:640] / 1000

            
        return _depth
    

    def escape_barrier(self, calc_z):
        cur_pose = self.agent.get_pose(print_option=False)
        thres = 1.0
        human_box_thres = 0.5
        if self.human_box_list[0] is not None:
            # print(f"human_box_list[1] : {self.human_box_list[1]}")
            human_box_size = self.human_box_list[1][2] * self.human_box_list[1][3] # TODO: human box size 맞는지 체크
        else:
            human_box_size = 0
        # print(f"human box size thres : {self.image_size * human_box_thres}")
        # print(f"real human box size : {human_box_size}")
        _num_rotate=0
        _depth = self.barrier_check()
        escape_radius = 0.2
        rot_dir_left = 1
        _depth_value_except_0 = _depth[_depth != 0]
        rospy.loginfo(f"rect depth mean : {np.mean(_depth)}")
        # rospy.loginfo(f"rect depth excetp 0 min : {_depth_value_except_0.min}")
        rospy.loginfo(f"calc_z  : {calc_z / 1000.0}")
        #and np.mean(_depth)< (calc_z-100)
        #and self.image_size * human_box_thres > human_box_size
        if (np.mean(_depth) < thres and calc_z!=0 and (np.mean(_depth)< ((calc_z/ 1000.0)-0.2) or self.agent.dist <((calc_z/1000.0)-0.2) )and not (self.start_location[0] - escape_radius < cur_pose[0] < self.start_location[0] + escape_radius and \
        self.start_location[1] - escape_radius < cur_pose[1] < self.start_location[1] + escape_radius)):
            _num_rotate = _num_rotate + 1
            rospy.sleep(1)
            print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!BARRIER!!!!!!!!!!!!!!!!!")
            self.agent.say('Barrier checking....', show_display=True)
            print("Barrier checking....")
            # self.agent.pose.head_pan_tilt(0, -self.tilt_angle)

            rospy.sleep(1)

            _depth = self.barrier_check()
            rospy.loginfo(f"rect depth : {np.mean(_depth)}")

            while (np.mean(_depth)< thres-0.1):

            ##########################BRANCH1. ROTATING########################    
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
                

            ##########################BRANCH2. MOVING########################    
                _depth = self.barrier_check()
                print(f"mean depth: {np.mean(_depth)}")
                self.agent.move_rel(0,-0.3,0, wait=False) ## TODO : go right
                # self.agent.move_rel(0,0.3,0, wait=False) ## TODO : go left
                rospy.sleep(1)


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
                    self.agent.say('Tiny object.', show_display=False)
                    break

        if tiny_object_exist:
            self.agent.say('I\'ll avoid it.', show_display=False)
            self.agent.move_rel(0,-0.3,0, wait=False) ## TODO : go right
            rospy.sleep(1)


    def stt_destination(self, stt_option, calc_z=0):
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
            self.escape_tiny()

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
            print("lets go to the last human position")
            if self.last_human_pos is not None:
                target_xyyaw = self.last_human_pos
                self.agent.move_rel(target_xyyaw[0], target_xyyaw[1], 0, wait=False)
            return False
        human_info_ary = copy.deepcopy(self.human_box_list)
        depth = np.asarray(self.d2pc.depth)
        twist, calc_z = self.human_reid_and_follower.follow(human_info_ary, depth, self.human_seg_pos)

        if calc_z > 1000:
            calc_z = calc_z + 500 #TODO : calc_z 과장할 정도 결정

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
            if twist.linear.x == 0 and twist.angular.z == 0:
                # change angular.z
                loc = self.check_human_pos(human_info_ary, location=True)
                if loc == 'lll':
                    print("left")
                    # twist.angular.z = -self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, self.stop_rotate_velocity, wait=True)
                    rospy.sleep(.5)
                if loc == 'll':
                    print("left")
                    # twist.angular.z = -self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, self.stop_rotate_velocity/2, wait=False)
                    rospy.sleep(.5)
                if loc == 'l':
                    print("left")
                    # twist.angular.z = -self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, self.stop_rotate_velocity/4, wait=False)
                    rospy.sleep(.5)
                if loc == 'r':
                    print("right")
                    # twist.angular.z = +self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, -self.stop_rotate_velocity/4, wait=False)
                    rospy.sleep(.5)
                if loc == 'rr':
                    print("right")
                    # twist.angular.z = +self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, -self.stop_rotate_velocity/2, wait=False)
                    rospy.sleep(.5)
                if loc == 'rrr':
                    print("right")
                    # twist.angular.z = +self.stop_rotate_velocity
                    self.agent.move_rel(0, 0, -self.stop_rotate_velocity, wait=True)
                    rospy.sleep(.5)

                if self.stt_destination(self.stt_option, calc_z):
                    return True

                return False

            target_xyyaw = self.calculate_twist_to_human(twist, calc_z)
            self.marker_maker.pub_marker([target_xyyaw[0], target_xyyaw[1], 1], 'base_link')
            # 2.4 move to human
            self.last_human_pos = target_xyyaw
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
            if self.stt_destination(self.stt_option, calc_z):
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
        self.yolo_bag_list = [6, 7, 8]
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
        self.bag_yolo_data = self.get_bag_by_x(data.data)
    
    def get_bag_by_x(self, arr):
        if arr is None:
            return None
        
        grouped_data = [arr[i:i+6] for i in range(0, len(arr), 6)]

        filtered_data = [group for group in grouped_data if (group[5] >= .75 and group[4] in self.yolo_bag_list)]

        sorted_data = sorted(filtered_data, key = lambda x: x[0])

        sorted_arr = [item for sublist in sorted_data for item in sublist]
        return sorted_arr


    def detect_bag_3d(self, bag_height, timeout=5.0, pointing_dir = 0):
        start_time = time.time()
        while time.time() - start_time <= timeout:
            _pc = self.agent.pc.reshape(480, 640)
            pc_np = np.array(_pc.tolist())[:, :, :3]
            bag_yolo_data = self.bag_yolo_data


            # self.agent.head_display_image_pubish(self.yolo_img)
            for idx in range(len(bag_yolo_data) // 6):
                
                item = bag_yolo_data[6 * idx: 6 * (idx + 1)]
                cent_x, cent_y, width, height, class_id, conf_percent = item
                
                # if class_id is not paper_bag
                #if class_id not in self.yolo_bag_list and conf_percent < .75:
                #    continue
                if idx == 0 and pointing_dir == -1:
                    continue

                '''
                BAG PICKING 0410
                '''
                if class_id == 6:
                    bag_height = 0.39
                elif class_id == 7:
                    bag_height = 0.33
                else:
                    bag_height = 0.285

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
            print("current position: ", self.agent.get_pose())
            self.marker_maker.pub_marker([mov_x, mov_y, 1], 'base_link')
            mov_x += 0.05
            self.agent.move_rel(mov_x, mov_y, wait=True)
            print("moved position: ", self.agent.get_pose())

            if object_size[1] >= object_size[2]:  # y >= z
                bag_orientation = -1.57
            else:
                bag_orientation = 0

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
        #target_base_xyz, object_size, bag_height = self.detect_bag_3d(bag_height, timeout=1)

        #0도일때
        for tilt_idx in range(2):
            if tilt_idx == 0:
                self.agent.pose.head_pan_tilt(0, -40)
            else:    
                self.agent.pose.head_pan_tilt(0, -30)
            print("Searching")
            rospy.sleep(2)
            target_base_xyz, object_size, bag_height = self.detect_bag_3d(bag_height, timeout=1, pointing_dir=direction)
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
    goal_radius = 0.5
    pose_save_time_period = 7
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
                                                   stop_thres=.4,
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
        yolo_process = subprocess.Popen(['bash', script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)


        ####################
        # agent.say("start yolo")
        rospy.sleep(3)
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
                while time.time() - start_searching_time <=7:  # check bag during 5 seconds
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
            #####    
    except Exception as e:
        print("error starting yolo")
        yolo_success = False
    # finally:
    #     yolo_process.terminate()

    if not yolo_success or not try_bag_picking:
       # no try bag picking
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


    # yolo_process.terminate()

    ######################
    # 2. human following
    byte_path = "/home/tidy/Robocup2024/byte.sh"
    seg_path = "/home/tidy/Robocup2024/seg.sh"
    byte_process = subprocess.Popen(['bash', byte_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
    seg_process = subprocess.Popen(['bash', seg_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
    demotrack_pub.publish(String('target'))
    agent.pose.head_pan_tilt(0, 0)
    agent.say("If you are arrived at the destination", show_display=True)
    rospy.sleep(3)
    agent.say("Please stand still", show_display=True)
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

        for i in range(len(track_queue)):
        # len(track_queue):
            cur_track = track_queue[len(track_queue)-i-1]
            # coordinate = track_queue.pop()
            while not agent.move_abs_coordinate_safe(cur_track):
                print("retry")
            calc_z= 2000

            human_info_ary = copy.deepcopy(human_following.human_box_list)
            depth = np.asarray(human_following.d2pc.depth)
            if human_info_ary[0] is not None:
                twist, calc_z = human_following.human_reid_and_follower.follow(human_info_ary, depth, human_following.human_seg_pos)
            human_following.escape_barrier(calc_z)
            human_following.escape_tiny()
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

    byte_process.terminate()
    seg_process.terminate()
    yolo_process.terminate()

    agent.say("Finish carry my luggage", show_display=True)

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

