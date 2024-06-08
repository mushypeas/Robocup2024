import rospy
from cv_bridge import CvBridge
import cv2

import subprocess
import time

import mediapipe as mp
import numpy as np
import sys
sys.path.append('.')
from utils.marker_maker import MarkerMaker
from utils.depth_to_pc import Depth2PC
from utils.axis_transform import Axis_transform

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from sklearn.preprocessing import StandardScaler


class BagInspection:
    def __init__(self, agent):
        self.agent = agent
        self.pointing_dir = -1 # left : -1, right : 1
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

        self.bag_yolo_data = [6,7,8]
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
            if class_id == 6:
                bag_height = 0.39
                bag_breadth = 0.41
                bag_length = 0.22
            elif class_id == 7:
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
            for i in range(5, 0, -1):
                self.agent.say(str(i))
                rospy.sleep(1)
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
            #self.agent.move_abs_coordinate(before_pick_pose, wait=True)
        self.agent.say("I am ready to follow you", show_display=True)
        rospy.sleep(2)



def carry_my_luggage(agent):
    # task params
    bag_search_limit_time = 15
    bag_height = 0.25
    try_bag_picking = True
    yolo_success = True

    bag_inspection = BagInspection(agent)

    # 0. start
    agent.say('start carry my luggage!')
    agent.pose.move_pose()
    rospy.sleep(2)
    agent.pose.head_pan_tilt(0, 0)

    
    if try_bag_picking:
        # [Bonus] 1. bag detection
        print("1-1. Hello, Please pointing where the bag is")
        agent.say("Please point Where the bag is", show_display=True)
        #########
        # 1.1 check human pointing direction
        bag_searching_start_time = time.time()
        while time.time() - bag_searching_start_time <= bag_search_limit_time:
            start_searching_time = time.time()
            while time.time() - start_searching_time <= 5:  # check bag during 5 seconds
                visualized_image, yaw, tilt = bag_inspection.cal_yaw_tilt()
                if visualized_image is not None:
                    agent.head_display_image_pubish(visualized_image)
            # check human direction
            if yaw is None:
                print("1-2. Please point again")
                agent.say("Please point again", show_display=True)
            else:
                break
            ########
        print("1-3.pointing direction:", 'right' if bag_inspection.pointing_dir == -1 else 'left')
        # 1.3 check shopping bag
        agent.pose.head_pan_tilt(0, -30)
        bag_inspection.run_bag_inspection(yaw, tilt)
    
    else:
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