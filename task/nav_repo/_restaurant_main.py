import sys
sys.path.append('task/nav_repo')
from restaurant_config import *

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import math

from std_msgs.msg import Int16MultiArray

from utils.depth_to_pc import Depth2PC
from utils.axis_transform import Axis_transform
from sensor_msgs.msg import LaserScan
from module.person_following_bot.follow import HumanReidAndFollower
from hsr_agent.agent import Agent
from std_srvs.srv import Empty, EmptyRequest

class Restaurant:
    def __init__(self, agent):
        #####jnpahk lidar callback#####
        self.agent = agent
        self.dist_thres = dist_thres
        self.unit_angle = 0.25 * ( math.pi / 180 )
        self.center_index = 481
        self.H = 480
        self.W = 640
        self.twist = Twist()
        self.axis_transform = Axis_transform()
        self.agent = Agent()
        self.d2pc = Depth2PC() # to depth
        self.human_yaw = 0

        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.byte_sub = rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_callback)
        rest_clinet = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        rest_clinet.call(EmptyRequest())


    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 5.0  # remove nans
        dist = data_np
        self.pathpoint = np.where(dist > self.dist_thres)[0].tolist()
        self.path_list = self.find_angles()

    def _human_yolo_callback(self, data):

        data_list = data.data
        if data_list[0] == -1:
            self.human_box_list = [None]
        else:
            self.human_box_list = [data_list[0], #[human_id, target_tlwh, target_score]
                                  np.asarray([data_list[1], data_list[2], data_list[3], data_list[4]], dtype=np.int64),
                                  data_list[5]]

            x = data_list[1]
            y = data_list[2]
            w = data_list[3]
            h = data_list[4]
            self.center = [y + int(h/2), x + int(w/2)]
            print(self.center)
            head_yaw_60 = (self.center[1] - 320) * 60 / 640
            self.human_yaw = head_yaw_60 * math.pi / 180

            # depth = np.asarray(self.d2pc.depth)

            # twist, calc_z = self.follow(self.human_box_list, depth, self.center)
            # target_xyyaw = self.calculate_twist_to_human(twist, calc_z)
            
            # self.human_yaw = target_xyyaw[2]
            # human_yaw_60 = target_xyyaw[2] * 180 / math.pi
            print("self.human_yaw: ", head_yaw_60)
            self.agent.pose.head_pan(head_yaw_60)
            

    def follow(self, human_info_ary, depth_frame, seg_human_point): # yolo box: list of bbox , frame : img
        twist = Twist()
        human_id, target_tlwh, target_score = human_info_ary
        (self.x, self.y, self.w, self.h) = [int(v) for v in target_tlwh]
        if self.y < 0:
            self.y = 0
        if self.y >= self.H:
            self.y = self.H-1
        if self.x < 0:
            self.x = 0
        if self.x >= self.W:
            self.x = self.W-1

        # cv2.rectangle(frame, (x, y), (x + w, y + h),(0, 255, 0), 2)
        calc_z = 0.0
        # if self.tilt_angle < math.radians(10):
        cropped_y = max(min(self.y + self.h//4, self.H-1), 0)
        cropped_x = max(min(self.x + self.w//2, self.W-1), 0)
        # else:
        # cropped_y = max(min(self.y + self.h // 3, self.H - 1), 0)
        # cropped_x = max(min(self.x + self.w // 2, self.W - 1), 0)
        calc_x, calc_z = (self.x + self.w / 2), depth_frame[cropped_y, cropped_x]
        if seg_human_point is not None : 
            calc_z = depth_frame[seg_human_point[1], seg_human_point[0]]
            if calc_z == 0:
                calc_z = depth_frame[cropped_y, cropped_x]
        # calc_z *= np.cos(self.tilt_angle)
        self.calc_z_prev = calc_z
        # twist = get_controls(calc_x, calc_z, Kp_l=1/5, Ki_l=0, Kd_l=0.1, Kp_a=-1/500, Ki_a=0, Kd_a=0,
        # 					 linear_max=self.linear_max, angular_max=self.angular_max)
        twist = self.twist
        # twist = new_get_controls(calc_x,calc_z)
        angular = (-1/500) * (calc_x-320)
        # print('linear: {} ,angular: {}  \n'.format(linear,angular))
        linear = calc_z / 1000 * .4

        
        twist.linear.x = linear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angular

        return twist, calc_z
        

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



    def index_to_angle(self, idx):
        return (idx - self.center_index) * self.unit_angle

    def find_angles(self):
        lst = self.pathpoint

        if not lst:
            return []

        segments = []
        start_idx = lst[0]
        end_idx = lst[0]

        for i in range(1, len(lst)):
            if lst[i] == lst[i - 1] +1:
                end_idx = lst[i]
            else:

                start_angle = self.index_to_angle(start_idx)
                end_angle = self.index_to_angle(end_idx)
                
                if end_angle - start_angle > interval_min_angle:
                    segments.append([start_angle, end_angle])

                start_idx = lst[i]
                end_idx = lst[i]

        start_angle = self.index_to_angle(start_idx)
        end_angle = self.index_to_angle(end_idx)

        if end_angle - start_angle > interval_min_angle:
            segments.append([start_angle, end_angle])

        return segments

    ## HELP Function 
    def move_rel(self, x, y, yaw=0):
        self.agent.move_rel(x, y, yaw=yaw)

    def heuristic(self, start, end):
        avg_angle = (start + end) / 2
        return -abs(avg_angle - self.human_yaw)

    def find_best_move(self, candidates):
        heuristic_values = [self.heuristic(start, end) for start, end in candidates]
        if not heuristic_values:
            return None
        best_idx = np.argmax(heuristic_values)
        return candidates[best_idx]
    
    def move_based_on_interval(self, interval):
        move_dis = 0.3
        avg_angle = (interval[0] + interval[1]) / 2
        move_x = move_dis * math.cos(avg_angle)
        move_y = move_dis * math.sin(avg_angle)
        move_yaw = avg_angle
        self.move_rel(move_x, move_y, yaw=move_yaw)

def restaurant(agent):
    r = Restaurant(agent)

    while not rospy.is_shutdown():
        try:
            candidates = r.path_list
        except AttributeError:
            continue

        print("candidates: ", candidates)

        if not candidates:
            continue
            ## TODO ##

        best_interval = r.find_best_move(candidates)

        while not best_interval:
            best_interval = r.find_best_move(candidates)

        r.move_based_on_interval(best_interval)
        rospy.sleep(2.)

    


    