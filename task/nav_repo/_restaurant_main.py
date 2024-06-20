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

import math

class Restaurant:
    def __init__(self, agent):
        #####jnpahk lidar callback#####
        self.agent = agent
        self.dist_thres = dist_thres
        self.unit_angle = unit_angle
        self.center_index = center_index
        self.H = 480
        self.W = 640
        self.twist = Twist()
        self.axis_transform = Axis_transform()
        self.agent = Agent()
        self.d2pc = Depth2PC() 
        
        self.human_yaw = None

        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.byte_sub = rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_callback)
        rest_clinet = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        rest_clinet.call(EmptyRequest())

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = max_lidar_range  # remove nans
        dist = data_np
        self.pathpoint = np.where(dist > self.dist_thres)[0].tolist()
        self.path_list = self.find_angles()

    def _human_yolo_callback(self, data):
        # ByteTrack
        data_list = data.data
        
        if data_list[0] == -1:
            self.human_box_list = [None]
            self.human_yaw = None
            
        else:
            self.human_box_list = [data_list[0], #[human_id, target_tlwh, target_score]
                                  np.asarray([data_list[1], data_list[2], data_list[3], data_list[4]], dtype=np.int64),
                                  data_list[5]]

            x = data_list[1]
            y = data_list[2]
            w = data_list[3]
            h = data_list[4]
            self.center = [y + int(h/2), x + int(w/2)]

            human_rad_in_cam = (self.center[1] - self.W / 2) / 640
            self.human_yaw = human_rad_in_cam + self.get_head_pan()

            print("self.human_yaw: ", self.human_yaw)
            self.head_pan(self.human_yaw)

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
        
    def head_pan(self, head_pan_rad):
        head_pan_deg = math.degrees(head_pan_rad)
        self.agent.pose.head_pan(head_pan_deg)
        
    def get_head_pan(self):
        ### TODO ###
        return self.agent.pose.joint_value
        
    def index_to_angle(self, idx):
        return (idx - self.center_index) * self.unit_angle

    def heuristic(self, start, end):
        avg_angle = (start + end) / 2
        
        if self.human_yaw:
            return -abs(avg_angle - self.human_yaw)
        
        else:
            ### TODO: heuristic function when human_yaw is None ###
            return 0

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

    


    