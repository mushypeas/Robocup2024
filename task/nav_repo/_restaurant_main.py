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
from std_srvs.srv import Empty, EmptyRequest

import math
import time

class Restaurant:
    def __init__(self, agent):
        self.agent = agent
        self.twist = Twist()
        self.axis_transform = Axis_transform()
        self.d2pc = Depth2PC() 
        
        self.min_dist = min_dist
        self.unit_rad = unit_rad
        self.center_index = center_index
        self.yolo_img_height = yolo_img_height
        self.yolo_img_width = yolo_img_width
        
        self.human_rad = None

        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.byte_sub = rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_callback)
        rest_clinet = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        rest_clinet.call(EmptyRequest())

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = max_dist  # remove nans
        self.dists = data_np
        self.indices_in_range = np.where(self.dists > self.min_dist)[0].tolist()
        self.candidates = self.find_candidates()

    def _human_yolo_callback(self, data):
        yolo_data = data.data
        
        if yolo_data[0] == -1:
            self.human_box_list = [None]
            self.human_rad = None
            
        else:
            human_id = yolo_data[0]
            human_x = yolo_data[1]
            human_y = yolo_data[2]
            human_w = yolo_data[3]
            human_h = yolo_data[4]
            target_score = yolo_data[5]
            
            self.human_box_list = [human_id, np.asarray([human_x, human_y, human_w, human_h], dtype=np.int64), target_score]
            
            human_center_x = human_x + human_w // 2
            human_rad_in_cam = calculate_human_rad(human_center_x, self.yolo_img_width)
            print("self.human_rad_in_cam: ", human_rad_in_cam)
            
            self.human_rad = human_rad_in_cam + self.get_head_pan()
            print("self.human_rad: ",self.human_rad)
            self.head_pan(self.human_rad)

    def find_candidates(self):
        indices_in_range = self.indices_in_range

        if not indices_in_range:
            return []

        segments = []
        start_idx = indices_in_range[0]
        end_idx = indices_in_range[0]

        for i in range(1, len(indices_in_range)):
            if indices_in_range[i] == indices_in_range[i - 1] + 1:
                end_idx = indices_in_range[i]
                
            else:
                start_rad = self.index_to_rad(start_idx)
                end_rad = self.index_to_rad(end_idx)
                avg_dist = (self.dists[start_idx] + self.dists[end_idx]) / 2
        
                if (end_rad - start_rad) * avg_dist > min_interval_arc_len:
                    segments.append([start_rad, end_rad, avg_dist])

                start_idx = indices_in_range[i]
                end_idx = indices_in_range[i]

        start_rad = self.index_to_rad(start_idx)
        end_rad = self.index_to_rad(end_idx)
        avg_dist = (self.dists[start_idx] + self.dists[end_idx]) / 2

        if (end_rad - start_rad) * avg_dist > min_interval_arc_len:
            segments.append([start_rad, end_rad, avg_dist])

        return segments

    ## HELP Function 
    def move_rel(self, x, y, yaw=0):
        self.agent.move_rel(x, y, yaw=yaw)
        
    def head_pan(self, head_pan_rad):
        head_pan_deg = math.degrees(head_pan_rad)
        self.agent.pose.head_pan(head_pan_deg)
        
    def get_head_pan(self):
        return self.agent.pose.joint_value['head_pan_joint']
        
    def index_to_rad(self, idx):
        return (idx - self.center_index) * self.unit_rad

    def heuristic(self, start_rad, end_rad, avg_dist):
        avg_rad = (start_rad + end_rad) / 2
        
        if self.human_rad:
            return -abs(avg_rad - self.human_rad)
        
        else:
            ### TODO: heuristic function when human_yaw is None ###
            ### Momentum + past human_yaw ###
            return 0

    def get_best_candidate(self, candidates):
        heuristic_values = [self.heuristic(start, end, avg_dist) for start, end, avg_dist in candidates]
        if not heuristic_values:
            return None
        best_idx = np.argmax(heuristic_values)
        return candidates[best_idx]
    
    def move_best_interval(self, interval):
        start_rad = interval[0]
        end_rad = interval[1]
        avg_dist = interval[2]
        
        move_dist = avg_dist / avg_dist_move_dist_ratio
        avg_rad = (start_rad + end_rad) / 2
        
        move_x = move_dist * math.cos(avg_rad)
        move_y = move_dist * math.sin(avg_rad)
        move_yaw = avg_rad
        self.move_rel(move_x, move_y, yaw=move_yaw)

def restaurant(agent):
    r = Restaurant(agent)
    moved_time = time.time()

    while not rospy.is_shutdown():
        if time.time() - moved_time < main_period:
            continue

        try:
            candidates = r.candidates
        except AttributeError:
            continue

        if not candidates:
            continue
            ## TODO ##
            ### If there is no candidate, what should we do? ###
            ### go back, turn around, etc... ###

        best_interval = r.get_best_candidate(candidates)

        while not best_interval:
            best_interval = r.get_best_candidate(candidates)

        print("best_interval", best_interval)
        
        r.move_best_interval(best_interval)

        moved_time = time.time()

    


    