import sys
sys.path.append('task/nav_repo')
from restaurant_config import *

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import math


from sensor_msgs.msg import LaserScan

class Restaurant:
    def __init__(self, agent):
        #####jnpahk lidar callback#####
        self.agent = agent
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.dist_thres = dist_thres
        self.unit_angle = 0.25 * ( math.pi / 180 )
        self.center_index = 481

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 5.0  # remove nans
        dist = data_np
        pathpoint = np.where(dist < self.dist_thres)[0].tolist()
        self.path_list = self.find_segments(pathpoint)

    def _destination_callback(self, data):
        self.destination_angle = None
        ## TODO ##
        pass
        
    def index_to_angle(self, idx):
        return (idx - self.center_index) * self.unit_angle

    def find_angles(self):
        lst = self.path_list

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

                segments.append(start_angle)
                segments.append(end_angle)
                start_idx = lst[i]
                end_idx = lst[i]

        start_angle = self.index_to_angle(start_idx)
        end_angle = self.index_to_angle(end_idx)

        segments.append(start_angle)
        segments.append(end_angle)

        return segments

    ## HELP Function 
    def move_abs(self, x, y, yaw=0):
        self.agent.move_abs(x, y, yaw)

    def heuristic(self, start, end):
        avg_angle = (start + end) / 2
        return -abs(avg_angle - self.destination_angle)

    def find_best_move(self, candidates):
        heuristic_values = [self.heuristic(start, end) for start, end in candidates]
        best_idx = np.argmax(heuristic_values)
        return candidates[best_idx]
    
    def move_based_on_interval(self, interval):
        move_dis = 0.3
        avg_angle = (interval[0] + interval[1]) / 2
        move_x = move_dis * math.cos(avg_angle)
        move_y = move_dis * math.sin(avg_angle)
        self.move_abs(move_x, move_y)

def restaurant(agent):
    r = Restaurant()

    while not rospy.is_shutdown():
        candidates = r.find_angles()

        if not candidates:
            pass
            ## TODO ##

        best_interval = r.find_best_move(candidates)

        r.move_based_on_interval(best_interval)


    