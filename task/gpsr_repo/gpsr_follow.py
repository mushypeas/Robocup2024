import rospy
from sensor_msgs.msg import LaserScan

import math
import numpy as np


## main frequency
main_freq = 1
main_period = 1.0 / main_freq

## LiDAR dist
min_dist = 1.0
max_dist = 5.0

## LiDAR index
lidar_index = 963
center_index = lidar_index // 2

## YOLO img size
yolo_img_height = 480
yolo_img_width = 640

min_interval_arc_len = 1.0
unit_rad = 0.23 * ( math.pi / 180 )
avg_dist_move_dist_ratio = 5

def calculate_human_rad(human_center_x, yolo_img_width):
    human_center_bias = human_center_x - yolo_img_width / 2
    return -human_center_bias / 1500

def index_to_rad(idx):
    return (idx - center_index) * unit_rad


class GPSRFollow:
    def __init__(self, agent, gpsr):
        self.agent = agent
        self.gpsr = gpsr

        self.human_rad = None
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.kpts_sub = rospy.Subscriber('human_pose_and_bbox', Float32MultiArray, self.hkpts_cb)


    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = max_dist  # remove nans
        self.dists = data_np
        self.indices_in_range = np.where(self.dists > min_dist)[0].tolist()
        self.candidates = self.find_candidates()

    def _hkpts_cb(self, data):
        human_keypoints = data.data
        num_features = 55
        split_kpts = [human_keypoints[i:i + num_features] for i in range(0, len(human_keypoints), num_features)]

        if not split_kpts:
            self.last_human_rad = self.human_rad
            self.human_rad = None
            return

        max_area = 20000
        max_idx = -1

        for idx, kpts in enumerate(split_kpts):
            human_w = kpts[2] - kpts[0]
            human_h = kpts[3] - kpts[1]

            if human_w * human_h > max_area:
                max_area = human_w * human_h
                max_idx = idx

        if max_idx == -1:
            self.last_human_rad = self.human_rad
            self.human_rad = None
            return

        human_center_x = split_kpts[max_idx][0] + split_kpts[max_idx][2] // 2

        self.human_rad = calculate_human_rad(human_center_x, yolo_img_width)

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
                start_rad = index_to_rad(start_idx)
                end_rad = index_to_rad(end_idx)
                min_dist = min(self.dists[start_idx:end_idx + 1])
        
                interval_arc_len = (end_rad - start_rad) * min_dist

                if interval_arc_len > min_interval_arc_len:
                    number_seg = int(interval_arc_len / min_interval_arc_len)
                    idx_len = end_idx - start_idx

                    for j in range(number_seg):
                        new_start_idx = start_idx + int(idx_len * j / number_seg)
                        new_end_idx = start_idx + int(idx_len * (j + 1) / number_seg)
                        new_min_dist = min(self.dists[new_start_idx:new_end_idx + 1])
                        new_start_rad = index_to_rad(new_start_idx)
                        new_end_rad = index_to_rad(new_end_idx)

                        segments.append([new_start_rad, new_end_rad, new_min_dist])

                start_idx = indices_in_range[i]
                end_idx = indices_in_range[i]

        start_rad = index_to_rad(start_idx)
        end_rad = index_to_rad(end_idx)
        min_dist = min(self.dists[start_idx:end_idx + 1])

        interval_arc_len = (end_rad - start_rad) * min_dist

        if interval_arc_len > min_interval_arc_len:
            number_seg = int(interval_arc_len / min_interval_arc_len)
            idx_len = end_idx - start_idx

            segments.append([start_rad, end_rad, min_dist])

            for j in range(number_seg):
                new_start_idx = start_idx + int(idx_len * j / number_seg)
                new_end_idx = start_idx + int(idx_len * (j + 1) / number_seg)
                new_min_dist = min(self.dists[new_start_idx:new_end_idx + 1])
                
                new_start_rad = index_to_rad(new_start_idx)
                new_end_rad = index_to_rad(new_end_idx)

                segments.append([new_start_rad, new_end_rad, new_min_dist])

        return segments

    ## HELP Function 
    def move_rel(self, x, y, yaw=0):
        self.agent.move_rel(x, y, yaw=yaw)

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

        if self.

        if self.human_rad and self.human_rad > start_rad and self.human_rad < end_rad:
            rospy.loginfo("Human is in the interval")
            move_x = move_dist * math.cos(self.human_rad)
            move_y = move_dist * math.sin(self.human_rad)
            move_yaw = self.human_rad

        elif self.human_rad:
            rospy.loginfo("Human is not in the interval")
            move_x = move_dist * math.cos(avg_rad)
            move_y = move_dist * math.sin(avg_rad)
            move_yaw = avg_rad

        else:
            rospy.logwarn("Human not detected")
            move_x = 0
            move_y = 0
            if self.last_human_rad:
                move_yaw = self.last_human_rad
                self.last_human_rad = None
            else:
                rospy.logwarn("No last human rad")
                move_yaw = 0
        
        self.move_rel(move_x, move_y, yaw=move_yaw)