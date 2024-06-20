import sys
sys.path.append('task/nav_repo')

import time
import os
import numpy as np
import math

import rospy
import roslaunch
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyRequest

from restaurant_config import *
from utils.marker_maker import MarkerMaker
from move_base_restaurant import MoveBaseRestaurant

class Restaurant:
    def __init__(self, agent):
        self.agent = agent
        self.move = MoveBaseRestaurant()
        
        self.human_rad = None

        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.byte_sub = rospy.Subscriber('/snu/carry_my_luggage_yolo', Int16MultiArray, self._human_yolo_callback)
        rest_clinet = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
        rest_clinet.call(EmptyRequest())

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = max_dist  # remove nans
        self.dists = data_np
        self.indices_in_range = np.where(self.dists > min_dist)[0].tolist()
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
            human_rad_in_cam = calculate_human_rad(human_center_x, yolo_img_width)
            
            self.human_rad = human_rad_in_cam
            print("self.human_rad: ",self.human_rad)

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
        
                if (end_rad - start_rad) * min_dist > min_interval_arc_len:
                    segments.append([start_rad, end_rad, min_dist])

                start_idx = indices_in_range[i]
                end_idx = indices_in_range[i]

        start_rad = index_to_rad(start_idx)
        end_rad = index_to_rad(end_idx)
        min_dist = min(self.dists[start_idx:end_idx + 1])

        if (end_rad - start_rad) * min_dist > min_interval_arc_len:
            segments.append([start_rad, end_rad, min_dist])

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
        
        move_x = move_dist * math.cos(avg_rad)
        move_y = move_dist * math.sin(avg_rad)
        if self.human_rad:
            move_yaw = self.human_rad
        else:
            move_yaw = 0
        self.move_rel(move_x, move_y, yaw=move_yaw)

def restaurant(agent):
    rospy.loginfo('Initialize Hector SLAM')
    # Kill existing nodes and replace them with others
    os.system('rosnode kill /pose_integrator')
    os.system('rosnode kill /move_base')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    slam_args = ['tidyboy_nav_stack', 'hector.launch']
    nav_args  = ['tidyboy_nav_stack', 'nav_stack.launch']
    slam_launch_file = roslaunch.rlutil.resolve_launch_arguments(slam_args)
    nav_launch_file  = roslaunch.rlutil.resolve_launch_arguments(nav_args)
    slam = roslaunch.parent.ROSLaunchParent(uuid,
                                            slam_launch_file,
                                            sigint_timeout=10.0,
                                            sigterm_timeout=5.0)
    nav  = roslaunch.parent.ROSLaunchParent(uuid,
                                            nav_launch_file,
                                            sigint_timeout=10.0,
                                            sigterm_timeout=5.0)
    slam.start()
    nav.start()
    rospy.sleep(3.)
    
    agent.say('start restaurant')
    marker_maker = MarkerMaker('/snu/robot_path_visu')
    
    ############################
    
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
            r.move_rel(0, 0, yaw=math.pi)
            moved_time = time.time()
            continue
            ## TODO ##
            ### If there is no candidate, what should we do? ###
            ### go back, turn around, etc... ###

        best_interval = r.get_best_candidate(candidates)

        while not best_interval:
            best_interval = r.get_best_candidate(candidates)
        
        r.move_best_interval(best_interval)

        moved_time = time.time()

    


    