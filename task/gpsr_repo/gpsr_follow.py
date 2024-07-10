import rospy
from sensor_msgs.msg import LaserScan

Class GPSRFollow:
    def __init__(self, agent):
        self.agent = agent

        self.human_rad = None
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)

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
        
        move_x = move_dist * math.cos(avg_rad)
        move_y = move_dist * math.sin(avg_rad)
        if self.human_rad:
            move_yaw = self.human_rad
        else:
            move_yaw = 0
        self.move_rel(move_x, move_y, yaw=move_yaw)