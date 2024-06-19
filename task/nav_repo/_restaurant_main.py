import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

class Restaurant:
    def __init__(self, dist_thres, angular_range):
        #####jnpahk lidar callback#####
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.dist_thres = dist_thres
        self.angular_range = angular_range

    def find_segments(lst):
        if not lst:
            return []

        segments = []
        start = lst[0]
        end = lst[0]

        for i in range(1, len(lst)):
            if lst[i] == lst[i - 1] + 1:
                end = lst[i]
            else:
                segments.append(start)
                segments.append(end)
                start = lst[i]
                end = lst[i]

        segments.append(start)
        segments.append(end)

        return segments

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 5.0  # remove nans
        self.ranges = data_np # jykim: save ranges data from LiDAR
        center_idx = data_np.shape[0] // 2
        ang_range = self.angular_range
        dist = np.mean(data_np[center_idx - ang_range: center_idx + ang_range])
        pathpoint = [dist > self.dist_thres]
        self.path_list = self.find_segments(pathpoint)

def restaurant(agent):
    pass