import rospy
import numpy as np
from geometry_msgs.msg import Twist


from sensor_msgs.msg import LaserScan

class Restaurant:
<<<<<<< HEAD
    def __init__(self, dist_thres, angular_range):
=======
    def __init__(self, dist_thres, angular_range, linear_max=0.8, angular_max=0.5):

>>>>>>> eb7316f (240619)
        #####jnpahk lidar callback#####
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.dist_thres = dist_thres
        self.angular_range = angular_range
        self.twist = Twist()
        self.linear_max = linear_max
        self.angular_mx = angular_max

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


        
    def new_get_controls(x,z):
        linear_max = self.linear_max
        angular_max = self.angular_max
        # linear = (z/1000-0.6)/3
        # linear = z/1000*0.4
        angular = (-1/500) * (x-320)
        # print('linear: {} ,angular: {}  \n'.format(linear,angular))
        linear = z / 1000 * .4

        # if linear > 0.8:
        # 	linear *= 0.8
        # else:
        # 	linear *= 0.4
        if linear > linear_max:
            linear = linear_max

        if angular > angular_max:
            angular = angular_max

        if linear < -0:
            linear = -0

        if angular < -angular_max:
            angular = -angular_max

        twist.linear.x = linear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angular

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 5.0  # remove nans
        self.ranges = data_np # jykim: save ranges data from LiDAR
        center_idx = data_np.shape[0] // 2
        ang_range = self.angular_range
        dist = np.mean(data_np[center_idx - ang_range: center_idx + ang_range])
        pathpoint = [dist > self.dist_thres]
        self.path_list = self.find_segments(pathpoint)
<<<<<<< HEAD
=======

>>>>>>> eb7316f (240619)

def restaurant(agent):
    pass