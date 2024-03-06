import rospy
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid

def cb(data):

    W = data.info.width
    H = data.info.height
    grid = np.array(data.data).reshape((W, H))
    grid = np.where(grid >= 80, 255, 0).astype(np.uint8)
    grid = np.flip(np.flip(grid, 0), 1).T
    cv2.imshow('grid', grid)
    cv2.waitKey(10)



if __name__ == '__main__':
    
    rospy.init_node('map_test', disable_signals=True)
    rospy.Subscriber('dynamic_obstacle_map', OccupancyGrid, cb)
    rospy.spin()
