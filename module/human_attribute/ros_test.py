import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ROS_sub():
    def __init__(self,
                 rgb_topic='/camera/rgb/image_raw',
                 depth_topic='/camera/depth/image_raw',
                 threshold_dist=0.5):
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback)

        self.threshold_dist = threshold_dist
        self.dist = None
        self.rgb_img = None
        self.depth_img = None
        self.bridge = CvBridge()



    def _rgb_callback(self, data):
        # rospy.loginfo('image_sub node started')
        self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        cv2.putText(self.rgb_img, "Door Closed.".format(self.dist), \
                    (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, \
                    (0, 255, 0), 3)
        cv2.imshow("rgb_img", self.rgb_img)
        cv2.waitKey(1)

    def _depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")



    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 0.0  # remove nans
        num_angles = data_np.shape # resolution of the lidar
        _width = 15 # ranges that you'll search
        center_idx = data_np.shape[0] // 2
        # print(num_angles)


if __name__ == '__main__':
    rospy.init_node('door_detection')
    ls = ROS_sub()
    rospy.spin()