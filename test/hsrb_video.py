import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image


class Hsrb_video:
    def __init__(self):
        rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.cb)
        self.rgb_img = None
        self.cnt = 0

    def cb(self, data):
        self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        cv2.imshow('dd', self.rgb_img)
        key = cv2.waitKey(1)
        if key == 32:
            cv2.imwrite(f'/home/tidy/bag_yolo_dataset/{self.cnt}.png', self.rgb_img)
            rospy.loginfo("SAVED!")
            self.cnt += 1
if __name__ == '__main__':
    rospy.init_node('test_video')
    hv = Hsrb_video()
    rospy.spin()
