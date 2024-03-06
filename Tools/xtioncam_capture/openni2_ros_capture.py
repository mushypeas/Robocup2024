# ros_launch #
# roslaunch openni2_launch openni2.launch
import argparse
import cv2
from playsound import playsound
import threading
import rospy
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, PointCloud2

def play_ding() :
    playsound("./ding.mp3")

parser = argparse.ArgumentParser(description='tidyboy capture')
parser.add_argument('--dir', type=str, default='image/',
                    help='saved directory path')
args = parser.parse_args()

class VisionController(object):
    def __init__(self, start_idx):
        self.rgb_image = None
        self.bridge = CvBridge()
        self.start_idx = start_idx

        # for realsense-ros
        rgb_topic = '/camera/rgb/image_rect_color'
        self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)


    def _rgb_callback(self,data):
        rgb_image = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        cv2.imshow('rgb image', rgb_image)
        key = cv2.waitKey(1)
        if key == 115 or key == 13:  # 's' or Enter
            print('saved', self.start_idx)
            cv2.imwrite(args.dir + str(self.start_idx) + '.png', rgb_image)
            self.start_idx += 1
            sound = threading.Thread(target=play_ding)
            sound.start()


if __name__ == '__main__':
    rospy.init_node('openni2_capture_baseline', anonymous=True)
    start_idx = int(input('start_idx : '))
    vision_controller = VisionController(start_idx)
    rospy.spin()