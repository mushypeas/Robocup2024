import argparse
import cv2
import os
from playsound import playsound
import threading
import rospy
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, PointCloud2

DEFAULT_PATH = 'image/'

def play_ding():
    playsound("./ding.mp3")

parser = argparse.ArgumentParser(description='tidyboy capture')
parser.add_argument('--dir', type=str, default=DEFAULT_PATH,
                    help='saved directory path')
parser.add_argument('--start_idx', type=int, required=True,
                    help='start index for image naming')
args = parser.parse_args()

if not os.path.exists(args.dir):
    print(f"Directory '{args.dir}' not found. creating...")
    os.makedirs(args.dir)

class VisionController(object):
    def __init__(self, start_idx):
        self.rgb_image = None
        self.bridge = CvBridge()
        self.start_idx = start_idx

        # for realsense-ros
        rgb_topic = '/camera/rgb/image_rect_color'
        self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)

    def _rgb_callback(self, data):
        rgb_image = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        cv2.imshow('rgb image', rgb_image)
        key = cv2.waitKey(1)
        if key == 115 or key == 13:  # 's' or Enter
            print('saved', self.start_idx)
            cv2.imwrite(args.dir + str(self.start_idx) + '.png', rgb_image)
            self.start_idx += 1
            sound = threading.Thread(target=play_ding)
            sound.start()

        if key == 100:  # 'd'
            self.start_idx -= 1
            print('deleted', self.start_idx)
            os.remove(args.dir + str(self.start_idx) + '.png')


if __name__ == '__main__':
    rospy.init_node('openni2_capture_baseline', anonymous=True)
    vision_controller = VisionController(args.start_idx)
    rospy.spin()
