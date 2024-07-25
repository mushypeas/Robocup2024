import argparse
import cv2
import os
import shutil
import random
import time
from playsound import playsound
import threading
import rospy
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from subprocess import call

# DEFAULT_PATH = 'image/'
DEFAULT_PATH = '../BoundingBox_Tool/Images/Test/'

def play_ding():
    playsound("./ding.mp3")

parser = argparse.ArgumentParser(description='tidyboy capture')
parser.add_argument('--dir', type=str, default=DEFAULT_PATH, help='saved directory path')
parser.add_argument('--start_idx', type=int, required=True, help='start index for image naming')
parser.add_argument('--input_classes', type=str, required=True, help='comma-separated list of input classes')
args = parser.parse_args()

if not os.path.exists(args.dir):
    print(f"Directory '{args.dir}' not found. creating...")
    os.makedirs(args.dir)

class VisionController(object):
    def __init__(self, start_idx, input_classes):
        self.rgb_image = None
        self.bridge = CvBridge()
        self.start_idx = start_idx
        self.input_classes = input_classes

        # for realsense-ros
        rgb_topic = '/camera/rgb/image_rect_color'
        self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)

    def _rgb_callback(self, data):
        rgb_image = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        cv2.imshow('rgb image with **ULTIMATE AUTOLABEL**', rgb_image)
        key = cv2.waitKey(1)
        if key == 115 or key == 13:  # 's' or Enter
            print('saved', self.start_idx)
            img_filename = os.path.join(args.dir, str(self.start_idx) + '.png')
            cv2.imwrite(img_filename, rgb_image)
            self.start_idx += 1
            sound = threading.Thread(target=play_ding)
            sound.start()

            # 잠시 대기하여 파일이 저장될 시간을 줍니다.
            time.sleep(1)

            # Call the autolabel script within the conda environment for the new image only
            call(['conda', 'run', '-n', 'yolov10', 'python', '../autolabel_v10/autolabel_ultimate_v2.py', '--input-classes', self.input_classes, '--source', img_filename])

        if key == 100:  # 'd'
            self.start_idx -= 1
            print('deleted', self.start_idx)
            os.remove(os.path.join(args.dir, str(self.start_idx) + '.png'))

if __name__ == '__main__':
    rospy.init_node('openni2_capture_baseline', anonymous=True)
    vision_controller = VisionController(args.start_idx, args.input_classes)
    rospy.spin()
