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

def play_ding(src = "./ding.mp3") :
    playsound(src)

parser = argparse.ArgumentParser(description='tidyboy capture')
parser.add_argument('--dir', type=str, default='image/',
                    help='saved directory path')
args = parser.parse_args()

class VisionController(object):
    def __init__(self, start_idx, loc = "init"):
        self.rgb_image = None
        self.bridge = CvBridge()
        self.start_idx = start_idx
        self.xbox_request = False
        # for realsense-ros
        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        if loc == "init":
            self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        elif loc=="xbox":
            self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback_xbox)

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

    def make_xbox_request(self):
        self.xbox_request=True
    def _rgb_callback_xbox(self, data):
        rgb_image = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1),
                                 cv2.COLOR_RGB2BGR)

        if self.xbox_request:  # 's' or Enter
            print('saved', self.start_idx)
            cv2.imshow('rgb image', rgb_image)
            cv2.waitKey(1)
            cv2.imwrite('image/' + str(self.start_idx) + '.png', rgb_image)
            self.start_idx += 1
            sound = threading.Thread(target=play_ding,args=('../Tools/xtioncam_capture/ding.mp3',))
            sound.start()
            self.xbox_request=False


if __name__ == '__main__':
    rospy.init_node('hsr_capture_baseline', anonymous=True)
    start_idx = int(input('start_idx : '))
    vision_controller = VisionController(start_idx)
    rospy.spin()