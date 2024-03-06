#!/usr/bin/env python3.5
# -*- coding: utf-8 -*-
import cv2
import rospy, os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

ROS_HZ = 4

BACKGROUND_HEIGHT = 600
BACKGROUND_WIDTH = 1024

IMAGE_HEIGHT = 600
IMAGE_WIDTH = 1024

BLACK = (0,0,0)
WHITE = (255,255,255)

IMAGE_START_HEIGHT = int((BACKGROUND_HEIGHT - IMAGE_HEIGHT)/2)
IMAGE_START_WIDTH = int((BACKGROUND_WIDTH - IMAGE_WIDTH)/2)
IMAGE_WIDTH_MARGIN = 30
TEXT_START_X = IMAGE_WIDTH + IMAGE_WIDTH_MARGIN
TEXT_START_Y = 50
TEXT_DISTANCE_Y = 70
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 2
thickness = 3

bridge = CvBridge()

class HeadDisplayController:
    def __init__(self):
        rospy.Subscriber('/hsr_head_img', Image, self.image_callback)
        rospy.Subscriber('/hsr_head_msg', String, self.string_callback)
        rospy.Subscriber('/hsr_head_file', String, self.filename_callback)
        self.camera_img = None
        self.text_splited_by_line = []
        self.full_screen = np.zeros((BACKGROUND_HEIGHT, BACKGROUND_WIDTH,3), dtype=np.uint8)
        self.show_image = True

    def image_callback(self, ros_data):
        camera_img = bridge.imgmsg_to_cv2(ros_data, desired_encoding='passthrough')
        camera_img = cv2.resize(camera_img, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.full_screen[:,:] = BLACK
        self.full_screen[IMAGE_START_HEIGHT:IMAGE_START_HEIGHT+IMAGE_HEIGHT, \
                        IMAGE_START_WIDTH:IMAGE_START_WIDTH+IMAGE_WIDTH,:3] = camera_img
        self.show_image = True

    def string_callback(self, ros_data):
        self.message = ros_data.data
        self.full_screen[:,:] = BLACK
        self.text_splited_by_line = self.message.split('\n')
        self.show_image = False

    def filename_callback(self, data):
        self.filename = data.data
        img = cv2.imread(self.filename, cv2.IMREAD_COLOR)
        self.full_screen = cv2.resize(img, (BACKGROUND_WIDTH, BACKGROUND_HEIGHT))
        self.show_image = True


def run():
    rospy.init_node('hsr_head_display_viewer', anonymous=True)
    rate = rospy.Rate(ROS_HZ) # 10hz
    head_image_controller = HeadDisplayController()

    while not rospy.is_shutdown():
        if head_image_controller.show_image == False:
            line_height = len(head_image_controller.text_splited_by_line)
            for i, line in enumerate(head_image_controller.text_splited_by_line):

                textsize = cv2.getTextSize(line, font, fontScale=fontScale, thickness=thickness)[0]
                textX = (BACKGROUND_WIDTH - textsize[0]) // 2
                textY = (BACKGROUND_HEIGHT + textsize[1]) / 2 - (line_height- 1) * TEXT_DISTANCE_Y // 2
                textY = textY + i * TEXT_DISTANCE_Y
                cv2.putText(head_image_controller.full_screen, line, (int(textX), int(textY)), font, fontScale=fontScale, color=WHITE, thickness=thickness)

        cv2.namedWindow('hsr_head', cv2.WND_PROP_FULLSCREEN)
        # FULL SCREEN MODE
        cv2.setWindowProperty('hsr_head',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
        cv2.imshow('hsr_head', head_image_controller.full_screen)
        cv2.waitKey(1)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
