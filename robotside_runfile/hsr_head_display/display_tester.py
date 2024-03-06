#!/usr/bin/env python3.5
# -*- coding: utf-8 -*-
import sys
import cv2
import rospy, os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


bridge = CvBridge()
def run():
    rospy.init_node('hsr_display_tester', anonymous=True)
    string_pub = rospy.Publisher('/hsr_head_msg', String, queue_size=10)
    file_pub = rospy.Publisher('/hsr_head_file', String, queue_size=10)
    img_pub = rospy.Publisher('/hsr_head_img', Image, queue_size=10)
    while not rospy.is_shutdown():
        input_str = input('mode(str / img / file): ')
        if input_str == 'str':
            input_msg = input('show msg: ')
            string_pub.publish(input_msg)
        elif input_str == 'file':
            file_name = input('filename : ')
            file_pub.publish(file_name)
        else:
            img_dir = input('image name: ')
            img = cv2.imread('images/'+img_dir, cv2.IMREAD_COLOR)
            img_pub.publish(bridge.cv2_to_imgmsg(img))


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
