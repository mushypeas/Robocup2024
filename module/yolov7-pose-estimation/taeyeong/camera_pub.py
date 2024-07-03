#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_camera():
    rospy.init_node('camera_publisher_node', anonymous=True)
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
    rate = rospy.Rate(30)  # 30Hz

    cap = cv2.VideoCapture(cv2.CAP_OPENNI2)
    if not cap.isOpened():
        rospy.logerr("Failed to open OpenNI2 device")
        return

    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read frame from OpenNI2 device")
            continue

        try:
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_camera()
    except rospy.ROSInterruptException:
        pass
