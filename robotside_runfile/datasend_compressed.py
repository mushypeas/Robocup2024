import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2



class DataSendController:
    def __init__(self):
        self.rgb_count = 0
        self.depth_count = 0
        self.bridge = CvBridge()

        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, self.rgb_callback)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, self.depth_callback)

        self.rgb_pub = rospy.Publisher('/snu/rgb_rect_compressed', CompressedImage, queue_size=1)
        self.depth_pub = rospy.Publisher('/snu/depth_rect_compressed', CompressedImage, queue_size=1)


    def rgb_callback(self, data):
        self.rgb_msg = data
        self.rgb_count += 1

    def depth_callback(self, data):
        self.depth_msg = data
        self.depth_count += 1

    def publish(self):
        global_stamp = rospy.Time.now()
        _rgb_msg = self.rgb_msg
        _depth_msg = self.depth_msg

        _rgb_msg.header.seq = global_stamp
        _depth_msg.header.seq = global_stamp

        self.rgb_pub.publish(self.compress_rgb(_rgb_msg))
        self.depth_pub.publish(self.compress_depth(_depth_msg))

    def compress_rgb(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        msg = CompressedImage()
        msg.header = data.header
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg',cv_img,[1,50])[1]).tobytes()
        return msg

    def compress_depth(self, data):
        image_np = self.bridge.imgmsg_to_cv2(data, "32FC1").astype(np.uint16)

        msg = CompressedImage()
        msg.header = data.header
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', image_np)[1]).tobytes()

        return msg



if __name__ == '__main__':
    rospy.init_node('data_sender')
    fps = 4
    r = rospy.Rate(fps)
    data_send_controller = DataSendController()
    while not rospy.is_shutdown():
        print(data_send_controller.depth_count, data_send_controller.rgb_count)
        if data_send_controller.depth_count > 0 and data_send_controller.rgb_count > 0:
            data_send_controller.publish()
        r.sleep()
