import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class PUBLISH_IMG:
    def __init__(self):
        self.node_name = "publish_img"
        rospy.init_node(self.node_name, anonymous=True)
        img_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        #img_topic = '/snu/yolo_img'
        self.publisher = rospy.Publisher(img_topic, Image, queue_size=10)
        self.timer_period = 0.1
        self.camera = cv2.VideoCapture(0) 
        self.br = CvBridge()
        self.timer = rospy.Timer(rospy.Duration(self.timer_period), self.timer_callback)

    def timer_callback(self, event):
        if self.camera.isOpened():
            success, img = self.camera.read()
            if not success: 
                rospy.loginfo("Ignoring empty camera frame.")
            else:
                try:
                    img_msg = self.br.cv2_to_imgmsg(img, encoding="bgr8")
                    self.publisher.publish(img_msg)
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: {0}".format(e))
                cv2.waitKey(1)

    def spin(self):
        rospy.spin()

def main():
    try:
        publisher = PUBLISH_IMG()
        publisher.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
