import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class SHOW_IMG:
    def __init__(self):
        self.node_name = "publish_img"
        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber('/snu/yolo_img', Image, self.img_callback)
        self.br = CvBridge()
        
    def img_callback(self, data):
        try:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        
        # Display image
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
        
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        show_img = SHOW_IMG()
        show_img.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
