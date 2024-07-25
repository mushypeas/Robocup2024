#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.contours = None
        self.data_header = None
        self.canny_pub = rospy.Publisher('/canny_result', Image, queue_size=10)
        
        self.rgb_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, self._rgb_callback)

    def _rgb_callback(self, data):

        frame = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        # cv2.imshow('original frame', frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        # cv2.imshow('gray', gray)

        edges = cv2.Canny(gray, 250, 350)

        # 푸리에 변환 적용
        rows, cols = edges.shape
        f = np.fft.fft2(edges)
        fshift = np.fft.fftshift(f)

        # Gaussian Low-Pass Filter
        crow, ccol = rows // 2, cols // 2
        mask = np.zeros((rows, cols), np.float32)
        sigma = 50  # Standard deviation for Gaussian filter
        x, y = np.ogrid[:rows, :cols]
        mask = np.exp(-((x - crow) ** 2 + (y - ccol) ** 2) / (2 * sigma ** 2))

        # Apply mask and inverse DFT
        fshift = fshift * mask
        f_ishift = np.fft.ifftshift(fshift)
        img_back = np.fft.ifft2(f_ishift)
        img_back = np.abs(img_back)


        
        img_back = (img_back - np.min(img_back)) / (np.max(img_back) - np.min(img_back)) * 255
        img_back = np.uint8(img_back)

        kernel = np.ones((1, 1), np.uint8)
        img_back = cv2.dilate(img_back, kernel, iterations=1)
        img_back = cv2.erode(img_back, kernel, iterations=1)

        morph = img_back

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tiny_exist = False
        tiny_loc = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if 200 < area < 5000:# 면적 기준으로 작은 물체 필터링 (적절히 조절 가능)
                print(area)
                x, y, w, h = cv2.boundingRect(contour)
                # cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if y+h//2 > 330 and y+h//2 < 450 and x+w//2 > 230 and x+w//2 < 410:

                    tiny_exist = True
                    # self.agent.say('Tiny object.', show_display=False)
                    cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    
        # cv2.imshow('morph', morph




        morph = cv2.bitwise_not(morph)


        morph = np.uint8(morph)
        canny_img_msg = self.bridge.cv2_to_imgmsg(morph, 'mono8')
        # canny_img_msg.header = data.data_header
        if canny_img_msg is not None:
            self.canny_pub.publish(canny_img_msg)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('image_processor_node', anonymous=True)
    image_processor = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    cv2.destroyAllWindows()

