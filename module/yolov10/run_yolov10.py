#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

import torch
import torch.backends.cudnn as cudnn
from numpy import random
from ultralytics import YOLOv10
import supervision as sv

import sys
sys.path.append('hsr_agent')
sys.path.append('../../hsr_agent')
from global_config import *

class HSR_Yolov10:
    def __init__(self):
        self.node_name = 'yolo_node'
        rospy.init_node(self.node_name, anonymous=True)

        # for hsr topic
        self.rgb_img = None
        self.bridge = CvBridge()

        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        self.yolo_pub = rospy.Publisher('/snu/yolo', Int16MultiArray, queue_size=10)
        # shlim /snu/yolo for carry my luggage bag detection; add confidence score
        self.yolo_with_conf_pub = rospy.Publisher('/snu/yolo_conf', Int16MultiArray, queue_size=10)
        self.yolo_img_pub = rospy.Publisher('/snu/yolo_img', Image, queue_size=10)

        self.ready()

    def _rgb_callback(self, img_msg):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        bbox_list, bag_bbox_list, img = self.detect()  # bag bbox list is the version added confidence scores
        if bbox_list is not None:
            self.yolo_publish(bbox_list)
            self.yolo_with_conf_publish(bag_bbox_list)
            self.yolo_img_publish(img)

    def ready(self):
        self.model = YOLOv10("weight/yolov10m_best.pt")  # path for weight file
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in OBJECT_LIST]

    def detect(self):
        if self.rgb_img is None:
            return None, None, None

        img = self.rgb_img
        model = self.model

        with torch.no_grad():  # Calculating gradients would cause a GPU memory leak
            results = model(img)[0]
            detections = sv.Detections.from_ultralytics(results)

        bbox_list = []
        bbox_with_conf_list = []

        colors = self.colors

        # Process detections
        if len(results):
            for box, cls, conf in zip(detections.xyxy, detections.class_id, detections.confidence):
                class_name = OBJECT_LIST[int(cls)][0]
                label = f'{class_name} {conf:.2f}'
                c1, c2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
                cent_x, cent_y = (c2[0] + c1[0]) // 2, (c2[1] + c1[1]) // 2
                width = c2[0] - c1[0]
                height = c2[1] - c1[1]

                bbox_list.append([cent_x, cent_y, width, height, int(cls)])
                bbox_with_conf_list.append([cent_x, cent_y, width, height, int(cls), int(conf * 100)])  # conf: bbox score

                cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), colors[int(cls)], 2)
                cv2.putText(img, label, (int(box[0]), int(box[1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

        return bbox_list, bbox_with_conf_list, img

    def yolo_publish(self, bbox_list):
        coord_list_msg = Int16MultiArray()
        coord_list_msg.data = [i for coord in bbox_list for i in coord]
        self.yolo_pub.publish(coord_list_msg)

    def yolo_with_conf_publish(self, bbox_list):
        coord_list_msg = Int16MultiArray()
        coord_list_msg.data = [i for coord in bbox_list for i in coord]
        self.yolo_with_conf_pub.publish(coord_list_msg)

    def yolo_img_publish(self, img):
        try:
            yolo_img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.yolo_img_pub.publish(yolo_img_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    yolov10_controller = HSR_Yolov10()

    with torch.no_grad():
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
