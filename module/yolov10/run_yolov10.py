import argparse
import cv2
import logging
import math
import torch
import torch.backends.cudnn as cudnn
from numpy import random
from ultralytics import YOLOv10
import numpy as np
import supervision as sv


import rospy
import ros_numpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Int16MultiArray, Float32MultiArray

import sys
sys.path.append('../../../robocup2024')
from hsr_agent.global_config import *

class custom_Yolov10:
    def __init__(self):
        # for hsr topic
        self.rgb_img = None
        self.pc = None
        self.object_len = 0
        self.yolo_bbox = []
        self.bridge = CvBridge()

        self._rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, self._rgb_callback)
        self._pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self._pc_callback, queue_size=1)
        self.yolo_pub = rospy.Publisher('/snu/yolo', Int16MultiArray, queue_size=10)

        #shlim /snu/yolo for carry my luggage bag detection; add confidence score
        self.yolo_with_conf_pub = rospy.Publisher('/snu/yolo_conf', Int16MultiArray, queue_size=10)
        self.yolo_img_pub = rospy.Publisher('/snu/yolo_img', Image, queue_size=10)
        self.ready()

    def _rgb_callback(self, img_msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def ready(self):
        self.model = YOLOv10("weight/yolov10m_240706_v3.pt")  #path for weight file
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in OBJECT_LIST]

    def detect(self):
        if self.rgb_img is None:
            return None
        
        img = self.rgb_img
        

        model = self.model

        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
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

                tl = 1
                color =colors[int(cls)] 

                bbox_list.append([cent_x, cent_y, width, height, int(cls)])
                bbox_with_conf_list.append([cent_x, cent_y, width, height, int(cls), int(conf * 100)]) #conf: bbox score

                cv2.rectangle(img, c1, c2, color, Thickness = tl, lineType = cv2.LINE_AA)
                
                tf = max(tl - 1, 1)  # font thickness
                t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
                c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
                cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
                cv2.putText(img, label,(c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
        # show result
        view_img = True
        if view_img:
            cv2.imshow('hsr_vision', img) # [378:456,528:612]
            cv2.waitKey(1)  # 1 millisecond
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
        yolo_img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.yolo_img_pub.publish(yolo_img_msg)


if __name__ == '__main__':

    rospy.init_node('hsr_yolov10', anonymous=True)
    yolov10_controller = custom_Yolov10()
    image_resolution = (480, 640, 3)
    r = rospy.Rate(5)
    with torch.no_grad():
        while not rospy.is_shutdown():
            bbox_list, bag_bbox_list, img = yolov10_controller.detect() # bag bbox list is the version added confidence scores
            if bbox_list is None or yolov10_controller.pc is None:
                continue

            yolov10_controller.yolo_publish(bbox_list)
            yolov10_controller.yolo_with_conf_publish(bag_bbox_list)    
            yolov10_controller.yolo_img_publish(img)
            r.sleep()
