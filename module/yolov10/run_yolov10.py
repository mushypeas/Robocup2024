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
sys.path.append('../../../Robocup2024')
from hsr_agent.global_config import *




class custom_Yolov10:
    def __init__(self):
        # for hsr topic
        self.rgb_img = None
        self.pc = None
        self.object_len = 0
        self.yolo_bbox = []
        self.bridge = CvBridge()

        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        self._pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self._pc_callback, queue_size=1)
        self.yolo_pub = rospy.Publisher('/snu/yolo', Int16MultiArray, queue_size=10)

        #shlim /snu/yolo for carry my luggage bag detection; add confidence score
        self.yolo_with_conf_pub = rospy.Publisher('/snu/yolo_conf', Int16MultiArray, queue_size=10)
        self.yolo_img_pub = rospy.Publisher('/snu/yolo_img', Image, queue_size=10)
        self.ready()

    def _rgb_callback(self, img_msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        rospy.loginfo('Received rgb callback')

    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)
        rospy.loginfo('Received pc callback')

    def ready(self):
        # self.model = YOLOv10("240621_v10m.pt")  #path for weight file
        self.model = YOLOv10("/home/tidy/Robocup2024/module/yolov10/weight/best.pt")  #path for weight file #240716 : cml만 학습한 weight. 수정필요 TODO -> 전체로 변경?
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in OBJECT_LIST]

    def detect(self):
        if self.rgb_img is None:
            return None, None, None
        
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
                class_name = OBJECT_LIST[int(cls)][0] #OBJECT_ LIST를 참조함.
                label = f'{class_name} {conf:.2f}'
                c1, c2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
                cent_x, cent_y = (c2[0] + c1[0]) // 2, (c2[1] + c1[1]) // 2
                width = c2[0] - c1[0]
                height = c2[1] - c1[1]

                bbox_list.append([cent_x, cent_y, width, height, int(cls)])
                bbox_with_conf_list.append([cent_x, cent_y, width, height, int(cls), int(conf * 100)]) #conf: bbox score

                cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), colors[int(cls)], 2)
                cv2.putText(img, label, (int(box[0]), int(box[1] - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
      
        # show result
        view_img = True
        if view_img:
            # cv2.imshow('hsr_vision', img) # [378:456,528:612]
            # cv2.waitKey(1)  # 1 millisecond
            print('i dont see image')
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

def get_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='weight/best_0704.pt', help='model.pt path(s)')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.3, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt = parser.parse_args()
    return opt

if __name__ == '__main__':
    opt = get_opt()
    print(opt)
    rospy.init_node('hsr_yolov10', anonymous=True)
    yolov10_controller = custom_Yolov10()
    image_resolution = (480, 640, 3)
    r = rospy.Rate(5)
    with torch.no_grad():
        while not rospy.is_shutdown():
            bbox_list, bag_bbox_list, img = yolov10_controller.detect() # bag bbox list is the version added confidence scores
            if bbox_list is None or yolov10_controller.pc is None:
                continue

            # _pc = yolov7_controller.pc.reshape(480, 640)
            # pc_np = np.array(_pc.tolist())[:, :, :3]
            yolov10_controller.yolo_publish(bbox_list)
            yolov10_controller.yolo_with_conf_publish(bag_bbox_list)
            # print(img)    
            yolov10_controller.yolo_img_publish(img)
            r.sleep()