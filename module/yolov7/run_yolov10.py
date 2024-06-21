import argparse
import cv2
import logging
import math
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from ultralytics.nn.tasks import attempt_load_one_weight
from ultralytics.data.augment import LetterBox
import numpy as np
from ultralytics.utils.ops import non_max_suppression, scale_coords
from ultralytics.utils.plotting import Annotator
from ultralytics.utils.torch_utils import select_device

import rospy
import ros_numpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Int16MultiArray, Float32MultiArray
import time

import sys
sys.path.append('../../../robocup2024')
from hsr_agent.global_config import *

class Yolov10:
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

    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def ready(self):
        weights, imgsz = opt.weights, opt.img_size
        
        rank = -1
        # Initialize
        logging.basicConfig(
        format="%(message)s",
        level=logging.INFO if rank in [-1, 0] else logging.WARN)

        device = select_device(opt.device)
        print('device', device)
        half = device.type != 'cpu'  # half precision only supported on CUDA
        self.half = half
        print('half', half)

        # Load model
        print(weights)
      
        model, _ = attempt_load_one_weight(weights, device=device)
        stride = int(model.stride.max())  # model stride

        #check_img_size
        divisor = 32
        
        new_size = math.ceil(imgsz / divisor) * divisor  # ceil gs-multiple
        if new_size != imgsz:
            print('WARNING: --img-size %g must be multiple of max stride %g, updating to %g' % (imgsz, divisor, new_size))
        imgsz = new_size
        #####

        #model = TracedModel(model, device, opt.img_size)

        if half:
            model.half()  # to FP16


        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        print('object list names', names)
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

        self.device, self.model, self.colors, self.names = device, model, colors, names

    def detect(self):
        if self.rgb_img is None:
            return None
        img, im0 = self.preprocess_img(self.rgb_img)
        device, model, colors, names = self.device, self.model, self.colors, self.names

        img = torch.from_numpy(img).to(device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)


        # Inference
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)[0]
        bbox_list = []
        bbox_with_conf_list = []

        # Process detections
        if len(pred):
            # Rescale boxes from img_size to im0 size
            pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], im0.shape).round()
            annotator = Annotator(im0, line_width=1)
            # Write results
            for *xyxy, conf, cls in reversed(pred):
                label = f'{names[int(cls)]} {conf:.2f}'
                c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                cent_x, cent_y = (c2[0] + c1[0]) // 2, (c2[1] + c1[1]) // 2
                width = c2[0] - c1[0]
                height = c2[1] - c1[1]

                annotator.box_label(xyxy, im0, label=label, color=colors[int(cls)])
                bbox_list.append([cent_x, cent_y, width, height, int(cls)])
                bbox_with_conf_list.append([cent_x, cent_y, width, height, int(cls), int(conf * 100)]) #conf: bbox score
        # show result
        view_img = True
        if view_img:
            cv2.imshow('hsr_vision', im0) # [378:456,528:612]
            cv2.waitKey(1)  # 1 millisecond
        return bbox_list, bbox_with_conf_list, im0

    def preprocess_img(self, img):
        img0 = img.copy()
        # LetterBox 클래스 인스턴스 생성
        letterbox = LetterBox(new_shape=(640, 640), auto = True)

        # __call__ 메서드 호출하여 이미지 리사이즈 및 패딩 추가
        img = letterbox(image = img)

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        return img, img0

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
    yolov10_controller = Yolov10()
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
