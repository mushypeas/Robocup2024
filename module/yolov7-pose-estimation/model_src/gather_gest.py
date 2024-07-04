#!/usr/bin/env python

import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts
from models.experimental import attempt_load

# 모델 로드
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = attempt_load('yolov7-w6-pose.pt', map_location=device)
model.to(device).eval()

# ROS 초기화
rospy.init_node('gest_estimation_node', anonymous=True)
bridge = CvBridge()

# 라벨 맵핑
label_dict = {'a': 'raising left', 'l': 'raising right', 'm': 'pointing right', 'z': 'pointing left'}
keypoint_labels = []

def preprocess(image):
    img = letterbox(image, 960, stride=64, auto=True)[0]
    img = img.transpose((2, 0, 1))[::-1]
    img = np.ascontiguousarray(img)
    img_tensor = torch.from_numpy(img).float().div(255.0).unsqueeze(0)
    return img_tensor.to(device), img.shape

def extract_keypoints(pred):
    if isinstance(pred, tuple):
        pred = pred[0]
    pred = non_max_suppression_kpt(pred, 0.25, 0.65, nc=model.yaml['nc'], nkpt=model.yaml['nkpt'], kpt_label=True)
    with torch.no_grad():
        output = output_to_keypoint(pred)
    return output

def scale_coords(coords, from_shape, to_shape):
    return coords

def origin_to_640x480(kpts):
    kpt640x480 = []
    for idx, num in enumerate(kpts):
        if idx % 3 == 2:
            kpt640x480.append(num)
        else:
            kpt640x480.append(num / 1.5)
    return kpt640x480

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # 프레임 전처리 및 예측 수행
    img, resized_shape = preprocess(frame)
    with torch.no_grad():
        pred = model(img)
    
    output = extract_keypoints(pred)

    # 키포인트 시각화
    if isinstance(output, np.ndarray) and output.shape[0] > 0:
        # 원본 이미지 크기로 좌표 조정
        output[:, 7:] = scale_coords(output[:, 7:], resized_shape, frame.shape)
        for idx in range(output.shape[0]):
            origin_kpt = output[idx, 7:].T
            kpt640x480 = origin_to_640x480(origin_kpt)

            plot_skeleton_kpts(frame, kpt640x480, 3)

    # 프레임 디스플레이
    cv2.imshow('Gest Estimation', frame)
    
    # 라벨링을 위한 키 입력 캡처
    key = cv2.waitKey(1) & 0xFF
    if key in [ord('a'), ord('l'), ord('m'), ord('z')]:
        label = label_dict[chr(key)]
        keypoint_labels.append((output, label))
        print(f"{label}로 라벨링됨")
    elif key == ord('q'):
        rospy.signal_shutdown('User requested shutdown.')

# 이미지 토픽 구독
image_topic = '/camera/rgb/image_rect_color'
rospy.Subscriber(image_topic, Image, image_callback)

# ROS 노드가 종료될 때까지 유지
rospy.spin()

# 수집된 데이터 저장
with open('gest_keypoint_labels.txt', 'w') as f:
    for output, label in keypoint_labels:
        if isinstance(output, np.ndarray):
            for idx in range(output.shape[0]):
                origin_kpt = output[idx, 7:].T
                kpt640x480 = origin_to_640x480(origin_kpt)
                keypoints_str = ' '.join([str(i) for i in kpt640x480])
                write_str = f"{keypoints_str} {label}\n"
                f.write(write_str)

cv2.destroyAllWindows()
