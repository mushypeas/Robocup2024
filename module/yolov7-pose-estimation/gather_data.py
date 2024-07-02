import cv2
import torch
import numpy as np
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts
from models.experimental import attempt_load

# 모델 로드
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = attempt_load('yolov7-w6-pose.pt', map_location=device)
model.to(device).eval()

# 비디오 캡처 설정
cap = cv2.VideoCapture(16)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 라벨 맵핑
label_dict = {'s': 'standing', 'l': 'lying', 't': 'sitting'}
keypoint_labels = []

def preprocess(image):
    img = letterbox(image, 960, stride=64, auto=True)[0]  # 다시 960으로 변경
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

    for idx, num in enumerate(origin_kpt):
        if idx % 3 == 2:
            kpt640x480.append(num)
        else:
            kpt640x480.append(num / 1.5)

    return kpt640x480


while True:
    ret, frame = cap.read()
    if not ret:
        break

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
    cv2.imshow('Pose Estimation', frame)

    # 라벨링을 위한 키 입력 캡처
    key = cv2.waitKey(1) & 0xFF
    if key in [ord('s'), ord('l'), ord('t')]:
        label = label_dict[chr(key)]
        keypoint_labels.append((output, label))
        print(f"{label}로 라벨링됨")

    # 종료 조건
    elif key == ord('q'):
        break

# 수집된 데이터 저장
with open('keypoint_labels.txt', 'w') as f:
    for output, label in keypoint_labels:
        if isinstance(output, np.ndarray):
            for idx in range(output.shape[0]):
                origin_kpt = output[idx, 7:].T
                kpt640x480 = origin_to_640x480(origin_kpt)
                keypoints_str = ' '.join([str(i) for i in kpt640x480])
                write_str = f"{keypoints_str} {label}\n"
                f.write(write_str)

cap.release()
cv2.destroyAllWindows()