import cv2
import supervision as sv
from ultralytics import YOLOv10
import numpy as np
import matplotlib.pyplot as plt

# YOLOv10 모델 로드
model = YOLOv10('module/yolov10/weight/yolov10m_best.pt')

# Supervision Annotator 설정
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# 웹캠 열기
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# matplotlib 설정
plt.ion()
fig, ax = plt.subplots()

def press(event):
    if event.key == 'q':
        plt.close(event.canvas.figure)

fig.canvas.mpl_connect('key_press_event', press)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # 프레임 전처리
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (640, 480))

    # 모델을 사용하여 객체 감지 수행
    results = model(img)[0]

    # 감지된 객체의 클래스 인덱스와 신뢰도 추출
    classes = results.boxes.cls.cpu().numpy()  # 클래스 인덱스
    confidences = results.boxes.conf.cpu().numpy()  # 신뢰도

    # 라벨 생성: "클래스 이름: 신뢰도" 형식으로 생성
    labels = [f"{model.names[int(cls)]}: {conf:.2f}" for cls, conf in zip(classes, confidences)]

    # 감지 결과로부터 Detections 객체 생성
    detections = sv.Detections.from_ultralytics(results)
    detections.labels = labels  # labels 속성을 수동으로 추가

    # 감지 결과를 이미지에 시각화
    annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
    annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)

    # 결과 프레임을 matplotlib를 사용하여 표시
    ax.clear()
    ax.imshow(cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB))
    ax.set_title('YOLOv10 Webcam Detection')
    ax.axis('off')
    plt.draw()
    plt.pause(0.001)

    # 'q' 키를 누르면 루프 종료
    if not plt.fignum_exists(fig.number):
        break

# 웹캠 및 윈도우 해제
cap.release()
plt.ioff()
plt.show()
