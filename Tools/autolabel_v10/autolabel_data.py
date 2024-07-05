import cv2
import os
from ultralytics import YOLOv10
import argparse
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='weight/v10m_240704_2.pt', help='model.pt path')
    parser.add_argument('--source', type=str, default='../BoundingBox_Tool/Images/Test', help='source directory containing images')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--label', default='../BoundingBox_Tool/Images/Test/Labeled', help='save results to Images/Test/Labeled')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--classnames', type=str, default='../BoundingBox_Tool/Images/Test/classnames.txt', help='file containing class names')
    return parser.parse_args()

def save_classnames(names, save_dir):
    # classnames_path = save_dir / 'classnames.txt'
    # with open(classnames_path, 'w') as file:
    #     file.write('\n'.join(names))

    upper_classnames_path = Path('../BoundingBox_Tool/Images/Test/classnames.txt')
    with open(upper_classnames_path, 'w') as file:
        file.write('\n'.join(names))

def check_and_create_classnames_file(classnames_path):
    if not os.path.exists(classnames_path):
        print(f"Please create {classnames_path} file with class names")

def save_labels(bbox_list, labelfilename, image_width, image_height):
    with open(labelfilename, 'w') as f:
        for bbox in bbox_list:
            class_id, x0, y0, x1, y1 = bbox
            xcenter = (x0 + x1) / 2 / image_width
            ycenter = (y0 + y1) / 2 / image_height
            width = (x1 - x0) / image_width
            height = (y1 - y0) / image_height
            f.write(f"{class_id} {xcenter} {ycenter} {width} {height}\n")

def yolov10_autolabel(args):
    model = YOLOv10(args.weights)
    os.makedirs(args.label, exist_ok=True)

    classnames_path = Path(args.classnames)
    check_and_create_classnames_file(classnames_path)

    # Load class names
    with open(classnames_path, 'r') as f:
        class_names = [line.strip() for line in f.readlines()]

    # Save class names to appropriate locations
    save_dir = Path(args.label)
    save_classnames(class_names, save_dir)

    for img_file in os.listdir(args.source):
        if img_file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff')):
            img_path = os.path.join(args.source, img_file)
            results = model.predict(source=img_path, imgsz=args.img_size, conf=args.conf_thres)
            annotated_image = results[0].plot()

            # 이미지 크기 얻기
            image = cv2.imread(img_path)
            image_height, image_width, _ = image.shape

            # 라벨 저장
            bbox_list = []
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                x0, y0, x1, y1 = box.xyxy[0]
                bbox_list.append((class_id, x0, y0, x1, y1))

            labelfilename = os.path.join(args.label, os.path.splitext(img_file)[0] + '.txt')
            save_labels(bbox_list, labelfilename, image_width, image_height)
            print(f"Labeled data saved to {labelfilename}")

            # 주석 이미지 저장
            output_path = os.path.join(args.label, img_file)
            cv2.imwrite(output_path, annotated_image)
            print(f"Annotated image saved to {output_path}")

if __name__ == "__main__":
    args = parse_args()
    yolov10_autolabel(args)
    print("***DONE!***")
