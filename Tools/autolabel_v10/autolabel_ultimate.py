import cv2
import os
import shutil
import random
from ultralytics import YOLOv10
import argparse
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='weight/yolov10m_240704_v2.pt', help='model.pt path')
    parser.add_argument('--source', type=str, default='../BoundingBox_Tool/Images/Test', help='source directory containing images')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--label', default='../BoundingBox_Tool/Images/Test/Labeled', help='save results to Images/Test/Labeled')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--classnames', type=str, default='../BoundingBox_Tool/Images/Test/classnames.txt', help='file containing class names')
    parser.add_argument('--input-classes', type=str, required=True, help='comma-separated list of input classes')
    return parser.parse_args()

def save_classnames(names, save_dir):
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

def generate_random_color():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

def yolov10_autolabel(args):
    model = YOLOv10(args.weights)
    os.makedirs(args.label, exist_ok=True)

    classnames_path = Path(args.classnames)
    check_and_create_classnames_file(classnames_path)

    # Load class names
    with open(classnames_path, 'r') as f:
        class_names = [line.strip() for line in f.readlines()]

    # Process input classes
    input_classes = [cls.strip() for cls in args.input_classes.split(',')]
    valid_input_classes = [cls for cls in input_classes if cls in class_names]
    if len(valid_input_classes) != len(input_classes):
        print(f"Warning: Some input classes are not in classnames.txt. Valid classes: {valid_input_classes}")
    else: 
        print(f"Input classes: {valid_input_classes}")

    # Create a mapping of class names to their indices
    class_to_index = {name: index for index, name in enumerate(class_names)}

    # Create color map for classes
    color_map = {class_name: generate_random_color() for class_name in valid_input_classes}

    # Create or recreate 'labeled_img' folder
    labeled_img_dir = os.path.join(args.label, 'labeled_img')
    if os.path.exists(labeled_img_dir):
        shutil.rmtree(labeled_img_dir)
    os.makedirs(labeled_img_dir)

    for img_file in os.listdir(args.source):
        if img_file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff')):
            img_path = os.path.join(args.source, img_file)
            results = model.predict(source=img_path, imgsz=args.img_size, conf=args.conf_thres)

            # Get image dimensions
            image = cv2.imread(img_path)
            image_height, image_width, _ = image.shape

            # Save labels
            bbox_list = []
            highest_conf_boxes = {}

            for box in results[0].boxes:
                class_id = int(box.cls[0])
                class_name = class_names[class_id]
                if class_name in valid_input_classes:  # Only process classes specified by the user
                    conf = float(box.conf[0])
                    x0, y0, x1, y1 = box.xyxy[0]

                    highest_conf_boxes[class_id] = (conf, (class_id, x0, y0, x1, y1))

            for _, bbox in highest_conf_boxes.values():
                bbox_list.append(bbox)

            if bbox_list:  # Only save if there are valid bounding boxes
                labelfilename = os.path.join(args.label, os.path.splitext(img_file)[0] + '.txt')
                save_labels(bbox_list, labelfilename, image_width, image_height)
                print(f"Labeled data saved to {labelfilename}")

                # Draw bounding boxes on the image
                for bbox in bbox_list:
                    class_id, x0, y0, x1, y1 = bbox
                    class_name = class_names[class_id]
                    color = color_map[class_name]
                    cv2.rectangle(image, (int(x0), int(y0)), (int(x1), int(y1)), color, 2)
                    label = f"{class_name}: {highest_conf_boxes[class_id][0]:.2f}"
                    cv2.putText(image, label, (int(x0), int(y0) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Save the annotated image in the 'labeled_img' folder
                labeled_image_path = os.path.join(labeled_img_dir, img_file)
                cv2.imwrite(labeled_image_path, image)
                print(f"Labeled image saved to {labeled_image_path}")

if __name__ == "__main__":
    print("\n***Start ULTIMATE Autolabel !***")
    args = parse_args()
    yolov10_autolabel(args)
    print("\n*** DONE! ***")
