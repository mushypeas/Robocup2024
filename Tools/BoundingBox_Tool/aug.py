import os
from tqdm import tqdm
import argparse
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--classnames-file', type=str, default='../BoundingBox_Tool/Images/Test/classnames.txt', help='path to classnames.txt')
    parser.add_argument('--labels-folder', type=str, default='../BoundingBox_Tool/Images/Test/Labeled', help='path to folder containing label files')
    parser.add_argument('--exp-labels-folder', type=str, default='../BoundingBox_Tool/Images/Test/exp/labels', help='path to folder containing exp label files')
    parser.add_argument('--width-threshold', type=float, nargs=2, default=(0.01, 0.5), help='bounding box width threshold (min, max)')
    parser.add_argument('--height-threshold', type=float, nargs=2, default=(0.01, 0.5), help='bounding box height threshold (min, max)')
    parser.add_argument('--center-threshold', type=float, nargs=4, default=(0.0, 1.0, 0.0, 1.0), help='bounding box center threshold (x_min, x_max, y_min, y_max)')
    parser.add_argument('--view-threshold', type=float, nargs=4, default=(0.25, 0.75, 0.25, 0.75), help='view area threshold (x_min, x_max, y_min, y_max)')
    return parser.parse_args()

def load_classnames(filepath):
    with open(filepath, 'r') as file:
        classnames = [line.strip() for line in file]
    return classnames

def is_in_center(center_x, center_y, center_threshold):
    cx_min, cx_max, cy_min, cy_max = center_threshold
    return cx_min <= center_x <= cx_max and cy_min <= center_y <= cy_max

def filter_outliers(filepath, width_threshold, height_threshold, center_threshold, view_threshold):
    with open(filepath, 'r') as file:
        lines = file.readlines()
    
    valid_lines = []
    for line in lines:
        parts = line.strip().split()
        class_id, center_x, center_y, width, height = map(float, parts)
        
        # 바운딩 박스 크기 기준으로 필터링
        if width < width_threshold[0] or width > width_threshold[1]:
            continue
        if height < height_threshold[0] or height > height_threshold[1]:
            continue
        
        # 중앙점 위치 기준으로 필터링
        if center_x < center_threshold[0] or center_x > center_threshold[1]:
            continue
        if center_y < center_threshold[2] or center_y > center_threshold[3]:
            continue
        
        # 중앙 view 영역 기준으로 필터링
        if not is_in_center(center_x, center_y, view_threshold):
            continue
        
        valid_lines.append(line)
    
    with open(filepath, 'w') as file:
        file.writelines(valid_lines)

def remove_class_from_file(filepath, class_index):
    with open(filepath, 'r') as file:
        lines = file.readlines()
    
    with open(filepath, 'w') as file:
        for line in lines:
            if not line.startswith(f"{class_index} "):
                file.write(line)

def process_labels(labels_folder, class_index, width_threshold, height_threshold, center_threshold, view_threshold):
    for txt_file in tqdm(os.listdir(labels_folder)):
        if txt_file.endswith(".txt"):
            filepath = os.path.join(labels_folder, txt_file)
            remove_class_from_file(filepath, class_index)
            filter_outliers(filepath, width_threshold, height_threshold, center_threshold, view_threshold)

def main():
    args = parse_args()

    # Load class names
    classnames = load_classnames(args.classnames_file)
    
    # Get class name to remove
    classname_to_remove = input(f"제거할 클래스 이름을 입력하세요 (가능한 이름: {', '.join(classnames)}): ")
    
    if classname_to_remove not in classnames:
        print(f"Cannot found {classname_to_remove}")
        return
    
    # Find class index
    class_index = classnames.index(classname_to_remove)
    
    # Process labels in the main folder
    process_labels(args.labels_folder, class_index, args.width_threshold, args.height_threshold, args.center_threshold, args.view_threshold)
    
    # Process labels in the exp folder
    process_labels(args.exp_labels_folder, class_index, args.width_threshold, args.height_threshold, args.center_threshold, args.view_threshold)

    print("[DONE] All labels are updated.")

if __name__ == "__main__":
    main()
