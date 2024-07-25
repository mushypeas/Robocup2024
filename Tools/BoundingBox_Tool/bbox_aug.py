import os
from tqdm import tqdm

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

def main():
    # directorh path setting
    classnames_file = "./Images/Test/classnames.txt"
    
    # load classnames 
    classnames = load_classnames(classnames_file)
    
    # 제거할 클래스 이름 입력
    classname_to_remove = input(f"제거할 클래스 이름을 입력하세요 (가능한 이름: {', '.join(classnames)}): ")
    
    if classname_to_remove not in classnames:
        print(f"Cannot found {classname_to_remove}")
        return
    
    # 클래스 인덱스 찾기
    class_index = classnames.index(classname_to_remove)
    width_threshold = (0.01, 0.5)  # 바운딩 박스 폭의 임계값 (최소, 최대)
    height_threshold = (0.01, 0.5)  # 바운딩 박스 높이의 임계값 (최소, 최대)
    center_threshold = (0.0, 1.0, 0.0, 1.0)  # 중앙점 위치의 임계값 (x_min, x_max, y_min, y_max)
    view_threshold = (0.25, 0.75, 0.25, 0.75)  # 중앙 view 영역의 임계값 (x_min, x_max, y_min, y_max)


    
    labels_folder = "./Images/Test/Labeled"
    
    # labels 폴더 내의 모든 txt 파일 처리
    for txt_file in tqdm(os.listdir(labels_folder)):
        if txt_file.endswith(".txt"):
            filepath = os.path.join(labels_folder, txt_file)
            remove_class_from_file(filepath, class_index)
            filter_outliers(filepath, width_threshold, height_threshold, center_threshold, view_threshold)


    labels_folder = "./Images/Test/exp/labels"
    # exp/labels 폴더 내의 모든 txt 파일 처리
    for txt_file in tqdm(os.listdir(labels_folder)):
        if txt_file.endswith(".txt"):
            filepath = os.path.join(labels_folder, txt_file)
            remove_class_from_file(filepath, class_index)
            filter_outliers(filepath, width_threshold, height_threshold, center_threshold, view_threshold)

    
    print("[DONE] All labels are updated.")

if __name__ == "__main__":
    main()

