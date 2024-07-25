import os

def delete_images_in_directory():
    # 이미지가 있는 디렉토리 경로
    image_dir = '../BoundingBox_Tool/Images/Test/'

    # 디렉토리가 존재하는지 확인
    if os.path.exists(image_dir):
        try:
            # 디렉토리 내의 모든 파일을 순회
            for filename in os.listdir(image_dir):
                file_path = os.path.join(image_dir, filename)
                # 파일이고 이미지 확장자를 가진 경우에만 삭제
                if os.path.isfile(file_path) and filename.lower().endswith(('.png', '.jpg', '.jpeg', '.tiff', '.bmp', '.gif')):
                    os.remove(file_path)
                    print(f"'{filename}' 이미지가 삭제되었습니다.")
            
            print(f"'{image_dir}' 디렉토리의 모든 이미지가 삭제되었습니다.")
        except Exception as e:
            print(f"이미지 삭제 중 오류가 발생했습니다: {e}")
    else:
        print(f"'{image_dir}' 디렉토리가 존재하지 않습니다.")

if __name__ == "__main__":
    delete_images_in_directory()