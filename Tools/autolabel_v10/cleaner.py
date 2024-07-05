import os
import shutil

def delete_labeled_directory():
    # 삭제할 디렉토리 경로
    labeled_dir = '../BoundingBox_Tool/Images/Test/Labeled/'

    # 디렉토리가 존재하는지 확인
    if os.path.exists(labeled_dir):
        try:
            # 디렉토리와 그 내용을 모두 삭제
            shutil.rmtree(labeled_dir)
            print(f"'{labeled_dir}' 디렉토리가 성공적으로 삭제되었습니다.")
        except Exception as e:
            print(f"'{labeled_dir}' 디렉토리 삭제 중 오류가 발생했습니다: {e}")
    else:
        print(f"'{labeled_dir}' 디렉토리가 존재하지 않습니다.")

if __name__ == "__main__":
    delete_labeled_directory()