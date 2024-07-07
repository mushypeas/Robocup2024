# 데이터 수집 절차

## 데이터 수집 Pure
1. **[환경 설정]** (venv) 및 Directory 접근 + 커맨드를 입력한다
    - Terminal에 `capture` 를 입력한다
    - [목적] (venv) virtual env에서 `../Tools/xtioncam_capture` 로 들어가기 위함 (데이터 수집 환경)
2. **[커맨드 입력]** 커맨드를 입력하면, camera view로 바라본 화각에서 창이 켜진다
    - Command
        
        ```bash
        python openni2_ros_capture.py
        ```
3. **[BoundingBox Tool 활용]** `../BoundingBox_Tool/bbox_snu.py` 열면
    - Bounding box를 딸 수 있는 LabelTool 창이 뜬다. 사진을 찍었음과 동시에 함께 라벨링이 적용되어 있음을 확인할 수 있다!
    

## 데이터 수집 with Ultimate AUTOLABEL
1. **[모델 준비] Autolabel을 위한 model** file 준비 
    - → `Tools/xtioncam_capture/weight/`에 Model .pt file 삽입
    - ../autolabel_v10/tidyup.sh 를 돌리면 Image와 Labeled folder가 사라진다
2. **[환경 설정]** (venv) 및 Directory 접근 + 커맨드를 입력한다
    - Terminal에 `capture` 를 입력한다
        - [목적] (venv) virtual env에서 `../Tools/xtioncam_capture` 로 들어가기 위함 (데이터 수집 환경)
3. **[커맨드 입력]** 커맨드를 입력하면, 아래와 같이 기존 camera view와 같은 형태로 rgb image with ULTIMATE AUTOLABEL 창이 함께 뜬다
    - Command
        
        ```bash
        ./capture.sh --idx [START INDEX] --input-class "[CLASS 1], [CLASS 2], [CLASS 3], ..."
        ```
        
        - 예시 : start index를 0으로 설정하여, “apple, orange, plum, banana, blue_milk, pink_milk, cucudas"에 대한 데이터를 수집하고 싶다면
            
            ```bash
            	./capture.sh --idx 0 --input-class "apple, orange, plum, banana, blue_milk, pink_milk, cucudas"
            ```
            
4. **[데이터 수집]** 데이터를 수집하듯이, rgb_image 창에서 엔터를 누르면서 다음 사진을 지속적으로 수집할 수 있다
    
    - (왼쪽 창) 이미지를 찍을 때마다 원본 이미지는 `../BoundingBox_Tool/Images/Test/` 에 저장된다
        - autolabeled image file과 labeled txt file 은 `../BoundingBox_Tool/Images/Test/Labeled` 에 저장된다
    - (가운데 창) 한 이미지를 찍을 때마다 Autolabel이 되었음을 보여준다.
        - 약간 텀이 있는 이유는,, shell에서 conda activate하기가 어려워서 `xtioncam_capture/openni2_ros_capture_v4.py`에서 내부적으로 하나 이미지 딸 때마다 conda 들어가서 autolabel 돌리기 때문이다.
5. **[BoundingBox Tool 활용]** `../BoundingBox_Tool/bbox_snu.py` 열면
    - Autolabel이 적용된 이미지들에 대하여 bbox를 딸 수 있는 LabelTool 창이 뜬다. 사진을 찍었음과 동시에 함께 라벨링이 적용되어 있음을 확인할 수 있다!
    