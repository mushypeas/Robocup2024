  # Robocup 2024 Tidyboy-DSPL


## Setup

1. Install Ros Noetic (Ubuntu 20.04) & cuda 11.1
2. Install HSR api (ros-noetic-tmc-desktop-full) [`Link`](https://docs.hsr.io/hsrb_user_manual_en/howto/pc_install.html#id2)
3. Install Python Packages (Python 3.7)
```shell
pip install -r requirements.txt
```
4. Install ros_numpy
```shell
sudo apt install ros-noetic-ros-numpy
```
5. install yolov7
```shell
cd module/yolov7
pip install -r requirements.txt
```
- issue: libcublas 나 버전 오류시, torch, torchvision을 내 cuda에 맞추어 재설치 필요.
- ex. for rtx-3090ti
 ```shell
pip install torch==1.10.1+cu111 torchvision==0.11.2+cu111 torchaudio==0.10.1 -f https://download.pytorch.org/whl/cu111/torch_stable.html
```

6. download weight file for yolov7
- link : [`Ours`](https://drive.google.com/file/d/1FwHFKKtF7xxDjzJNCN6lbHH6rvJKraZq/view?usp=sharing), [`yolov7-tiny.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-tiny.pt)
- move weight file in yolov7/weight/
- (todo) 현재는 serve breakfast와 clean the table용 물체만 학습되어 있음.

7. Settings for Bytetrack (Human re-id model)
- link : https://github.com/ifzhang/ByteTrack
- Install all the dependencies on the link
- Please try the folllowing command if you encountered an error while installing dependencies ...
```shell
sudo apt install python3.7-dev
```
- add pretrained weight in
```shell
module/ByteTrack/pretrained/bytetrack_s_mot17.pth.tar
```
- To execute Bytetrack and publish topic /snu/carry_my_luggage_yolo
```shell
python module/ByteTrack/tools/demo_track.py
```

8. Settings for stt
```shell
sudo apt install libportaudio2
```

9. Settings for Air-Clothing-MA (Human Attribute model)
- link : https://github.com/ai4r/Air-Clothing-MA

- Install all the dependencies on the link
- Install RoIAlign in Air-Clothing-MA in modules/human_attribute
```shell
git clone https://github.com/longcw/RoIAlign.pytorch.git
```
```shell
cd RoIAlign
python setup.py install
```

- Download yolo-v3 model from [here](https://drive.google.com/drive/folders/18etoB_1Sz3rvXIz9oJwKpKZ7IkSlgMHj?usp=drive_link) and put in 'modules/human_attribute'.

[//]: # (- original old one: &#40;https://drive.google.com/file/d/1yCz6pc6qHJD2Zcz8ldDmJ3NzE8wjaiT6/view&#41;)

10. Settings qrcode
```shell
 sudo apt install libzbar0
 pip install pyzbar
```

## Run Yolov7

``` shell
hsrb_mode
cd module/yolov7
python run_yolov7.py
```

## Run task code
``` shell
hsrb_mode
python main_client.py
```

## Testing
``` shell
hsrb_mode
python test_client.py
```
