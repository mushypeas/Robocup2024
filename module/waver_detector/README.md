# waver_detector module

### Tasks which requires this module

- restaurant

### Setup Manual

1. Make sure [**opencv-python**](https://pypi.org/project/opencv-python/) is installed.
2. In `run_openpose.py`, change `BASE_DIR` variable to the model directory, which will contain the caffe model weights.
(This manual uses `~/robocup2024/module/waver_detector/models/` as `BASE_DIR`, which is the default.)
3. Change directory to the `BASE_DIR` and download the official OpenPose weight files (COCO)
    
    ```bash
    foo@bar:~$> cd ~/robocup2024/module/waver_detector/models/
    
    foo@bar:~/robocup2024/module/waver_detector/models$> wget https://raw.githubusercontent.com/CMU-Perceptual-Computing-Lab/openpose/master/models/pose/coco/pose_deploy_linevec.prototxt
    
    foo@bar:~/robocup2024/module/waver_detector/models$> wget http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel
    ```
    

### How to run?

```bash
foo@bar:~$> hsrb_mode

foo@bar:~$> source ~/robocup2024/venv/bin/activate

foo@bar:~/robocup2024/module/waver_detector$> python run_openpose.py
```

### Published Topics

- `/snu/openpose`: Image. This topic visualizes the OpenPose skeleton of every person detected and bounding boxes of hand raising people.
- `/snu/openpose/bbox`: Int16MultiArray. This topic contains the coordinate of every bounding box that the module detects.
data ← [*box_0_top_left_x, box_0_top_left_y, box_0_bottom_right_x, box_0_bottom_right_y, … , box_N_top_left_x, box_N_top_left_y, box_N_bottom_right_x, box_N_bottom_right_y*]

### Subscribed Topics

- `/hsrb/head_rgbd_sensor/rgb/image_rect_color`: Image. A rectified color image from the head.