import os
import sys
sys.path.append('/home/tidy/Robocup2024/hsr_agent')
sys.path.append('/home/tidy/Robocup2024/')
from global_config_utils import make_object_list

is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

# data topic name.
RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

# 기존 경로 : 'weight/best_240409.pt'
# YOLO weight 변경 시 경로 변경
# yolo_weight_path = 'weight/best_0704.pt'
yolo_weight_path = 'weight/best_for_final_not_tiny.pt'
# yolo_classnames_path = 'weight/best_for_final.cn'
yolo_classnames_path = 'weight/YOLOV10-M-SNU-0703.cn'

try:
    import pdb;
    OBJECT_LIST = make_object_list(yolo_classnames_path)
    pdb.set_trace()
except:
    print('Error: Cannot load object list')
    pass

FINAL = True


if FINAL:
    print('[GLOBAL CONFIG] FINAL mode')
    # real robot
    ABS_POSITION = {

        # CTT
        'pos_target_table' : [5.2366, -1.3278, 0.0333],
        'pos_dishwasher': [5.703, -2.9454, -1.5737]

    }


    TABLE_DIMENSION = {
        # width, depth, height
        'tab_target_table': [1.18,0.735,0.73],
        'tab_dishwasher' : [0.41,0.32,0.12],
        
    }

    TINY_OBJECTS = ['spoon', 'fork', 'knife']

    # added by lsh
    ARENA_EDGES = [[0.611, 2.440], [9.101, 2.457], [9.473, 1.872], [9.425, -6.256], [0.878, -6.291]]

    # added by sujin
    # for gpsr
    LOCATION_MAP = {
        "bedroom": ['bed', 'bedside_table', 'shelf'],
        "kitchen": ['pantry', 'trashbin', 'dishwasher', 'potted_plant', 'kitchen_table', 'chairs',
                    'refrigerator', 'sink'],
        "study": ['cabinet', 'coatrack', 'desk', 'armchair', 'desk_lamp', 'waste_basket', 'exit'],
        "living_room": ['tv_stand', 'storage_rack', 'lamp', 'side_tables', 'side_table', 'sofa', 'entrance']
    }
