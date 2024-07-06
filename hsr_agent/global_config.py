import os
import sys
sys.path.append('/home/tidy/Robocup2024/hsr_agent')
sys.path.append('/home/tidy/Robocup2024/')
from global_config_utils import make_object_list

is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

is_yolov10 = True

# data topic name.
RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

if is_yolov10==False:
    yolo_weight_path = 'weight/best_0704.pt'
    # yolo_weight_path = 'weight/best_for_final_not_tiny.pt'
    yolo_classnames_path = 'weight/best_0704.cn'


if is_yolov10:
#     yolo_weight_path = 'weight/pnu_final.pt'
#     yolo_classnames_path = 'weight/pnu_final.cn'
    yolo_weight_path = 'weight/collectedbysnu_240704and05.pt'
    yolo_classnames_path = 'weight/collectedbysnu_240704and05.cn'


try:
        OBJECT_LIST = make_object_list(yolo_classnames_path, is_yolov10)
except:
    print('Error: Cannot load object list')
    pass

PNU = True


if PNU:
    print('[GLOBAL CONFIG] PNU mode')

    ABS_POSITION = {
        # CTT
        'pos_target_table' : [5.2366, -1.3278, 0.0333],
        'pos_dishwasher': [5.703, -2.7454, -1.5737]

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
