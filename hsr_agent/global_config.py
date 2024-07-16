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
    yolo_weight_path = 'weight/0715v2.pt'
    yolo_classnames_path = 'weight/0715v2.cn'


try:
        OBJECT_LIST = make_object_list(yolo_classnames_path, is_yolov10)
except:
    print('Error: Cannot load object list')
    pass

Eindhoven = True


if Eindhoven:
    print('[GLOBAL CONFIG] Eindhoven mode')

    ABS_POSITION = {
        # CTT
        # 'pos_target_table' : [7.3643, 3.7794, 3.131], # 병주가 직접땀. 재문파트가 이쌍했음.
        # 'pos_target_table' : [6.3748, 2.6865, 1.5975],# 병주가 직접땀. 재문파트가 이쌍했음.
        'pos_target_table' : [5.3226, 3.6826, -0.0141], # 병주가 직접땀. 재문파트가 이쌍했음. 3번 position
        'pos_dishwasher': [7.5986, 4.116, 0.0285] # 병주가 직접땀. 재문파트가 이쌍했음.

    }


    TABLE_DIMENSION = {
        # width, depth, height
        'tab_target_table': [0.78, 2.00, 0.785] ,
        'tab_dishwasher' : [0.600, 0.610, 0.340] # 병주가 직접 둘째날 아침에 잰거임. 아주 정확하지 않을수도 있음.
        
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
