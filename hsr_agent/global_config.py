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
    yolo_classnames_path = 'weight/best_0704.cn'


if is_yolov10:
    yolo_weight_path = 'weight/0715v2.pt'
    # yolo_weight_path = 'weight/0716v3.pt' (실험 시도 가능 )
    yolo_classnames_path = 'weight/classnames.cn'


try:
    print(is_yolov10)
    OBJECT_LIST = make_object_list(yolo_classnames_path, is_yolov10)
    
except:
    print('Error: Cannot load object list')
    pass

Eindhoven = True


if Eindhoven:
    print('[GLOBAL CONFIG] Eindhoven mode')

    ABS_POSITION = {
        'pos_dining_table_1' :  [1,1,1],
        'pos_dining_table_2' :  [1,1,1],
        'pos_dining_table_3' :  [1,1,1],
        'pos_dining_table_4' :  [1,1,1],
        'pos_dining_table_5' :  [7.8103, 3.2688, 3.1361],
        'pos_dining_table_6' :  [1,1,1],
        'pos_dining_table_7' :  [1,1,1],

        # 'pos_office_table_1' : [1,1,1],
        # 'pos_office_table_2' : [1,1,1],
        # 'pos_office_table_3' : [1,1,1],
        # 'pos_office_table_4' : [1,1,1],
        # 'pos_office_table_5' : [1,1,1],

        # 'pos_livingroom_table_1' : [1,1,1],
        # 'pos_livingroom_table_2' : [1,1,1],
        # 'pos_livingroom_table_3' : [1,1,1],
        # 'pos_livingroom_table_4' : [1,1,1],
        # 'pos_livingroom_table_5' : [1,1,1],

        'pos_dishwasher': [8.1414, 3.6023, -0.0372]
        

    }


    TABLE_DIMENSION = {
        # width, depth, height
        'tab_dining_table': [0.78, 2.00, 0.770] ,
        # 'tab_office_table': [0.897, 2.02, 0.75],
        # 'tab_livingroom_table': [0.905, 0.905, 0.48],


        'tab_dishwasher' : [0.600, 0.610, 0.340], # 병주가 직접 둘째날 아침에 잰거임. 아주 정확하지 않을수도 있음.
        
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
