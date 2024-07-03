import os
import sys
sys.path.append('/home/tidy/Robocup2024/hsr_agent')
from global_config_utils import make_object_list

is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

# data topic name.
RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

# 기존 경로 : 'weight/best_240409.pt'
# YOLO weight 변경 시 경로 변경
yolo_weight_path = 'weight/best_0704.pt'
yolo_classnames_path = 'weight/best_0704.cn'

try:
    OBJECT_LIST = make_object_list(yolo_classnames_path)
except:
    pass

FINAL = True


if FINAL:
    print('[GLOBAL CONFIG] FINAL mode')
    # real robot
    ABS_POSITION = {

        # CTT
        'target_table' : [6.6449, 0.3005, 0.0422],
        'dishwasher': [2.6256, -1.7107, 3.0623],

    }


    TABLE_DIMENSION = {
        # width, depth, height
        'target_table' : [0.55, 0.75, 0.730],
        'dishwasher' : [0.65, 0.75, 0.595],


        # clean the table
        'dishwasher_rack' : [0.520,0.60,0.400],
        'dishwasher' : [0.520,0.60,0.830] #height is not precise
        
    }

    OBJECT_LIST = [
        # name, item_id, itemtype, grasping_type[front:0, top:1, bowl:2, plate:3]  2: 'spoon', 3: 'fork', 4: 'plate', 5: 'bowl', 0: 'mug', 1: 'knife', 
        ['cracker', 0, 5, 0],
        ['sugar', 1, 2, 0],
        ['jello_red', 2, 2, 0],
        ['jello_black', 3, 2, 0],
        ['coffee_can', 4, 2, 0],
        ['tuna_can', 5, 2, 0],
        ['pringles', 6, 5, 0],
        ['mustard', 7, 2, 0],
        ['tomato_soup', 8, 2, 0],
        ['pear', 9, 3, 0],
        ['peach', 10, 3, 0],
        ['apple', 11, 3, 0],
        ['strawberry', 12, 3, 0],
        ['orange', 13, 3, 0],
        ['banana', 14, 3, 0],
        ['plum', 15, 3, 0],
        ['lemon', 16, 3, 0],
        ['bowl', 17, 6, 2],
        ['mug', 18, 6, 0],
        ['plate', 19, 6, 3],
        ['knife', 20, 6, 1],
        ['fork', 21, 6, 1],
        ['spoon', 22, 6, 1],
        ['tennis_ball', 23, 4, 0],
        ['golf_ball', 24, 4, 0],
        ['base_ball', 25, 4, 0],
        ['soccer_ball', 26, 4, 0],
        ['soft_ball', 27, 4, 0],
        ['cube', 28, 4, 0],
        ['dice', 29, 4, 0],
        ['wine', 30, 1, 0],
        ['ice_tea', 31, 1, 0],
        ['orange_juice', 32, 1, 0],
        ['milk', 33, 1, 0],
        ['tropical_juice', 34, 1, 0],
        ['juice_pack', 35, 1, 0],
        ['cereal_red', 36, 1, 0],
        ['cereal_yellow', 37, 1, 0],
        ['coke', 38, 1, 0],
        ['sponge', 39, 0, 1],
        ['scrub', 40, 0, 0],
        ['spam', 41, 2, 0],
        ['shopping_bag_1', 42, 7, 0],
        ['shopping_bag_2', 43, 7, 0],
        ['shopping_bag_3', 44, 7, 0],
        ['cereal_black', 45, 7, 0],
        ['dishwasher_tablet', 46, 7, 0],
    ]

    OBJECT_TYPES = [
        "cleaning",  # 0
        "drink",  # 1
        "food",  # 2
        "fruit",  # 3
        "toy",  # 4
        "snack",  # 5
        "dish",  # 6
        "bag",  # 7
    ]

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
