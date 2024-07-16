import os
import sys
sys.path.append('hsr_agent')
from global_config_utils import make_object_list

RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

# YOLO weight 변경 시 경로 변경
yolo_weight_path = 'weight/best_for_final_not_tiny.pt'
yolo_classnames_path = 'weight/best_for_final.cn'

try:
    OBJECT_LIST = make_object_list(yolo_classnames_path)
    print('[GLOBAL CONFIG] OBJECT_LIST loaded')
except:
    print('[GLOBAL CONFIG] OBJECT_LIST NOT loaded')
    pass


ABS_POSITION = {

    #insepction
    'inspection': [4.8333, 2.7961, 1.6308],

    # storing grocery
    'grocery_table': [4.9945, -1.2884, 0.0201],
    'grocery_shelf': [5.3142, -0.125, 1.5797],
    'grocery_shelf_door': [5.4142, -0.125, 1.5797],

    #clean the table
    '원탁앞60센치' : [2.9768, -1.1441, -0.0054],
    '식기세척기앞60센치' : [2.5437, -1.4757, -3.1153],

    # serve breakfast
    '원탁앞60센치' : [2.9768, -1.1441, -0.0054],
    '식기세척기닫힘60센치': [1.9481, -1.4913, -3.0755],

    # recptionist
    'start': [4.7578, -1.6402, -1.5496],
    'cloth_scan': [4.7578, -1.6402, -1.5496],
    'seat_scan' : [6.7194, -0.3494, -0.7164],
    'seat_scan_bypass': [6.7071, -0.3502, -3.038],

    # stickler for the rules
    'kitchen_search': [3.3146, 0.4319, -2.2959],
    'living_room_search': [4.9844, 0.2595, -0.8542],
    'study_search': [5.1834, 1.9487, 2.7205],
    'bedroom_search': [6.7134, 3.401, -0.6504],
    'shoe_warning': [5.6829, -2.9312, 2.2687],
    'bin_littering': [1.9497, -1.9686, 1.8865],
    'bar_drink': [3.1751, -2.4041, 1.2635],
    'bedroom_doublecheck' : [6.7134, 3.401, -0.6504],
    'bedroom_search_reverse': [5.2946, 3.5653, -2.3053],
}


TABLE_DIMENSION = {
    # width, depth, height
    'grocery_table': [0.738, 2.00, 0.772],
    'grocery_shelf': [
        [0.792, 0.285, 0],
        [0.792, 0.285, 0.413],
        [0.792, 0.285, 0.734],
        [0.792, 0.285, 1.058],
    ],
}

OBJECT_TYPES = [
    "cleaning_supply",  # 0
    "drink",  # 1
    "food",  # 2
    "fruit",  # 3
    "decoration",  # 4
    "snack",  # 5
    "dish",  # 6
    "unknown",  # 7
]

TINY_OBJECTS = ['strawberry', 'candle']

HEAVY_OBJECTS = ['coke_big']
