import os
import sys
sys.path.append('../../hsr_agent')
sys.path.append('hsr_agent')
sys.path.append('../../hsr_agent')
from global_config_utils import make_object_list

RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

# YOLO weight 변경 시 경로 변경
yolo_weight_path = 'weight/0716v3.pt'
yolo_classnames_path = 'weight/classnames.cn'

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
    'living_living_1': [3.8317, -0.2223, 1.5071],
    'living_living_2': [3.7858, 2.9424, 0.111],
    'grocery_table': [5.6348, 3.881, -0.0264],
    'grocery_shelf': [5.6019, 4.9918, 3.127],
    'grocery_shelf_door': [5.7569, 5.07, 3.13],

    #clean the table
    'hallway' : [3.8827, -0.5537, -0.0261],
    'livingroom' : [8.9285, 0.6183, 1.5787],
    '원탁앞60센치' : [2.9768, -1.1441, -0.0054],
    '식기세척기앞60센치' : [2.5437, -1.4757, -3.1153],

    # serve breakfast
    '원탁앞60센치' : [2.9768, -1.1441, -0.0054],
    '식기세척기닫힘60센치': [1.9481, -1.4913, -3.0755],

    # recptionist
    'start': [2.6694, 0.0088, 3.1294],
    'cloth_scan': [2.6694, 0.0088, 3.1294],
    'seat_scan': [7.2618, -1.1397, -0.0927],
    'seat_scan_bypass': [7.0752, -1.0906, 2.8539],

    # stickler for the rules
    'office_search': [3.9124, 1.8329, 2.3017],
    'office_search2': [3.9013, 3.7423, -1.5981], # leave
    'kitchen_search': [4.7953, 3.7805, 0.0278],
    'kitchen_search2': [8.0138, 1.5989, 1.5747],
    'livingroom_search': [6.8303, 0.4463, -0.8153],
    'hallway_search': [3.8517, 0.6339, -2.3486],
    'office_leave1': [3.7782, 2.8632, 0.0525],
    'office_leave2': [4.7751, 2.9022, 0.0008],

    'bin_littering': [5.346, 1.4374, 2.3295],
    'bar_drink': [5.3231, 5.5267, -1.7753], # currently cabinet
    'shoe_warning': [2.0195, -0.7274, 1.6346],

    'kitchen_living_middle': [8.6032, 0.9316, 3.0649],
    'livingroom_leave': [4.9864, -0.4961, -3.1398],
    'hallway_enter': [3.6486, -0.5246, 1.5538],
}


TABLE_DIMENSION = {
    # width, depth, height
    'grocery_table': [1.00, 0.738, 0.78],
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
