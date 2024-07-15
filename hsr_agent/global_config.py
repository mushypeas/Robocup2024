import os
import sys
sys.path.append('hsr_agent')
from global_config_utils import make_object_list

is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

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

    '원탁': [0.89, 0.89, 0.735],
    '식탁용식기세척기': [0.595, 0.595, 0.845],
    
    # Storing Groceries
    'grocery_table': [0.736, 1.10, 0.73],
    'grocery_shelf': [
        [0.76, 0.36, 0],
        # [0.76, 0.36, 0.44],
        [0.76, 0.36, 0.804],
        [0.76, 0.36, 1.174],
    ],
}

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

HEAVY_OBJECTS = ['cheezit']

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


if is_sim: # sim mode
    print('[GLOBAL CONFIG] Sim mode. WARNING!!!!!!!!!!')
    PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

    # gazebo
    ABS_POSITION = {
        'arena_out': [-2.45, 5.1, -1.5708],
        'zero': [0.0, 0.0, 0.0],
        'start_1': [-1.0916, 2.541, -1.487],
        'dev_front': [-1.018, 0.190, -3.061],
        'breakfast_table': [-1.3, 0.65, -3.1416],
        'table_front': [0.2, 0.0, 0.0],
        'table_side': [1.5, -1.00, 1.5708],
        'shelf_front': [7.245, 1.051, 0],
        'sofa_view': [-0.008, -0.043, -0.801],
        'door_handle': [-2.327, 3.75, 1.61],
        'handle_front': [0.311, -0.009, 0.016],
        'dishwasher_gpsr': [6.531, 0.946, 1.57],
        'side_table': [3.291, 0.7065, 1.57],
        'grocery_table': [3.291, 0.7065, 1.57],  # dist = 0.9
        'kitchen_table': [1.5916, -2.7794, 0.0313], # dist = 0.6 /mjgu
        'kitchen_table_ready' : [1.0016, -2.7794, 0.0313], # dist = 2.0 /mjgu
        'pantry': [7.245, 1.051, 0],  # dist = 0.9
        'desk': [2.52, -3.815, -1.57],  # dist = 0.9
        'gpsr_start': [1.3372, 1.1288, 0.5792],
        'shelf': [5.4, -4.93, -1.57],  # dist = 0.9
        'bedside_table': [7.615, -4.56, 0],  # dist = 0.6
        'sink': [6.1375, -1.91, 3.14],  # dist = 0.6
        'cabinet': [3.45, -4.44, 0]  # dist = 0.6
    }
    
    TINY_OBJECTS = ['spoon', 'fork', 'knife']

    TABLE_DIMENSION = {
        # width, depth, height
        'kitchen_table': [0.8, 0.8, 0.72],
        'desk': [1.505, 0.705, 0.8],
        'side table': [0.495, 0.495, 0.395],
        'sink': [0.515, 0.455, 0.945],
        'dishwasher_gpsr': [0.6, 0.6, 0.84],
        'cabinet': [0.9, 0.46, 0.66],
        'shelf': [0.26, 0.36, 0.445],
        'shelf_1f': [0.26, 0.36, 0.445],
        'shelf_2f': [0.26, 0.36, 0.735],
        'shelf_3f': [0.26, 0.36, 1.06],
        'shelf_4f': [0.26, 0.36, 1.4],
        'storage_rack': [0.445, 0.9, 0.42],
        'storage_rack_1f': [0.445, 0.9, 0.42],
        'storage_rack_2f': [0.445, 0.9, 0.88],
        'storage_rack_3f': [0.445, 0.9, 1.34],
        'pantry': [0.76, 0.26, 0.45],
        'pantry_1f': [0.76, 0.26, 0.45],
        'pantry_2f': [0.76, 0.26, 0.74],
        'pantry_3f': [0.76, 0.26, 1.06],
        'pantry_4f': [0.76, 0.26, 1.405],
        'bedside_table': [0.5, 0.45, 0.575],
        'breakfast_table': [0.6, 0.4, 0.625],
        'grocery_table': [0.495, 0.495, 0.40],
        'grocery_table_pose': [0.495, 0.495, 0.42],
        'grocery_table_pose1': [0.495, 0.495, 0.45],
        'grocery_table_pose2': [0.495, 0.495, 0.46],
        'dishwasher_handle': [0.60, 0.60, 0.84],  # [0.6, 0.4, 0.8]
        'door_handle': [0, 0, 0.96],
    }

    LOCATION_MAP = {
        "bedroom": ['bed', 'bedside_table', 'shelf'],
        "kitchen": ['pantry', 'trashbin', 'dishwasher', 'potted_plant', 'kitchen_table', 'chairs',
                    'refrigerator', 'sink'],
        "study": ['cabinet', 'coatrack', 'desk', 'armchair', 'desk_lamp', 'waste_basket', 'exit'],
        "living room": ['tv_stand', 'storage_rack', 'lamp', 'side_tables', 'side_table', 'sofa', 'entrance']
    }
    
    # TODO AREA_EDGES FOR sim_mode must be updated
    ARENA_EDGES = [[1.167, -0.2321], [7.0443, -0.1863], [7.015, 2.5457], [8.2162, 2.6681], [8.2485, 6.0065], \
                   [5.6399, 5.8781], [5.5177, 3.6958], [4.7759, 3.6698], [4.7012, 2.4829], [0.9986, 2.0893]]
    

