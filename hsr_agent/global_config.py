import os
import sys
sys.path.append('hsr_agent')
from global_config_utils import make_object_list

is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

# data topic name.
RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

# 기존 경로 : 'weight/best_240409.pt'
# YOLO weight 변경 시 경로 변경
yolo_weight_path = 'weight/best_for_final_tiny_added.pt'
yolo_classnames_path = 'weight/best_for_final.cn'

try:
    OBJECT_LIST = make_object_list(yolo_classnames_path)
except:
    pass

AIIS = False
FINAL = False
PNU = True # 240630 mjgu 추가

if PNU: # 240630 mjgu 추가
    print('[GLOBAL CONFIG] PNU mode')
    # real robot
    ABS_POSITION_PNU = {

        #insepction
        'inspection': [4.8333, 2.7961, 1.6308],

        # storing grocery
        '쉘프': [1.9958, -0.7921, -3.1012], # 영어로 고치기 (feat.영재선배님)
        '원탁앞90센치': [2.6643, -1.1615, -0.0232], # 영어로 고치기 (feat.영재선배님)

        #clean the table
        '원탁앞60센치' : [2.9768, -1.1441, -0.0054], # 영어로 고치기 (feat.영재선배님)
        '식기세척기앞60센치' : [2.5437, -1.4757, -3.1153], # 영어로 고치기 (feat.영재선배님)

        # serve breakfast
        'initial_location_pnu' :[0.0053, 0.0243, -0.0228], # (세면대 방 출발선) 240630 mjgu 추가
        'picking_location_pnu': [4.3083, -1.5883, 0.0044], # (kitchen_table 근처) 240630 mjgu 추가
        'kitchen_table_pnu' : [6.5185, -2.7109, -1.536], # 240630 mjgu 추가
        'breakfast_table_pnu': [5.3689, -1.3234, 0.0665], # 240630 mjgu 추가

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
        'breakfast_table_pnu': [0.735, 1.18, 0.73], # (나무색) 240630 mjgu 추가
        'kitchen_table_pnu': [0.4, 0.802, 0.602], #(흰색) 240630 mjgu 추가
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


if FINAL:
    print('[GLOBAL CONFIG] FINAL mode')
    # real robot
    ABS_POSITION = {

        #insepction
        'inspection': [4.8333, 2.7961, 1.6308],

        # storing grocery
        '쉘프': [1.9958, -0.7921, -3.1012], # 영어로 고치기 (feat.영재선배님)
        '원탁앞90센치': [2.6643, -1.1615, -0.0232], # 영어로 고치기 (feat.영재선배님)

        #clean the table
        '원탁앞60센치' : [2.9768, -1.1441, -0.0054], # 영어로 고치기 (feat.영재선배님)
        '식기세척기앞60센치' : [2.5437, -1.4757, -3.1153], # 영어로 고치기 (feat.영재선배님)

        # serve breakfast
        'kitchen_table' : [3.0768, -1.1441, -0.0054], #2.9286, -1.1145, 0.0052 (가까움)
        'breakfast_table': [2.4481, -1.4913, -3.0755],

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

        '원탁': [0.89, 0.89, 0.735], # 영어로 고치기 (feat.영재선배님)
        '식탁용식기세척기': [0.595, 0.595, 0.845], # 영어로 고치기 (feat.영재선배님)
        'breakfast_table': [0.595, 0.595, 0.845],
        'kitchen_table': [0.89, 0.89, 0.735],
        
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

if AIIS:
    print('[GLOBAL CONFIG] AIIS mode')
    # real robot
    ABS_POSITION = {
        'insp_target': [6.2869, 3.5307, 0],
        'arena_out': [-2.487, 5.65, -1.561],
        'zero': [0.0, 0.0, 0.0],
        'dev_front': [-1.018, 0.190, -3.061],
        'table_front': [6.6449, 0.3005, 0.0422],
        'table_side': [7.3455, -1.0624, 1.551],
        'table_back': [8.6478, 0.0623, 3.1025],
        'sofa_view': [-0.008, -0.043, -0.801],
        'door_handle': [1.4227, 1.1802, -1.5106],


        # storing grocery
        'grocery_table': [-0.7854, -0.2033, -1.548],
        'grocery_shelf': [-1.091, -0.1197, 1.6165],

        # serve breakfast
        'breakfast_table': [5.1527, 0.2285, 0.0], # mjgu 240530
        'kitchen_table' : [6.086, -1.1229, 0.0], # mjgu 240530
        # 필요할 경우 우회 지점 설정 -> 'breakfast_table_bypass_testday' : [1.7554, 0.9174, 3.1374], #mjgu 240505

        # clean the table

        'dishwasher_front': [2.6256, -1.7107, 3.0623], #bjkim2 0505
        'clean_table_front' : [5.2608, 0.2969, -0.0126], #bjkim2 0505 # HEIGHT SHOULD BE REALLLLLLY PRECISE
        'rack_close_position1': [2.0321, -0.9574, -1.5822], #bjkim 0512
        'rack_close_position2': [1.6463, -0.9664, -1.5655],
        'rack_close_position3': [1.6434, -0.9569, -1.9500],


        # receptionist
        # 'cloth_scan': [1.7869, 0.0919, -3.1073],  # [2.5404, 0.3225, -3.1168] near door
        # 'cloth_scan' : [3.1471, 0.1392, -3.0298], # 0505
        'cloth_scan': [5.3575, -0.8901, -1.5689], # 0531
        # 'cloth_scan' : [1.0330, -2.2140, -1.4835], # AIIS
        # 'handle_front': [1.5502, 0.0104, -3.1301],
        # 'door_bypass': [2.507, 0.1598, 1.535],
        # 'seat_scan': [2.4192, 0.2234, 1.576], #[2.5198, 0.0942, 1.5773],
        # 'seat_scan': [1.3810, 2.2950, 0.0445], # AIIS
        # 'seat_scan' : [7.195, -0.8567, -0.9396], # 0505
        'seat_scan' : [7.1866, -0.8539, -0.8736], # 0514
        # 'seat_scan_bypass': [1.8008, 0.0949, -2.2551],  # [7.4416, 4.5881, -1.5132] far bypass
        'seat_scan_bypass': [7.1009, -0.8733, 2.8402], # 0505
        # 'start': [2.0208, -1.3355, 2.3405],
        # 'start': [1.7869, 0.0919, -3.1073], # AIIS-safe-cloth
        # 'start' : [3.1471, 0.1392, -3.0298], # 0505\
        'start': [5.3575, -0.8901, -1.5689], # 0531
        # 'start': [-1.7020, -1.3990, -3.0880], # AIIS
        # 'start_receptionist': [-1.7020, -1.3990, -3.0880], # AIIS

        # stickler
        # 'stickler_search': [5.7458, 1.4637, 1.5781],
        # 'forbidden_scan': [3.998, 1.592, 3.139],
        # 'stickler_forbidden_room_front': [2.4505, 1.65, -3.0966],
        # 'bar': [7.5292, 3.547, 1.6503],
        # 'bin': [1.7214, 0.7673, 1.1897],
        # 'no_littering_search1': [4.7126, 1.3649, -3.0343],
        # 'no_littering_search2': [5.5347, 1.4413, 1.2616],
        # 'no_littering_search3': [7.4141, 3.0249, 1.666],

        #0707 1957 global config for stickler - lsh
        #0411 stickler config-1

        # AIIS
        # 'bedroom_search': [-2.5427, 0.216, 1.2345],
        # 'kitchen_search': [0.6884, -1.0065, -0.7347],
        # 'living_room_search': [0.6534, -0.6374, 0.9569],
        # 'study_search': [0.027, 0.3138, -2.1569],

        # 0505
        'kitchen_search': [3.2691, 0.3223, -2.1086],
        'living_room_search': [5.932, -0.357, -0.4455],
        'study_search': [5.2668, 1.273, 2.5436],
        'bedroom_search': [6.9826, 3.0422, -0.6487],


        # AIIS
        # 'shoe_warning': [1.0093, -2.4509, -1.5534],
        # 'bin_littering': [1.846, -2.4151, -1.5907],
        # 'bar_drink': [0.945, -1.2866, -0.0441],
        # 'bedroom_doublecheck' : [-2.5427, 0.216, 1.2345],
        # 'study_search_reverse': [-0.0189, 0.2843, 0.0333],
        # 'bedroom_search_reverse': [-2.5306, 0.1937, -0.3405],

        # 0505
        'shoe_warning': [3.1471, 0.1392, -3.0298],
        'bin_littering': [2.6641, -1.6283, 3.1113],
        'bar_drink': [2.4819, -2.6883, 0.6981],
        #'bedroom_doublecheck' : [6.4953, 3.4738, -0.6583],
        'bedroom_doublecheck' : [6.9826, 3.0422, -0.6487],
        # 'study_search_reverse': [-0.0189, 0.2843, 0.0333],
        'bedroom_search_reverse': [6.4518, 3.4936, 3.1033], #수정필요


        # gpsr
        'kitchen_table': [2.1348, -2.7771, -0.0066], 
        'taxi': [6.2415, 3.3874, 1.5591],
        'side_table': [2.5619, -0.0344, 1.5821],
        'side_tables': [2.5619, -0.0344, 1.5821],
        'sofa': [3.7381, 0.6181, 2.3207],
        'storage_rack': [3.5983, -1.0546, -0.0079],
        # 'desk': [1.6335, -4.7401, -0.0524],
        'cabinet': [4.0185, -4.9778, 0.0594],
        'shelf': [5.8276, -5.2125, -1.5813],
        'bedside_table': [8.344, -4.7478, -0.0446],
        'sink': [6.2943, -1.3015, 3.065],
        'lamp': [3.7726, 0.7358, 1.0435],
        'bookshelf': [1.6211, 1.1404, 3.047],
        'tv_stand': [1.7771, -1.0172, -1.6089],
        'coatrack': [2.0199, -5.1707, -2.2595],
        'dishwasher_gpsr': [9.0631, -0.1084, -1.5204],
        'refrigerator': [7.142, -2.1261, 3.0864],
        'bed': [7.7814, -4.4426, -1.0038],
        'armchair': [1.8318, -5.814, 1.517],
        'desk_lamp': [2.9492, -5.8113, 1.5511],
        'waste_basket': [2.9492, -5.8113, 1.5511],
        'entrance': [1.0214, 0.0173, -0.0081],
        'exit': [1.7556, -3.837, -3.1345],
        'trashbin': [7.6864, -1.8229, -0.8876],
        'trash_bin': [7.6864, -1.8229, -0.8876],
        'chairs': [8.0663, -0.327, 1.5792],  # TODO: kitchen_table
        'pantry': [6.7433, 1.2517, 1.5273],
        'kitchen': [5.9569, 0.0252, 0.1178],
        'bedroom': [7.1038, -3.5726, -1.5305],
        'bed_room': [7.1038, -3.5726, -1.5305],
        'study': [3.0395, -3.331, -1.5554],
        'study_room': [3.0395, -3.331, -1.5554],
        'living_room': [2, 0, 0],


        # final
        'final_kitchen_table' : [7.0491, -0.1961, 0.0064],
        'final_round_table' : [6.9514, -1.5522, -1.5492],
    }


    TABLE_DIMENSION = {
        # width, depth, height

        'kitchen_table': [0.89, 0.89, 0.735],
        'breakfast_table': [0.595, 0.595, 0.845],
        'grocery_table': [0.65, 1.2, 0.42],
        'grocery_table_pose': [0.55, 0.75, 0.42 + 0.055], # +055
        'grocery_table_pose1': [0.55, 0.75, 0.42 + 0.065], # +065
        'grocery_table_pose2': [0.55, 0.75, 0.42 - 0.02], # -020

        'door_handle': [0, 0, 0.96],
        # width, depth, height
        
        # clean the table
        'clean_table' : [0.55,0.75,0.715],
        'dishwasher_door' : [0.595,0.60,0.165], #height is not precise
        'dishwasher_rack' : [0.520,0.60,0.400],
        'dishwasher' : [0.520,0.60,0.830], #height is not precise

        'desk': [1.505, 0.705, 0.8],
        'side_table': [0.495, 0.495, 0.395],
        'side-tables': [0.495, 0.495, 0.395],
        'sink': [0.515, 0.455, 0.945],
        'dishwasher': [0.65, 0.75, 0.595],
        'dishwasher_table': [0.65, 0.75, 0.42],
        'cabinet': [0.9, 0.46, 0.66],
        'grocery_shelf_1f': [0.765, 0.357, 0.805],
        'grocery_shelf_2f': [0.765, 0.357, 1.11],
        'grocery_shelf_3f': [0.765, 0.357, 1.4],
        'grocery_shelf_4f': [0.765, 0.357, 1.4],
        'storage_rack': [0.445, 0.9, 0.42],
        'storage_rack_1f': [0.445, 0.9, 0.42],
        'storage_rack_2f': [0.445, 0.9, 0.88],
        'storage_rack_3f': [0.445, 0.9, 1.34],
        'pantry': [0.76, 0.26, 0.45],
        'pantry_1f': [0.76, 0.26, 0.45],
        'pantry_2f': [0.76, 0.26, 0.74],
        'pantry_3f': [0.76, 0.26, 1.06],
        'pantry_4f': [0.76, 0.26, 1.405],
        'bedside_table': [0.5, 0.45, 0.58],

        # clean the table
        'dishwasher_handle': [0.65, 0.75, 0.60], #bjkim, but not modified, still France version

        # final
        'final_kitchen_table': [1.505, 0.705, 0.8],
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

elif is_sim: # sim mode
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
    

