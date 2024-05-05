import os
is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

# data topic name.
RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'


AIIS = True

if AIIS:
    print('[GLOBAL CONFIG] AIIS mode')
    # real robot
    ABS_POSITION = {
        'insp_target': [6.2869, 3.5307, 0],
        'arena_out': [-2.487, 5.65, -1.561],
        'zero': [0.0, 0.0, 0.0],
        'dev_front': [-1.018, 0.190, -3.061],
        'breakfast_table': [6.4946, 1.2412, -3.1284],
        'table_front': [6.6449, 0.3005, 0.0422],
        'table_side': [7.3455, -1.0624, 1.551],
        'table_back': [8.6478, 0.0623, 3.1025],
        'sofa_view': [-0.008, -0.043, -0.801],
        'door_handle': [1.4227, 1.1802, -1.5106],


        # storing grocery
        'grocery_table': [3.7086, -0.1316 -1.58], #bjkim
        'shelf_front': [2.4902, -0.8467, -3.14], #bjkim

        # serve breakfast
        'breakfast_table_testday' : [6.3927, -1.0289, -0.0093], #mjgu 240505
        'testday_breakfast_table_nearby' : [1.3235, -0.0452, 1.5803],
        # 필요할 경우, (의자 등) 'breakfast_table_bypass_testday' : [1.7554, 0.9174, 3.1374], #mjgu 240505
        'kitchen_table_testday' : [5.0999, 0.5681, 0.0124], #mjgu 240505

        'initial_position' : [0.9951, -3.7974, 1.5979], # mjgu 240504 
        'breakfast_table': [1.7478, -1.2796, 0.0], #mjgu
        'kitchen_table_front_far' : [0.9951, -2.7733, 0.0088], #mjgu 240504
        'kitchen_table_front' : [1.7258, -2.8179, 0.0302], #mjgu 240504
        'breakfast_table_front': [1.4045, -1.3402, 0.0091], #bjkim2 [1.6, -1.2796, 0.0]

        # clean the table
        'dishwasher': [2.0148, 0.8449, 1.6256], #bjkim2
        'dishwasher_rack': [3.4552, -0.6194, -1.5674], #bjkim2
        'opened_dishwasher': [2.0148, 0.8449, 1.6256], #bjkim2
        'clean_table_front' : [6.3029, -1.0222, 0.0161], #bjkim2

        # receptionist
        # 'cloth_scan': [1.7869, 0.0919, -3.1073],  # [2.5404, 0.3225, -3.1168] near door
        'cloth_scan' : [3.1471, 0.1392, -3.0298], # 0505
        # 'cloth_scan' : [1.0330, -2.2140, -1.4835], # AIIS
        # 'handle_front': [1.5502, 0.0104, -3.1301],
        # 'door_bypass': [2.507, 0.1598, 1.535],
        # 'seat_scan': [2.4192, 0.2234, 1.576], #[2.5198, 0.0942, 1.5773],
        # 'seat_scan': [1.3810, 2.2950, 0.0445], # AIIS
        'seat_scan' : [7.195, -0.8567, -0.9396], # 0505
        # 'seat_scan_bypass': [1.8008, 0.0949, -2.2551],  # [7.4416, 4.5881, -1.5132] far bypass
        'seat_scan_bypass': [7.1009, -0.8733, 2.8402], # 0505
        # 'start': [2.0208, -1.3355, 2.3405],
        # 'start': [1.7869, 0.0919, -3.1073], # AIIS-safe-cloth
        'start' : [3.1471, 0.1392, -3.0298], # 0505
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
        'bedroom_search': [6.4953, 3.4738, -0.6583],

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
        'bedroom_doublecheck' : [6.4953, 3.4738, -0.6583],
        # 'study_search_reverse': [-0.0189, 0.2843, 0.0333],
        'bedroom_search_reverse': [6.4518, 3.4936, 3.1033],


        # gpsr
        'kitchen_table': [2.1348, -2.7771, -0.0066], #mjgu
        'taxi': [6.2415, 3.3874, 1.5591],
        'gpsr_start': [2, 0, 0],
        'side_table': [2.5619, -0.0344, 1.5821],
        'side_tables': [2.5619, -0.0344, 1.5821],
        'sofa': [3.7381, 0.6181, 2.3207],
        'storage_rack': [3.5983, -1.0546, -0.0079],
        'desk': [1.6335, -4.7401, -0.0524],
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
        # testday용 추가 - mjgu 240504
        'kitchen_table_testday': [0.8, 0.8, 0.715],
        'breakfast_table_testday': [0.6, 0.4, 0.625],        
        # width, depth, height
        'kitchen_table': [0.55, 0.75, 0.735],
        'breakfast_table': [0.55, 0.75, 0.715], #이거 715인데 왜 테이프에 735라 써놧어
        'grocery_table': [0.55, 0.75, 0.730],
        'grocery_table_pose': [0.55, 0.75, 0.785], # +055
        'grocery_table_pose1': [0.55, 0.75, 0.795], # +065
        'grocery_table_pose2': [0.55, 0.75, 0.70], # +030
        'door_handle': [0, 0, 0.96],
        # width, depth, height
        'desk': [1.505, 0.705, 0.8],
        'side_table': [0.495, 0.495, 0.395],
        'side-tables': [0.495, 0.495, 0.395],
        'sink': [0.515, 0.455, 0.945],
        'dishwasher': [0.65, 0.75, 0.595],
        'dishwasher_table': [0.65, 0.75, 0.42],
        'cabinet': [0.9, 0.46, 0.66],
        'shelf': [0.765, 0.357, 0.805],
        'shelf_1f': [0.765, 0.357, 0.855 + 0.025],
        'shelf_2f': [0.765, 0.357, 1.15 + 0.025],
        'shelf_3f': [0.765, 0.357, 1.4 + 0.025],
        'shelf_4f': [0.765, 0.357, 1.4 + 0.025],
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

    OBJECT_LIST = [
        # name, item_id, itemtype, grasping_type[front:0, top:1, bowl:2, plate:3]
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
    OBJECT_LIST = [
        # name, item_id, itemtype, grasping_type[front:0, top:1, bowl:2, plate:3]
        ['water', 0, 0, 0],
        ['milk', 1, 0, 0],
        ['coke', 2, 0, 0],
        ['tonic', 3, 0, 0],
        ['bubble_tea', 4, 0, 0],
        ['ice tea', 5, 0, 0],
        ['cereal', 6, 2, 0],
        ['tuna_can', 7, 2, 0],
        ['coffee_jar', 8, 2, 0],
        ['sugar', 9, 2, 0],
        ['mustard', 10, 2, 0],
        ['apple', 11, 3, 0],
        ['peach', 12, 3, 0],
        ['orange', 13, 3, 0],
        ['banana', 14, 3, 0],
        ['strawberry', 15, 3, 0],
        ['pockys', 16, 4, 0],
        ['pringles', 17, 4, 0],
        ['spoon', 18, 5, 1],
        ['fork', 19, 5, 1],
        ['plate', 20, 5, 3],
        ['bowl', 21, 5, 2],
        ['mug', 22, 5, 0],
        ['knife', 23, 5, 1],
        ['paperbag', 24, 1, 1],
        ['tab', 25, 0, 0]
    ]

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

    OBJECT_TYPES = [
        "food",  # 0
        "fruit",  # 1
        "kitchen",  # 2
        "shape",  # 3
        "drink",  # 4
        "task",  # 5
    ]
    # TODO AREA_EDGES FOR sim_mode must be updated
    ARENA_EDGES = [[1.167, -0.2321], [7.0443, -0.1863], [7.015, 2.5457], [8.2162, 2.6681], [8.2485, 6.0065], \
                   [5.6399, 5.8781], [5.5177, 3.6958], [4.7759, 3.6698], [4.7012, 2.4829], [0.9986, 2.0893]]


elif False:
    print('[WARNING] please stop right now')
    # RGB_TOPIC = '/snu/rgb_rect_raw'
    # DEPTH_TOPIC = '/snu/depth_rect_raw'
    # PC_TOPIC = '/snu/points'
    PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points_fixed'   #for 73 robot
    ABS_POSITION = {
        'arena_out': [0.0, 0.0, 0.0],
        'zero': [0.0, 0.0, 0.0],
        'dev_front': [0.0, 0.0, 0.0],
        'breakfast_table': [6.4844, -0.4418, -1.5614],
        'table_front': [6.7171, 0.2106, 0.0067],

        'table_side': [1.519, -1.001, 1.615],
        # serve breakfast
        'kitchen_entrance' : [4.131, 0.1607, 0.0226],
        # storing groceries
        'shelf_front': [2.9903, 3.6003, 1.4964],
        'grocery_table': [3.4176, 2.7049, 0.0384],
        'dishwasher' : [9.0737, 2.7295, 1.624],

        'sofa_view': [2.5436, 1.2884, -1.6177],
        'door_handle': [1.6718, 0.1612, -3.0811],
        'cloth_scan': [1.6718, 0.1612, -3.0811],
        'door_bypass': [1.6718, 0.1612, -3.0811],
        'seat_scan': [2.5646, 0.5886, -1.5718],
        'seat_scan_bypass': [2.5646, 0.5886, -1.5718],
        'sofa_front': [2.5646, 0.5886, -1.5718],

        #stickler for the rules
        'living_room_search': [3.1688, 1.3108, -1.59],
        'office_search': [3.2534, 2.4367, 1.5922],
        'kitchen_search': [5.4816, 0.1591, 0.0411],
        'bedroom_search': [7.5031, 2.81, 1.4711],
        'stickler_forbidden_room_front': [3.7707, 0.9244, -0.5702],
        'shoe_warning': [1.4915, 0.7425, -1.6057],
        'kitchen_table': [6.8759, 0.2216, 0.0326],
        'bar_drink': [6.4021, 1.101, -3.1054],
        'bin_littering': [2.5534, 2.4934, 3.0933],

        # receptionist
        'cloth_scan': [2.1035, 0.1388, -3.0278]  ,  # [2.5404, 0.3225, -3.1168] near door
        'handle_front': [0.311, -0.009, 0.016],
        'door_bypass': [-1.2198, 2.6009, 1.698],
        'seat_scan': [2.6464, 0.2434, -1.6043],

        'seat_scan_1500': [2.4096, 0.4396, -1.4934],
        'seat_scan_1300': [2.6464, 0.2434, -1.6043],

        'seat_scan_bypass': [2.3973, 1.0645, 3.0917],

        # gpsr
        # 'breakfast_table': [6.4844, -0.4418, -1.5614],
        # 'grocery_table': [3.4176, 2.7049, 0.0384],
        # 'kitchen_table': [6.8759, 0.2216, 0.0326],
        'shelf': [2.7292, 3.5637, 1.5569],
        'taxi': [0.0, 0.0, 0.0],
        'kitchen': [5.4816, 0.1591, 0.0411],
        'living_room': [3.1688, 1.3108, -1.59],
        'office': [3.2534, 2.4367, 1.5922],
        'bedroom': [7.5031, 2.81, 1.4711],
        'gpsr_start': [2.6054, 0.5917, -1.5534],
        ## clean the table
        'clean_table_front': [8.9271, 0.3594, 3.1027],
        'dishwasher_door': [8.1775, 3.5906, 0.0587]

    }
    # for gpsr
    LOCATION_MAP = {
        "bedroom": ['small_shelf', 'cupboard', 'big_shelf', 'bed'],
        "kitchen": ['kitchen_shelf', 'pantry', 'dinner_table', 'kitchen_bin', 'fridge', 'washing_machine', 'sink',
                    'kitchen_table', 'breakfast_table'],
        "office": ['desk', 'show_rack', 'bin', 'office_shelf'],
        "living room": ['house_plant', 'coat_rack', 'sola', 'couch_table', 'tv', 'side_table', 'book_shelf']
    }
    OBJECT_LIST = [
        # name, item_id, itemtype, grasping_type[front:0, top:1, bowl:2, plate:3]
        ['cracker', 0, 0, 0],
        ['sugar', 1, 0, 0],
        ['jello red', 2, 0, 0],
        ['jello black', 3, 0, 0],
        ['spam', 4, 0, 0],
        ['coffee can', 5, 0, 0],
        ['tuna can', 6, 0, 0],
        ['pringles', 7, 0, 0],
        ['mustard', 8, 0, 0],
        ['tomato soup', 9, 0, 0],
        ['pear', 10, 1, 0],
        ['peach', 11, 1, 0],
        ['apple', 12, 1, 0],
        ['strawberry', 13, 1, 0],
        ['orange', 14, 1, 0],
        ['plum', 15, 1, 0],
        ['lemon', 16, 1, 0],
        ['bowl', 17, 2, 2],
        ['mug', 18, 2, 0],
        ['plate', 19, 2, 3],
        ['knife', 20, 2, 1],
        ['fork', 21, 2, 1],
        ['spoon', 22, 2, 1],
        ['tennis ball', 23, 3, 0],
        ['golf ball', 24, 3, 0],
        ['base ball', 25, 3, 0],
        ['water', 26, 4, 0],
        ['bubble tea', 27, 4, 0],
        ['tonic', 28, 4, 0],
        ['coke', 29, 4, 0],
        ['ice tea', 30, 4, 0],
        ['milk', 31, 4, 0],
        ['cereal', 32, 5, 0],
        ['shopping bag', 33, 5, 0],
        ['dishwasher tablet', 34, 5, 1],
        ['cube', 35, 5, 0],
        ['banana', 36, 1, 0],
    ]

    '''
    Insert TABLE_DIMENSION OBJECT_TYPES
    in hsrb_mode just for compatiability (otherwise it won't run)
    YOU need to modify it before running
    '''
    TABLE_DIMENSION = {
        # width, depth, height
        'kitchen_table': [0.73, 0.4, 0.74],
        'breakfast_table': [0.4, 0.4, 0.615],
        #'dishwasher': [1dishwa.1, 0.3, 0.91],
        'dishwasher_handle': [0.55, 0.4, 0.72],
        'dishwasher': [0.32, 0.41, 0.6],
        'door_handle': [0, 0, 0.96],
        'shelf_1f': [0.56, 0.23, 0.78],
        'shelf_2f': [0.56, 0.23, 1.12],
        'shelf': [0.56, 0.23, 0.78],
        'grocery_table': [0.4, 0.4, 0.615],
    }

    OBJECT_TYPES = [
        "food",  # 0
        "fruit",  # 1
        "kitchen",  # 2
        "shape",  # 3
        "drink",  # 4
        "task",  # 5
    ]

    TINY_OBJECTS = ['spoon', 'fork', 'knife']

    ARENA_EDGES = [[0.96,-2.14], [10,-1.9], [9.93, 5.04], [0.925, 4.73]]

