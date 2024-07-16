import os
import sys
sys.path.append('../../hsr_agent')
sys.path.append('hsr_agent')
from global_config_utils import make_object_list

# is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

RGB_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
DEPTH_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

# 기존 경로 : 'weight/best_240409.pt'
# YOLO weight 변경 시 경로 변경
yolo_weight_path = 'weight/0716v1.pt' 
yolo_classnames_path = 'weight/classnames.cn'

try:
    OBJECT_LIST = make_object_list(yolo_classnames_path)
except:
    print('Error: Cannot load object list')
    pass

Robocup = True
# FINAL = False
# PNU = False # 240630 mjgu 추가


global config 

if Robocup: # 240710 mjgu 추가
    print('[GLOBAL CONFIG] !!!Robocup mode!!!')

    ABS_POSITION_Robocup = {
        # serve breakfast
        'Entrance' :[0, 0, 0],          
        'picking_location': [8.1305, 4.1887, 0.0141], # (좌표 6) dish washer 앞 60cm 지점
        'kitchen_counter' : [7.9502, 5.3833, 0.0163], # 80cm 앞. pick_table 후보 1
        'kitchen_cabinet' : [4.8601, 5.3832, -3.1016], # 60cm 앞. pick_table 후보 2
        'dish_washer' : [8.1305, 4.1887, 0.0141], # 60cm 앞. pick_table 후보 3
        'dinner_table_counter': [5.3689, -1.3234, 0.0665], # (좌표 5) 60m 앞
        'dinner_table_cabinet': [5.1436, 4.2293, 0.0502], # (좌표 8) 60m 앞
    }

        # 좌표 1. 문 앞에서 직진 3m 정도 한 지점 [3.3894, 0.1851, 0.0061]
        # 좌표 2. 좌표 1에서 좌측 3m 정도 간 지점 [3.356, 3.1656, 0.044]
        # 좌표 3. 좌표 2에서 앞으로 1m 정도 간 지점 (kitchen_basket이 pickup table일 경우 60cm 앞 평행한 지점 [4.8702, 3.4057, -0.0066]
        # 좌표 4. kitchen_cabinet 앞 60cm 지점 [4.8601, 5.3832, -3.1016]
        # 좌표 5. 좌표 4에서 dinner_table 앞 60cm 지점 [5.1436, 4.2293, 0.0502]
        # 좌표 6. dish washer 앞 60cm 지점 [8.1305, 4.1887, 0.0141]
        # 좌표 7. kitchen counter 앞 80cm 지점 [7.9502, 5.3833, 0.0163]
        # 좌표 8. dinner table 앞 60cm 지점 (좌표 5의 반대편) [7.4391, 4.3351, 3.1374]
        # 좌표 9. dinner table 옆 80cm~1m 지점 (안전하게 가기 위한 체크 포인트) [6.226, 2.2863, 0.0322]

        # *** 다른 방의 가구를 가져와서 사용할 가능성이 존재함. 
        # ex) living room에 있는 원탁 테이블 (낮지만 원탁임)

    TABLE_DIMENSION = {
        # width, depth, height
        'kitchen_counter': [0.61, 1.82, 0.91],
        'kitchen_cabinet': [0.792, 0.285, 1.058],
        'dish_washer': [0.60, 0.595, 0.85],
        'dinner_table': [0.78, 2.00, 0.772],
        'test_table' : [0.8, 0.8, 0.815], # dinner table보다 4cm 높음 
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

    ARENA_EDGES = [[0.611, 2.440], [9.101, 2.457], [9.473, 1.872], [9.425, -6.256], [0.878, -6.291]] # 수정 필요


# if PNU: # 240630 mjgu 추가
#     print('[GLOBAL CONFIG] PNU mode')
#     # real robot
#     ABS_POSITION_PNU = {

#         # serve breakfast
#         'initial_location_pnu' :[0.0053, 0.0243, -0.0228], # (세면대 방 출발선) 240630 mjgu 추가
#         'picking_location_pnu': [4.3083, -1.5883, 0.0044], # (kitchen_table 근처) 240630 mjgu 추가
#         'kitchen_table_pnu' : [6.5185, -2.7109, -1.536], # 240630 mjgu 추가
#         'breakfast_table_pnu': [5.3689, -1.3234, 0.0665], # 240630 mjgu 추가
#     }


#     TABLE_DIMENSION = {
#         # width, depth, height
#         'breakfast_table_pnu': [0.735, 1.18, 0.73], # (나무색) 240630 mjgu 추가
#         'kitchen_table_pnu': [0.80, 0.40, 0.602], #(흰색) 240630 mjgu 추가
#     }


#     OBJECT_TYPES = [
#         "cleaning",  # 0
#         "drink",  # 1
#         "food",  # 2
#         "fruit",  # 3
#         "toy",  # 4
#         "snack",  # 5
#         "dish",  # 6
#         "bag",  # 7
#     ]

#     TINY_OBJECTS = ['spoon', 'fork', 'knife']

#     # added by lsh
#     ARENA_EDGES = [[0.611, 2.440], [9.101, 2.457], [9.473, 1.872], [9.425, -6.256], [0.878, -6.291]]


# if FINAL:
#     print('[GLOBAL CONFIG] FINAL mode')
#     # real robot
#     ABS_POSITION = {
#         # serve breakfast
#         'kitchen_table' : [3.0768, -1.1441, -0.0054], #2.9286, -1.1145, 0.0052 (가까움)
#         'breakfast_table': [2.4481, -1.4913, -3.0755],
#     }


#     TABLE_DIMENSION = {
#         # width, depth, height
#         'breakfast_table': [0.595, 0.595, 0.845],
#         'kitchen_table': [0.89, 0.89, 0.735],
        
#     }


#     OBJECT_TYPES = [
#         "cleaning",  # 0
#         "drink",  # 1
#         "food",  # 2
#         "fruit",  # 3
#         "toy",  # 4
#         "snack",  # 5
#         "dish",  # 6
#         "bag",  # 7
#     ]

#     TINY_OBJECTS = ['spoon', 'fork', 'knife']

#     # added by lsh
#     ARENA_EDGES = [[0.611, 2.440], [9.101, 2.457], [9.473, 1.872], [9.425, -6.256], [0.878, -6.291]]

#     # added by sujin

# if AIIS:
#     print('[GLOBAL CONFIG] AIIS mode')
#     # real robot
#     ABS_POSITION = {
#         'insp_target': [6.2869, 3.5307, 0],
#         'arena_out': [-2.487, 5.65, -1.561],
#         'zero': [0.0, 0.0, 0.0],
#         'dev_front': [-1.018, 0.190, -3.061],
#         'table_front': [6.6449, 0.3005, 0.0422],
#         'table_side': [7.3455, -1.0624, 1.551],
#         'table_back': [8.6478, 0.0623, 3.1025],
#         'sofa_view': [-0.008, -0.043, -0.801],
#         'door_handle': [1.4227, 1.1802, -1.5106],


#         # storing grocery
#         'grocery_table': [-0.7854, -0.2033, -1.548],
#         'grocery_shelf': [-1.091, -0.1197, 1.6165],

#         # serve breakfast
#         'breakfast_table': [5.1527, 0.2285, 0.0], # mjgu 240530
#         'kitchen_table' : [6.086, -1.1229, 0.0], # mjgu 240530
#         # 필요할 경우 우회 지점 설정 -> 'breakfast_table_bypass_testday' : [1.7554, 0.9174, 3.1374], #mjgu 240505

#         # clean the table

#         'dishwasher_front': [2.6256, -1.7107, 3.0623], #bjkim2 0505
#         'clean_table_front' : [5.2608, 0.2969, -0.0126], #bjkim2 0505 # HEIGHT SHOULD BE REALLLLLLY PRECISE
#         'rack_close_position1': [2.0321, -0.9574, -1.5822], #bjkim 0512
#         'rack_close_position2': [1.6463, -0.9664, -1.5655],
#         'rack_close_position3': [1.6434, -0.9569, -1.9500],


   
#         # AIIS
#         # 'bedroom_search': [-2.5427, 0.216, 1.2345],
#         # 'kitchen_search': [0.6884, -1.0065, -0.7347],
#         # 'living_room_search': [0.6534, -0.6374, 0.9569],
#         # 'study_search': [0.027, 0.3138, -2.1569],

#         # 0505
#         'kitchen_search': [3.2691, 0.3223, -2.1086],
#         'living_room_search': [5.932, -0.357, -0.4455],
#         'study_search': [5.2668, 1.273, 2.5436],
#         'bedroom_search': [6.9826, 3.0422, -0.6487],


#         # final
#         'final_kitchen_table' : [7.0491, -0.1961, 0.0064],
#         'final_round_table' : [6.9514, -1.5522, -1.5492],
#     }


#     TABLE_DIMENSION = {
#         # width, depth, height

#         'kitchen_table': [0.89, 0.89, 0.735],
#         'breakfast_table': [0.595, 0.595, 0.845],
#         'grocery_table': [0.65, 1.2, 0.42],

#         # final
#         'final_kitchen_table': [1.505, 0.705, 0.8],
#     }


#     OBJECT_TYPES = [
#         "cleaning",  # 0
#         "drink",  # 1
#         "food",  # 2
#         "fruit",  # 3
#         "toy",  # 4
#         "snack",  # 5
#         "dish",  # 6
#         "bag",  # 7
#     ]

#     TINY_OBJECTS = ['spoon', 'fork', 'knife']

#     # added by lsh
#     ARENA_EDGES = [[0.611, 2.440], [9.101, 2.457], [9.473, 1.872], [9.425, -6.256], [0.878, -6.291]]

# elif is_sim: # sim mode
    # print('[GLOBAL CONFIG] Sim mode. WARNING!!!!!!!!!!')
    # PC_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'

    # # gazebo
    # ABS_POSITION = {
    #     'arena_out': [-2.45, 5.1, -1.5708],
    #     'zero': [0.0, 0.0, 0.0],
    #     'start_1': [-1.0916, 2.541, -1.487],
    #     'dev_front': [-1.018, 0.190, -3.061],
    #     'breakfast_table': [-1.3, 0.65, -3.1416],
    #     'table_front': [0.2, 0.0, 0.0],
    #     'table_side': [1.5, -1.00, 1.5708],
    #     'shelf_front': [7.245, 1.051, 0],
    #     'sofa_view': [-0.008, -0.043, -0.801],
    #     'door_handle': [-2.327, 3.75, 1.61],
    #     'handle_front': [0.311, -0.009, 0.016],
    #     'dishwasher_gpsr': [6.531, 0.946, 1.57],
    #     'side_table': [3.291, 0.7065, 1.57],
    #     'grocery_table': [3.291, 0.7065, 1.57],  # dist = 0.9
    #     'kitchen_table': [1.5916, -2.7794, 0.0313], # dist = 0.6 /mjgu
    #     'kitchen_table_ready' : [1.0016, -2.7794, 0.0313], # dist = 2.0 /mjgu
    #     'pantry': [7.245, 1.051, 0],  # dist = 0.9
    #     'desk': [2.52, -3.815, -1.57],  # dist = 0.9
    #     'gpsr_start': [1.3372, 1.1288, 0.5792],
    #     'shelf': [5.4, -4.93, -1.57],  # dist = 0.9
    #     'bedside_table': [7.615, -4.56, 0],  # dist = 0.6
    #     'sink': [6.1375, -1.91, 3.14],  # dist = 0.6
    #     'cabinet': [3.45, -4.44, 0]  # dist = 0.6
    # }
    
    # TINY_OBJECTS = ['spoon', 'fork', 'knife']

    # TABLE_DIMENSION = {
    #     # width, depth, height
    #     'kitchen_table': [0.8, 0.8, 0.72],
    #     'desk': [1.505, 0.705, 0.8],
    #     'side table': [0.495, 0.495, 0.395],
    #     'sink': [0.515, 0.455, 0.945],
    #     'dishwasher_gpsr': [0.6, 0.6, 0.84],
    #     'cabinet': [0.9, 0.46, 0.66],
    #     'shelf': [0.26, 0.36, 0.445],
    #     'shelf_1f': [0.26, 0.36, 0.445],
    #     'shelf_2f': [0.26, 0.36, 0.735],
    #     'shelf_3f': [0.26, 0.36, 1.06],
    #     'shelf_4f': [0.26, 0.36, 1.4],
    #     'storage_rack': [0.445, 0.9, 0.42],
    #     'storage_rack_1f': [0.445, 0.9, 0.42],
    #     'storage_rack_2f': [0.445, 0.9, 0.88],
    #     'storage_rack_3f': [0.445, 0.9, 1.34],
    #     'pantry': [0.76, 0.26, 0.45],
    #     'pantry_1f': [0.76, 0.26, 0.45],
    #     'pantry_2f': [0.76, 0.26, 0.74],
    #     'pantry_3f': [0.76, 0.26, 1.06],
    #     'pantry_4f': [0.76, 0.26, 1.405],
    #     'bedside_table': [0.5, 0.45, 0.575],
    #     'breakfast_table': [0.6, 0.4, 0.625],
    #     'grocery_table': [0.495, 0.495, 0.40],
    #     'grocery_table_pose': [0.495, 0.495, 0.42],
    #     'grocery_table_pose1': [0.495, 0.495, 0.45],
    #     'grocery_table_pose2': [0.495, 0.495, 0.46],
    #     'dishwasher_handle': [0.60, 0.60, 0.84],  # [0.6, 0.4, 0.8]
    #     'door_handle': [0, 0, 0.96],
    # }

    # LOCATION_MAP = {
    #     "bedroom": ['bed', 'bedside_table', 'shelf'],
    #     "kitchen": ['pantry', 'trashbin', 'dishwasher', 'potted_plant', 'kitchen_table', 'chairs',
    #                 'refrigerator', 'sink'],
    #     "study": ['cabinet', 'coatrack', 'desk', 'armchair', 'desk_lamp', 'waste_basket', 'exit'],
    #     "living room": ['tv_stand', 'storage_rack', 'lamp', 'side_tables', 'side_table', 'sofa', 'entrance']
    # }
    
    # # TODO AREA_EDGES FOR sim_mode must be updated
    # ARENA_EDGES = [[1.167, -0.2321], [7.0443, -0.1863], [7.015, 2.5457], [8.2162, 2.6681], [8.2485, 6.0065], \
    #                [5.6399, 5.8781], [5.5177, 3.6958], [4.7759, 3.6698], [4.7012, 2.4829], [0.9986, 2.0893]]
    

