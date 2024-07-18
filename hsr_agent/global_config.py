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
yolo_weight_path = 'weight/0716v3.pt' 
yolo_classnames_path = 'weight/classnames.cn'

try:
    OBJECT_LIST = make_object_list(yolo_classnames_path)
except:
    print('Error: Cannot load object list')
    pass

Robocup = True

global config 

if Robocup: # 240710 mjgu 추가
    print('[GLOBAL CONFIG] !!!Robocup mode!!!')

    pick_table =  'dish_washer'
    place_table = 'dinner_table'

    ABS_POSITION_Robocup = {
        # serve breakfast
        'Entrance' :[0, 0, 0],       
        'hallway' : [3.8827, -0.5537, -0.0261],   # bj 땀gi
        'livingroom' : [8.9285, 0.6183, 1.5787], # bj 땀
        'picking_location': [7.9502, 5.3833, 0.0163], # (좌표 6) dish washer 앞 60cm 지점 
        'kitchen_counter' : [8.5652, 4.8247, -0.0005], # 80cm 앞. pick_table 후보 1 (bj, )
        # 'kitchen_cabinet' : [4.8601, 5.3832, -3.1016], # 60cm 앞. pick_table 후보 2
        'dish_washer' : [8.5045, 3.6366, 0.0131], # 60cm 앞. pick_table 후보 3 (병주가 땀, )
        'dinner_table': [8.5045, 4.9366, 0.0131], # dish_washer 쪽 , place_table 위치
        }

    TABLE_DIMENSION = {
        # width, depth, height
        'kitchen_counter': [0.61, 1.82, 0.91],
        'kitchen_cabinet': [0.792, 0.285, 1.058],
        'dish_washer': [0.60, 0.72, 0.85],
        'dinner_table': [0.785, 2.00, 0.775],
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

