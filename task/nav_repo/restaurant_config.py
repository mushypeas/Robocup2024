import math


min_interval_arc_len = 0.7
unit_rad = 0.25 * ( math.pi / 180 )
avg_dist_move_dist_ratio = 5

def calculate_human_rad(human_center_x, yolo_img_width):
    human_center_bias = human_center_x - yolo_img_width / 2
    return -human_center_bias / 640

def index_to_rad(idx):
    return (idx - center_index) * unit_rad

################ Maybe Constant? ################

## main frequency
main_freq = 0.5
main_period = 1.0 / main_freq

## LiDAR dist
min_dist = 1.0
max_dist = 5.0

## LiDAR index
lidar_index = 963
center_index = lidar_index // 2

## YOLO img size
yolo_img_height = 480
yolo_img_width = 640
