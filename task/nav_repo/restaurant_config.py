import math

## main frequency
main_freq = 0.5
main_period = 1.0 / main_freq

## LiDAR
min_dist = 1.0
max_dist = 5.0

lidar_index = 963
center_index = lidar_index // 2

min_interval_arc_len = 0.7
unit_rad = 0.25 * ( math.pi / 180 )

yolo_img_height = 480
yolo_img_width = 640

def calculate_human_rad(human_center_x, yolo_img_width):
    return (human_center_x - yolo_img_width / 2) / 640