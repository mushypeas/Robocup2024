import math

dist_thres = 1.0
max_lidar_range = 5.0

lidar_index = 963
center_index = lidar_index // 2

interval_min_angle = 0.3
unit_angle = 0.25 * ( math.pi / 180 )

yolo_img_height = 480
yolo_img_width = 640

def calculate_human_rad(human_center_x, yolo_img_width):
    return (human_center_x - yolo_img_width / 2) / 640