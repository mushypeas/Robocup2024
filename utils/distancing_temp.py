import numpy as np
from utils.axis_transform import Axis_transform
from hsr_agent.global_config import TABLE_DIMENSION


def distancing_table(pointcloud, table, raw=False, dist=0.6):
    # Transform from depth camera to base_link
    axis_transform = Axis_transform()
    pointcloud = np.array(pointcloud.tolist())[:, :, :3]
    pointcloud = pointcloud.reshape(-1, 3)
    base_link_pc = axis_transform.tf_camera_to_base(pointcloud, multi_dimention=True)
    # [x, y, z] == [front, left, up]
    table_height = TABLE_DIMENSION[table][2]
    table_height_range = [table_height - 0.01, table_height + 0.01]
    base_link_pc = base_link_pc[np.where((base_link_pc[:, 2] > table_height_range[0])
                     & (base_link_pc[:, 2] < table_height_range[1]))]

    robot_to_table_dist = round(np.nanmin(base_link_pc[:, 0]), 4)
    print('robot_to_table_dist', robot_to_table_dist)
    if raw:
        return robot_to_table_dist
    return robot_to_table_dist - dist






