import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray, ColorRGBA
import struct
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
import numpy as np
import ros_numpy
from open3d import geometry, camera
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from hsr_agent.global_config import *
from sklearn.cluster import DBSCAN

import sys
sys.path.append('../../../robocup2024')
from hsr_agent.global_config import OBJECT_LIST, TINY_OBJECTS, TABLE_DIMENSION, ABS_POSITION_PNU
from module.yolov7.utils_yolo.axis_transform import Axis_transform


class YoloModule:
    def __init__(self, object_list):
        self.yolo_sub = rospy.Subscriber('/snu/yolo', Int16MultiArray, self._yolo_callback)
        self.pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self._pc_callback, queue_size=1)
        self.object_list = object_list
        self.yolo_bbox = []
        self.object_3d_list = []
        self.pc = None

        self.custom_pc_pub = rospy.Publisher('/snu/points', PointCloud2, queue_size=10)

        self.object_len = 0
        self.axis_transform = Axis_transform()
        self.mark_pub = rospy.Publisher('/snu/object_3d', Marker, queue_size=100)
        self.object_pc_pub = rospy.Publisher('/snu/object_points', PointCloud2, queue_size=1)
        self._depth_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image,
                                           self._depth_callback)

    def _yolo_callback(self, msg):
        # [[cent_x, cent_y, width, height, class_id], ..]
        self.yolo_bbox = np.array(msg.data).reshape(-1, 5)

    def _depth_callback(self, data):
        self.stamp = data.header.stamp
        self.depth_for_pc = geometry.Image(ros_numpy.numpify(data).astype(np.uint16))
        self.depth_image = ros_numpy.numpify(data).astype(np.uint16)

    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)


    def find_object_info_by_name(self, name):
        for item in self.object_list:
            if item[0] == name:
                return item
        return None

    def find_id_by_name(self, name):
        for item in self.object_list:
            if item[0] == name:
                return item[1]
        return None

    def find_name_by_id(self, idx):
        return self.object_list[int(idx)][0]

    def find_box_by_name(self, name):
        item_id = self.find_id_by_name(name)
        for bbox in self.yolo_bbox:
            #print(bbox)
            if bbox[4] == item_id:
                return bbox
        return None

    def find_type_by_id(self, item_id):
        object_list_np = np.array(self.object_list)
        return self.object_list[np.where(object_list_np[:, 1].astype(int) == item_id)[0][0]][2]

    def find_grasping_type_by_id(self, item_id):
        object_list_np = np.array(self.object_list)
        return self.object_list[np.where(object_list_np[:, 1].astype(int) == item_id)[0][0]][3]

    def find_3d_points_by_name(self, points_3d_list, name):
        points_3d_list = np.array(points_3d_list)
        try:
            return points_3d_list[np.where(points_3d_list[:, 3] == self.find_id_by_name(name))[0][0]][:3]
        except IndexError:
            return None

    #find minimum distance pointcloud in object
    # todo : delete
    def get_target_object_pc(self, bbox):
        pc_points = self.pc
        pc_msg = rospy.wait_for_message('/pointcloud_from_depth')
        pc = ros_numpy.numpify(pc_msg)
        x_cloud = pc['x']
        y_cloud = pc['y']
        z_cloud = pc['z']

        _pc = np.stack([x_cloud, y_cloud, z_cloud], axis=-1)
        _pc = pc_points
        cent_x, cent_y, width, height, class_id = bbox

        half_height = width // 2
        cropped_pc_z = _pc[cent_y-half_height: cent_y+half_height,
                           cent_x, 2]
        min_z_idx = np.nanargmin(cropped_pc_z) + cent_y-half_height
        target_pc = _pc[min_z_idx, cent_x, :]
        return target_pc

    def calculate_dist_to_pick(self, base_to_object_xyz, grasping_type):
        # [ 0.91695352 -0.0770967   0.53439366]
        # [side: 0, top: 1, bowl: 2, plate: 3, shelf: 4, bag:5]
        if grasping_type == 0:  # side [ex. milk, cereal]
            camera_to_base_offset = [-0.65, -0.1, 0]
        elif grasping_type == 1:  # top [ex. spoon]
            camera_to_base_offset = [-0.5, -0.07, 0]
        elif grasping_type == 2:  # bowl
            camera_to_base_offset = [-0.5, -0.1, 0]
        elif grasping_type == 3:  # dish
            camera_to_base_offset = [-0.5, -0.1, 0]
        elif grasping_type == 4:  # shelf
            camera_to_base_offset = [-0.38, -0.1, 0]
        elif grasping_type == 5:  # bag
            camera_to_base_offset = [-0.44, -0.04, 0]
        elif grasping_type == 6:  # inclined pick
            camera_to_base_offset = [-0.43, -0.1, 0]
            
        dist_to_pick = [i + j for i, j in zip(base_to_object_xyz, camera_to_base_offset)]
        return dist_to_pick

    def make_point_marker(self, idx):
        marker = Marker()
        marker.header.frame_id = "base_link" #'camera_rgb_optical_frame'#"base_link"
        marker.ns = "unit_vector"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.color = ColorRGBA(1, 1, 0, 1)
        marker.scale.x = 0.005

        marker.points = []
        marker.id = idx*10 + 1

        return marker

    def draw_line_marker(self, base_points, idx):
        marker = self.make_point_marker(idx)
        for p in base_points:
            marker.points.append(Point(p[0], p[1], p[2]))
        self.mark_pub.publish(marker)

    def detect_cutlery(self, name, cutlery_list):
        if name in cutlery_list:
            return self.find_object_info_by_name(name)
        return None


    def detect_3d_safe(self, table, dist=0.6, depth=0.8, item_list=None):
        _pc = self.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        # base_link [+x, +y, +z] = [front, left, up]
        if self.object_len != len(self.yolo_bbox):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.action = marker.DELETEALL
            self.mark_pub.publish(marker)
        self.object_len = len(self.yolo_bbox)

        object_3d_list = []
        for idx, item in enumerate(self.yolo_bbox):
            cent_x, cent_y, width, height, class_id = item
            class_name = self.find_name_by_id(class_id)
            if item_list is not None and class_name not in item_list:
                rospy.logwarn(f"Ignoring {class_name}...")
                continue
            start_x = cent_x - (width // 2)
            start_y = cent_y - (height // 2)
            object_pc = pc_np[start_y:start_y+height, start_x:start_x+width]
            object_pc = object_pc.reshape(-1, 3)
            points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

            table_height = ABS_POSITION_PNU[table][2]
            height_offset = 0.01
            front_threshold = dist + depth

            # exception1 : tiny height object
            if OBJECT_LIST[class_id][0] in TINY_OBJECTS: # ['spoon', 'fork', 'knife']
                height_offset = 0.01 # 0 에서 0.01 사이
                print('tiny', OBJECT_LIST[class_id])
            # exception2 : objects in shelf
            if 'shelf' in table:
                front_threshold = dist + depth
            if 'pantry' in table:
                front_threshold = 1.3

            height_threshold = [table_height - height_offset, 1.5]
            print("POINTS SHAPE ; ", points_by_base_link.shape)
            # 1. hard-constraint thresholding by fixed parameters
            points_by_base_link = points_by_base_link[np.where((points_by_base_link[:, 0] < front_threshold)
                                                             & (points_by_base_link[:, 2] > height_threshold[0])
                                                             & (points_by_base_link[:, 2] < height_threshold[1]))]
            rospy.loginfo(f'{OBJECT_LIST[class_id][0]} points 3: {points_by_base_link.shape}')
            try:
                # # 2. soft-constraint thresholding by objects depth
                object_depth = 0.1  # 10cm
                point_min_x = round(np.nanmin(points_by_base_link[:, 0]), 4)
                points_by_base_link = points_by_base_link[
                    np.where((points_by_base_link[:, 0] < point_min_x + object_depth))]

                # 3. select min_max points for draw 3d box
                min_points = [round(np.nanmin(points_by_base_link[:, 0]), 4),
                              round(np.nanmin(points_by_base_link[:, 1]), 4),
                              round(np.nanmin(points_by_base_link[:, 2]), 4)]
                max_points = [round(np.nanmax(points_by_base_link[:, 0]), 4),
                              round(np.nanmax(points_by_base_link[:, 1]), 4),
                              round(np.nanmax(points_by_base_link[:, 2]), 4)]
            except ValueError:
                rospy.logerr(f'[YOLO] zero-size array for {OBJECT_LIST[class_id][0]}. Try searching again.')
                return []

            # print('item', OBJECT_LIST[class_id][0])
            # print('x', min_points[0], max_points[0], end=' ')
            # print('y', min_points[1], max_points[1], end=' ')
            # print('z', min_points[2], max_points[2])

            # TODO: all shelfs...
            # if not (table == 'shelf_1f' or table == 'shelf_2f' or table == 'shelf') and min_points[2] >= table_height + 0.03:
            #     print('behind objects')
            #     continue

            object_3d_list.append([min_points[0],                           # x
                                   (min_points[1] + max_points[1]) / 2,     # y
                                   (min_points[2] + max_points[2]) / 2,     # z
                                   class_id])                               # class_id
            cube_points = [[min_points[0], min_points[1], min_points[2]],  #1
                           [min_points[0], min_points[1], max_points[2]],  #2
                           [min_points[0], max_points[1], max_points[2]],  #3
                           [min_points[0], max_points[1], min_points[2]],  #4
                           [min_points[0], min_points[1], min_points[2]],  #5
                           [max_points[0], min_points[1], min_points[2]],  #6
                           [max_points[0], min_points[1], max_points[2]],  #7
                           [max_points[0], max_points[1], max_points[2]],  #8
                           [max_points[0], max_points[1], min_points[2]],  #9
                           [max_points[0], min_points[1], min_points[2]],  #10
                           [max_points[0], max_points[1], min_points[2]],  #11
                           [min_points[0], max_points[1], min_points[2]],  #12
                           [min_points[0], max_points[1], min_points[2]],  #13
                           [min_points[0], max_points[1], max_points[2]],  #14
                           [max_points[0], max_points[1], max_points[2]],  #15
                           [max_points[0], min_points[1], max_points[2]],  #16
                           [min_points[0], min_points[1], max_points[2]]]  #17

            self.draw_line_marker(cube_points, idx)

        self.object_3d_list = object_3d_list
        return self.object_3d_list


    def detect_3d(self, table, dist=0.6, shelf_depth=None):
        _pc = self.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        # base_link [+x, +y, +z] = [front, left, up]
        if self.object_len != len(self.yolo_bbox):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.action = marker.DELETEALL
            self.mark_pub.publish(marker)
        self.object_len = len(self.yolo_bbox)
        object_3d_list = []

        for idx, item in enumerate(self.yolo_bbox):
            cent_x, cent_y, width, height, class_id = item
            start_x = cent_x - (width // 2)
            start_y = cent_y - (height // 2)
            object_pc = pc_np[start_y:start_y+height, start_x:start_x+width]
            object_pc = object_pc.reshape(-1, 3)
            points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

            table_depth = dist
            table_height = TABLE_DIMENSION[table][2]
            height_offset = 0
            front_threshold = table_depth + 0.8

            # exception1 : tiny height object
            if OBJECT_LIST[class_id][0] in TINY_OBJECTS: # ['spoon', 'fork', 'knife']
                height_offset = -0.02
                print('tiny', OBJECT_LIST[class_id])
            # exception2 : objects in shelf
            if 'shelf' in table:
                front_threshold = table_depth + shelf_depth
            if 'pantry' in table:
                front_threshold = 1.3

            height_threshold = [table_height - height_offset, 1.5]

            # 1. hard-constraint thresholding by fixed parameters
            points_by_base_link = points_by_base_link[np.where((points_by_base_link[:, 0] < front_threshold)
                                                             & (points_by_base_link[:, 2] > height_threshold[0])
                                                             & (points_by_base_link[:, 2] < height_threshold[1]))]

            try:
                # # 2. soft-constraint thresholding by objects depth
                object_depth = 0.1  # 10cm
                print(f"found object id: {class_id}")
                print(f"found object: {OBJECT_LIST[class_id][0]}")
                print(f"points_by_base_link: {points_by_base_link}")
                point_min_x = round(np.nanmin(points_by_base_link[:, 0]), 4)
                points_by_base_link = points_by_base_link[
                    np.where((points_by_base_link[:, 0] < point_min_x + object_depth))]

                # 3. select min_max points for draw 3d box
                min_points = [round(np.nanmin(points_by_base_link[:, 0]), 4),
                              round(np.nanmin(points_by_base_link[:, 1]), 4),
                              round(np.nanmin(points_by_base_link[:, 2]), 4)]
                max_points = [round(np.nanmax(points_by_base_link[:, 0]), 4),
                              round(np.nanmax(points_by_base_link[:, 1]), 4),
                              round(np.nanmax(points_by_base_link[:, 2]), 4)]
            except ValueError:
                print('[YOLO] zero-size array for ', OBJECT_LIST[class_id][0])
                continue

            # print('item', OBJECT_LIST[class_id][0])
            # print('x', min_points[0], max_points[0], end=' ')
            # print('y', min_points[1], max_points[1], end=' ')
            # print('z', min_points[2], max_points[2])

            # TODO: all shelfs...
            # if not (table == 'shelf_1f' or table == 'shelf_2f' or table == 'shelf') and min_points[2] >= table_height + 0.03:
            #     print('behind objects')
            #     continue

            object_3d_list.append([min_points[0],                           # x
                                   (min_points[1] + max_points[1]) / 2,     # y
                                   (min_points[2] + max_points[2]) / 2,     # z
                                   class_id])                               # class_id
            cube_points = [[min_points[0], min_points[1], min_points[2]],  #1
                           [min_points[0], min_points[1], max_points[2]],  #2
                           [min_points[0], max_points[1], max_points[2]],  #3
                           [min_points[0], max_points[1], min_points[2]],  #4
                           [min_points[0], min_points[1], min_points[2]],  #5
                           [max_points[0], min_points[1], min_points[2]],  #6
                           [max_points[0], min_points[1], max_points[2]],  #7
                           [max_points[0], max_points[1], max_points[2]],  #8
                           [max_points[0], max_points[1], min_points[2]],  #9
                           [max_points[0], min_points[1], min_points[2]],  #10
                           [max_points[0], max_points[1], min_points[2]],  #11
                           [min_points[0], max_points[1], min_points[2]],  #12
                           [min_points[0], max_points[1], min_points[2]],  #13
                           [min_points[0], max_points[1], max_points[2]],  #14
                           [max_points[0], max_points[1], max_points[2]],  #15
                           [max_points[0], min_points[1], max_points[2]],  #16
                           [min_points[0], min_points[1], max_points[2]]]  #17

            self.draw_line_marker(cube_points, idx)

        self.object_3d_list = object_3d_list
        return self.object_3d_list

    def detect_3d_unseen(self, table):
        _pc = self.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        # base_link [+x, +y, +z] = [front, left, up]

        object_pc = pc_np.reshape(-1, 3)
        points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

        # 1. ransac to search table plane
        # best_eq, best_inliers = self.plane.fit(points_by_base_link, thresh=0.01, maxIteration=100)

        table_depth = 0.7

        table_height = 0.81#TABLE_DIMENSION[table][2]
        # else:
        #     table_height = abs(best_eq[3]) #TABLE_DIMENSION[table][2]
        #     if table_height <= 0.3 or abs(best_eq[2]) < 0.1:
        #         return False
        #
        # print('best_eq', best_eq, table_height)

        height_offset = -0.02
        front_threshold = table_depth + 0.5
        robot_max_height = 1.7


        height_threshold = [table_height - height_offset, robot_max_height]

        # 1. hard-constraint thresholding by fixed parameters
        points_by_base_link = points_by_base_link[np.where((points_by_base_link[:, 0] < front_threshold)
                                                         & (points_by_base_link[:, 2] > height_threshold[0])
                                                         & (points_by_base_link[:, 2] < height_threshold[1]))]


        # clustering = KMeans(n_clusters=8).fit(points_by_base_link)
        object_3d_list = []
        clustering = DBSCAN(eps=0.01, min_samples=5).fit(points_by_base_link)
        print('clustering.labels_', max(clustering.labels_))

        if self.object_len != len(clustering.labels_):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.action = marker.DELETEALL
            self.mark_pub.publish(marker)
        self.object_len = len(clustering.labels_)

        for c in range(max(clustering.labels_)+1):
            clustered_point = points_by_base_link[np.where((clustering.labels_ == c))]

            min_points = [round(np.nanmin(clustered_point[:, 0]), 4),
                          round(np.nanmin(clustered_point[:, 1]), 4),
                          round(np.nanmin(clustered_point[:, 2]), 4)]
            max_points = [round(np.nanmax(clustered_point[:, 0]), 4),
                          round(np.nanmax(clustered_point[:, 1]), 4),
                          round(np.nanmax(clustered_point[:, 2]), 4)]
            object_3d_list.append([min_points[0],  # x
                                   (min_points[1] + max_points[1]) / 2,  # y
                                   (min_points[2] + max_points[2]) / 2,  # z
                                   ])
            cube_points = [[min_points[0], min_points[1], min_points[2]],  # 1
                           [min_points[0], min_points[1], max_points[2]],  # 2
                           [min_points[0], max_points[1], max_points[2]],  # 3
                           [min_points[0], max_points[1], min_points[2]],  # 4
                           [min_points[0], min_points[1], min_points[2]],  # 5
                           [max_points[0], min_points[1], min_points[2]],  # 6
                           [max_points[0], min_points[1], max_points[2]],  # 7
                           [max_points[0], max_points[1], max_points[2]],  # 8
                           [max_points[0], max_points[1], min_points[2]],  # 9
                           [max_points[0], min_points[1], min_points[2]],  # 10
                           [max_points[0], max_points[1], min_points[2]],  # 11
                           [min_points[0], max_points[1], min_points[2]],  # 12
                           [min_points[0], max_points[1], min_points[2]],  # 13
                           [min_points[0], max_points[1], max_points[2]],  # 14
                           [max_points[0], max_points[1], max_points[2]],  # 15
                           [max_points[0], min_points[1], max_points[2]],  # 16
                           [min_points[0], min_points[1], max_points[2]]]  # 17
            self.draw_line_marker(cube_points, c)
        self.points_marker_for_cluster(points_by_base_link, clustering.labels_)
        object_3d_list = np.array(object_3d_list)
        object_3d_list = object_3d_list[object_3d_list[:, 0].argsort()]

        return object_3d_list

    def points_marker_for_cluster(self, points, cluster_labels):
        full_points = np.zeros(shape=(0,6))
        for c in range(max(cluster_labels)+1):
            clustered_point = points[np.where((cluster_labels == c))]
            print('clustered_point', clustered_point.shape)
            if clustered_point.shape[0] > 8000 or clustered_point.shape[0] < 100:
                continue
            color = np.zeros(shape=(clustered_point.shape[0], 3))
            color[:] = self.colors[c]
            clustered_point = np.concatenate((clustered_point, color), axis=1)
            full_points = np.concatenate((full_points, clustered_point), axis=0)

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = full_points.astype(dtype).tobytes()

        fields = [PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzrgb')]
        pc_msg = PointCloud2(
            height=1,
            width=full_points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 6),
            row_step=(itemsize * 6 * full_points.shape[0]),
            data=data
        )
        pc_msg.header.frame_id = 'base_link'
        pc_msg.header.stamp = self.stamp
        self.object_pc_pub.publish(pc_msg)

if __name__ == '__main__':
    rospy.init_node('test_yolo_module')
    yolo_module = YoloModule(OBJECT_LIST)
    r = rospy.Rate(5)
    print('ready')
    while not rospy.is_shutdown():
        if yolo_module.pc is None:
            continue
        print(yolo_module.detect_3d('kitchen_table'))
        r.sleep()