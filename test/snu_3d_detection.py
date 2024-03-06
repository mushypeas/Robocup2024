import rospy
from std_msgs.msg import Int16MultiArray, ColorRGBA, Float32MultiArray, Header
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
import numpy as np
from cv_bridge import CvBridge
import ros_numpy
from open3d import geometry, camera
# from Plane import Plane
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from matplotlib import pyplot as plt
plt.style.use('ggplot')

from sklearn.cluster import DBSCAN

import sys
sys.path.append('../../robocup2024')
from hsr_agent.global_config import OBJECT_LIST, TINY_OBJECTS, TABLE_DIMENSION
from module.yolov7.utils_yolo.axis_transform import Axis_transform
from hsr_agent.agent import Agent
from hsr_agent.global_config import *

OBJECT_LIST = [
    # name, item_id, itemtype, grasping_type[front:0, top:1, bowl:2, plate:3]
    ['water',  0, 2, 0],
    ['milk',   1, 5, 1],
    ['coke',    2, 5, 2],
    ['tonic',    3, 0, 0],
    ['bubble tea',    4, 5, 3],
    ['ice tea',   5, 5, 1],
    ['tuna can',    6, 5, 1],
    ['coffee jar',     7, 5, 0],
    ['sugar',     7, 5, 0],
    ['mustard',     7, 5, 0],
    ['apple',     7, 5, 0],
    ['peach',     7, 5, 0],
    ['orange',     7, 5, 0],
    ['banana',     7, 5, 0],
    ['strawberry',     7, 5, 0],
    ['pockys',     7, 5, 0],
    ['pringles',     7, 5, 0],
    ['spoon',     7, 5, 0],
    ['fork',     7, 5, 0],
    ['plate',     7, 5, 0],
    ['bowl',     7, 5, 0],
    ['mug',     7, 5, 0],
    ['knife',     7, 5, 0],
    ['cereal',     7, 5, 0],
]


class YoloModule:
    def __init__(self, object_list):
        self.yolo_sub = rospy.Subscriber('/snu/yolo', Int16MultiArray, self._yolo_callback)
        self._rgb_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, self._rgb_callback)
        self.pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self._pc_callback, queue_size=1)
        self._depth_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image,
                                           self._depth_callback)
        self.object_list = object_list
        self.yolo_bbox = []
        self.object_3d_list = []
        self.rgb_img = None
        self.pc = None
        self.bridge = CvBridge()
        # ##### camera intrinsic matrix #####
        camera_info_msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/camera_info', CameraInfo)
        k = np.array(camera_info_msg.K).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]

        self.camera_intrinsic = [fx, fy, cx, cy]
        self.camera_info = camera.PinholeCameraIntrinsic()
        self.custom_pc_pub = rospy.Publisher('/snu/points', PointCloud2, queue_size=10)

        self.object_len = 0
        self.axis_transform = Axis_transform()
        self.mark_pub = rospy.Publisher('/snu/object_3d', Marker, queue_size=100)
        self.object_pc_pub = rospy.Publisher('/snu/object_points', PointCloud2, queue_size=1)
        self.colors = [list(np.random.choice(range(256), size=3)) for i in range(500)]
        # self.plane = Plane()

    def _yolo_callback(self, msg):
        # [[cent_x, cent_y, width, height, class_id], ..]
        self.yolo_bbox = np.array(msg.data).reshape(-1, 5)

    def _rgb_callback(self, img_msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(img_msg, 'rgb8')

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
        return self.object_list[idx][0]

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

    def find_3d_points_by_name(self, points_3d_list, name):
        points_3d_list = np.array(points_3d_list)
        try:
            return points_3d_list[np.where(points_3d_list[:, 3] == self.find_id_by_name(name))[0][0]][:3]
        except IndexError:
            return None
    def _depth_callback(self, data):
        self.stamp = data.header.stamp
        self.depth_for_pc = geometry.Image(ros_numpy.numpify(data).astype(np.uint16))
        self.depth_image = ros_numpy.numpify(data).astype(np.uint16)
    def depth_to_pc(self):
        if self.rgb_img is None:
            return
        depth = self.depth_for_pc
        stamp = self.stamp
        self.camera_info.set_intrinsics(640, 480, self.camera_intrinsic[0], self.camera_intrinsic[1],
                                        self.camera_intrinsic[2], self.camera_intrinsic[3])
        pc = geometry.PointCloud.create_from_depth_image(depth, self.camera_info, project_valid_depth_only=False)
        pc_points = np.asarray(pc.points)  # only points,
        # pc_points = pc_points.reshape(480, 640, 3)  # points + rgb

        rgb = self.rgb_img.reshape(-1, 3)
        rgb = rgb / 255
        pc_data = np.hstack((pc_points, rgb))


        # ######## add
        # pc_data[:, :3] = self.axis_transform.tf_camera_to_base(pc_data[:, :3], multi_dimention=True)
        # table = 'kitchen_table'
        #
        # table_height = TABLE_DIMENSION[table][2]
        # height_offset = 0
        #
        # height_threshold = [table_height - height_offset, 1.7]
        #
        #
        # # 1. hard-constraint thresholding by fixed parameters
        # print('pc_data', pc_data.shape)
        # points_by_base_link = pc_data
        # pc_data = points_by_base_link[np.where((points_by_base_link[:, 2] > height_threshold[0])
        #                                                    & (points_by_base_link[:, 2] < height_threshold[1]))]
        # print('pc_data', pc_data.shape)
        #
        # #############


        ##### pointcloud2 parameters #####
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        pc_to_pub = PointCloud2()
        pc_to_pub.header.frame_id = 'head_rgbd_sensor_rgb_frame'
        pc_to_pub.header.stamp = stamp
        pc_to_pub.width = 640  # 720 * 1280 * 4
        pc_to_pub.height = 480
        pc_to_pub.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
                            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
                            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)]
        pc_to_pub.is_bigendian = False
        pc_to_pub.point_step = np.dtype(np.float32).itemsize * 6
        pc_to_pub.row_step = (itemsize * 6 & pc_data.shape[0])
        pc_to_pub.is_dense = False
        pc_to_pub.data = pc_data.astype(dtype).tobytes()  # 720 * 1280 * 32

        self.custom_pc_pub.publish(pc_to_pub)

        return pc_points.reshape(480, 640, 3)

    def calculate_dist_to_pick(self, base_to_object_xyz, grasping_type):
        # [ 0.91695352 -0.0770967   0.53439366]
        # [side: 0, top: 1, bowl: 2, plate: 3, shelf: 4]
        if grasping_type == 0:  # side [ex. milk, cereal]
            camera_to_base_offset = [-0.65, -0.1, 0]
            dist_to_pick = [i+j for i, j in zip(base_to_object_xyz, camera_to_base_offset)]
            return dist_to_pick
        elif grasping_type == 1:  # top [ex. spoon]
            camera_to_base_offset = [-0.45, -0.1, 0]
            dist_to_pick = [i + j for i, j in zip(base_to_object_xyz, camera_to_base_offset)]
            return dist_to_pick
        elif grasping_type == 2:  # bowl
            camera_to_base_offset = [-0.5, -0.1, 0]
            dist_to_pick = [i + j for i, j in zip(base_to_object_xyz, camera_to_base_offset)]
            return dist_to_pick
        elif grasping_type == 3:  # dish
            camera_to_base_offset = [-0.5, -0.1, 0]
            dist_to_pick = [i + j for i, j in zip(base_to_object_xyz, camera_to_base_offset)]
            return dist_to_pick
        elif grasping_type == 4:  # shelf
            camera_to_base_offset = [-0.3, -0.1, 0]
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

    def detect_3d(self, table):
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
        obj_points = np.empty(shape=[0, 3], dtype=np.float)
        for idx, item in enumerate(self.yolo_bbox):
            cent_x, cent_y, width, height, class_id = item
            start_x = cent_x - (width // 2)
            start_y = cent_y - (height // 2)
            object_pc = pc_np[start_y:start_y+height, start_x:start_x+width]
            object_pc = object_pc.reshape(-1, 3)
            points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

            table_depth = 0.4
            front_threshold = table_depth + 0.5
            table_height = TABLE_DIMENSION[table][2]
            height_offset = 0
            table_width = TABLE_DIMENSION[table][0]

            # exception1 : tiny height object
            if OBJECT_LIST[class_id][0] in TINY_OBJECTS: # ['spoon', 'fork', 'knife']
                height_offset = 0.02
                print('tiny', OBJECT_LIST[class_id])
            # exception2 : objects in shelf
            if table == 'shelf_1f' or table == 'shelf_2f':
                front_threshold = 1.35

            height_threshold = [table_height - height_offset, 1.7]
            width_threshold = [-table_width / 2, table_width / 2]

            # 1. hard-constraint thresholding by fixed parameters
            points_by_base_link = points_by_base_link[np.where((points_by_base_link[:, 0] < front_threshold)
                                                        & (points_by_base_link[:, 1] > width_threshold[0])
                                                        & (points_by_base_link[:, 1] < width_threshold[1])
                                                        & (points_by_base_link[:, 2] > height_threshold[0])
                                                        & (points_by_base_link[:, 2] < height_threshold[1]))]
            obj_points = np.concatenate((obj_points, points_by_base_link), axis=0)
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
                print('[YOLO] zero-size array for ', OBJECT_LIST[class_id][0])
                continue

            print('item', OBJECT_LIST[class_id][0])
            print('x', min_points[0], max_points[0], end=' ')
            print('y', min_points[1], max_points[1], end=' ')
            print('z', min_points[2], max_points[2])
            if min_points[2] >= table_height + 0.03:
                print('behind objects')
                continue

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
        self.points_marker(obj_points)
        return self.object_3d_list

    def detect_2d(self):
        _pc = self.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        object_pc_list = []
        for idx, item in enumerate(self.yolo_bbox):
            cent_x, cent_y, width, height, class_id = item
            start_x = cent_x - (width // 2)
            start_y = cent_y - (height // 2)
            object_pc = pc_np[cent_y, cent_x]
            object_pc = object_pc.reshape(-1,3)
            print(OBJECT_LIST[class_id][0], object_pc)
            points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)
            object_pc_list.append(points_by_base_link[0])
        res = np.array(object_pc_list)
        print('res', res.shape)
        res = res[res[:, 0].argsort()]
        print('after res ', res)
        return res

    def detect_3d_unseen(self, table):
        _pc = self.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        # base_link [+x, +y, +z] = [front, left, up]

        object_pc = pc_np.reshape(-1, 3)
        points_by_base_link = self.axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)

        # 1. ransac to search table plane
        # best_eq, best_inliers = self.plane.fit(points_by_base_link, thresh=0.01, maxIteration=100)

        table_depth = 0.7
        hard_coded = True
        if hard_coded:
            table_height = 0.71#TABLE_DIMENSION[table][2]
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
        visualize_cluster = False
        if visualize_cluster:
            ax = plt.axes(projection='3d')
            print('clustering.labels_', max(clustering.labels_))
            ax.scatter(points_by_base_link[:,0], points_by_base_link[:,1], points_by_base_link[:,2], c=clustering.labels_, s=0.1)
            plt.show()
        object_3d_list = np.array(object_3d_list)
        object_3d_list = object_3d_list[object_3d_list[:, 0].argsort()]

        return object_3d_list

    def points_marker(self, points, color='white'):
        if color == 'red':
            color = np.zeros(shape=(points.shape[0], 3))
            color[:] = [1.0, 0.0, 0.0]
        else:
            color = np.zeros(shape=(points.shape[0], 3))
            color[:] = [0.0, 1.0, 0.0]

        points = np.concatenate((points, color), axis=1)
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = points.astype(dtype).tobytes()

        fields = [PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzrgb')]
        pc_msg = PointCloud2(
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 6),
            row_step=(itemsize * 6 * points.shape[0]),
            data=data
        )
        pc_msg.header.frame_id = 'base_link'
        pc_msg.header.stamp = self.stamp
        self.object_pc_pub.publish(pc_msg)

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
    import time
    rospy.init_node('snu_3d_object_detection', anonymous=True)
    yolo_module = YoloModule(OBJECT_LIST)
    agent = Agent()
    ### task params #################
    pick_table = 'kitchen_table'
    pick_location = 'table_front'
    #################################
    success = 7
    fail = 2
    while not rospy.is_shutdown():
        agent.move_abs(pick_location)
        agent.pose.table_search_pose()
        start_time = time.time()
        # input('command: ')
        if yolo_module.pc is None:
            continue
        # table_base_to_object_xyz = yolo_module.detect_2d()[0]
        table_base_to_object_xyz = yolo_module.detect_3d_unseen('kitchen_table')[0]
        table_base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, 0)


        agent.move_rel(-0.15, 0)
        agent.pose.pick_side_pose_by_arm_lift_joint(table_base_xyz[2])
        agent.open_gripper()

        agent.move_rel(0, table_base_xyz[1], wait=True)
        agent.move_rel(table_base_xyz[0] + 0.15, 0, wait=True)
        s_f = input('ready:')
        if s_f == 's':
            success +=1
        else:
            fail += 1
        print('success', success, 'fail', fail)
        agent.grasp()


        # 3. move
        agent.move_rel(-0.4, 0, wait=True)
        rospy.sleep(1.0)
        agent.open_gripper()
        agent.pose.table_search_pose()

