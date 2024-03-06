import rospy
import ros_numpy
import numpy as np
import struct
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from open3d import geometry, visualization, camera
from utils.axis_transform import Axis_transform
from hsr_agent.global_config import *
from sklearn.decomposition import PCA

class Depth2PC:
    def __init__(self):
        self.depth_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image, self.depth_cb)
        self.rgb_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, self.rgb_sub)
        self.depth = None
        self.stamp = None
        self.rgb = None
        self.pc_pub = rospy.Publisher('pointcloud_from_depth', PointCloud2, queue_size=10)
        self.pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self.pc_cb)

        camera_info_msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/camera_info', CameraInfo)
        ##### camera intrinsic matrix #####
        k = np.array(camera_info_msg.K).reshape(3, 3)
        fx = k[0, 0]
        fy = k[1, 1]
        cx = k[0, 2]
        cy = k[1, 2]
        self.camera_intrinsic = [fx, fy, cx, cy]
        self.camera_info = camera.PinholeCameraIntrinsic()
        self.axis_transform = Axis_transform()

    def depth_cb(self, data):
        self.stamp = data.header.stamp
        self.depth = geometry.Image(ros_numpy.numpify(data).astype(np.uint16))  # (720, 1280)



    def pc_cb(self, data):
        data = ros_numpy.numpify(data)

    def rgb_sub(self, data):
        self.rgb = geometry.Image(ros_numpy.numpify(data).astype(np.uint8))


    def pc_for_target_object(self, cx, cy):
        if self.camera_intrinsic is not None and self.depth is not None and self.rgb is not None:
            self.camera_info.set_intrinsics(640, 480, self.camera_intrinsic[0], self.camera_intrinsic[1],
                                            self.camera_intrinsic[2], self.camera_intrinsic[3])
            rgbd_image = geometry.RGBDImage.create_from_color_and_depth(self.rgb, self.depth, convert_rgb_to_intensity=False)
            pc = geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.camera_info, project_valid_depth_only=False)
            pc_points_2d = np.asarray(pc.points).reshape((480, 640, -1))  # only points, (1280 * 720, 3)
            pc_roi = pc_points_2d[cy - 30: cy + 30, cx - 30: cx + 30].reshape(-1, 3)
            # print("pc_roi", pc_roi)
            xyz = np.average(pc_roi[~np.any(np.isnan(pc_roi), axis=1)], axis=0)
            return xyz
    def pc_for_bbox(self, cx, cy, w, h):
        x1 = cx - w // 2
        x2 = cx + w // 2
        y1 = cy - h // 2
        y2 = cy + h // 2
        if self.camera_intrinsic is not None and self.depth is not None and self.rgb is not None:
            self.camera_info.set_intrinsics(640, 480, self.camera_intrinsic[0], self.camera_intrinsic[1],
                                            self.camera_intrinsic[2], self.camera_intrinsic[3])
            rgbd_image = geometry.RGBDImage.create_from_color_and_depth(self.rgb, self.depth, convert_rgb_to_intensity=False)
            pc = geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.camera_info, project_valid_depth_only=False)
            pc_points_2d = np.asarray(pc.points).reshape((480, 640, -1))  # only points, (1280 * 720, 3)
            pc_roi = pc_points_2d[y1: y2, x1: x2].reshape(-1, 3)
            return pc_roi
        else :
            return None
    def get_pc(self):
        if self.camera_intrinsic is not None and self.depth is not None and self.rgb is not None:
            self.camera_info.set_intrinsics(640, 480, self.camera_intrinsic[0], self.camera_intrinsic[1],
                                            self.camera_intrinsic[2], self.camera_intrinsic[3])
            rgbd_image = geometry.RGBDImage.create_from_color_and_depth(self.rgb, self.depth, convert_rgb_to_intensity=False)
            pc = geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.camera_info, project_valid_depth_only=False)
            pc_points_2d = np.asarray(pc.points).reshape((480, 640, -1))  # only points, (1280 * 720, 3)
            return pc_points_2d
        else :
            return None

    def main(self):
        if self.camera_intrinsic is not None and self.depth is not None and self.rgb is not None:
            self.camera_info.set_intrinsics(640, 480, self.camera_intrinsic[0], self.camera_intrinsic[1],
                                            self.camera_intrinsic[2], self.camera_intrinsic[3])
            rgbd_image = geometry.RGBDImage.create_from_color_and_depth(self.rgb, self.depth, convert_rgb_to_intensity=False)
            pc = geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.camera_info, project_valid_depth_only=False)
            pc_points = np.asarray(pc.points)  # only points, (1280 * 720, 3)
            pc_colors = np.asarray(pc.colors)
            pc_data = np.hstack((pc_points, pc_colors)).reshape(480, 640, 6)  # points + rgb
            pc_data = pc_data.astype(np.float32)
            pc_to_pub = PointCloud2()
            pf = []

            #### pointfield parameters #####
            pf_x = PointField()
            pf_x.name = 'x'
            pf_x.offset = 0
            pf_x.datatype = 7
            pf_x.count = 1
            pf.append(pf_x)

            pf_y = PointField()
            pf_y.name = 'y'
            pf_y.offset = 4
            pf_y.datatype = 7
            pf_y.count = 1
            pf.append(pf_y)

            pf_z = PointField()
            pf_z.name = 'z'
            pf_z.offset = 8
            pf_z.datatype = 7
            pf_z.count = 1
            pf.append(pf_z)

            pf_b = PointField()
            pf_b.name = 'b'
            pf_b.offset = 12
            pf_b.datatype = 7
            pf_b.count = 1
            pf.append(pf_b)

            pf_g = PointField()
            pf_g.name = 'g'
            pf_g.offset = 16
            pf_g.datatype = 7
            pf_g.count = 1
            pf.append(pf_g)

            pf_r = PointField()
            pf_r.name = 'r'
            pf_r.offset = 20
            pf_r.datatype = 7
            pf_r.count = 1
            pf.append(pf_r)

            ##### pointcloud2 parameters #####
            pc_to_pub.header.frame_id = 'head_rgbd_sensor_frame'
            pc_to_pub.width = 640
            pc_to_pub.height = 480
            pc_to_pub.fields = pf
            pc_to_pub.is_bigendian = False
            pc_to_pub.point_step = 24
            pc_to_pub.header.stamp = self.stamp
            pc_to_pub.row_step = 640 * 24
            pc_to_pub.is_dense = False
            pc_to_pub.data = pc_data.tobytes()
            self.pc_pub.publish(pc_to_pub)

if __name__ == '__main__':
    rospy.init_node('depth_to_pc')
    depth2pc = Depth2PC()
    while True:
        # depth2pc.main()
        depth2pc.pc_for_target_object(550,330)
        rospy.sleep(0.5)






