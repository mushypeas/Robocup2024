import rospy
import sys
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from open3d import geometry, camera


class DataReceiveController:
    def __init__(self):
        self.bridge = CvBridge()
        camera_info_msg = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/camera_info', CameraInfo)
        k = np.array(camera_info_msg.K).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]

        self.camera_intrinsic = [fx, fy, cx, cy]
        self.camera_info = camera.PinholeCameraIntrinsic()

        rospy.Subscriber("/snu/rgb_rect_compressed", CompressedImage, self.rgb_callback)
        rospy.Subscriber("/snu/depth_rect_compressed", CompressedImage, self.depth_callback)


        self.rgb_pub = rospy.Publisher('/snu/rgb_rect_raw', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/snu/depth_rect_raw', Image, queue_size=1)
        self.pc_pub = rospy.Publisher('/snu/points', PointCloud2, queue_size=10)


    def rgb_callback(self, data):
        if sys.getsizeof(data.data) > 1000:
            np_arr = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, 1)
            self.rgb_img = image_np
            img_msg = self.bridge.cv2_to_imgmsg(image_np, "bgr8")
            img_msg.header = data.header
            img_msg.header.frame_id = "head_rgbd_sensor_rgb_frame"
            img_msg.width = 640
            img_msg.height = 480
            self.rgb_pub.publish(img_msg)

    def depth_callback(self, data):
        if sys.getsizeof(data.data) > 1000:
            image_np = self.bridge.compressed_imgmsg_to_cv2(data)
            image_np = image_np.astype(np.uint16)
            self.depth_img = image_np

            depth_msg = Image()
            depth_msg.header = data.header
            depth_msg.header.frame_id = "head_rgbd_sensor_rgb_frame"
            depth_msg.width = 640
            depth_msg.height = 480
            depth_msg.encoding = "16UC1"
            depth_msg.step = 640 * 2
            depth_msg.data = image_np.tobytes()
            self.stamp = depth_msg.header.stamp
            self.depth_pub.publish(depth_msg)
            self.depth_to_pc()

    def depth_to_pc(self):
        if self.rgb_img is None:
            return
        depth = geometry.Image(self.depth_img)
        stamp = self.stamp
        self.camera_info.set_intrinsics(640, 480, self.camera_intrinsic[0], self.camera_intrinsic[1],
                                        self.camera_intrinsic[2], self.camera_intrinsic[3])
        pc = geometry.PointCloud.create_from_depth_image(depth, self.camera_info, project_valid_depth_only=False)
        pc_points = np.asarray(pc.points)  # only points,

        rgb = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2RGB).reshape(-1, 3)
        rgb = rgb / 255
        pc_data = np.hstack((pc_points, rgb))

        ##### pointcloud2 parameters #####
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        pc_msg = PointCloud2()
        pc_msg.header.frame_id = 'head_rgbd_sensor_rgb_frame'
        pc_msg.header.stamp = stamp
        pc_msg.width = 640
        pc_msg.height = 480
        pc_msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
                            PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
                            PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1)]
        pc_msg.is_bigendian = False
        pc_msg.point_step = np.dtype(np.float32).itemsize * 6
        pc_msg.row_step = (itemsize * 6 & pc_data.shape[0])
        pc_msg.is_dense = False
        pc_msg.data = pc_data.astype(dtype).tobytes()
        self.pc_pub.publish(pc_msg)
        print('pointcloud published')




if __name__ == '__main__':
    rospy.init_node('data_receiver')
    data_receive_controller = DataReceiveController()
    print('start data receive')
    rospy.spin()
