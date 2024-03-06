import rospy
import numpy as np
import sys
sys.path.append('./')
from hsr_agent.agent import Agent
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import PointCloud2, CameraInfo, PointField
from utils.axis_transform import Axis_transform
from utils.depth_to_pc import Depth2PC
import copy

class FloorYoloDetection:
    def __init__(self):
        rospy.Subscriber('/snu/yolo_conf', Int16MultiArray, self.cb)
        self.pc_pub = rospy.Publisher('/snu/head_rgbd_sensor/pointcloud', PointCloud2, queue_size=10)
        self.yolo_data = None

    def cb(self, data):
        self.yolo_data = data

    def publish_pointcloud(self, pt_np):
        assert pt_np.shape[-1] == 3
        h, w, _ = pt_np.shape
        pc_data = pt_np.astype(np.float32)
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

        ##### pointcloud2 parameters #####
        pc_to_pub.header.frame_id = 'head_rgbd_sensor_rgb_frame'
        pc_to_pub.width = w
        pc_to_pub.height = h
        pc_to_pub.fields = pf
        pc_to_pub.is_bigendian = False
        pc_to_pub.point_step = 12
        pc_to_pub.header.stamp = rospy.Time.now()
        pc_to_pub.row_step = w * 12
        pc_to_pub.is_dense = False
        pc_to_pub.data = pc_data.tobytes()
        self.pc_pub.publish(pc_to_pub)


if __name__ == '__main__':
    rospy.init_node('floor_yolo')
    agent = Agent()
    d2pc = Depth2PC()
    yolo_detect = FloorYoloDetection()
    axis_transform = Axis_transform()
    while not rospy.is_shutdown():
        # _pc = agent.pc.reshape(480, 640)
        # pc_np = np.array(_pc.tolist())[:, :, :3]
        pc_np = d2pc.get_pc()
        # print("pc_np", pc_np.shape)
        if yolo_detect.yolo_data is None or yolo_detect.yolo_data == []:
            # print("no data")
            pass

        else:
            bag_yolo_data = yolo_detect.yolo_data.data
            for idx in range(len(bag_yolo_data) // 6):
                item = bag_yolo_data[6 * idx: 6 * (idx + 1)]
                cent_x, cent_y, width, height, class_id, conf_percent = item
                start_x = cent_x - (width // 2)
                start_y = cent_y - (height // 2)

                object_pc = pc_np[start_y:start_y + height // 2, start_x:start_x + width]
                object_pc = object_pc.reshape(-1, 3)
                points_by_base_link = axis_transform.tf_camera_to_base(object_pc, multi_dimention=True)
                # lift up process
                points_by_base_link[:, 2] = 0.4
                lifted_object_pc = axis_transform.transform_coordinate_array('base_link', 'head_rgbd_sensor_rgb_frame',\
                                                                             points_by_base_link).reshape(height // 2, width, 3)
                # pc_np[start_y:start_y + height, start_x:start_x + width] = lifted_object_pc.reshape(height, width, 3)
                yolo_detect.publish_pointcloud(lifted_object_pc)
