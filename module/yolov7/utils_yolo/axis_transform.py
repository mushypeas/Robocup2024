import rospy
from tf import TransformListener, Transformer
import tf.transformations
import numpy as np


class Axis_transform:
    def __init__(self):
        self.listener = TransformListener()


    def tf_camera_to_base(self, camera_point, multi_dimention=False):
        if multi_dimention:
            return self.transform_coordinate_array('head_rgbd_sensor_rgb_frame', 'base_link', camera_point)
        else:
            return self.transform_coordinate('head_rgbd_sensor_rgb_frame', 'base_link',
                                      [camera_point[0], camera_point[1], camera_point[2]])


    def transform_coordinate_array(self, from_tf, to_tf, src_point_array):
        # src_point must be xyz!
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(to_tf, from_tf, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        R = self.listener.fromTranslationRotation(trans, rot)
        src_point_array = np.concatenate([src_point_array, np.ones((src_point_array.shape[0], 1))], axis=1)
        out = np.dot(R, src_point_array.T)
        out = out.T
        return out[:, 0:-1]

if __name__ == '__main__':
    rospy.init_node('test_hsr_tf')
    axis_tf = Axis_transform()
    print(axis_tf.get_pose())
    # origin = np.array(([0.09911522, 0.04511706, 1.01600003],
    #                     [0.09911522, 0.04511706, 1.01600003]))
    # print(origin)
    # base_link = axis_tf.transform_coordinate_array('head_rgbd_sensor_link', 'base_link', origin)
    # print(base_link)
    # camera_link = axis_tf.transform_coordinate_array('base_link', 'head_rgbd_sensor_link', base_link)
    # print(camera_link)

