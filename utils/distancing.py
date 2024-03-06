import rospy
import numpy as np
import tf
from hsr_agent.global_config import TABLE_DIMENSION

def distancing(pointcloud, table, dist=0.6, timeout=3., raw=False):
    table_height = TABLE_DIMENSION[table][2]
    # Transform from depth camera to base_link
    listener = tf.TransformListener()
    st = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform('/base_link', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))
            R = listener.fromTranslationRotation(trans, rot)
            X = pointcloud['x']
            Y = pointcloud['y']
            Z = pointcloud['z']
            H = np.ones(X.shape)
            PC = np.stack([X, Y, Z, H], axis=-1)
            PC = np.reshape(PC, (-1, 4))
            PC = PC[np.logical_not(np.isnan(PC).sum(axis=-1))]
            basePC = np.matmul(PC, R.T)
            filteredPC = basePC[np.abs(basePC[:, 2] - table_height) < 0.01]
            medianx = np.min(filteredPC[:, 0])
            filteredPC = filteredPC[np.abs(filteredPC[:, 0] - medianx) < 0.05]
            corr = np.mean(filteredPC[:, 0])
            if raw:
                return corr
            return corr - dist

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if rospy.get_time() - st > timeout:
                rospy.logwarn('Distancing failed due to tf Exceptions')
                return None

def distancing_horizontal(pointcloud, table, thres=0.05, timeout=3.):
    table_height = TABLE_DIMENSION[table][2]
    # Transform from depth camera to base_link
    listener = tf.TransformListener()
    st = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform('/base_link', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))
            R = listener.fromTranslationRotation(trans, rot)
            X = pointcloud['x']
            Y = pointcloud['y']
            Z = pointcloud['z']
            H = np.ones(X.shape)
            PC = np.stack([X, Y, Z, H], axis=-1)
            PC = np.reshape(PC, (-1, 4))
            PC = PC[np.logical_not(np.isnan(PC).sum(axis=-1))]
            basePC = np.matmul(PC, R.T)

            filteredPC = basePC[np.abs(basePC[:, 2] - table_height) < 0.01]
            min_x_dist = np.nanmin(filteredPC[:, 0])
            print('min_x_dist', min_x_dist)
            x_dist_threshold = thres
            x_filtered_pc = filteredPC[np.where(min_x_dist + x_dist_threshold >= filteredPC[:, 0])]
            y_min, y_max = np.nanmin(x_filtered_pc[:, 1]), np.nanmax(x_filtered_pc[:, 1])
            print('y_min, y_max', y_min, y_max)
            y_center = (y_min + y_max) / 2
            return y_center

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if rospy.get_time() - st > timeout:
                rospy.logwarn('Distancing failed due to tf Exceptions')
                return None
