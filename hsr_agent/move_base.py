import rospy
import tf
import tf.transformations
from utils.simple_action_client import SimpleActionClient
import numpy as np
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion,PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

NAVIGATION_STATUS = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

class MoveBase:
    def __init__(self, ABS_POSITION):
        self.abs_position = ABS_POSITION
        self.base_action_client = SimpleActionClient('/move_base/move', MoveBaseAction, "base_action_client")

        self.base_action_client.wait_for_server(timeout=2)
        
        self.listener = tf.TransformListener()
        # jykim
        self.initial_pose_pub = rospy.Publisher('/laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=10)

    def initial_pose(self, position):
        goal_x, goal_y, goal_yaw = self.abs_position[position]
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = goal_x
        msg.pose.pose.position.y = goal_y
        msg.pose.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0., 0., goal_yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = np.eye(6).flatten().tolist()
        # msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        self.initial_pose_pub.publish(msg)
        return

    def move_abs_test(self, goal_x, goal_y, goal_yaw=None, wait=True):
        
        if goal_yaw is None: goal_yaw = 0.
        rospy.loginfo(f"Moving to {goal_x, goal_y, goal_yaw}")
        
        self.base_action_client.wait_for_server(timeout=2)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose
        # send message to the action server
        self.base_action_client.send_goal(goal)
        if wait:
            # wait for the action server to complete the order
            self.base_action_client.wait_for_result()
            # print result of navigation
            action_state = self.base_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded.")
            else:
                rospy.logerr(f"Navigation Failed! (ERROR: GOAL {action_state})")
                return False

    # add wait argument by yspark and shlim
    def move_abs(self, position, wait=True):
        rospy.loginfo(f"Moving to {position}")
        # goal_x, goal_y, goal_yaw = self.abs_position[position]
        self.move_abs_coordinate(self.abs_position[position], wait)
        # self.base_action_client.wait_for_server(timeout=2)


    # added by shlim
    def move_abs_coordinate(self, coordinate, wait=True):
        rospy.loginfo(f"Moving to {coordinate}")
        goal_x, goal_y, goal_yaw = coordinate
        self.base_action_client.wait_for_server(timeout=2)

        pose = PoseStamped()
        # pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose
        # send message to the action server
        self.base_action_client.send_goal(goal)
        if wait:
            # wait for the action server to complete the order
            self.base_action_client.wait_for_result()
            # print result of navigation
            action_state = self.base_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded.")
            else:
                rospy.logerr(f"Navigation Failed! (GOAL {action_state})")
                return False
    # added by sujin for gpsr
    def move_abs_by_point(self, position, wait=True):
        goal_x, goal_y, goal_yaw = position
        self.base_action_client.wait_for_server(timeout=2)
        rospy.loginfo(f"Moving to {position}")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose
        # send message to the action server
        self.base_action_client.send_goal(goal)
        if wait:
            # wait for the action server to complete the order
            self.base_action_client.wait_for_result()
            # print result of navigation
            action_state = self.base_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded.")
            else:
                rospy.logerr(f"Navigation Failed! (ERROR: GOAL {action_state})")
                return False

    def move_rel(self, x, y, yaw=0, wait=False):
        self.base_action_client.wait_for_server(timeout=2)
        rospy.loginfo(f"Moving {x, y, yaw} relative to current position")

        pose = PoseStamped()
        # pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position = Point(x, y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose
        # send message to the action server
        self.base_action_client.send_goal(goal)
        # wait for the action server to complete the order
        if wait:
            self.base_action_client.wait_for_result(timeout=rospy.Duration(8))
            # print result of navigation
            action_state = self.base_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded.")
                return True
            else:
                rospy.logerr(f"Navigation Failed! (ERROR: GOAL {action_state})")
                self.base_action_client.cancel_all_goals()
                return False

        return True

    def get_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('map',
                                                             'base_footprint',
                                                             rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        euler_angles = tf.transformations.euler_from_quaternion(rot)
        yaw = euler_angles[2]

        return [trans[0], trans[1], yaw]

    def tf_camera_to_base(self, camera_point):
        return self.transform_coordinate('head_rgbd_sensor_rgb_frame', 'base_link',
                                  [camera_point[0], camera_point[1], camera_point[2]])

    def transform_coordinate(self, from_tf, to_tf, src_point):
        # src_point must be xyz!
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(to_tf, from_tf, rospy.Time(0))
                # euler = tf.transformations.euler_from_quaternion(rot)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        R = self.listener.fromTranslationRotation(trans, rot)
        out = np.dot(R, np.array([src_point[0], src_point[1], src_point[2], 1]))
        return out[0:-1]

    def transform_coordinate_array(self, from_tf, to_tf, src_point_array):
        # src_point must be xyz!
        while not rospy.is_shutdown():
            try:
                (_trans, _rot) = self.listener.lookupTransform(to_tf, from_tf, rospy.Time(0))
                # euler = tf.transformations.euler_from_quaternion(rot)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        R = self.listener.fromTranslationRotation(_trans, _rot)
        src_point_array = np.concatenate([src_point_array, np.ones((src_point_array.shape[0], 1))], axis=1)
        out = np.dot(R, src_point_array.T)
        out = out.T
        return out[:, 0:-1]
