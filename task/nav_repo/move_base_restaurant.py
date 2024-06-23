import rospy
import tf

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from utils.simple_action_client import SimpleActionClient
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped

class MoveBaseRestaurant:
    def __init__(self):
        self.base_action_client = SimpleActionClient('/move_base', MoveBaseAction, "base_action_client")
        self.base_action_client.wait_for_server(timeout=2)
        self.initial_pose_pub = rospy.Publisher('/laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=10)
        self.listener = tf.TransformListener()
        
    def get_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        euler_angles = tf.transformations.euler_from_quaternion(rot)
        yaw = euler_angles[2]

        return [trans[0], trans[1], yaw]
    
    def move_rel(self, x, y, yaw=0, wait=False):
        self_x, self_y, self_yaw = self.get_pose()
        goal_x, goal_y, goal_yaw = x + self_x, y + self_y, yaw + self_yaw
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
                rospy.logerr("Navigation FAILED!!!!")
                return False
