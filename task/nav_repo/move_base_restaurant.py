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
        
    def move_rel(self, x, y, yaw=0, wait=False):
        self.base_action_client.wait_for_server()

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
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
            self.base_action_client.wait_for_result()
            # print result of navigation
            action_state = self.base_action_client.get_state()
            print("action state", action_state)
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded.")
                return True
            else:
                rospy.loginfo("Navigation FAILED!!!!")
                return False

        return True