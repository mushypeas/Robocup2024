from geometry_msgs.msg import Point, PoseStamped, Quaternion
import rospy
import tf.transformations
from hsrb_interface import Robot
rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher('goal', PoseStamped, queue_size=10)

# wait to establish connection between the navigation interface
# move_base and navigation_log_recorder node
while pub.get_num_connections() < 2:
    rospy.sleep(0.1)

# input goal pose

goal_x, goal_y, goal_yaw =4.349187868004667, 2.5028621883979887, 1.5496914794986787

# fill ROS message
goal = PoseStamped()
# goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"
goal.pose.position = Point(goal_x, goal_y, 0)
quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
goal.pose.orientation = Quaternion(*quat)

# publish ROS message
pub.publish(goal)
