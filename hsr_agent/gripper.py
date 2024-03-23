import rospy
from utils.simple_action_client import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal


class Gripper:
    def __init__(self):
        self._follow_joint_trajectory_client = SimpleActionClient(
            "/hsrb/gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            "_follow_joint_trajectory_client"
        )
        self._grasp_client = SimpleActionClient(
            "/hsrb/gripper_controller/grasp",
            GripperApplyEffortAction,
            "_follow_joint_trajectory_client"
        )

    def grasp(self, force=1, wait=True):
        goal = GripperApplyEffortGoal()
        _HAND_MOMENT_ARM_LENGTH = 0.07
        goal.effort = - force * _HAND_MOMENT_ARM_LENGTH
        self._grasp_client.send_goal(goal)
        if wait:
            self._grasp_client.wait_for_result(rospy.Duration(1.0))


    def gripper_command(self, open_angle):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['hand_motor_joint']
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rospy.Duration(1.0))
        ]

        self._follow_joint_trajectory_client.send_goal(goal)
        self._follow_joint_trajectory_client.wait_for_result(rospy.Duration(5.0))
