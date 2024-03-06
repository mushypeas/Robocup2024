import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

sys.path.append('../')
from Tools.xtioncam_capture.hsr_capture import VisionController
from hsr_agent.agent import Agent
class xbox_controller:
    def __init__(self, agent, vision_controller):
        self.agent = agent
        rospy.sleep(2)
        rospy.Subscriber('/hsrb/joy', Joy, self.control_robot)
        self.max_velo = 0.2 #spec : 0.222m/s
        self.isMoving = False
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.buttons_isPressed = [False]*4 # ABXY
        self.isMovepose = True
        self.isGripped = False
        self.vision_controller = vision_controller

    def control_robot(self, data):
        if data.axes[2] < -1*0.2 and not(data.axes[0] == 0 and data.axes[1] == 0 and data.axes[3] == 0):
            self.isMoving = True
            twist = Twist()
            twist.linear.x = data.axes[1] * self.max_velo
            twist.linear.y = data.axes[0] * self.max_velo
            twist.angular.z = data.axes[3]
            self.vel_pub.publish(twist)
        if self.isMoving and (data.axes[2] < -1*0.2 and data.axes[0] == 0 and data.axes[1] == 0 and data.axes[3] == 0):
            self.isMoving = False
            self.vel_pub.publish(Twist())

        for i in range(4):
            if self.buttons_isPressed[i] is False and data.buttons[i] == 1:
                self.buttons_isPressed[i] = True
                if i == 2:
                    print('X pressed')
                    self.agent.get_pose()
                elif i == 3:
                    print('Y pressed')
                    if self.isMovepose:
                        self.agent.pose.table_search_pose()
                    else:
                        self.agent.pose.move_pose()
                    self.isMovepose = not self.isMovepose
                elif i == 0:
                    print('A pressed')
                    self.vision_controller.make_xbox_request()
                else: # i == 1
                    print('B pressed')
                    if self.isGripped:
                        self.agent.open_gripper()
                    else:
                        self.agent.grasp()
                    self.isGripped = not self.isGripped
            if self.buttons_isPressed[i] is True and data.buttons[i] == 0:
                self.buttons_isPressed[i] = False

if __name__ == '__main__':
    rospy.init_node('tidyboy_xbox_controller')
    agent = Agent()
    start_idx = int(input('start_idx : '))
    vision_controller = VisionController(start_idx, 'xbox')
    xbox_controller = xbox_controller(agent,vision_controller)
    rospy.spin()
