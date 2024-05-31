import os
import rospy
import numpy as np
import tf
import time
import math
import subprocess
import cv2
from collections import deque

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16MultiArray
from utils.marker_maker import MarkerMaker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MoveBaseStandalone:
    def __init__(self):
        self.base_action_client = SimpleActionClient('/move_base', MoveBaseAction, "base_action_client")
        self.base_action_client.wait_for_server(timeout=2)
        self.initial_pose_pub = rospy.Publisher('/laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=10)
        self.track_queue = deque()
        self.save_one_time = True

    def move_zero(self, agent):
        while self.track_queue:
            cur_track = self.track_queue.pop()
            agent.move_abs_coordinate_safe(cur_track):

        rospy.loginfo("Returned to zero position.")

    def move_abs(self, agent, goal_x, goal_y, goal_yaw=None):
        self.base_action_client.wait_for_server(5)
        while not rospy.is_shutdown():
            if goal_yaw is None: goal_yaw = 0.
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "base_link"
            pose.pose.position = Point(goal_x, goal_y, 0)
            quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
            pose.pose.orientation = Quaternion(*quat)
            goal = MoveBaseGoal()
            goal.target_pose = pose
            self.base_action_client.send_goal(goal)
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                action_state = self.base_action_client.get_state()
                if action_state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Navigation Succeeded.")
                    return True
                elif action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED:
                    rospy.logwarn("Invalid Navigation Target")
                    return False
                else:
                    pass

    def move_to_edge(self, agent, edge_x, edge_y):
        rospy.loginfo(f"Moving to edge detected position: x={edge_x}, y={edge_y}")
        return self.move_abs(agent, edge_x, edge_y)


class EdgeDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.canny_pub = rospy.Publisher('/canny_edges', Image, queue_size=10)

    def process_frame(self, data):
        frame = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 150)
        rows, cols = edges.shape
        f = np.fft.fft2(edges)
        fshift = np.fft.fftshift(f)
        crow, ccol = rows // 2, cols // 2
        sigma = 50
        x, y = np.ogrid[:rows, :cols]
        mask = np.exp(-((x - crow) ** 2 + (y - ccol) ** 2) / (2 * sigma ** 2))
        fshift = fshift * mask
        f_ishift = np.fft.ifftshift(fshift)
        img_back = np.fft.ifft2(f_ishift)
        img_back = np.abs(img_back)
        img_back = (img_back - np.min(img_back)) / (np.max(img_back) - np.min(img_back)) * 255
        img_back = np.uint8(img_back)
        kernel = np.ones((1, 1), np.uint8)
        img_back = cv2.dilate(img_back, kernel, iterations=1)
        img_back = cv2.erode(img_back, kernel, iterations=1)
        morph = img_back
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.contours = contours
        morph = cv2.bitwise_not(morph)
        morph = np.uint8(morph)
        canny_img_msg = self.bridge.cv2_to_imgmsg(morph, 'mono8')
        if canny_img_msg is not None:
            self.canny_pub.publish(canny_img_msg)
        return contours


def restaurant(agent):
    rospy.loginfo('Initializing Edge Detection Navigation')
    move = MoveBaseStandalone()
    edge_detector = EdgeDetector()

    rospy.Subscriber("/camera/rgb/image_raw", Image, edge_detector.process_frame)
    agent.say('start restaurant')
    marker_maker = MarkerMaker('/snu/robot_path_visu')

    for _ in range(10):
        try:
            agent.head_show_image('Neutral')
            agent.pose.head_tilt(0)
            agent.pose.restaurant_move_pose()
            agent.say('Please wave your hand to order')
            rospy.sleep(3.)
            start_time = time.time()

            while True:
                now = time.time()
                if now - start_time > 5:
                    agent.say('Please wave your hand to order')
                    rospy.sleep(3.)
                    start_time = now

                contours = edge_detector.contours
                if not contours: continue

                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M['m00'] == 0: continue

                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                rospy.loginfo(f"Detected hand wave at: x={cx}, y={cy}")

                # Transform image coordinates to robot coordinates if needed
                target_x, target_y = transform_image_to_robot_coordinates(cx, cy)
                move_success = move.move_to_edge(agent, target_x, target_y)
                if move_success:
                    agent.say("I found the customer. I will calculate the pathway toward the customer.")
                    rospy.sleep(4)
                    marker_maker.pub_marker([target_x, target_y, 1], 'base_link')
                    break

            while not rospy.is_shutdown():
                agent.head_show_image('red')
                agent.say('Please say items you like to order in proper format after the ding sound')
                rospy.sleep(4.5)
                agent.head_show_image('green')
                result = agent.stt(5.)
                raw, item_parsed = result
                if len(item_parsed) == 0:
                    agent.say('I am sorry that I could not recognize what you said. Please answer me again.')
                    agent.head_show_image('STT Fail')
                    rospy.sleep(6.)
                    continue
                rospy.loginfo(f'STT Result: {raw} => {item_parsed}')
                agent.head_show_text(f'{item_parsed}')
                rospy.sleep(1.)
                agent.say('Is this your order? Please say Yes or No to confirm after the ding sound')

                rospy.sleep(6.)
                _, confirm_parsed = agent.stt(3.)
                if confirm_parsed != 'yes':
                    agent.say('I am sorry that I misunderstand your order. Please answer me again.')
                    agent.head_show_text('Speech Recognition Failed!')
                    rospy.sleep(6.0)
                    continue
                agent.say('I received your order!')
                agent.head_show_image('Neutral')
                rospy.sleep(3.)
                break

            move.move_zero(agent)
            rospy.sleep(1.)
            agent.say(f'Bartender, please give me {item_parsed}.')
            rospy.sleep(2.)
            agent.pose.restaurant_give_pose()
            rospy.sleep(2.)
            for i in range(5, 0, -1):
                agent.say(str(i))
                rospy.sleep(1)
            agent.say('Thank you!')
            rospy.sleep(3.)
            # 4. Give the customer items they ordered
            agent.pose.restaurant_move_pose()
            goback_flag = move.move_abs(agent, target_x, target_y)
            if not goback_flag:
                move.move_to_edge(agent, target_x, target_y)
            agent.pose.restaurant_give_pose()
            agent.head_show_image('Take Menu')
            agent.say(f'Here is your {item_parsed}, Take menu in ')
            rospy.sleep(4.)
            for i in range(5, 0, -1):
                agent.say(str(i))
                rospy.sleep(1)

            # 5. Return to the start position (zero pose)
            agent.pose.restaurant_move_pose()
            agent.head_show_image('Neutral')
            move.move_zero(agent)
            rospy.sleep(3.)
        except KeyboardInterrupt:
            break
    agent.say('The task is finished!')

def transform_image_to_robot_coordinates(cx, cy):
    # 여기에 카메라 좌표를 로봇 좌표로 변환하는 코드를 작성합니다.
    # 예시로 단순 변환을 사용합니다.
    target_x = cx * 0.05
    target_y = cy * 0.05
    return target