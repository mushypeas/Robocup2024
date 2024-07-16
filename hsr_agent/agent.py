from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseGoal
# jykim
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
import tf.transformations
import cv2
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, Image
import ros_numpy
from hsr_agent.global_config import *

# hsr module
from hsr_agent.move_base import MoveBase
from hsr_agent.joint_pose import JointPose
from hsr_agent.gripper import Gripper
from hsr_agent.tts import TTS

# yolo
from module.yolov7.yolo_module import YoloModule
from open3d import geometry

# stt
# from module.stt.stt_client import stt_client
from module.stt.cloud_stt_hsr_mic import stt_client_hsr_mic
import numpy as np
from utils.distancing import distancing
import copy
from transformers import Speech2TextProcessor, Speech2TextForConditionalGeneration
from utils.simple_action_client import SimpleActionClient
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
from cv_bridge import CvBridge
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Twist
from utils.axis_transform import Axis_transform
from utils.in_arena_check import InArena
import time
from utils.marker_maker import MarkerMaker


class Agent:
    def __init__(self):
        
        # head display
        self.head_display_file_pub = rospy.Publisher('/hsr_head_file', String, queue_size=10)
        self.head_text_pub = rospy.Publisher('/hsr_head_msg', String, queue_size=10)
        self.head_display_image_pub = rospy.Publisher('/hsr_head_img', Image, queue_size=10)
        # rgb
        self.rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, self._rgb_callback)
        # depth
        self._depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image,self._depth_callback)
        # pointcloud
        self._pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self._pc_callback)
        # lidar
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.pc = None
        self.bridge = CvBridge()
        # cur_pose
        self.cur_pose = None
        self.cur_pose_sub = rospy.Subscriber('/hsrb/omni_base_controller/state',
                                             JointTrajectoryControllerState, self._cur_pose_callback)
        self.cur_vel_sub = rospy.Subscriber('/base_velocity', Twist, self._cur_vel_callback)
        self.joint_trajectory_client = SimpleActionClient(
            '/hsrb/head_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction,
            'joint_trajectory_client')
        self.joint_trajectory_client.wait_for_server(2)
        self.marker_maker = MarkerMaker('/snu/robot_path_visu')

        # make sure the controller is running
        # This doesn't work sometimes...
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers', timeout=5.0)
        
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                rospy.loginfo(f"{c.name} is {c.state}")
                if c.name == 'head_trajectory_controller' and c.state == 'running':
                    running = True


        # hsr module instantiate
        self.move_base = MoveBase(ABS_POSITION_Robocup) #
        self.gripper = Gripper()
        self.pose = JointPose(TABLE_DIMENSION, self.gripper)
        self.tts = TTS()

        # object
        self.object_list = OBJECT_LIST
        self.table_dimension = TABLE_DIMENSION  # for gpsr

        # yolo
        self.yolo_module = YoloModule(OBJECT_LIST)
        # jykim static-map

        static_topic_name = '/static_obstacle_ros_map'

        grid = rospy.wait_for_message(static_topic_name, OccupancyGrid, timeout=5.0)
        # map meta-info
        self.static_res = grid.info.resolution
        self.static_w = grid.info.width
        self.static_h = grid.info.height
        # map grid
        grid = np.array(grid.data).reshape(self.static_w, self.static_h)
        grid = np.flip(np.flip(grid, axis=0), axis=1).T
        grid = np.where(grid >= 80, 255, 0).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.static_grid = cv2.dilate(grid, kernel, iterations=1)
        # jykim odom
        self.odom_listener = tf.TransformListener()
        # jykim dynamic-map
        self.dynamic_obstacles = None
        self._dyn_map_sub = rospy.Subscriber('/dynamic_obstacle_map',
                                             OccupancyGrid, self._dyn_map_callback)
        self.isstopped = True
        self.last_moved_time = time.time()
        self.cur_vel = [0.0, 0.0, 0.0] # x,y,yaw
        self.axis_transform = Axis_transform()

        # return if the point is in arena
        self.arena_check = InArena(ARENA_EDGES)
        # for carry my luggage (todo)
        rospy.loginfo('HSR agent is ready.')

    def _rgb_callback(self, data):
        # rospy.loginfo('image_sub node started')
        self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 5.0  # remove nans
        self.ranges = data_np # jykim: save ranges data from LiDAR
        center_idx = data_np.shape[0] // 2
        self.dist = np.mean(data_np[center_idx - 15: center_idx + 15])

    def _depth_callback(self, data):
        self.stamp = data.header.stamp
        self.depth_for_pc = geometry.Image(ros_numpy.numpify(data).astype(np.uint16))
        self.depth_image = ros_numpy.numpify(data).astype(np.uint16)

    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)

    def _cur_pose_callback(self, pose_msg):
        self.cur_pose = pose_msg.actual.positions

    def _cur_vel_callback(self, msg):
        x, y, z = msg.linear.x, msg.linear.y, msg.linear.z
        xw, yw, zw = msg.angular.x, msg.angular.y, msg.angular.z
        self.cur_vel = [x, y, zw]
        # rospy.loginfo("{:.3f}{:.3f}{:.3f}{:.3f}{:.3f}{:.3f}".format(x,y,z,xw,yw,zw))
        if x == 0 and y == 0 and z == 0 and zw == 0:
            self.isstopped = True
        else:
            self.isstopped = False

        if round(x, 4) != 0 or round(y, 4) != 0:
            self.last_moved_time = time.time()
    def _dyn_map_callback(self, grid):

        # Transform Static Map First (To Egocentric)
        st = rospy.get_time()
        while not rospy.is_shutdown():
            try:
                trans, rot = self.odom_listener.lookupTransform('/base_link',
                                                                '/map',
                                                                rospy.Time(0))
                x = trans[0]
                y = trans[1]
                yaw = tf.transformations.euler_from_quaternion(rot)[-1]
                break
                '''
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    if rospy.get_time() - st > 0.5:
                        rospy.logwarn('dyn tf failed')
                        # if too late, return this and receive next dynamic map
                        return
                '''
            except (tf.LookupException):
                rospy.logwarn('Lookup')
                if rospy.get_time() - st > 0.5:
                    rospy.logwarn('dyn tf failed')
                    # if too late, return this and receive next dynamic map
                    return
            except (tf.ConnectivityException):
                rospy.logwarn('Connectivity')
                if rospy.get_time() - st > 0.5:
                    rospy.logwarn('dyn tf failed')
                    # if too late, return this and receive next dynamic map
                    return
            except (tf.ExtrapolationException) as e:
                rospy.logwarn(e)
                if rospy.get_time() - st > 0.5:
                    rospy.logwarn('dyn tf failed')
                    # if too late, return this and receive next dynamic map
                    return
        SO2 = cv2.getRotationMatrix2D((self.static_w//2, self.static_h//2),
                                      yaw / np.pi * 180, 1)
        T = np.array([[0., 0., -y / self.static_res],
                      [0., 0., -x / self.static_res]])
        SE2 = SO2 + T
        static = cv2.warpAffine(self.static_grid, SE2,
                                (self.static_w, self.static_h),
                                flags=cv2.INTER_NEAREST, borderValue=0)
        # Dynamic Obstacle Map
        dyn_w = grid.info.width
        dyn_h = grid.info.height
        dyn_res = grid.info.resolution
        grid = np.array(grid.data).reshape((dyn_w, dyn_h))
        grid = np.flip(np.flip(grid, 0), 1).T # dynamic obstacle map
        dyn = np.where(grid >= 80, 255, 0).astype(np.uint8)
        # resize dynamic obstacle map to fit dynamic map to static resolution
        scale_factor = dyn_res / self.static_res
        dyn_w = int(dyn_w * scale_factor)
        dyn_h = int(dyn_h * scale_factor)
        dyn = cv2.resize(dyn, (dyn_w, dyn_h), interpolation=cv2.INTER_NEAREST)
        SO2 = cv2.getRotationMatrix2D((dyn_w//2, dyn_h//2),
                                      yaw / np.pi * 180, 1)
        dyn = cv2.warpAffine(dyn, SO2,
                             (dyn_w, dyn_h),
                             flags=cv2.INTER_NEAREST, borderValue=0)
        self.dynamic_obstacles_with_static = dyn
        static = static[self.static_h//2-dyn_h//2: self.static_h//2+dyn_h//2,
                        self.static_w//2-dyn_w//2: self.static_w//2+dyn_w//2]
        # Mask dynamic map with static map to discard static obstacles
        mask = np.logical_not(static > 0)
        dynamic_obstacles = (mask * dyn).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.dynamic_obstacles = cv2.morphologyEx(dynamic_obstacles,
                                                  cv2.MORPH_OPEN,
                                                  kernel,
                                                  iterations=2)
        vis = np.hstack([static, dyn, self.dynamic_obstacles])
        #cv2.imshow('dynamic obstalces', vis)
        #cv2.waitKey(10)
        return


    def initial_pose(self, position):
        self.move_base.initial_pose(position)

    def move_abs(self, position, wait=True):
        self.move_base.move_abs(position, wait)

    def move_abs_coordinate(self, coordinate, wait=True):
        self.move_base.move_abs_coordinate(coordinate, wait)

    def move_rel(self, x, y, yaw=0, wait=False):
        return self.move_base.move_rel(x, y, yaw, wait)
    
    def move_rel_AFAP(self, x, y, yaw=0, interval = 0.05):
        return self.move_base.move_rel_AFAP(x, y, yaw, interval)
    
    def move_distancing(self, place, dist=0.6, timeout=3.0):
        # Move for distancing
        # tilt = 0
        # if place in ['kitchen_table', 'breakfast_table']:
        #     tilt = -30
        # if place in ['shelf_1f', 'shelf_2f']:
        #     tilt = 5
        # self.pose.move_pose()
        # self.pose.head_tilt(tilt)
        st = rospy.get_time()
        while self.pc is None:
            if rospy.get_time() - st > timeout:
                rospy.logwarn('Distancing failed due to none type point cloud')
                return
        timeout -= rospy.get_time() - st
        corr = distancing(self.pc, place, dist, timeout)
        rospy.loginfo(f'{round(corr, 6)} m')
        # Check whether the return of distancing method is None
        if corr is None:
            return
        if abs(corr) > 0.3:
            rospy.logwarn('Distancing ignored due to extreme estimation')
            return
        self.move_rel(corr, 0., wait=True)
        return
    
    # jykim
    def inspection_obstacle_ahead(self, thresh=0.35, angle=45):

        N = len(self.ranges)
        '''
        edited by lsh
        '''
        covering_range = N // 4
        cur_vel_baselink = copy.deepcopy(self.cur_vel)
        cur_yaw = np.arctan2(cur_vel_baselink[1], cur_vel_baselink[0]) / np.pi * 180
        cur_yaw = -cur_yaw + covering_range // 2
        cur_yaw_idx = int(cur_yaw) * 4
        if 0 <= (cur_yaw_idx - (angle * 4 // 2)) and (cur_yaw_idx + (angle * 4 // 2)) < N:
            inspection_ranges = self.ranges[cur_yaw_idx - (angle * 4 // 2): cur_yaw_idx + (angle * 4 // 2)]
        else:
            return False

        '''
        -------
        '''
        #return np.any(inspection_ranges < thresh)
        # print("max obstacle distance", inspection_ranges.max())
        if not np.any(inspection_ranges < thresh):
            return False
        if self.dynamic_obstacles is None:
            return False
        else:
            H, W = self.dynamic_obstacles.shape
            insp_dist = int(2.2 / self.static_res)
            insp_area = self.dynamic_obstacles[H//2-insp_dist//2:H//2+insp_dist//2,
                                               W//2-insp_dist//2:W//2+insp_dist//2]
            '''
            ratio = (insp_area > 0).sum() / insp_area.size
            return (ratio > 0.1)
            '''
            return np.any(insp_area > 200)

    def move_abs_coordinate_safe(self, coordinate, thresh=0.35, timeout=5.0, giveup_timeout=8.0, angle=45):
        rospy.loginfo(f"Moving to {coordinate}")
        goal_x, goal_y, goal_yaw = coordinate
        self.move_base.base_action_client.wait_for_server(2)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)
        # send message to the action server
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()
        # before_moved_time = self.last_moved_time

        self.move_base.base_action_client.send_goal(goal)
        rospy.sleep(1)
        time_trapped = time.time()
        while self.move_base.base_action_client.get_state() != GoalStatus.SUCCEEDED:
            # print("state", self.move_base.base_action_client.get_state())
            if time.time() - self.last_moved_time > giveup_timeout:
                rospy.logwarn('Giveup moving!')
                self.move_base.base_action_client.cancel_all_goals()
                return True
            if self.inspection_obstacle_ahead(thresh=thresh, angle=angle):
                time_trapped = time.time()
                rospy.logwarn('Something Ahead !')
                self.move_base.base_action_client.cancel_all_goals()
                while self.inspection_obstacle_ahead(thresh=thresh, angle=angle):
                    if time.time() - self.last_moved_time > giveup_timeout:
                        rospy.logwarn('Giveup moving!')
                        self.move_base.base_action_client.cancel_all_goals()
                        return True
                    # 5초 이상 안움직이면 회전해서 빠져 나가게끔 하자
                    # move rel
                    rospy.logwarn('Still something in region')
                    if time.time() - time_trapped > timeout:
                        cur_pose = self.get_pose()
                        if np.linalg.norm(np.asarray(cur_pose[:2], dtype=np.float32) - np.asarray([goal_x, goal_y], dtype=np.float32)) < .15:
                            self.move_base.base_action_client.cancel_all_goals()
                            self.move_abs_coordinate([cur_pose[0], cur_pose[1], goal_yaw], wait=False)
                            return True
                        rospy.logwarn("move backward")
                        self.move_rel(-0.5, 0, wait=True)
                        break
                    rospy.sleep(0.1)
                time_trapped = time.time()
                goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base.base_action_client.send_goal(goal)

            else:
                if time.time() - time_trapped > timeout:
                    time_trapped = time.time()
                    if self.isstopped:
                        rospy.loginfo("timeout, send goal again")
                        self.move_base.base_action_client.cancel_all_goals()
                        cur_pose = self.get_pose()
                        if np.linalg.norm(np.asarray(cur_pose[:2], dtype=np.float32) - np.asarray([goal_x, goal_y], dtype=np.float32)) < .15:
                            print(np.linalg.norm(np.asarray(cur_pose[:2], dtype=np.float32) - np.asarray([goal_x, goal_y], dtype=np.float32)))
                            self.move_abs_coordinate([cur_pose[0], cur_pose[1], goal_yaw], wait=False)

                            return True
                        goal.target_pose.header.stamp = rospy.Time.now()
                        self.move_base.base_action_client.send_goal(goal)
            rospy.sleep(0.01)
        return True

    def move_abs_safe(self, position, thresh=0.01, timeout=3.0, giveup_timeout=50.0, angle=30):
        rospy.loginfo(f'Moving to {position}')
        # goal_x, goal_y, goal_yaw = self.move_base.abs_position[position]
        self.move_abs_coordinate_safe(self.move_base.abs_position[position], thresh, timeout, giveup_timeout, angle)
        # self.move_base.base_action_client.wait_for_server(2)

    def move_abs_by_point(self, position):
        self.move_base.move_abs_by_point(position)

    def get_pose(self, print_option=True):
        cur_p = self.move_base.get_pose()
        if print_option:
            print([round(cur_p[0], 4), round(cur_p[1], 4), round(cur_p[2], 4)])
        return self.move_base.get_pose()

    # tts
    def say(self, sentence, show_display=False):
        self.tts.say(sentence)
        if show_display:
            self.head_show_text(sentence)

    # stt
    def stt(self, sec=5., mode=None):
        return stt_client_hsr_mic(sec=sec, mode=mode)
        # return stt_client(sec=sec)
        pass

    # gripper
    def open_gripper(self, wait=True):
        rospy.loginfo("Gripper opened")
        self.gripper.gripper_command(1.0, wait=wait)

    def grasp(self, force=1.0, weak=False, wait=True):
        rospy.loginfo("Gripper closed")
        if weak:
            # self.gripper.grasp(0.1)
            self.gripper.grasp(0.05, wait=wait)
        else:
            self.gripper.grasp(force, wait=wait)

    def grasp_degree(self, radian):
        self.gripper.gripper_command(radian)

    def door_open(self, thres=1.5):
        while self.dist <= thres:
            rospy.sleep(1.0)
            print('door closed')
        self.say('door open'); rospy.sleep(1)
        self.say('three'); rospy.sleep(1)
        self.say('two');   rospy.sleep(1)
        self.say('one');   rospy.sleep(1)
        return True

    def head_show_image(self, file_name='images/snu.png'):
        if file_name == 'fork' or file_name == 'knife' or file_name == 'spoon':
            file_name = 'images/cutlery.png'
        elif file_name == 'plate':
            file_name = 'images/plate.png'
        elif file_name == 'bowl':
            file_name = 'images/bowl.png'
        elif file_name == 'snu':
            file_name = 'images/snu.png'
        elif file_name == 'red':
            file_name = 'images/red.png'
        elif file_name == 'green':
            file_name = 'images/green.png'
        elif file_name == 'STT Fail':
            file_name = 'images/STT Fail.png'
        elif file_name == 'Neutral':
            file_name = 'images/Neutral.png'
        elif file_name == 'Take Menu':
            file_name = 'images/Take Menu.png'
        else:
            pass
        self.head_display_file_pub.publish(file_name)
    def head_show_text(self, show_text):
        self.head_text_pub.publish(show_text)
    def head_display_image_pubish(self, image):
        head_msg = self.bridge.cv2_to_imgmsg(image, encoding='passthrough')
        self.head_display_image_pub.publish(head_msg)
    def is_occluded(self, point, frame='map'):
        assert frame == 'map' or frame == 'base_link'
        if len(point) == 2:
            point = [point[0], point[1], 1]
        if frame != 'base_link':
            point = self.axis_transform.transform_coordinate(frame, 'base_link', point)

        h, w = self.dynamic_obstacles_with_static.shape
        target_x_in_pixel = max(min(h - 1, h // 2 - int(point[0] / 0.05)), 0)
        target_y_in_pixel = max(min(w - 1, w // 2 - int(point[1] / 0.05)), 0)
        if self.dynamic_obstacles_with_static[target_y_in_pixel, target_x_in_pixel] > 220:

            return True
        else:
            return False



if __name__ == '__main__':
    rospy.init_node('in_agent_py', disable_signals=True)
    agent = Agent()
    agent.say('hello')
    rospy.spin()


