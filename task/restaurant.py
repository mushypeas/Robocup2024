import os
import rospy
import roslaunch
import numpy as np
from utils.simple_action_client import SimpleActionClient
import tf
import time
import math
import subprocess

from cv_bridge import CvBridge
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion,PoseWithCovarianceStamped, Twist
from sklearn.cluster import KMeans
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import LaserScan, Image
from utils.marker_maker import MarkerMaker
from playsound import playsound
from std_srvs.srv import Trigger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import dynamic_reconfigure.client
import Levenshtein


## To Check ##
cleaning_supplies = ["soap", "dishwasher tab", "washcloth", "sponges"]
drinks = ["cola", "ice tea", "water", "milk", "big coke", "fanta", "dubbelfris"]
food = ["cornflakes", "pea soup", "curry", "pancake mix", "hagelslag", "sausages", "mayonaise"]
decorations = ["candle"]
fruits = ["pear", "plum", "peach", "lemon", "orange", "strawberry", "banana", "apple"]
snacks = ["stroopwafel", "candy", "liquorice", "crisps", "pringles", "tictac"]
dishes = ["spoon", "plate", "cup", "fork", "bowl", "knife"]

object_list = cleaning_supplies + drinks + food + decorations + fruits + snacks + dishes

min_interval_arc_len = 0.8
unit_rad = 0.25 * ( math.pi / 180 )
avg_dist_move_dist_ratio = 3
################ Maybe Constant? ################
## main frequency
main_freq = 1
main_period = 1.0 / main_freq

## LiDAR dist
min_dist = 1.0
max_dist = 5.0

## LiDAR index
lidar_index = 963
center_index = lidar_index // 2

## YOLO img size
yolo_img_height = 480
yolo_img_width = 640

doing_lookup = False
'''

foo@bar:~/robocup2024/module/waver_detector$> python run_openpose.py


'''

## TO Check func ##

def cluster(word, arr):
    distances = [(Levenshtein.distance(word, a), a) for a in arr]
    closest_match = min(distances, key=lambda x: x[0])
    return closest_match[1]

def calculate_human_rad(human_center_x, yolo_img_width):
    human_center_bias = human_center_x - yolo_img_width / 2
    return -human_center_bias / 640

def index_to_rad(idx):
    return (idx - center_index) * unit_rad



## main ##

class MoveBaseStandalone:
    def __init__(self):
        self.base_action_client = SimpleActionClient('/move_base', MoveBaseAction, "base_action_client")
        self.base_action_client.wait_for_server(timeout=2)
        self.bridge = CvBridge()
        # jykim
        self.initial_pose_pub = rospy.Publisher('/laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=10)
        # jnpahk
        self.lidar_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self._lidar_callback)
        self.openpose_sub = rospy.Subscriber('/snu/openpose', Image, self._openpose_callback)
        self.listener = tf.TransformListener()
        self.last_checked_time = time.time()
        self.last_checked_pos = [0, 0]

        self.seg_img = None # jnpahk
        self.openpose_image = None


        rospy.Subscriber('/deeplab_ros_node/segmentation', Image, self._segment_cb)
        # reset map
        # rospy.wait_for_service('/reset_map')


    # def _openpose_callback(self, data):
    #     data_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    #     self.openpose_image = data_img

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = max_dist  # remove nans
        self.dists = data_np
        self.indices_in_range = np.where(self.dists > min_dist)[0].tolist()
        self.candidates = self.find_candidates()


    def _segment_cb(self, data): #jnpahk

        data_img = self.bridge.imgmsg_to_cv2(data, 'mono16')
        self.seg_img = data_img

    def find_candidates(self):
        indices_in_range = self.indices_in_range

        if not indices_in_range:
            return []

        segments = []
        start_idx = indices_in_range[0]
        end_idx = indices_in_range[0]

        for i in range(1, len(indices_in_range)):
            if indices_in_range[i] == indices_in_range[i - 1] + 1:
                end_idx = indices_in_range[i]
                
            else:
                start_rad = index_to_rad(start_idx)
                end_rad = index_to_rad(end_idx)
                min_dist = min(self.dists[start_idx:end_idx + 1])
        
                interval_arc_len = (end_rad - start_rad) * min_dist

                if interval_arc_len > min_interval_arc_len:
                    number_seg = int(interval_arc_len / min_interval_arc_len)
                    idx_len = end_idx - start_idx

                    for j in range(number_seg):
                        new_start_idx = start_idx + int(idx_len * j / number_seg)
                        new_end_idx = start_idx + int(idx_len * (j + 1) / number_seg)
                        new_min_dist = min(self.dists[new_start_idx:new_end_idx + 1])
                        new_start_rad = index_to_rad(new_start_idx)
                        new_end_rad = index_to_rad(new_end_idx)

                        segments.append([new_start_rad, new_end_rad, new_min_dist])

                start_idx = indices_in_range[i]
                end_idx = indices_in_range[i]

        start_rad = index_to_rad(start_idx)
        end_rad = index_to_rad(end_idx)
        min_dist = min(self.dists[start_idx:end_idx + 1])

        interval_arc_len = (end_rad - start_rad) * min_dist

        if interval_arc_len > min_interval_arc_len:
            number_seg = int(interval_arc_len / min_interval_arc_len)
            idx_len = end_idx - start_idx

            for j in range(number_seg):
                new_start_idx = start_idx + int(idx_len * j / number_seg)
                new_end_idx = start_idx + int(idx_len * (j + 1) / number_seg)
                new_min_dist = min(self.dists[new_start_idx:new_end_idx + 1])
                
                new_start_rad = index_to_rad(new_start_idx)
                new_end_rad = index_to_rad(new_end_idx)

                segments.append([new_start_rad, new_end_rad, new_min_dist])

        return segments
    

    def barrier_stop(self, agent, barrier_stop_thres=0.5): #jnpahk
        depth = agent.depth_image / 1000
        depth = depth[depth>0]
        if np.any(depth < barrier_stop_thres):
            return True
        else:
            return False

    
    def human_stop(self, agent, human_stop_thres=0.6): #jnpahk
        depth = agent.depth_image / 1000
        # depth = depth[depth>0]

        h, w = self.seg_img.shape
        # seg_img = self.seg_img[:, w*4 : w//4 * 3]
        seg_img = self.seg_img[:,:]

        # valid_depth_mask = depth > 0
        human_mask = (seg_img == 15) 
        human_y, human_x = np.where(human_mask)
        human_depth_values = depth[human_y, human_x]

        if human_depth_values[human_depth_values>0].size != 0 and (np.min(human_depth_values[human_depth_values>0]) < human_stop_thres): 
            print("human detected. finish.")
            return True
        else:
            return False


    def turn_around(self, angle=120):
        while not rospy.is_shutdown():
            # goal topic generation
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            cur_pos = self.get_pose()
            goal_x, goal_y, goal_yaw = cur_pos
            goal_yaw = goal_yaw + angle * math.pi / 180
            pose.pose.position = Point(goal_x, goal_y, 0)
            quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
            pose.pose.orientation = Quaternion(*quat)
            goal = MoveBaseGoal()
            goal.target_pose = pose
            # send message to the action server
            self.base_action_client.send_goal(goal)
            # Retry navigation to the customer until it success
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                action_state = self.base_action_client.get_state()
                if action_state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Turn around succeeded.")
                    return True
                elif action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED:
                    rospy.logwarn("Turn around aborted/rejected.")
                    return False
                else:
                    print(action_state)
                    pass

    def move_zero(self, agent):
        self.base_action_client.wait_for_server(timeout=2)
        goal = MoveBaseGoal()
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(0, 0, 0)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        goal.target_pose = pose

        self.last_checked_pos = self.get_pose()
        self.last_checked_time = time.time()
        
        self.base_action_client.send_goal(goal)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            action_state = self.base_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                rospy.loginfo('Success move zero by action state')
                return
            elif action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED:
                #reset = rospy.ServiceProxy('/reset_map', Trigger)
                #reset()
                #rospy.sleep(3.0)
                rospy.logwarn("Move zero aborted/rejected. Turn around.")
                self.base_action_client.send_goal(goal)

            elif self.barrier_stop(agent): #jnpahk
                rospy.logwarn("Barrier detected. Turn around.")
                agent.move_base.base_action_client.cancel_all_goals()
                self.turn_around()
                self.base_action_client.send_goal(goal)
            
            else:
                cur_pos = self.get_pose()
                if (time.time() - self.last_checked_time) > 3 and abs(self.last_checked_pos[0] - cur_pos[0]) < 0.1 and abs(self.last_checked_pos[1] - cur_pos[1]) < 0.1:
                    agent.move_base.base_action_client.cancel_all_goals()
                    print("No movement detected. Trying to find the best interval.")
                    try:
                        candidates = self.candidates
                    except AttributeError:
                        continue
                    best_interval = self.get_best_candidate(candidates)

                    maware_count = 0

                    while not best_interval:
                        best_interval = self.get_best_candidate(candidates)

                        maware_count += 1

                        if maware_count > 10:
                            self.turn_around()
                            maware_count = 0

                    self.move_best_interval(best_interval, agent)

                    if abs(self.last_checked_pos[0]) < 0.5 and abs(self.last_checked_pos[1]) < 0.5:
                        rospy.loginfo("Success move zero by distancing")
                        return 

                    self.base_action_client.send_goal(goal)

            if (time.time() - self.last_checked_time) > 3:
                self.last_checked_pos = self.get_pose()
                self.last_checked_time = time.time()

    def move_customer(self, agent, goal_x, goal_y, goal_yaw=None):
        r = 0.3 # 0.5
        self.base_action_client.wait_for_server(5)
        theta = 0.
        rotate_delta = 30.
        spin_count = 0

        self.last_checked_pos = self.get_pose()
        self.last_checked_time = time.time()

        while not rospy.is_shutdown():
            # goal topic generation
            _goal_x = goal_x - r * math.cos(math.pi * (theta / 180))
            _goal_y = goal_y + r * math.sin(math.pi * (theta / 180))
            _goal_yaw = -math.pi*(theta/180)
            if goal_yaw is None: goal_yaw = 0.
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position = Point(_goal_x, _goal_y, 0)
            quat = tf.transformations.quaternion_from_euler(0, 0, _goal_yaw)
            pose.pose.orientation = Quaternion(*quat)
            goal = MoveBaseGoal()
            goal.target_pose = pose

            # send message to the action server
            self.base_action_client.send_goal(goal)
            
            # Retry navigation to the customer until it success
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                action_state = self.base_action_client.get_state()

                if (action_state == GoalStatus.SUCCEEDED and not doing_lookup) or self.human_stop(agent): #jnpahk
                    rospy.loginfo("Move Customer Succeeded.")
                    return _goal_x, _goal_y, _goal_yaw
                
                elif action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED:
                    rospy.logwarn("Move Customer Aborted.")
                    agent.say("I am searching \n the valid pathway. \n Please hold.", show_display=True)
                    rospy.sleep(3)
                    theta = (theta + rotate_delta) % 360
                    spin_count += 1

                    # if spin_count % 8 == 0:
                    #    reset = rospy.ServiceProxy('/reset_map', Trigger)
                    #    reset()
                    #    rospy.sleep(3.0)
  
                    if spin_count % 8 == 0:
                        r += 0.1

                    break

                elif self.barrier_stop(agent): #jnpahk
                    rospy.logwarn("Barrier detected. Turn around.")
                    agent.move_base.base_action_client.cancel_all_goals()
                    self.turn_around()
                    self.base_action_client.send_goal(goal)

                else:
                    cur_pos = self.get_pose()
                    if (time.time() - self.last_checked_time) > 3 and abs(self.last_checked_pos[0] - cur_pos[0]) < 0.1 and abs(self.last_checked_pos[1] - cur_pos[1]) < 0.1:
                        agent.move_base.base_action_client.cancel_all_goals()
                        print("No movement detected. \n Trying to find the best interval.")
                        try:
                            candidates = self.candidates
                        except AttributeError:
                            continue
                        best_interval = self.get_best_candidate(candidates)
                        maware_count = 0

                        while not best_interval:
                            best_interval = self.get_best_candidate(candidates)

                            maware_count += 1

                            if maware_count > 10:
                                self.turn_around()
                                maware_count = 0

                        self.move_best_interval(best_interval, agent)

                        if abs(_goal_x - self.last_checked_pos[0]) < r + 0.1 and abs(_goal_y - self.last_checked_pos[1]) < r + 0.1:
                            rospy.loginfo("Finish move customer by distancing.")
                            return _goal_x, _goal_y, _goal_yaw

                        self.base_action_client.send_goal(goal)

                if (time.time() - self.last_checked_time) > 3:
                    self.last_checked_pos = self.get_pose()
                    self.last_checked_time = time.time()

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

    def move_abs(self, agent, goal_x, goal_y, goal_yaw=None):
        self.base_action_client.wait_for_server(5)

        self.last_checked_pos = self.get_pose()
        self.last_checked_time = time.time()
        
        while not rospy.is_shutdown():
            # goal topic generation
            if goal_yaw is None: goal_yaw = 0.
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
            # Retry navigation to the customer until it success
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                action_state = self.base_action_client.get_state()
                if action_state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Move abs Succeeded.")
                    return True
                elif action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED:
                    rospy.logwarn("Move abs Aborted/Rejected.")
                    return False
                else:
                    pass


    
    def heuristic(self, start_rad, end_rad, avg_dist):
        avg_rad = (start_rad + end_rad) / 2
        
        # if self.human_rad:
        return -abs(avg_rad) #current center
        
        # else:
        #     ### TODO: heuristic function when human_yaw is None ###
        #     ### Momentum + past human_yaw ###
        #     return 0

    def get_best_candidate(self, candidates):
        heuristic_values = [self.heuristic(start, end, avg_dist) for start, end, avg_dist in candidates]
        # if not heuristic_values:
        #     return None
        if heuristic_values == []:
            return False
        best_idx = np.argmax(heuristic_values)
        return candidates[best_idx]
    
    def move_best_interval(self, interval, agent):

        start_rad = interval[0]
        end_rad = interval[1]
        avg_dist = interval[2]
        
        move_dist = avg_dist / avg_dist_move_dist_ratio
        avg_rad = (start_rad + end_rad) / 2
        
        x = move_dist * math.cos(avg_rad)
        y = move_dist * math.sin(avg_rad)
        yaw = 0
        # if self.human_rad:
        #     move_yaw = self.human_rad
        # else:
        #     move_yaw = 0
        cur_x, cur_y, cur_yaw = self.get_pose()
        goal_x = cur_x + x * math.cos(cur_yaw) + y * math.sin(cur_yaw)
        goal_y = cur_y + x * math.sin(cur_yaw) - y * math.cos(cur_yaw)
        goal_yaw = cur_yaw

        self.base_action_client.wait_for_server(5)

        self.last_checked_pos = self.get_pose()
        self.last_checked_time = time.time()

        while not rospy.is_shutdown():
            # goal topic generation
            if goal_yaw is None: goal_yaw = 0.
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
            # Retry navigation to the customer until it success
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                action_state = self.base_action_client.get_state()
                if action_state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Move best interval Succeeded.")
                    return True
                
                
                elif action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED:
                    rospy.logwarn("Move best interval Aborted/Rejected.")
                    self.turn_around()
                    return False
                elif self.barrier_stop(agent): #jnpahk
                    rospy.logwarn("Barrier detected. Turn around.")
                    agent.move_base.base_action_client.cancel_all_goals()

                    self.turn_around()
                    self.base_action_client.send_goal(goal)
                

                
                else:
                    cur_pos = self.get_pose()
                    if (time.time() - self.last_checked_time) > 3 and abs(self.last_checked_pos[0] - cur_pos[0]) < 0.1 and abs(self.last_checked_pos[1] - cur_pos[1]) < 0.1:
                        rospy.logwarn('STOP move_best_interval')
                        return False

                if (time.time() - self.last_checked_time) > 3:
                    self.last_checked_pos = self.get_pose()
                    self.last_checked_time = time.time()


def nav_target_from_pc(pc, table, robot_ori, K=100):

    median_models = []
    table = table[np.all(table >= 0, 1)] # remain valid keypoints only
    for _ in range(K):
        human_model = []
        for coords in table:
            centX = coords[0]
            centY = coords[1]
            deltaX = int(np.random.randn() * 5)
            deltaY = int(np.random.randn() * 5)
            imageX = np.clip(centX + deltaX, 0, 639)
            imageY = np.clip(centY + deltaY, 0, 479)
            sample = pc[imageY, imageX]
            X = sample['x']
            Y = sample['y']
            Z = sample['z']
            sample_cent = np.array([X, Y, Z])
            if np.isnan(sample_cent).any(): continue
            human_model.append(sample_cent)
        if len(human_model) <= 0: continue
        human_model = np.array(human_model)
        sorted_model = human_model[human_model[:, 2].argsort()]
        N = len(sorted_model)
        median_models.append(sorted_model[N//2 - N//4: N//2 + N//4, :].mean(0))
    if len(median_models) <= 0: return None
    mean_model = np.array(median_models).mean(0)
    Dx = mean_model[2]
    Dy = -mean_model[0]
    if robot_ori == 0:
        return Dx, Dy
    else:
        D = np.sqrt(Dx*Dx + Dy*Dy)
        alpha = np.arctan2(Dy, Dx)
        theta = robot_ori + alpha
        return D * np.cos(theta), D * np.sin(theta)


def get_one_item(agent):
    while not rospy.is_shutdown():
        agent.head_show_image('red')
        agent.say('Please say items you \n like to order in proper format\n !!!after the ding sound!!!', show_display=True)
        rospy.sleep(4.5)
        agent.head_show_image('green')
        result = agent.stt(5.)
        print('stt_result', result)
        item_parsed = cluster(result, object_list)
        if len(item_parsed) == 0:
            agent.say('I am sorry that I could not\n recognize what you said.\n Please answer me again.', show_display=True)
            agent.head_show_image('STT Fail')
            rospy.sleep(6.)
            continue
        # rospy.loginfo(f'STT Result: {raw} => {item_parsed}')
        agent.head_show_text(f'{item_parsed}')
        print('item parsed', item_parsed)
        rospy.sleep(1.)
        agent.say('Is this your order? \n Please say Yes or No to confirm \n !!!after the ding sound!!!', show_display=True)

        rospy.sleep(6.)
        result = agent.stt(3.)
        print('stt_result', result)
        confirm_parsed = cluster(result, ['yes', 'no'])
        print('item parsed', confirm_parsed)

        if confirm_parsed != 'yes':
            agent.say('I am sorry that \n I misunderstand your order.\n Please answer me again.', show_display=True)
            agent.head_show_text('Speech Recognition Failed!')
            rospy.sleep(6.0)
            continue
        
        return item_parsed

def restaurant(agent):

    rospy.loginfo('Initialize Hector SLAM')
    # Kill existing nodes and replace them with others
    os.system('rosnode kill /pose_integrator')
    os.system('rosnode kill /move_base')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    #jnpahk depth camera on

    head_map_client = dynamic_reconfigure.client.Client("/tmc_map_merger/inputs/head_rgbd_sensor")
    head_map_client.update_configuration({"enable": True})

    slam_args = ['tidyboy_nav_stack', 'hector.launch']
    nav_args  = ['tidyboy_nav_stack', 'nav_stack.launch']
    slam_launch_file = roslaunch.rlutil.resolve_launch_arguments(slam_args)
    nav_launch_file  = roslaunch.rlutil.resolve_launch_arguments(nav_args)
    slam = roslaunch.parent.ROSLaunchParent(uuid,
                                            slam_launch_file,
                                            sigint_timeout=10.0,
                                            sigterm_timeout=5.0)
    nav  = roslaunch.parent.ROSLaunchParent(uuid,
                                            nav_launch_file,
                                            sigint_timeout=10.0,
                                            sigterm_timeout=5.0)
    slam.start()
    nav.start()
    rospy.sleep(5.)
    # StanAlone MoveBase
    move = MoveBaseStandalone()
    agent.say('start restaurant', show_display=True)
    marker_maker = MarkerMaker('/snu/robot_path_visu')

    openpose_path = "/home/tidy/Robocup2024/restaurant_openpose.sh"
    seg_path = "/home/tidy/Robocup2024/seg.sh"
    whisper_path = "/home/tidy/Robocup2024/whisper.sh"
    
    openpose_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {openpose_path}; exec bash']
    seg_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {seg_path}; exec bash']
    whisper_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {whisper_path}; exec bash']

    byte_process = subprocess.Popen(openpose_command)
    seg_process = subprocess.Popen(seg_command)
    whisper_process = subprocess.Popen(whisper_command)
    
    print('OpenPose process started')
    print('Segmentation DeepLab process started')






    for _ in range(10):
        try:

            agent.head_show_image('Neutral')
            agent.pose.head_tilt(0)
            agent.pose.restaurant_move_pose()
            agent.say('Please wave your hand to order', show_display=True)
            rospy.sleep(3.)
            start_time = time.time()
            Dx = 0
            Dy = 0
            offset = 0
            first_lookup = 0
            failure_count = 0
            robot_ori = 0.

            while True:
                doing_lookup = True
                # HRI: tell customers to wave their hand
                first_lookup += 1
                if first_lookup % 3 == 0:
                    if (first_lookup // 3) % 3 == 0:
                        robot_ori = 0.
                        move.move_abs(agent, 0, 0, 0)
                    elif (first_lookup // 3) % 3 == 1:
                        robot_ori = -20. * np.pi / 180
                        move.move_abs(agent, 0, 0, -20. * np.pi / 180)
                    elif (first_lookup // 3) % 3 == 2:
                        robot_ori = 20. * np.pi / 180
                        move.move_abs(agent, 0, 0, 20. * np.pi / 180)
                    rospy.sleep(3)
                now = time.time()
                if now-start_time > 5:
                    agent.say('Please wave your hand to order', show_display=True)
                    rospy.sleep(3.)
                    start_time = now

                doing_lookup = False

                # Wait message of OpenPose results
                # Continue if no valid input received
                msg = rospy.wait_for_message('/snu/openpose/bbox', Int16MultiArray)

                '''
                This topic contains the coordinate of every bounding box that the module detects.
                data ← [*box_0_top_left_x, box_0_top_left_y, box_0_bottom_right_x, box_0_bottom_right_y, … ,
                box_N_top_left_x, box_N_top_left_y, box_N_bottom_right_x, box_N_bottom_right_y*]
                '''
                if len(msg.data) == 0: continue

                openpose_image = rospy.wait_for_message('/snu/openpose', Image)
                if openpose_image is not None:
                    agent.head_display_image_pubish(openpose_image)

                table_arr = np.array(msg.data)
                table_arr = table_arr.reshape(len(table_arr) // 34, 17, 2)  # num of people? 
                pc = agent.pc
                if pc is None: continue

                # Calculate the area of each bounding box and find the index of the largest one
                areas = []
                for box in table_arr:
                    top_left_x, top_left_y = box[0]
                    bottom_right_x, bottom_right_y = box[1]
                    area = (bottom_right_x - top_left_x) * (bottom_right_y - top_left_y)
                    areas.append(area)
                
                largest_box_index = np.argmax(areas)
                target = table_arr[largest_box_index]
                
                # Select the target customer with the largest bounding box
                failure_count = 0

                Dx, Dy = nav_target_from_pc(pc, target, robot_ori)
                print(f'Computed Target Dx: {Dx}   Dy: {Dy}')
                if np.isnan(Dx) or np.isnan(Dy):
                    failure_count += 1
                    if failure_count >= 10:
                        failure_count = 0
                        offset += 0.5
                        move.move_abs(agent, offset, 0)
                else:
                    break

            agent.say("I found the customer.\n I will calculate the pathway \n toward the customer.", show_display=False)
            rospy.sleep(4)
            marker_maker.pub_marker([offset + Dx, Dy, 1], 'base_link')
            customer_x, customer_y, customer_yaw = move.move_customer(agent, offset + Dx, Dy)
            rospy.sleep(1.)
            
            item1 = get_one_item()
            item2 = get_one_item()
            
            total_item = item1 + " " + item2
            
            agent.say('Do you have more order? \n Please say Yes or No \n !!!after the ding sound!!!', show_display=True)

            rospy.sleep(6.)
            result = agent.stt(3.)
            print('stt_result', result)
            confirm_parsed = cluster(result, ['yes', 'no'])
            print('item parsed', confirm_parsed)

            if confirm_parsed == 'yes':
                item3 = get_one_item()
                total_item = total_item + ' ' + item3
                        
            agent.say('I received your order!', show_display=True)
            agent.head_show_image('Neutral')
            rospy.sleep(3.)
            
            move.move_zero(agent)
            rospy.sleep(1.)
            agent.say(f'Bartender, please give me {total_item}.', show_display=True)
            rospy.sleep(2.)
            agent.pose.restaurant_give_pose()
            rospy.sleep(2.)
            for i in range(5, 0, -1):
                agent.say(str(i), show_display=True)
                rospy.sleep(1)
            agent.say('Thank you!')
            rospy.sleep(3.)
            # 4. Give the customer items they ordered
            agent.pose.restaurant_move_pose()
            goback_flag = move.move_abs(agent, customer_x, customer_y, customer_yaw)
            if not goback_flag:
                move.move_customer(agent, offset + Dx, Dy)
            agent.pose.restaurant_give_pose()
            agent.head_show_image('Take Menu')
            agent.say(f'Here is your {total_item}, Take menu in ', show_display=True)
            rospy.sleep(4.)
            for i in range(5, 0, -1):
                agent.say(str(i), show_display=True)
                rospy.sleep(1)

            # 5. Return to the start position (zero pose)
            agent.pose.restaurant_move_pose()
            agent.head_show_image('Neutral')
            move.move_zero(agent)
            rospy.sleep(3.)
        except KeyboardInterrupt:
            break
    nav.shutdown()
    slam.shutdown()
    agent.say('The task is finished!', show_display=True)


