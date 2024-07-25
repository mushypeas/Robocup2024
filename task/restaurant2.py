import os
import rospy
import roslaunch
import numpy as np
from utils.simple_action_client import SimpleActionClient
import tf
import time
import math


from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion,PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sklearn.cluster import KMeans
from std_msgs.msg import Int16MultiArray
from utils.marker_maker import MarkerMaker
import dynamic_reconfigure.client
from playsound import playsound

# class MoveBaseStandalone:
#     def __init__(self):
#         self.base_action_client = SimpleActionClient('/move_base/move', MoveBaseAction, "base_action_client")
#         self.base_action_client.wait_for_server(timeout=2)
#         # jykim
#         self.initial_pose_pub = rospy.Publisher('/laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=10)
#
#     def move_abs(self, agent, goal_x, goal_y, goal_yaw=None, wait=True):
#         theta = 0
#         r = 0.8
#         while True:
#             _goal_x = goal_x - r * math.cos(math.pi*(theta/180))
#             _goal_y = goal_y + r * math.sin(math.pi * (theta / 180))
#             _goal_yaw = -math.pi*(theta/180)
#             if goal_yaw is None: goal_yaw = 0.
#             self.base_action_client.wait_for_server()
#             pose = PoseStamped()
#             pose.header.stamp = rospy.Time.now()
#             pose.header.frame_id = "map"
#             pose.pose.position = Point(_goal_x, _goal_y, 0)
#             quat = tf.transformations.quaternion_from_euler(0, 0, _goal_yaw)
#             pose.pose.orientation = Quaternion(*quat)
#
#             goal = MoveBaseGoal()
#             goal.target_pose = pose
#             # send message to the action server
#             self.base_action_client.send_goal(goal)
#             if wait:
#                 # wait for the action server to complete the order
#                 self.base_action_client.wait_for_result()
#                 # print result of navigation
#                 action_state = self.base_action_client.get_state()
#                 if action_state == GoalStatus.SUCCEEDED:
#                     rospy.loginfo("Navigation Succeeded.")
#                     break
#                 else:
#                     rospy.loginfo("Navigation FAILED!!!!")
#                     agent.say('re-route')
#                     #playsound('./Tools/xtioncam_capture/ding_3x.mp3')
#                     theta = (theta+30)%360
#                     continue
#
#
#     def move_rel(self, x, y, yaw=0, wait=False):
#         self.base_action_client.wait_for_server()
#
#         pose = PoseStamped()
#         pose.header.stamp = rospy.Time.now()
#         pose.header.frame_id = "base_link"
#         pose.pose.position = Point(x, y, 0)
#         quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
#         pose.pose.orientation = Quaternion(*quat)
#
#         goal = MoveBaseGoal()
#         goal.target_pose = pose
#         # send message to the action server
#         self.base_action_client.send_goal(goal)
#         # wait for the action server to complete the order
#         if wait:
#             self.base_action_client.wait_for_result()
#             # print result of navigation
#             action_state = self.base_action_client.get_state()
#             print("action state", action_state)
#             if action_state == GoalStatus.SUCCEEDED:
#                 rospy.loginfo("Navigation Succeeded.")
#                 return True
#             else:
#                 rospy.loginfo("Navigation FAILED!!!!")
#                 return False
#
#         return True

'''
def nav_target_from_pc(pc, tlx, tly, brx, bry):

    seg = pc[tly: bry, tlx: brx]
    X = seg['x']
    Y = seg['y']
    Z = seg['z']
    idx = np.logical_not(np.isnan(Z))
    depth_flat = Z[idx].flatten()
    side_flat = X[idx].flatten()
    projection = np.vstack([depth_flat, side_flat]).T
    rospy.loginfo('clustering started ...')
    labels = KMeans(n_clusters=2,
                    tol=1e-3,
                    algorithm='auto').fit_predict(projection)
    if (labels == 1).sum() >= (labels == 0).sum():
        projection = projection[labels == 1]
    else:
        projection = projection[labels == 0]
    return np.mean(projection[:, 0]) - 0.5, -np.mean(projection[:, 1])
'''

def nav_target_from_pc(pc, table, K=100):

    median_models = []
    table = table[np.all(table >= 0, 1)] # remain valid keypoints only
    print(table)
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
    print(median_models)
    if len(median_models) <= 0: return None
    mean_model = np.array(median_models).mean(0)
    return mean_model[2] - 0.5, -mean_model[0]

def head_circle_cb(config):
    rospy.loginfo(config)

def head_map_cb(config):
    rospy.loginfo(config)
def restaurant(agent):
    rospy.loginfo('Initialize Hector SLAM')
    # Kill existing nodes and replace them with others
    # os.system('rosnode kill /pose_integrator')
    # os.system('rosnode kill /move_base')
    head_map_client = dynamic_reconfigure.client.Client("/tmc_map_merger/inputs/head_rgbd_sensor",
                                                        config_callback=head_map_cb)
    head_map_client.update_configuration({"enable": True})
    head_obstacle_client = dynamic_reconfigure.client.Client(
        '/tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle', config_callback=head_circle_cb)
    head_obstacle_client.update_configuration({"forbid_radius": 0.05,
                                               "obstacle_radius": 0.3,
                                               "obstacle_occupancy": 80
                                               })
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    rospy.sleep(4)
    '''
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["./task/hector.launch"])
    launch.start()
    '''
    # slam_args = ['tidyboy_nav_stack', 'hector.launch']
    # nav_args  = ['tidyboy_nav_stack', 'nav_stack.launch']
    # slam_launch_file = roslaunch.rlutil.resolve_launch_arguments(slam_args)
    # nav_launch_file  = roslaunch.rlutil.resolve_launch_arguments(nav_args)
    # slam = roslaunch.parent.ROSLaunchParent(uuid,
    #                                         slam_launch_file,
    #                                         sigint_timeout=10.0,
    #                                         sigterm_timeout=5.0)
    # nav  = roslaunch.parent.ROSLaunchParent(uuid,
    #                                         nav_launch_file,
    #                                         sigint_timeout=10.0,
    #                                         sigterm_timeout=5.0)
    # slam.start()
    # nav.start()
    rospy.sleep(3.)
    # StanAlone MoveBase
    # move = MoveBaseStandalone()
    agent.say('start restaurant')
    #marker_maker = MarkerMaker('/snu/robot_path_visu')

    for _ in range(100):
        try:
            agent.head_show_image('Neutral')
            agent.pose.head_tilt(0)
            agent.pose.restaurant_move_pose()
            agent.say('Please wave your hand to order')
            rospy.sleep(3.)
            start_time = time.time()
            Dx = 0
            Dy = 0
            offset = 0

            while True:
                now = time.time()
                if now-start_time > 7:
                    agent.say('Please wave your hand to order')
                    rospy.sleep(3.)
                    start_time = now


                msg = rospy.wait_for_message('/snu/openpose/bbox', Int16MultiArray)
                if len(msg.data) == 0: continue
                
                # bbox_arr = np.array(msg.data)
                # # bbox_arr = bbox_arr.reshape(len(bbox_arr)//4, 4).tolist()
                # if len(bbox_arr) == 0: continue
                
                table_arr = np.array(msg.data)
                print(table_arr)
                table_arr = table_arr.reshape(len(table_arr)//34, 17, 2)
                pc = agent.pc
                if pc is None: continue
                '''
                target = bbox_arr[np.random.randint(len(bbox_arr))]
                Dx, Dy = nav_target_from_pc(pc, target[0], target[1], target[2], target[3])
                '''
                target = table_arr[np.random.randint(len(table_arr))]
                failure_count = 0
                try:
                    Dx, Dy = nav_target_from_pc(pc, target)
                    print(f'{Dx}   {Dy}')
                    break
                except TypeError as e:
                    print('Type Error?', e)
                    failure_count += 1
                    if failure_count >= 50:
                        offset += 0.5
                        agent.move_abs_coordinate([offset, 0, 0], wait=True)
                    else:
                        continue
                #Dx -= 0.6
                #Dy -= np.sign(Dy) * 0.1
                #if np.allclose(np.array([Dx, Dy]), np.zeros(2), rtol=0., atol=1e-1): continue
                #break
            #marker_maker.pub_marker([offset + Dx , Dy, 1], 'base_link')
            agent.move_abs_coordinate_restaurant([offset + Dx, Dy, 0])
            #move.move_abs(1., 0., wait=True)
            rospy.sleep(1.)
            while not rospy.is_shutdown():
                agent.head_show_image('red')
                agent.say('Please say items you like to order in proper format after the ding sound')
                rospy.sleep(4.5)
                # TODO Order Image & STT
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
            agent.move_abs_coordinate([0., 0., 0.], wait=True)
            rospy.sleep(1.)
            agent.say(f'Bartender, please give me {item_parsed}.')
            rospy.sleep(2.)
            agent.pose.restaurant_give_pose()
            rospy.sleep(7.)
            agent.say('Thank you!')
            rospy.sleep(3.)
            # 4. Give the customer items they ordered
            agent.pose.restaurant_move_pose()
            agent.move_abs_coordinate_restaurant([offset + Dx, Dy, 0])
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
            agent.move_abs_coordinate([0., 0., 0.], wait=True)
            rospy.sleep(3.)
        except KeyboardInterrupt:
            break
    # nav.shutdown()
    # slam.shutdown()
    agent.say('The task is finished!')



