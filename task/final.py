import rospy
from std_srvs.srv import Empty, EmptyRequest
from utils.distancing import distancing

# tech spec
# 1. pick & place with unseen object detection
# 2. chat gpt with stt

def final(agent):
    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    phase = int(input('phase: '))
    if phase == 1:
        agent.pose.move_pose()
        # 0. reach to the kitchen table
        agent.move_abs_safe('final_kitchen_table')
        agent.pose.head_tilt(10)
        agent.say("Hello, time to have dinner?", show_display=True)
        # 1. mug
        input('0')

    # 2. wine spill
    if phase <= 1:
        # explain
        agent.pose.head_tilt(-10)
        rospy.sleep(4)
        agent.pose.head_tilt(10)
        agent.say("Okay lets have some wine,\nbut it seems like\nthe cup is not prepared.\nI'll get it for you.", show_display=True)
        rospy.sleep(5)

        agent.move_abs_safe('shelf_front')
        rospy.sleep(1)

        agent.pose.head_tilt(0)

        while True:
            try:
                rospy.sleep(1)
                table_item_list = agent.yolo_module.detect_3d('shelf_2f')
                table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, 'mug')
                grasping_type = 4
                base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
                agent.say('ready')
                break
            except:
                print('cannot find the mug')
        agent.pose.pick_shelf_pose('shelf_2f')
        agent.open_gripper()
        agent.move_rel(0, base_xyz[1], wait=True)
        agent.move_rel(base_xyz[0], 0, wait=True)
        agent.grasp()
        rospy.sleep(1)
        agent.move_rel(-0.5, 0, wait=True)
        agent.pose.move_pose()
        agent.move_abs_safe('final_kitchen_table')  # desk

        # spill the wine
        rospy.sleep(1)
        dist_to_table = distancing(agent.yolo_module.pc, 'final_kitchen_table', dist=0.9)
        agent.move_rel(dist_to_table, 0, wait=True)

        agent.pose.place_side_pose('final_kitchen_table')
        rospy.sleep(2)
        agent.move_rel(0.5, 0, wait=True)
        agent.open_gripper()
        agent.move_rel(-0.5, 0, wait=True)
        agent.pose.move_pose()
        agent.move_abs_safe('final_kitchen_table')

        #  mug end
        input('1')


    # 2. spill the wine
    if phase <= 2:
        agent.move_abs_safe('final_kitchen_table')
        agent.say("Yeah sure,\ngive me the wine.", show_display=True)
        agent.pose.pick_shelf_pose('shelf_3f')
        agent.move_rel(0.15, 0)

        agent.open_gripper()
        input('2-1')
        agent.grasp()

        agent.pose.table_search_go_pose()
        while True:
            try:
                rospy.sleep(1)
                table_item_list = agent.yolo_module.detect_3d('desk')
                table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, 'mug')
                base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type=0)
                agent.say('ready')
                break
            except:
                print('cannot find the mug')
                continue

        #  spill
        object_height = 0.14
        agent.move_rel(-0.35, 0, wait=True)
        agent.pose.spill_object_pose(object_height, table='final_kitchen_table')
        agent.move_rel(base_xyz[0] - 0.05 + 0.28 + 0.08 + 0.1, base_xyz[1] + 0.3, wait=True)  # offset fix

        input('2-1')
        agent.pose.wrist_roll(90)
        agent.pose.wrist_roll(0)
        agent.move_rel(0, 0.07, wait=True)
        agent.pose.arm_lift_object_table_down(0.1, table='final_kitchen_table')
        agent.open_gripper()
        rospy.sleep(1.0)
        agent.move_rel(-0.4, 0)
        agent.pose.move_pose()
        input('2')

    if phase <= 3:
        agent.say('Okay. I will fetch\nsome breads for you.', show_display=True)
        rospy.sleep(3.0)
        agent.move_abs_safe('final_round_table')

        agent.pose.table_search_pose()
        while True:
            try:
                rospy.sleep(1)
                table_item_list = agent.yolo_module.detect_3d('kitchen_table')
                table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, 'pear')
                base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type=0)
                agent.say('ready')
                break
            except:
                print('cannot find the apple')
                continue

        agent.pose.pick_bread_final_pose('sink')
        agent.open_gripper()

        agent.move_rel(0, base_xyz[1] + 0.15, wait=True)  # offset fix
        agent.move_rel(base_xyz[0] + 0.3, 0, wait=True)  # offset fix
        rospy.sleep(3)
        agent.grasp()

        agent.move_rel(-0.6, 0, wait=True)
        agent.pose.move_pose(vertical=True)
        agent.move_abs_safe('final_kitchen_table')

        # place the bread
        dist_to_table = distancing(agent.yolo_module.pc, 'final_kitchen_table')
        agent.move_rel(dist_to_table, 0)

        agent.pose.neutral_pose(vertical=True)

        agent.move_rel(-0.3, -0.3, wait=True)  # fix
        agent.pose.pick_bread_final_pose('pantry_3f')
        agent.move_rel(0.45, 0, wait=True)  # fix

        agent.pose.arm_lift_up(0.4) # fix
        agent.open_gripper()
        agent.say('Here is the bread', show_display=True)
        rospy.sleep(5.0)
        agent.move_rel(-0.3, 0)
        agent.pose.move_pose()

        agent.say('Bon appetit.', show_display=True)
        rospy.sleep(1)
        input('3')



    # 5. unseen object detection
    if phase <= 4:
        agent.pose.move_pose()
        agent.say('Okay. I hope you \nenjoyed your dinner.', show_display=True)
        rospy.sleep(3)
        agent.say('and I will clean up \nthe table for you!', show_display=True)
        rospy.sleep(3)

        agent.move_abs_safe('final_kitchen_table')
        dist_to_table = distancing(agent.yolo_module.pc, 'final_kitchen_table')
        agent.move_rel(dist_to_table, 0)

        agent.pose.table_search_pose_breakfast()
        while True:
            try:
                rospy.sleep(1)
                table_item_list = agent.yolo_module.detect_3d('desk')
                table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, 'bowl')
                base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, 2)
                agent.say('ready')
                break
            except:
                print('cannot find the bowl')
            continue

        agent.move_rel(-0.15, 0)
        agent.pose.pick_bowl_pose_last(table='desk')
        agent.open_gripper()
        agent.move_rel(0, base_xyz[1], wait=True)
        agent.move_rel(base_xyz[0] + 0.18, 0, wait=True)
        agent.pose.arm_lift_up(0.59)
        agent.grasp()
        agent.pose.arm_lift_up(0.67)
        agent.move_rel(-0.4, 0)

        agent.pose.neutral_pose()

        agent.move_abs_safe('sink')


        dist_to_table = distancing(agent.yolo_module.pc, 'final_kitchen_table')
        agent.move_rel(dist_to_table, 0)

        #placing in the sink
        agent.pose.neutral_pose()
        agent.pose.head_tilt(0)
        agent.pose.arm_lift_up(0.69)
        agent.move_rel(0.3, 0, wait=True)

        input('4')
        agent.pose.arm_flex(-50) # todo
        agent.open_gripper()

        agent.pose.arm_flex(0) # todo
        agent.pose.move_pose()

    if phase <= 5:
        # 6. byebye pose in front of audiences
        agent.move_abs_safe('final_kitchen_table')  # fix
        # fix : make a byebye pose
        agent.say('Thank you for watching\n our demo', show_display=True)
        agent.pose.bye1_pose()
        agent.open_gripper()
        agent.pose.bye2_pose()
        while True:
            agent.pose.bye1_pose()
            agent.pose.bye2_pose()
