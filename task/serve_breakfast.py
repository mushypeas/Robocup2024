import rospy
from utils.distancing import distancing
from std_srvs.srv import Empty, EmptyRequest
def serve_breakfast(agent):
    ### task params #################
    milk_height = 0.13  # [m]
    cereal_height = 0.24  # [m]
    pick_table = 'breakfast_table'
    place_table = 'kitchen_table'

    pick_position = 'breakfast_table'
    pick_position_bypass = 'kitchen_entrance'
    place_position = 'kitchen_table'
    item_list = ['bowl', 'cracker', 'cereal_red', 'cereal_black', 'milk', 'scrub', 'spoon', 'fork', 'knife']  # 'bowl', 'cereal', 'milk', 'spoon'

    bowl_to_cereal_spill_xy = [-0.2, 0.15]  # bowl to cereal
    bowl_to_milk_spill_xy = [-0.1, 0.05]  # bowl to milk
    bowl_to_spoon_place_xy = [0, -0.13]  # bowl to spoon

    # try except params
    default_base_xyz = [0.376, -0.1, 0.823]
    # bonus params
    milk_bonus = True
    cereal_bonus = True

    ###########################
    #### test param ##########
    pick_only_mode = False

    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    ### task start ##
    # agent.door_open()
    agent.say('start serve breakfast')
    agent.move_rel(2.0, 0, wait=False)
    agent.move_abs('breakfast_bypass')


    ###########################
    while True:
        # 1. go to breakfast table
        agent.pose.move_pose()
        rospy.sleep(0.5)
        agent.move_abs(pick_position)
        dist_to_table = distancing(agent.yolo_module.pc, pick_table)
        agent.move_rel(dist_to_table, 0)

        # 2. search
        agent.pose.table_search_pose_breakfast()
        rospy.sleep(2)

        # 2.1 detect all objects in pick_table
        table_item_list = agent.yolo_module.detect_3d(pick_table)

        is_detected = False
        for item in item_list:
            print('item', item)
            name, item_id, itemtype, grasping_type = agent.yolo_module.find_object_info_by_name(item)
            for table_item in table_item_list:
                if item_id == table_item[0]:

                    # 2.2 select target_object_pc
                    table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, name)
                    is_detected = True
                    break
            if is_detected:
                break

        if not is_detected:
            continue


        # print('table_base_to_object_xyz', table_base_to_object_xyz)

        # 2.3 calculate dist with offset
        try:
            base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
        except:
            continue
        print('gripper_to_object_xyz', base_xyz)
        agent.move_rel(-0.15, 0)

        # 4. pick
        if item == 'bowl':
            agent.pose.pick_bowl_pose_last(table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0] + 0.18, 0, wait=True)
            agent.pose.arm_lift_up(0.625)
            agent.grasp()
            agent.pose.arm_lift_up(0.69)
        elif item == 'spoon' or item == 'fork' or item == 'knife':
            agent.pose.pick_top_pose_last(table='breakfast_table')
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0] + 0.15 + 0.1, 0, wait=True)
            agent.pose.arm_lift_up(0.605)
            agent.grasp()
            rospy.sleep(0.5)
            agent.pose.arm_lift_up(0.69)
        else:   # milk, cereal (cracker, pringles)
            if item == 'cereal_red' or item == 'cracker' or item == 'pringles':
                object_height = cereal_height / 2
            else: # milk
                object_height = milk_height / 2
            agent.pose.pick_object_side_pose(object_height, table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1]-0.01, wait=True)
            agent.move_rel(base_xyz[0] + 0.15, 0, wait=True) # + 0.15
            if item == 'cereal_red' or item == 'cracker' or item == 'pringles':
                agent.grasp()
            else: # milk
                agent.grasp(weak=True)
                rospy.sleep(0.5)
            rospy.sleep(0.5)


        # 5. return pose
        agent.move_rel(-0.4, 0)
        agent.pose.table_search_go_pose()
        agent.pose.check_grasp()

        if pick_only_mode:
            agent.open_gripper()
            continue

        # 6. go place pos
        agent.move_abs(place_position)
        # rospy.sleep(1)
        # dist_to_table = distancing(agent.yolo_module.pc, place_table, dist=0.7)
        # print('dist_to_table', dist_to_table)
        # agent.move_rel(dist_to_table, 0)

        if item != 'bowl':
            try:
                rospy.sleep(1)
                table_item_list = agent.yolo_module.detect_3d(place_table)
                table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, 'bowl')
                base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type=2) # 2 == bowl
                print('bowl base_xyz',base_xyz)
            except:
                base_xyz = default_base_xyz
                print('no bowl detected. set default base_xyz', base_xyz)
                cereal_bonus = False
                milk_bonus = False

        # 7. place
        if item == 'bowl':
            agent.pose.place_bowl_pose()
            agent.move_rel(0.3, -0.15, wait=True)
            agent.pose.arm_lift_object_table_down(0.18, table=place_table)
            agent.open_gripper()
            rospy.sleep(1.0) # gazebo wait
            agent.move_rel(-0.3, 0)
        elif item == 'cereal_red' or item == 'cracker':
            if cereal_bonus:
                object_height = cereal_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_cereal_spill_xy[0], base_xyz[1]+bowl_to_cereal_spill_xy[1], wait=True)
                agent.pose.wrist_roll(120)
                agent.pose.wrist_roll(0)
                agent.move_rel(0, 0.15, wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0) # gazebo wait
                agent.move_rel(-0.3, 0)
            else:
                object_height = cereal_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_cereal_spill_xy[0], base_xyz[1]+bowl_to_cereal_spill_xy[1]+0.25, wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0)  # gazebo wait
                agent.move_rel(-0.3, 0)
        elif item == 'milk' or item == 'scrub':
            if milk_bonus:
                object_height = milk_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_milk_spill_xy[0]+0.2, base_xyz[1]+bowl_to_milk_spill_xy[1]+0.08, wait=True)
                agent.pose.wrist_roll(80)
                agent.pose.wrist_roll(0)
                agent.move_rel(0.1, 0.07, wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0) # gazebo wait
                agent.move_rel(-0.4, 0)
            else:
                object_height = milk_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0] + bowl_to_milk_spill_xy[0], base_xyz[1] + bowl_to_milk_spill_xy[1]+0.17,
                               wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0)  # gazebo wait
                agent.move_rel(-0.4, 0)
        elif item == 'spoon' or item == 'fork' or item == 'knife':
            agent.move_rel(0, base_xyz[1]+bowl_to_spoon_place_xy[1], wait=True)
            agent.pose.place_top_pose(0.05, table=place_table)
            agent.move_rel(base_xyz[0]+bowl_to_spoon_place_xy[0]+0.1, 0, wait=True)
            agent.pose.arm_lift_object_table_down(0.23, table=place_table) # top_pose = 0.2
            agent.open_gripper()
            rospy.sleep(2.0) # gazebo wait
            agent.move_rel(-0.4, 0)