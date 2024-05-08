import rospy
from std_srvs.srv import Empty, EmptyRequest
from utils.distancing import distancing, distancing_horizontal
from hsr_agent.agent import Agent

def clean_the_table(agent: Agent):

    ### task params #################
    pick_table = 'breakfast_table'
    pick_position = 'clean_table_front'
    place_position = 'dishwasher_rack'
    tab_name = 'dishwasher_tablet'
    item_list = ['mug', 'bowl', 'plate', 'fork', 'spoon', 'knife']
    plate_radius = 0.10
    base_to_arm_dist = 0.5
    safe_flag = 0
    grasp_sleep = 0.5
    no_distancing_mode = True
    short_move = 2.0

    cutlery_box_position = [0, 0]
    place_position_dict = {'mug': [0, -0.24], 'bowl': [0.12, 0], 'plate': [0.1, -0.24], 'fork': cutlery_box_position,
                    'knife': cutlery_box_position, 'spoon': cutlery_box_position, tab_name: [0.05, 0.0]}

    is_using_check_grasp_dict = {'mug': True, 'bowl': True, 'plate': True,
                                 'fork': False, 'knife': False, 'spoon': False, tab_name: True}     # do not use check_grasp for motions that scrapes the table
    id_item_dict = {46: tab_name, 2: 'spoon', 3: 'fork', 4: 'plate', 5: 'bowl', 0: 'mug', 1: 'knife', 46: 'tab'}
    miss_count = 0
    is_picked = False
    double_check_item_list = ['plate', 'bowl']  # items that have ambiguous values when using check_grasp


    ###########################

    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())
    agent.grasp()
    agent.pose.move_pose()


    # agent.door_open()
    # agent.move_rel(2.0, 0, wait=True)
    agent.say('start clean the table', show_display=True)

    import pdb; pdb.set_trace()
    while True:
        # 1. go to pick table
        if safe_flag == 0:
            rospy.sleep(0.5)
            print('1. go to pick table')
            agent.move_abs(pick_position)
            agent.pose.table_search_pose()
            

        # 2.0 search
        agent.pose.table_search_pose()
        rospy.sleep(1)
        table_item_list = agent.yolo_module.detect_3d(pick_table) # eg. [0.8,0.0,0.7,20] (20 means knife) ()
        table_item_id_list = [table_item[3] for table_item in table_item_list] # eg. [20] (=knife)

        print("2.0 table_item_list:", table_item_list)
        print('    table_item_id_list:', table_item_id_list)

        # 2.1 select target_object_pc
        is_detected = False
        for item in item_list: # Item list is set already
            name, item_id, itemtype, grasping_type = agent.yolo_module.find_object_info_by_name(item)
            for table_item in table_item_list:
                if item_id == table_item[3]:
                    table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, name) # [0.8,0.0,0.7]
                    is_detected = True
                    pick_item_id = item_id # 20 (knife)
                    break
            if is_detected:
                break

        if not is_detected: # If nothing detected, go back! if something was, go next line.
            continue

        ########################################
        # Keep going only if detected
        ########################################

        # 2.2 calculate x, y and go.
        print('2.2 table_base_to_object_xyz', table_base_to_object_xyz)
        base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
        if item == 'plate':
            if no_distancing_mode:
                pass
            else:
                rospy.sleep(2)
                dist_to_base_x = distancing(agent.yolo_module.pc, pick_table, raw=True)
                plate_xyz = base_xyz
                dist_plate_x = plate_xyz[0] - (dist_to_base_x - base_to_arm_dist) + plate_radius
                print('dist_plate_x', dist_plate_x)
        agent.move_rel(-0.15, 0)

        print('2.3. current_item:', item)

        # Move Y direction
        if safe_flag == 0: # If safe_flag is 0, move y direction, and start from the beginning. 
            agent.move_rel(0, base_xyz[1], wait=True)
            safe_flag += 1
            continue

        #####################################################

        safe_flag = 0
        is_using_check_grasp = is_using_check_grasp_dict[item]      # choose miss detection between check_grasp and yolo

        if item == 'bowl':
            agent.pose.bring_bowl_pose(table=pick_table)
            agent.open_gripper()
            # agent.move_rel(0, base_xyz[1] + 0.04, wait=True)
            # agent.move_rel(base_xyz[0] + 0.15, 0, wait=True)

            agent.move_rel(base_xyz[0] + 0.15, base_xyz[1] + 0.04, wait=True)

            agent.pose.pick_bowl_max_pose(table=pick_table, height=-0.13) # modified from -0.1 to -0.13
            agent.grasp()
            rospy.sleep(0.5)

            is_picked = agent.pose.check_grasp()
            print(f"2.4 grasp value of item '{item}': {is_picked}")

            agent.pose.pick_up_bowl_pose(table=pick_table)
            agent.move_rel(-0.2, 0)


        elif item == 'mug':
            agent.pose.pick_side_cup_pose(table=pick_table)
            agent.open_gripper()
            # agent.move_rel(0, base_xyz[1]+0.03, wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.12, table=pick_table)
            # agent.move_rel(base_xyz[0]+0.20, 0, wait=True)
            agent.move_rel(base_xyz[0]+0.20, base_xyz[1] + 0.03, wait=True)

            agent.grasp()

            is_picked = agent.pose.check_grasp(threshold=-1.45)  # modify
            print(f"2.4 grasp value of item '{item}': {is_picked}")

            agent.move_rel(-0.2, 0)
        elif item == 'fork' or item == 'spoon' or item == 'knife':
            agent.pose.pick_top_pose(table=pick_table)
            agent.open_gripper()

            # agent.move_rel(0, base_xyz[1]-0.01, wait=True)
            # agent.move_rel(base_xyz[0] + 0.25, 0, wait=True)

            agent.move_rel(base_xyz[0] + 0.25, base_xyz[1] - 0.01, wait=True)

            agent.pose.arm_lift_top_table_down(height=-0.015, table=pick_table) # modify

            agent.grasp()
            agent.pose.arm_flex(-60)
            # agent.pose.pick_up_bowl_pose(table=pick_table)
            agent.move_rel(-0.17, 0)

        elif item == tab_name:                               # modify
            agent.pose.pick_top_pose(table=pick_table)
            agent.open_gripper()

            # agent.move_rel(0, base_xyz[1]+0.02, wait=True)
            # agent.move_rel(base_xyz[0] + 0.35, 0, wait=True)

            agent.move_rel(base_xyz[0] + 0.35, base_xyz[1] + 0.02, wait=True) # modify

            agent.pose.arm_lift_top_table_down(height=-0.005, table=pick_table) # modify
            agent.grasp()

            is_picked = agent.pose.check_grasp(threshold=-1.45)     # modify this for tab
            print(f"2.4 grasp value of item '{item}': {is_picked}")

            agent.pose.arm_flex(-60)
            # agent.pose.pick_up_bowl_pose(table=pick_table)
            agent.move_rel(-0.17, 0)
        # elif item == 'plate':
        #     hand_over_mode = True
        #     if hand_over_mode:
        #         agent.move_rel(-0.26, 0, wait=True)
        #         agent.pose.pick_plate_pose_fold(table=pick_table)
        #         agent.open_gripper()
        #         agent.say('please hand me the plate', show_display=True)
        #         rospy.sleep(2)
        #         agent.say('five');
        #         rospy.sleep(1)
        #         agent.say('four');
        #         rospy.sleep(1)
        #         agent.say('three')
        #         rospy.sleep(1)
        #         agent.say('two')
        #         rospy.sleep(1)
        #         agent.say('one')
        #         rospy.sleep(1)
        #         agent.grasp()
        #         rospy.sleep(1)
        #         is_picked = True
        #     else:
        #         agent.pose.bring_bowl_pose(table=pick_table)
        #         agent.open_gripper()
        #         # agent.move_rel(0, base_xyz[1] + 0.04, wait=True)
        #         # agent.move_rel(base_xyz[0] + 0.15, 0, wait=True)

        #         agent.move_rel(base_xyz[0] + 0.15, base_xyz[1] + 0.04, wait=True)

        #         agent.pose.pick_bowl_max_pose(table=pick_table, height=-0.13)
        #         agent.grasp()
        #         rospy.sleep(0.5)

        #         is_picked = agent.pose.check_grasp()
        #         print(f"2.4 grasp value of item '{item}': {is_picked}")

        #         agent.pose.pick_up_bowl_pose(table=pick_table)
        #         agent.move_rel(-0.3, 0)

        elif item == 'plate':
            agent.pose.pick_plate_pose(table=pick_table)
            agent.open_gripper()
            # agent.move_rel(0, plate_xyz[1], wait=True)
            # agent.move_rel(plate_xyz[0] + 0.18, 0, wait=True)
        
            agent.move_rel(base_xyz[0] + 0.18, base_xyz[1], wait=True)
        
            agent.pose.arm_lift_top_table_down(height=-0.025, table=pick_table)
        
            agent.move_rel(-base_xyz[0]-0.06, 0, wait=True)
            agent.pose.arm_lift_top_table_down(height=0.05, table=pick_table)
        
            agent.move_rel(-0.18, 0, wait=True)
            agent.pose.pick_plate_pose_fold(table=pick_table)
        
            agent.move_rel(0.075, 0, wait=True)  # slightly move forward
            agent.grasp()
        
            is_picked = agent.pose.check_grasp()
            print(f"2.4 grasp value of item '{item}': {is_picked}")

        # 3. return pose
        agent.pose.table_search_pose()

        if is_using_check_grasp:        # when it detects miss with check_grasp
            if is_picked:
                agent.say(f'I picked the {item}', show_display=True)
            else:
                if item in double_check_item_list:
                    rospy.sleep(0.5)
                    agent.move_abs(pick_position)
                    rospy.sleep(1)
                    current_table_item_list = agent.yolo_module.detect_3d(pick_table)
                    current_table_item_id_list = [current_table_item[3] for current_table_item in current_table_item_list]

                    table_item_id_list.sort()
                    current_table_item_id_list.sort()
                    if table_item_id_list == current_table_item_id_list:
                        miss_count += 1

                        if miss_count > 1:
                            agent.say(f'please hand me the {item}')
                            agent.open_gripper()

                            rospy.sleep(1)
                            agent.say('five');
                            rospy.sleep(1)
                            agent.say('four');
                            rospy.sleep(1)
                            agent.say('three');
                            rospy.sleep(1)
                            agent.say('two');
                            rospy.sleep(1)
                            agent.say('one');
                            rospy.sleep(1)

                            agent.grasp()
                        else:
                            print(f"3.3 you didn't pick the current item! (item: {item})")
                            continue
                    elif len(table_item_id_list) > len(current_table_item_id_list):
                        for i in range(len(current_table_item_id_list)):
                            table_item_id = table_item_id_list[i]
                            current_table_item_id = current_table_item_id_list[i]

                            if table_item_id != current_table_item_id:
                                item = id_item_dict[table_item_id]
                                print(f'target: {id_item_dict[pick_item_id]}')
                                print(f'picked: {item}')
                                agent.say(f'I picked the {item}', show_display=True)
                                break
                else:
                    miss_count += 1
                    if miss_count > 1:
                        agent.say(f'please hand me the {item}')
                        agent.open_gripper()

                        rospy.sleep(1)
                        agent.say('five');
                        rospy.sleep(1)
                        agent.say('four');
                        rospy.sleep(1)
                        agent.say('three');
                        rospy.sleep(1)
                        agent.say('two');
                        rospy.sleep(1)
                        agent.say('one');
                        rospy.sleep(1)

                        agent.grasp()
                    else:
                        print(f"3.1 you didn't pick the current item! (item: {item})")
                        continue
        else:       # when it detects miss with yolo vision
            rospy.sleep(0.5)
            agent.move_abs(pick_position)
            rospy.sleep(1)
            current_table_item_list = agent.yolo_module.detect_3d(pick_table)
            current_table_item_id_list = [current_table_item[3] for current_table_item in current_table_item_list]

            table_item_id_list.sort()
            current_table_item_id_list.sort()

            print('3.2 table_item_id_list', table_item_id_list)
            print('3.2 current_table_item_id_list', current_table_item_id_list)

            # if pick_item_id in current_table_item_id_list:
            if table_item_id_list == current_table_item_id_list:
                miss_count += 1

                if miss_count > 1:
                    agent.say(f'please hand me the {item}')
                    agent.open_gripper()

                    rospy.sleep(1)
                    agent.say('five');
                    rospy.sleep(1)
                    agent.say('four');
                    rospy.sleep(1)
                    agent.say('three');
                    rospy.sleep(1)
                    agent.say('two');
                    rospy.sleep(1)
                    agent.say('one');
                    rospy.sleep(1)

                    agent.grasp()
                else:
                    print(f"3.3 you didn't pick the current item! (item: {item})")
                    continue
            elif len(table_item_id_list) > len(current_table_item_id_list):
                for i in range(len(current_table_item_id_list)):
                    table_item_id = table_item_id_list[i]
                    current_table_item_id = current_table_item_id_list[i]

                    if table_item_id != current_table_item_id:
                        item = id_item_dict[table_item_id]
                        print(f'target: {id_item_dict[pick_item_id]}')
                        print(f'picked: {item}')
                        agent.say(f'I picked the {item}', show_display=True)
                        break

        miss_count = 0

        # 4. go place pos
          

        # rospy.sleep(5)
        pdb.set_trace()
        
        agent.move_abs(place_position)
        rospy.sleep(short_move)
        agent.pose.move_pose()
        agent.pose.head_tilt(-30) # look at the table you stupid robot

        agent.move_rel(-0.2,0, wait=True)
        # import pdb; pdb.set_trace()

         # for coding only bjkim 24th

        dishwasher = 'dishwasher'
        dishwasher_x = distancing(agent.yolo_module.pc, 'dishwasher_table', dist=0.5)

        agent.move_rel(dishwasher_x, 0, wait=True) # modify

        agent.pose.arm_lift_up(0.49)

        # dishwasher_y = distancing_horizontal(agent.yolo_module.pc, dishwasher_table)

        agent.move_rel(0, place_position_dict[item][1], wait=True)
        agent.move_rel(place_position_dict[item][0], 0, wait=True)

        arm_lift_value = agent.pose.place_cutlery_pose(table=dishwasher) # temp
        agent.pose.arm_lift_up(arm_lift_value - 0.1)
        
        # agent.move_rel(0.27 + place_position_dict[item][0], place_position_dict[item][1], wait=True)

        agent.open_gripper()

        # agent.pose.pick_up_bowl_pose(table=place_table)
        agent.pose.arm_lift_up(0.3)
        rospy.sleep(1)
        agent.move_rel(-0.3, 0)
        agent.pose.move_pose()
