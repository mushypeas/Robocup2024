import rospy
from std_srvs.srv import Empty, EmptyRequest
from utils.distancing import distancing, distancing_horizontal
from hsr_agent.agent import Agent

def clean_the_table(agent: Agent):
    
    # things to be modified: pick_table, pick_position, place_postion 182 267

    ### task params #############################################
    # ABS_POSITIONS #
    pick_position = 'pos_dining_table_5'
    place_position = 'pos_dishwasher'
    # close_position1 = 'rack_close_position1'
    # close_position2 = 'rack_close_position2'
    # close_position3 = 'rack_close_position3'
    # open_position1 = 'rack_open_position1'

    # TABLE_DIMENSIONS #
    pick_table = 'tab_dining_table'
    # dishwasher_door = 'dishwasher_door'
    # dishwasher_rack = 'dishwasher_rack'
    place_table = 'tab_dishwasher'

    # POSE PARAMETERS #

    rack_close_arm_lift = 0.19
    rack_close_wrist_flex = -70

    # MODE PARAMETERS #
    no_distancing_mode = True
    door_open_mode = True
    picking_mode = True
    placing_mode = True
    rack_close_mode = True
# 
    # item_list = ['bowl','cup','fork', 'spoon', 'knife','plate'] #만약 cutlery와 plate가 멀다면.

    item_list = [ 'bowl','plate',  'fork', 'spoon', 'knife', 'cup' ] #만약 cutlery와 plate가 멀다면.
    plate_radius = 0.10
    base_to_arm_dist = 0.5

    short_move = 2.0

    default_x = 0.10

    cutlery_box_position = [default_x, 0.10] # 두번쨰 try에서 0.08정도?
    place_position_dict = {'bowl': [default_x, 0.15], 'cup': [default_x, 0.17], 'plate': [0.10, -0.10], #원래 plate 대회 시작 직전에 0.15 x ㅕㅆ음.
                            'fork': cutlery_box_position, 'knife': cutlery_box_position, 'spoon': cutlery_box_position,  }

    is_using_check_grasp_dict = {'cup': True, 'bowl': True, 'plate': True,
                                 'fork': False, 'knife': False, 'spoon': False}     # do not use check_grasp for motions that scrapes the table
    miss_count = 0
    is_picked = False
    double_check_item_list = ['plate', 'bowl']  # items that have ambiguous values when using check_grasp
    num_gripped_items = 0


    #############################################################

    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    safe_flag = 0

    agent.grasp()
    agent.pose.table_search_pose()

    if door_open_mode:
        agent.door_open()
        agent.move_rel(2.0, 0, wait=True)
        agent.say('Please remove all \n chairs to office', show_display=True)
        rospy.sleep(3)
        agent.say('Please remove all \n chairs to office', show_display=True)
        rospy.sleep(7)
        agent.say('Repeat, \n Please remove all \n chairs to office', show_display=True)
        rospy.sleep(5)
        
        # agent.move_abs_safe('living_living_1')
        # agent.move_abs_safe('living_living_2')
        agent.say('I will go to kitchen.',show_display=True)
        agent.move_abs_safe('hallway')
        agent.move_abs_safe('livingroom')
    
    agent.say('start clean the table', show_display=True)
    # agent.say('Please pull off the chair, which is infront of dishes, to the living room', show_display=True)
    # rospy.sleep(5)
    # import pdb; pdb.set_trace()

    while True:
        if picking_mode:
            
            # 1. go to pick table
            if safe_flag == 0:
                rospy.sleep(0.5)
                agent.move_abs(pick_position)
                

            # 2.0 search
            rospy.sleep(1)
            table_item_list = agent.yolo_module.detect_3d(pick_table) # eg. [0.8,0.0,0.7,20] (20 means knife) ()
            table_item_id_list = [table_item[3] for table_item in table_item_list] # eg. [20] (=knife)

            print("2.0 table_item_list:", table_item_list)
            print('    table_item_id_list:', table_item_id_list)

            # 2.1 select target_object_pc
            is_detected = False
            for item in item_list: # Item list is set already
                # import pdb; pdb.set_trace()
                name, item_id, _, grasping_type = agent.yolo_module.find_object_info_by_name(item)
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

            ####### safe_flag mountain #######

            print(pick_table)

            safe_flag = 0
            is_using_check_grasp = is_using_check_grasp_dict[item]      # choose miss detection between check_grasp and yolo
            
            if item == 'bowl':
                
                agent.pose.bring_bowl_pose(table=pick_table) # 살짝 들림
                agent.open_gripper()

                
                agent.move_rel(base_xyz[0] + 0.17, base_xyz[1] + 0.02, wait=True) # 0.17 0.04

                agent.pose.pick_bowl_max_pose_bj(table=pick_table, height=-0.1) # dining table이 나오면 사용!!
                # agent.pose.pick_bowl_max_pose(table=pick_table, height=-0.1) # office table, livingroom table 경우 사용! # 90도 가까움, -0.1 for 2023
               
                agent.grasp()
                rospy.sleep(0.5)

                is_picked = agent.pose.check_grasp()
                print(f"2.4 grasp value of item '{item}': {is_picked}")

                agent.pose.pick_up_bowl_pose(table=pick_table) # 위로 많이 들림.
                agent.move_rel(-0.2, 0)

                agent.say('I picked a bowl.', show_display=True)
                ##############################
                # # Hand me over Mode ########################
                # agent.move_rel(-0.35, 0, wait=True)
                # agent.pose.pick_plate_pose_fold(table=pick_table)
                # agent.open_gripper()
                # agent.say('please hand me the bowl', show_display=True)
                # rospy.sleep(4)
                # agent.say('five');
                # rospy.sleep(1)
                # agent.say('four');
                # rospy.sleep(1)
                # agent.say('three')
                # rospy.sleep(1)
                # agent.say('two')
                # rospy.sleep(1)
                # agent.say('one')
                # rospy.sleep(1)
                # agent.grasp()
                # rospy.sleep(1)
                # is_picked = True


            elif item == 'cup':

                # agent.pose.pick_side_pose_by_height(height = 0.77) # dining table의 경우 (0.77 + 0.01)
                # # agent.pose.pick_side_pose_by_height(height = 0.76) # office table의 경우 (0.75 + 0.01)
                # # agent.pose.pick_side_pose_by_height(height = 0.49) # livingroom table의 경우 (0.48 + 0.01)
                
                # # agent.pose.pick_side_cup_pose(table=pick_table)
                # agent.open_gripper()
                # agent.move_rel(0, base_xyz[1] + 0.03, wait=True)
                # agent.move_rel(base_xyz[0]+0.20, 0, wait=True)
                
                # agent.grasp()

                # is_picked = agent.pose.check_grasp(threshold=-1.45)  # modify
                # print(f"2.4 grasp value of item '{item}': {is_picked}")

                # agent.move_rel(-0.2, 0)

                # agent.say('I picked a cup.', show_display=True)

                 # # Hand me over Mode ########################
                agent.move_rel(-0.35, 0, wait=True)
                agent.pose.pick_plate_pose_fold(table=pick_table)
                agent.open_gripper()
                agent.say('please hand me the cup', show_display=True)
                rospy.sleep(4)
                agent.say('five');
                rospy.sleep(1)
                agent.say('four');
                rospy.sleep(1)
                agent.say('three')
                rospy.sleep(1)
                agent.say('two')
                rospy.sleep(1)
                agent.say('one')
                rospy.sleep(1)
                agent.grasp()
                rospy.sleep(1)
                is_picked = True

            elif item == 'fork' or item == 'spoon' or item == 'knife':
                agent.pose.pick_top_pose(table=pick_table)
                agent.open_gripper()

                agent.move_rel(base_xyz[0] + 0.25, base_xyz[1] - 0.01, wait=True)

                agent.pose.arm_lift_top_table_down(height=-0.015, table=pick_table) # modify

                agent.grasp()
                agent.pose.arm_flex(-60)
                
                agent.move_rel(-0.17, 0)

                agent.say('I picked a cutlery.', show_display=True)

            # elif item == 'plate':
            #     h = agent.pose.pick_plate_pose(table=pick_table)
            #     agent.open_gripper()
            #     # agent.move_rel(0, plate_xyz[1], wait=True)
            #     # agent.move_rel(plate_xyz[0] + 0.18, 0, wait=True)
            #     진기명기_move_offset = 0.05 # 0.03이었음
            #     진기명기_grasp_offset = 0.02
            
            #     agent.move_rel(base_xyz[0] + 0.18, base_xyz[1], wait=True)
            
            #     agent.pose.arm_lift_up(h - 진기명기_grasp_offset)
            
            #     agent.move_rel(-base_xyz[0]-진기명기_move_offset, 0, wait=True)

            #     agent.pose.arm_lift_up(h)
            
            #     agent.move_rel(-0.18, 0, wait=True)
            #     agent.pose.pick_plate_pose_fold(table=pick_table)
            
            #     agent.move_rel(0.075-진기명기_move_offset, 0, wait=True)  # slightly move forward
            #     agent.grasp()
            
            #     is_picked = agent.pose.check_grasp()
            #     print(f"2.4 grasp value of item '{item}': {is_picked}")

            #     agent.say('I picked a plate.', show_display=True)
            
            # PLATE HAND OVER MODE  
            elif item == 'plate':
                hand_over_mode = True
                if hand_over_mode:
                    agent.move_rel(-0.35, 0, wait=True)
                    agent.pose.pick_plate_pose_fold(table=pick_table)
                    agent.open_gripper()
                    agent.say('please hand me the plate', show_display=True)
                    rospy.sleep(4)
                    agent.say('five');
                    rospy.sleep(1)
                    agent.say('four');
                    rospy.sleep(1)
                    agent.say('three')
                    rospy.sleep(1)
                    agent.say('two')
                    rospy.sleep(1)
                    agent.say('one')
                    rospy.sleep(1)
                    agent.grasp()
                    rospy.sleep(1)
                    is_picked = True
                else:
                    agent.pose.bring_bowl_pose(table=pick_table)
                    agent.open_gripper()
                    # agent.move_rel(0, base_xyz[1] + 0.04, wait=True)
                    # agent.move_rel(base_xyz[0] + 0.15, 0, wait=True)

                    agent.move_rel(base_xyz[0] + 0.15, base_xyz[1] + 0.04, wait=True)

                    agent.pose.pick_bowl_max_pose(table=pick_table, height=-0.13)
                    agent.grasp()
                    rospy.sleep(0.5)

                    is_picked = agent.pose.check_grasp()
                    print(f"2.4 grasp value of item '{item}': {is_picked}")

                    agent.pose.pick_up_bowl_pose(table=pick_table)
                    agent.move_rel(-0.3, 0)



        ########################################
        ## PLACING ##
        ########################################

        if placing_mode:
            if picking_mode == False:
                item = 'spoon'
                
            num_gripped_items += 1
            agent.pose.move_pose()      
            agent.move_abs(place_position) # 실제 대회때는 반드시 추가


            agent.pose.arm_lift_up(0.30) # 2층 랙도 열려있을 경우 수정 필요함.

            
            agent.move_rel(0, place_position_dict[item][1], wait=True)
            rospy.sleep(short_move)
            while agent.move_rel(place_position_dict[item][0], 0, wait=True)==False:
                agent.say('Hold on.')
                rospy.sleep(0.5)
                agent.move_rel(place_position_dict[item][0]-0.02, 0, wait=True)
            agent.move_rel(place_position_dict[item][0], 0, wait=True)
            rospy.sleep(short_move)

            arm_lift_value = agent.pose.place_cutlery_pose(table=place_table) # temp
            

            # if item == 'bowl':
            #     agent.pose.wrist_roll(-90)
            #     agent.pose.wrist_flex(-25)
            if item == 'cup':
                agent.pose.wrist_roll(180)

            # agent.pose.arm_lift_up(arm_lift_value - 0.1) # 던지더라하게 가자.도 안전
        

            agent.open_gripper()

            rospy.sleep(short_move)

            agent.pose.arm_lift_up(arm_lift_value) # 1층 랙 2층 랙 다 열림

            rospy.sleep(short_move)

            agent.pose.table_search_pose()



        # if num_gripped_items == 4:
        #     #Rack Close Action
        #     agent.grasp()
        #     agent.move_abs(close_position1)

        #     agent.pose.place_cutlery_pose(table=dishwasher)

        #     agent.pose.arm_lift_up(rack_close_arm_lift) #parameter 1
        #     agent.pose.wrist_flex(-70)
            
        #     agent.move_abs(close_position2)
        #     agent.move_abs(close_position3)
        #     agent.move_abs(close_position2)
        #     agent.move_abs(close_position1)
            
        #     agent.say('I finished cleaning the table', show_display=True)

        #     break