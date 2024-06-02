# Version 2. Offset version (Vanishing distancing)
## Offset 버전 수정 중 (민정)

import rospy
from std_msgs.msg import String
from utils.distancing import distancing
from std_srvs.srv import Empty, EmptyRequest
from hsr_agent.agent import Agent

### About this code ###
# (Task) 아래의 코드는 2024년 Robocup - Stage 1. 'Serve Breakfast' task에 대한 코드임.
#        총 4개의 object를 사용하며 [bowl,cereal,milk,spoon] 순서임.
# (Task order) Initial point -> Picking place -> Distancing -> Picking object -> displaying or saying -> Distancing -> Placing -> ... -> Finish
#              이때, milk, cereal은 흘리지 않고 bowl에 부어야하며, spoon은 bowl 옆에 두어야 함.
# (Version) Version 1은 Distancing 을 사용하였으며, Version 2에서는 SLAM에 기반한 Offset에 의존함.
# (Location) 942동 212호 Robot room
# (Start point) 문 밖에서 시작하므로 zero가 start point임.
# (Spill) wrist_roll : -110도 정도 (-1.92 radian)

# 위치 관련 내용은 global_config.py 파일에서 ABS_POSITION에 있음.
# joint, pose 관련은 hsr_agent 폴더에 joint_pose.py 파일 참고.
# 현 위치 반환 함수 : self.move_base.get_pose(), return self.move_base.get_pose()

# Head display 관련 : HSR에 직접 키보드 연결 후 "python3 /hsr_head_display/hsr_head_monitor.py" 실행

def serve_breakfast(agent: Agent):

    ### task params #################
    mustard_height = 0.19  # [m]
    spam_height = 0.083  # [m]

    agent.pose.table_search_pose_breakfast_initial()
    rospy.sleep(0)

    pick_table = 'breakfast_table' 
    place_table = 'kitchen_table'  

    pick_position = 'breakfast_table'  # pick_position_bypass = 'kitchen_entrance' // 만일을 대비한 우회 지점
    place_position = 'kitchen_table'

    # Item List
    item_list = ['bowl', 'spam', 'mustard', 'spoon'] # 대회 당일에는 item_list 가 인지할 수 있는 것들을 가능한 한 추가 ex)'cracker', 'scrub', 'fork', 'knife' ...

    bowl_to_spam_spill_xy = [-0.03, 0.08]  # bowl to cereal
    spam_placing_xy = [0, 0.30]

    bowl_to_mustard_spill_xy = [-0.104, 0.04]  # bowl to milk
    mustard_placing_xy = [0, 0.15]

    bowl_to_spoon_place_xy = [0, -0.13]  # bowl to spoon

    # try except params
    default_base_xyz = [6.2082, -1.1592, 0.022] # serving table 기준
    # bonus params

    mustard_bonus = True
    spam_bonus = True

    #### test param ###########
    pick_only_mode = False

    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    ### task start ##
    # agent.move_abs('zero') -> Initial location
    # agent.door_open()
    agent.say('Hi, I will serve breakfast for you')
    # agent.move_rel(7.5, 0.6, wait=False) 직선 거리 이동 후, picking location 으로 이동
    agent.move_abs(pick_table)

    ###########################
    while True:
        # 1. go to kitchen table
        # import pdb; pdb.set_trace()
        agent.move_abs(pick_position) # Distancing을 위해 60cm 떨어진 지점임.

         # 2. search
        agent.pose.table_search_pose_breakfast_initial()
        rospy.sleep(0.2)

         # 2.1 detect all objects in pick_table // id_list = [17, 22, 33, 36]
        table_item_list = agent.yolo_module.detect_3d(pick_table)

        is_detected = False
        
        agent.say('Wait for a moment')
        
        for item in item_list:
            print('item', item)
            name, item_id, itemtype, grasping_type = agent.yolo_module.find_object_info_by_name(item)
            for table_item in table_item_list:
                if item_id == table_item[3]:
                     # 2.2 select target_object_pc
                    table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, name)
                    is_detected = True
                    break
            if is_detected:
                break

        print('table_base_to_object_xyz', table_base_to_object_xyz)

        # 2.3 calculate dist with offset
        try:
            base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
        except:
            continue
        print('gripper_to_object_xyz', base_xyz)

        # 4. pick (순서: bowl -> spam -> mustard -> spoon)
        if item == 'bowl':
            agent.say('I will pick a bowl', show_display=False)
            agent.head_show_text('I will pick a Bowl') # text display 추가``
            agent.pose.pick_bowl_pose_last(table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1] - 0.05, wait=True)
            agent.move_rel(base_xyz[0]+ 0.05, 0, wait=True)
            agent.pose.arm_lift_up(0.48) # 0.48에 맞춰서 아래로 내려감
            rospy.sleep(1.0)
            agent.grasp()
            agent.pose.arm_lift_up(0.6) # 0.6 높이에 맞춰서 팔을 들어올림

        elif item == 'spam':
            object_height = spam_height / 2
            agent.say('I will pick a spam', show_display=False)
            agent.head_show_text('I will pick a Spam') # text display 추가
            agent.pose.pick_object_side_pose(object_height, table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1]-0.05, wait=True)
            agent.move_rel(base_xyz[0]+0.08, 0 , wait=True)
            agent.grasp()
            rospy.sleep(0.5)

        elif item == 'mustard':
            object_height = mustard_height / 2
            agent.say('I will pick a mustard', show_display=False)
            agent.head_show_text('I will pick a Mustard') # text display 추가
            agent.pose.pick_object_side_pose(object_height, table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0], 0, wait=True)
            agent.grasp()
            rospy.sleep(0.5)

        else:                    # 기존 elif item == 'spoon' or item == 'fork' or item == 'knife':
            if item == 'spoon':
                # import pdb; pdb.set_trace()
                agent.say('I will pick a spoon', show_display=False)
                agent.head_show_text('I will pick a Spoon') # text display 추가
                agent.pose.pick_top_pose_last(table=pick_table)
                agent.open_gripper()
                agent.move_rel(0, base_xyz[1], wait=True)
                agent.move_rel(base_xyz[0], 0, wait=True)
                agent.pose.pick_top_pose(table=pick_table)
                agent.grasp()
                rospy.sleep(0.2)

        # 5. return pose
        agent.move_rel(-0.6, 0) # 부딪힘 방지
        agent.say('I will moving. Please be careful')
        
        agent.pose.table_search_pose_breakfast_initial()

        # 기존 코드 -> agent.pose.table_search_go_pose()
        agent.pose.check_grasp()
        if pick_only_mode:
            agent.open_gripper()
            continue

        # 6. go place pos
        agent.move_abs(place_position)
        agent.pose.holding_pose() # 대회 당일 의자나 아래 부분에 장애물이 있을 것도 고려해야 함. 현재 고려 x.
        agent.move_rel(-0.2, 0, wait=True)

        # pdb.set_trace()

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
                spam_bonus = True
                mustard_bonus = True

        # 7. place
        if item == 'bowl': # 주석 풀기
            agent.pose.place_bowl_pose()
            agent.move_rel(0, -0.18, wait=True)
            rospy.sleep(0.1)
            agent.move_rel(0.55, 0, wait=True)
            # agent.pose.arm_lift_object_table_down(0.28, table=place_table)
            agent.open_gripper()
            rospy.sleep(2.0) # gazebo wait
            agent.pose.arm_lift_up(0.68)
            agent.move_rel(-0.8, 0)

        elif item == 'spam': # 'cereal_red' # elif로 바꿔주기
            # pdb.set_trace()
            if spam_bonus: #붓기를 시도할것임.
                object_height = spam_height / 2
                # 7.1 spill
                
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_spam_spill_xy[0], 0, wait=True)
                agent.move_rel(0 , base_xyz[1]+bowl_to_spam_spill_xy[1], wait=True)
                agent.pose.wrist_roll(120) # 붓는 중
                agent.pose.wrist_roll(0) # 붓기 완료
                agent.move_rel(spam_placing_xy[0], spam_placing_xy[1], wait=True) # 옆에 두기
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0) # gazebo wait
                agent.move_rel(-0.6, 0, wait=True)
            else: #붓기는 건너뛰고 placing만 진행
                object_height = spam_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_spam_spill_xy[0], base_xyz[1]+bowl_to_spam_spill_xy[1]+0.25, wait=True) #옆에 두기.
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0)  # gazebo wait
                agent.move_rel(-0.6, 0, wait=True)
             
        elif item == 'mustard':         # 기존 코드 : elif item == 'milk' or item == 'scrub': 
            if mustard_bonus:
                object_height = mustard_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_mustard_spill_xy[0], base_xyz[1]+bowl_to_mustard_spill_xy[1], wait=True)
                agent.pose.wrist_roll(130)
                agent.pose.wrist_roll(0)
                agent.move_rel(mustard_placing_xy[0], mustard_placing_xy[1], wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0) # gazebo wait
                agent.move_rel(-0.4, 0,wait=True)
            else:
                object_height = mustard_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0] + bowl_to_mustard_spill_xy[0], base_xyz[1] + bowl_to_mustard_spill_xy[1]+0.17,
                               wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0)  # gazebo wait
                agent.move_rel(-0.4, 0,wait=True)
        
        elif item == 'spoon': # 기존 코드 : elif item == 'spoon' or item == 'fork' or item == 'knife':
            agent.move_rel(-0.2,0)
            # agent.move_rel(0, base_xyz[1]+bowl_to_spoon_place_xy[1], wait=True)
            agent.pose.place_top_pose(0.05, table=place_table)
            agent.move_rel(base_xyz[0]+bowl_to_spoon_place_xy[0]+0.1, 0, wait=True)
            agent.pose.arm_lift_object_table_down(0.23, table=place_table) # top_pose = 0.2
            agent.open_gripper()
            rospy.sleep(2.0) # gazebo wait
            agent.move_rel(-0.4, 0,wait=True)
            agent.pose.arm_lift_up(0.68)
            agent.move_rel(-0.8, 0)            
            agent.pose.neutral_pose()

        # 8. (모의고사용) 복귀
        agent.pose.table_search_pose_breakfast_initial()
        agent.say('I will moving. Please be careful')


