import rospy
from utils.distancing import distancing
from std_srvs.srv import Empty, EmptyRequest
from hsr_agent.agent import Agent

def serve_breakfast(agent: Agent):
    ### 942동 로봇방 212호 실험 ###
    ##모의고사용 'breakfast_table_testday' : [0.5649, -0.0299, 1.5483],
    ##모의고사용 'testday_breakfast_table_nearby' : [1.3235, -0.0452, 1.5803],
    ##모의고사용 kitchen_table_testday [2.1372, 0.3393, 1.5513]

    # start point #
    # Initial location = zero
    
    # <Preparing table> object detect 후 pick_side_pose (올라오면서 팔을 펼치고 올라옴 - 팔 부상 가능성)
    # wrist_roll : -110 -> 0 (cereal, milk both) 아마도 -1.92 radian //joint_pose.py 594 line에서 
    # spoon은 pick_top_pose 후 앞으로 이동


    ### task params #################
    mustard_height = 0.19  # [m] 기존 milk_height
    spam_height = 0.083  # [m] 기존 cereal_height
    pick_table = 'kitchen_table_testday' # 모의고사용 testday
    place_table = 'breakfast_table_testday'  # 모의고사용 testday

    pick_position = 'kitchen_table_testday' # 기존 'kitchen_table_front'
    # pick_position_bypass = 'kitchen_entrance' 주석처리 함. 의자가 있을 경우 우회해야 함.
    place_position = 'breakfast_table_testday' # 기존 'breakfast_table_front'
    # breakfast_table_front 도 위치 정해서 global config 에 추가하기.
    # item_list = ['bowl', 'cracker', 'cereal_red', 'milk', 'scrub', 'spoon', 'fork', 'knife']
    # 모의고사용. 대회 당일에는 item_list 가 인지할 수 있는 것들을 가능한 한 추가해야 함. list는 위를 참고.
    item_list = [ 'bowl', 'spam', 'mustard', 'spoon'] # milk 추가, cereal_red (=spam)

    bowl_to_spam_spill_xy = [-0.2, 0.15]  # bowl to cereal
    bowl_to_mustard_spill_xy = [-0.1, 0.05]  # bowl to milk
    bowl_to_spoon_place_xy = [0, -0.13]  # bowl to spoon

    # try except params
    default_base_xyz = [5.0999, 0.5681, 0.0124]
    # bonus params
    mustard_bonus = True
    spam_bonus = True

    ###########################
    #### test param ##########
    pick_only_mode = False

    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    #########################
    ### Initial Location은 global_config.py에 ABS_POSITION에 있음
    ### 942동 212호 냉장고 옆 문을 시작점으로 가정. 문이 열렸다고 가정.
    # 로봇을 초기 위치로 이동시키는 함수를 호출
    agent.move_abs('zero')
    #########################


    ### task start ##
    agent.door_open()
    agent.say('Hi, I will serve you breakfast') # 멘트 수정
    agent.move_rel(7.5, 0.6, wait=False)
    agent.move_abs('kitchen_table_testday')
    # 기존) agent.move_abs('breakst_bypass') # 실험해서 정확한 좌표로 수정할 것. 위치 : kitchen table 앞


    ########################### 
    while True:
        # 1. go to kitchen table -> item picking 해야 함.

        agent.pose.neutral_pose()
        # agent.pose.move_pose() 주석 처리함. (모의고사용) -> move_pose는 /hsr_agent/joint_pose.py 에서 정의 (70 line)
        rospy.sleep(0.5)                                    # 움직이는 동안 대기하는 시간

        agent.move_abs(pick_position) # kitchen_table_front 는 60cm 떨어진 곳이어야 함.
        dist_to_table = distancing(agent.yolo_module.pc, pick_table) # kitchen table 좌표 확인
        agent.move_rel(dist_to_table, 0)
         # def get_pose 관련 설명 (아래 참고)
         # hsr_agent -> agent.py에서 262번 줄 def move_distancing 에서 if문 활성화 (if kitchen~)
         # self.move_base.get_pose(), return self.move_base.get_pose() -> 현 위치 반환 함수

         # 2. search
        agent.pose.table_search_pose_breakfast_initial() # /hsr_agent/joint_pose.py 에서 pose 추가 ///혹은/// 81 line 수정
         # agent.pose.
        rospy.sleep(3) # 움직이는 동안 대기하는 시간

         # 2.1 detect all objects in pick_table
        ##################################################### 재문님 도와주신 코드 yolo number unmatch일 시, 지정해서 매칭
        # id_list = [17, 22, 33, 36]
        table_item_list = agent.yolo_module.detect_3d(pick_table)
        # 아이템 ID 순서대로 처리하기 위한 ID 리스트
        ordered_item_ids = [17, 7, 41, 22]

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

        # print('table_base_to_object_xyz', table_base_to_object_xyz)

         # 2.3 calculate dist with offset
        try: # base_xyz는 table까지의 거리 계산 후 count해서 값을 반환 
            base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
        except:
            continue
        print('gripper_to_object_xyz', base_xyz)
        agent.move_rel(-0.20, 0) 
        

         # 4. pick (순서: bowl-> spam -> mustard -> spoon)
        if item == 'bowl':
            agent.pose.pick_bowl_pose_last(table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0] + 0.20, 0, wait=True)
            agent.pose.arm_lift_up(0.510)
            agent.grasp()
            agent.pose.arm_lift_up(0.8) # 0.8로 수정해보기
            # picking 후, arm을 끌어서 다른 물체를 건드리는 상황이 발생하지 않도록 수정 필요

        elif item == 'spam' or 'mustard' :  # milk, cereal

            if item == 'spam': 
                object_height = spam_height / 2
            else: # milk
                object_height = mustard_height / 2
            agent.pose.pick_object_side_pose(object_height, table=pick_table)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1]-0.01, wait=True)
            agent.move_rel(base_xyz[0] + 0.15, 0, wait=True) # + 0.15

            if item == 'spam': 
                agent.grasp()
            else: # milk
                agent.grasp()
                rospy.sleep(0.5)
            rospy.sleep(0.5)

        else:                    # 기존 elif item == 'spoon' or item == 'fork' or item == 'knife':
            if item == 'spoon':
                agent.pose.pick_top_pose_last(table='breakfast_table')
                agent.open_gripper()
                agent.move_rel(0, base_xyz[1], wait=True)
                agent.move_rel(base_xyz[0] + 0.15 + 0.1, 0, wait=True)
                agent.pose.arm_lift_up(0.520)
                agent.grasp()
                rospy.sleep(0.5)
                agent.pose.arm_lift_up(0.68)
                rospy.sleep(3)

         # 5. return pose

        agent.move_rel(-1.0, 0)
        
        agent.pose.neutral_pose()

         # 기존 코드 -> agent.pose.table_search_go_pose()
        agent.pose.check_grasp()

        agent.pose.check_grasp()
        if pick_only_mode:
            agent.open_gripper()
            continue


        pdb.set_trace()

        # 6. go place pos
        agent.move_abs(place_position)
        dist_to_table = distancing(agent.yolo_module.pc, place_table, dist=0.8)
        print('dist_to_table', dist_to_table)
        agent.move_rel(dist_to_table, 0)
        
        item = 'mustard'

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
                spam_bonus = False
                milk_bonus = True

        # 7. place
        if item == 'bowl':
            agent.pose.place_bowl_pose()
            agent.move_rel(0.3, 0, wait=True)
            agent.pose.arm_lift_object_table_down(0.18, table=place_table)
            agent.open_gripper()
            rospy.sleep(2.0) # gazebo wait
            agent.move_rel(-0.3, 0)

        # (모의고사용) 기존 코드 -> elif item == 'cereal_red' or item == 'cracker':   
        elif item == 'spam': # 기존 elif item == 'cereal_red'
            if 'spam' :
                object_height = spam_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_spam_spill_xy[0], base_xyz[1]+bowl_to_spam_spill_xy[1], wait=True)
                agent.pose.wrist_roll(120)
                agent.pose.wrist_roll(0)
                agent.move_rel(0, 0.15, wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0) # gazebo wait
                agent.move_rel(-0.3, 0)
            else:
                object_height = spam_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_spam_spill_xy[0], base_xyz[1]+bowl_to_spam_spill_xy[1]+0.25, wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0)  # gazebo wait
                agent.move_rel(-0.3, 0)
        # (모의고사용) 기존 코드 -> elif item == 'milk' or item == 'scrub':              
        elif item == 'mustard':
            if milk_bonus:
                object_height = mustard_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0]+bowl_to_mustard_spill_xy[0]+0.2, base_xyz[1]+bowl_to_mustard_spill_xy[1]+0.08, wait=True)
                agent.pose.wrist_roll(110)
                agent.pose.wrist_roll(0)
                agent.move_rel(0.1, 0.07, wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0) # gazebo wait
                agent.move_rel(-0.4, 0)
            else:
                object_height = mustard_height / 2
                # 7.1 spill
                agent.pose.spill_object_pose(object_height, table=place_table)
                agent.move_rel(base_xyz[0] + bowl_to_mustard_spill_xy[0], base_xyz[1] + bowl_to_mustard_spill_xy[1]+0.17,
                               wait=True)
                agent.pose.arm_lift_object_table_down(object_height, table=place_table)
                agent.open_gripper()
                rospy.sleep(1.0)  # gazebo wait
                agent.move_rel(-0.4, 0)
        # (모의고사용) 기존 코드 -> elif item == 'spoon' or item == 'fork' or item == 'knife':
        elif item == 'spoon':
            agent.move_rel(0, base_xyz[1]+bowl_to_spoon_place_xy[1], wait=True)
            agent.pose.place_top_pose(0.05, table=place_table)
            agent.move_rel(base_xyz[0]+bowl_to_spoon_place_xy[0]+0.1, 0, wait=True)
            agent.pose.arm_lift_object_table_down(0.23, table=place_table) # top_pose = 0.2
            agent.open_gripper()
            rospy.sleep(2.0) # gazebo wait
            agent.move_rel(-0.4, 0)

        
        # 8. (모의고사용) 복귀
        agent.pose.neutral_pose()
        agent.say('I will serve you soon!') 

######