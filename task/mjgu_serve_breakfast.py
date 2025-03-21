import rospy
from utils.distancing import distancing
# 따로, 정의가 필요하다면.. utils에 파일 만들고 import 해서 사용하기(?)
from std_srvs.srv import Empty, EmptyRequest
def serve_breakfast(agent):
    ### 942동 로봇방 212호 실험 ###
    # joint_pose에 kitchen table과 breakfast table....그냥 이름 다 바꿔야할 듯?

    # start point #
    # outside of the door - 아직 좌표 모름 #
    # 2 Tables - preparing table (getp 좌표 [2.074, -2.6953, 0.0159]),
    #            serving table (getp 좌표 [1.1877, -1.1908, 0.0079])
    # <Preparing table> object detect 후 pick_side_pose (올라오면서 팔을 펼치고 올라옴 - 팔 부상 가능성)
    # wrist_roll : -110 -> 0 (cereal, milk both) 아마도 -1.92 radian //joint_pose.py 594 line에서 
    # spoon은 pick_top_pose 후 앞으로 이동

    ### task params #################
    milk_height = 0.14  # [m]
    cereal_height = 0.13  # [m]
    pick_table = 'kitchen_table'
    place_table = 'breakfast_table'

    pick_position = 'kitchen_table_front'
    # pick_position_bypass = 'kitchen_entrance' 주석처리 함. 의자가 있을 경우 우회해야 함.
    place_position = 'breakfast_table_front'
    # breakfast_table_front 도 위치 정해서 global config 에 추가하기.
    # item_list = ['bowl', 'cracker', 'cereal_red', 'milk', 'scrub', 'spoon', 'fork', 'knife']
    # 모의고사용. 대회 당일에는 item_list 가 인지할 수 있는 것들을 가능한 한 추가해야 함. list는 위를 참고.
    item_list = [ 'bowl', 'spoon', 'milk', 'cereal_red']

    bowl_to_cereal_spill_xy = [-0.2, 0.15]  # bowl to cereal
    bowl_to_milk_spill_xy = [-0.1, 0.05]  # bowl to milk
    bowl_to_spoon_place_xy = [0, -0.13]  # bowl to spoon

    # try except params
    # default_base_xyz = [0.376, -0.1, 0.823]  (수정) 주석처리함.
    # bonus params
    milk_bonus = False
    cereal_bonus = True

    ###########################
    #### test param ##########
    pick_only_mode = False

    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    #########################
    ### Initial Location ### 942동 212호 냉장고 옆 문을 시작점으로 가정. 문이 열렸다고 가정.
    initial_x = 2.2  # 2.2 정도?
    initial_y = -3.5  # -3.5 정도?
    initial_z = 1.58  # yaw 인듯?

    # 로봇을 초기 위치로 이동시키는 함수를 호출
    agent.move_abs(initial_x, initial_y, initial_z)
    #########################


    ### task start ##
    agent.door_open()
    agent.say('Hi, I will serve you breakfast') # 멘트 수정
    agent.move_rel(2.0, 0, wait=False)
    agent.move.abs('kitchen_table_front')
    # 기존) agent.move_abs('breakfast_bypass') # 실험해서 정확한 좌표로 수정할 것. 위치 : kitchen table 앞


    ###########################
    while True:
        # 1. go to kitchen table -> item picking 해야 함.

        # Initial pose 정의 (모의고사용) 
        agent.pose.neutral.pose()
        # agent.pose.move_pose() 주석 처리함. (모의고사용) -> move_pose는 /hsr_agent/joint_pose.py 에서 정의 (70 line)
        rospy.sleep(0.5)                                    # 움직이는 동안 대기하는 시간

        # kitchen table 앞 picking 할 위치(60cm 떨어진 곳)으로 이동
        agent.move_abs(pick_position) # kitchen_table_front 는 60cm 떨어진 곳이어야 함.
        dist_to_table = distancing(agent.yolo_module.pc, pick_table) # kitchen table 좌표 확인
        agent.move_rel(dist_to_table, 0.6) # 기존 코드 -> (dist_to_table,0)
        # def get_pose 관련 설명 (아래 참고)
        # hsr_agent -> agent.py에서 262번 줄 def move_distancing 에서 if문 활성화 (if kitchen~)
        # self.move_base.get_pose(), return self.move_base.get_pose() -> 현 위치 반환 함수

        # 2. search
        agent.pose.table_search_pose_breakfast_inital() # /hsr_agent/joint_pose.py 에서 pose 추가 ///혹은/// 81 line 수정
        # agent.pose.
        rospy.sleep(2) # 움직이는 동안 대기하는 시간

        # 2.1 detect all objects in pick_table
        # 아이템 리스트 초기화
        item_list = ['bowl', 'mustard', 'spam', 'spoon']

        # 각 아이템에 대한 처리
        for item in item_list:
            # 해당 아이템의 정보 검색
            name, item_id, item_type, grasping_type = agent.yolo_module.find_object_info_by_name(item)
            print(f'Processing item: {name}, item_id: {item_id}')

            # 테이블 위의 아이템을 순회하며 해당 아이템이 있는지 확인
            is_detected = False
            for table_item in table_item_list:
                # table_item[3]는 아이템의 이름을 나타내는 부분으로 가정
                if table_item[3] == item:
                    # 아이템을 찾으면 위치 정보를 출력
                    table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, name)
                    print(f'{item} found at position {table_base_to_object_xyz}')
                    is_detected = True
                    break
    
        # 아이템이 테이블 위에 없다면 메시지 출력
        if not is_detected:
            print(f'{item} not found on the table.')

            print('table_base_to_object_xyz', table_base_to_object_xyz)

            # 2.3 calculate dist with offset
            try: # base_xyz는 table까지의 거리 계산 후 count해서 값을 반환 
                base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
            except:
                continue
            print('gripper_to_object_xyz', base_xyz)
            agent.move_rel(-0.15, 0) # 꼭 사용해야 하는 코드인가..?

            # 4. pick # pick 할 때 애초에 부피가 큰 cereal, milk, bowl 먼저
            if item == 'bowl':
                agent.pose.pick_bowl_pose_last(table=pick_table)
                agent.open_gripper()
                agent.move_rel(0, base_xyz[1], wait=True)
                agent.move_rel(base_xyz[0] + 0.18, 0, wait=True)
                agent.pose.arm_lift_up(0.625)
                agent.grasp()
                agent.pose.arm_lift_up(0.69) # 0.8로 수정해보기
                # picking 후, arm을 끌어서 다른 물체를 건드리는 상황이 발생하지 않도록 수정 필요

            elif item == 'spoon' or item == 'fork' or item == 'knife':
                agent.pose.pick_top_pose_last(table='breakfast_table')
                agent.open_gripper()
                agent.move_rel(0, base_xyz[1], wait=True)
                agent.move_rel(base_xyz[0] + 0.15 + 0.1, 0, wait=True)
                agent.pose.arm_lift_up(0.605)
                agent.grasp()
                rospy.sleep(0.5)
                agent.pose.arm_lift_up(0.69)

            else:   # milk, cereal
                # (모의고사용) 기존 코드 -> if item == 'cereal_red' or item == 'cracker':
                if item == 'cereal_red':
                    object_height = cereal_height / 2
                else: # milk
                    object_height = milk_height / 2
                agent.pose.pick_object_side_pose(object_height, table=pick_table)
                agent.open_gripper()
                agent.move_rel(0, base_xyz[1]-0.01, wait=True)
                agent.move_rel(base_xyz[0] + 0.15, 0, wait=True) # + 0.15
                # (모의고사용) 기존 코드 -> if item == 'cereal_red' or item == 'cracker':
                if item == 'cereal_red':
                    agent.grasp()
                else: # milk
                    agent.grasp(weak=True)
                    rospy.sleep(0.5)
                rospy.sleep(0.5)


            # 5. return pose
            # (모의고사용) 기존 코드 -> agent.move_rel(-0.4, 0)
            agent.move_rel(-0.8, 0)
            agent.pose.neutral.pose()
            # (모의고사용) 기존 코드 ->         agent.pose.table_search_go_pose()
            agent.pose.check_grasp()

            if pick_only_mode:
                agent.open_gripper()
                continue

            # 6. go place pos
            agent.move_abs(place_position)
            rospy.sleep(5)
            dist_to_table = distancing(agent.yolo_module.pc, place_table, dist=0.7)
            print('dist_to_table', dist_to_table)
            agent.move_rel(dist_to_table, 0)

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
            # (모의고사용) 기존 코드 -> elif item == 'cereal_red' or item == 'cracker':   
            elif item == 'cereal_red':
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
            # (모의고사용) 기존 코드 -> elif item == 'milk' or item == 'scrub':              
            elif item == 'milk':
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
            agent.say('Have a nice breakfast!')       
            agent.move_abs(initial_x, initial_y, initial_z)