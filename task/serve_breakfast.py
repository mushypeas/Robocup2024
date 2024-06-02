# Version 2. Offset version (Vanishing distancing)

## 24.06.02 재문님 수정해주신 코드 돌려보기 편 (placing 부분 수정하면 됨)

import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
from hsr_agent.agent import Agent

### About this code ###
# (Task) 아래의 코드는 2024년 Robocup - Stage 1. 'Serve Breakfast' task에 대한 코드임.
#        총 4개의 object를 사용하며 [bowl,cereal,milk,spoon] 순서로 picking 함. (시간 단축을 위한 순서 배정)
# (Task order) Initial point -> Move to Picking place -> Distancing -> Picking object -> displaying or saying -> Distancing -> Placing -> ... -> Finish
#              이때, milk, cereal은 흘리지 않고 bowl에 부어야하며, spoon은 bowl 옆에 두어야 함.
#              + Rule 추가 : Picking 시, picking object를 display에 표시 혹은 tts로 알려주어야 함.
# (Version) Version 1은 Distancing 을 사용하였으며, Version 2에서는 SLAM에 기반한 Offset 코드임. / 향후, distancing을 제외한 Offset 버전으로 진행할 듯. (24.06.02)
# (Location) 942동 212호 Robot room
# (Start point) 문 밖에서 시작하므로 zero가 start point임.
# (Spill) wrist_roll : -110도 정도 (-1.92 radian) / Mustard와 같은 높이가 길쭉한 편에 나오는 입구가 좁은 경우 -130도 (빠른 속도를 원할 시 -140도) 정도는 되어야 할 듯.
#                       각도의 경사가 커질수록 낙하속도가 빨라지므로 흘릴 가능성이 높아짐. offset 좌표가 중요할 듯 
# 위치 관련 내용은 global_config.py 파일에서 ABS_POSITION에 있음. (AIIS와 Simulation mode 둘 다 있으니 유의해서 보기) -> 거의 table dimension과 table location만 보면 됨.
# joint, pose 관련은 hsr_agent 폴더에 joint_pose.py 파일 참고.
# 현 위치 반환 함수 : self.move_base.get_pose(), return self.move_base.get_pose()
# Head display 관련 : HSR에 직접 키보드 연결 후 "python3 /hsr_head_display/hsr_head_monitor.py" 실행

<<<<<<< HEAD
# 아래는 재문님이 수정 및 재작성해주신 코드. 깔끔함에 감사 (-_-)(_ _) 꾸벅.

=======
>>>>>>> 60df98a84e4f4dfc0b0655f6a1d2193cf09bf290
class ServeBreakfast:

    def __init__(self, agent: Agent):
        self.agent = agent

        # !!! Test params !!!
        self.picking_test_mode = False
        self.attemp_pouring = {
            'spam': True,
            'mustard': True,
        }

        self.item_list = ['bowl', 'spam', 'mustard', 'spoon'] # In order
        
        # !!! Measured Distances !!!
        self.dist_to_pick_table = 0.93
        self.dist_to_place_table = 0.97
        self.item_height = {
            'spam': 0.083,
            'mustard': 0.19,
        }

        # !!! Hard-Coded Offsets !!!
        self.pick_front_bias = [0.03, 0.00, -0.03]  # [x, y, height]
        self.pick_top_bias = [-0.03, 0.00, -0.015]  # [x, y, height]
        self.pick_bowl_bias = [0.0, 0.00, -0.11]    # [x, y, height]
        self.pour_offsets = { # [x, y, angle]
            'spam': [-0.03, 0.08, 120],
            'mustard': [-0.104, 0.04, 130],
        }
        self.arm_lift_height = 0.68
        self.place_offsets = { # [x, y, height]
            'bowl': [self.dist_to_place_table - 0.50, -0.1, 0],
            'spam': [0.0, 0.30, 0],
            'mustard': [0.0, 0.15, 0],
            'spoon': [0.1, -0.13, 0.23]
        }

        self.pick_table = 'breakfast_table'
        self.pick_table_depth = self.agent.table_dimension[self.pick_table][1] 
        self.pick_table_height = self.agent.table_dimension[self.pick_table][2]
        self.pick_table_head_angle = np.arctan(
            (self.pick_table_height - 1.1) / self.dist_to_pick_table # 1.1: HSR height
        )

        self.place_table = 'kitchen_table'
        self.place_table_depth = self.agent.table_dimension[self.place_table][1]

        self.default_base_xyz = [6.2082, -1.1592, 0.022] # serving table 기준


    def search_item(self, item):
        table_item_list = np.zeros(0)
        table_search_attempts = 0
        while table_item_list.size == 0:
            rospy.sleep(0.2) # give some time for the YOLO to update
            table_item_list = np.array(self.agent.yolo_module.detect_3d_safe(
                table='grocery_table',
                dist=self.dist_to_pick_table,
                depth=self.pick_table_depth,
                item_list=self.item_list
            ))
            table_search_attempts += 1
            if table_search_attempts == 20:
                self.agent.say('Searching is taking a while...')
            if table_search_attempts == 60:
                self.agent.say('Still searching...')
            if table_search_attempts == 100:
                self.agent.say('What the...')
                rospy.logwarn('Finish Serve Breakfast.')
                return

        for table_item_info in table_item_list:
            table_item_id = int(table_item_info[3])
            table_item_name = self.agent.yolo_module.find_name_by_id(table_item_id)
            if table_item_name == item:
                grasping_type = self.agent.yolo_module.find_grasping_type_by_id(table_item_id)
                break

        try:
            table_base_to_object_xyz = self.agent.yolo_module.find_3d_points_by_name(table_item_list, table_item_name)
            table_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)  # inclined grasping type
            rospy.loginfo(f"Base to object:   {table_base_to_object_xyz}")
            rospy.loginfo(f"Table base :      {table_base_xyz}")
            return (grasping_type, table_base_xyz)
        except Exception as e:
            rospy.logerr(f'[ERROR] table_base_to_object_xyz: {table_base_to_object_xyz}\n{e}')
            return None


    def pick_item(self, item, table_base_xyz):

        if item == 'bowl':
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.open_gripper(wait=False)
            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=self.pick_bowl_bias[2])
            self.agent.grasp()
            self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
            self.agent.move_rel(-0.4, 0, wait=False)

        elif item in ['spam', 'mustard']:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.pose.pick_side_pose_by_height(height=self.pick_table_height + self.pick_front_bias[2] + self.item_height[item]/2)
            self.agent.open_gripper(wait=False)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) # wait for grasping manually
            self.agent.move_rel(-0.4, 0, wait=False)

        else:
            if item == 'spoon':
                self.agent.pose.pick_top_pose_by_height(height=self.pick_table_height + self.pick_top_bias[2])
                table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
                self.agent.open_gripper(wait=False)
                self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
                self.agent.grasp(wait=False)
                rospy.sleep(0.5) # wait for grasping manually
                self.agent.pose.arm_flex(-60)


    def pour_item(self, item, table_base_xyz):

        self.agent.pose.spill_object_pose(self.item_height[item]/2, table=self.place_table)
        self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
        self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
        self.agent.pose.wrist_roll(0) # 붓기 완료


    def place_item(self, item, table_base_xyz):

        if item == 'bowl': # 주석 풀기
            self.agent.pose.place_bowl_pose()
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True)

        elif item in ['spam', 'mustard']:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.place_offsets[item])]
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True) # 옆에 두기
            self.agent.pose.arm_lift_object_table_down(self.item_height[item]/2, table=self.place_table)
        
        elif item == 'spoon':
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.place_offsets[item])]
            self.agent.pose.place_top_pose(0.05, table=self.place_table)
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.pose.arm_lift_object_table_down(self.place_offsets[item][2], table=self.place_table) # top_pose = 0.2
            
        self.agent.open_gripper()
        self.agent.pose.arm_lift_up(self.arm_lift_height)
        self.agent.move_rel(-0.4, 0, wait=False)
        rospy.sleep(1) # wait manually


    
    def check_grasp(self, grasping_type):
        if grasping_type == 0:
            return self.agent.pose.check_grasp()
        else:
            return True

    
    def run(self):
        stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_client.call(EmptyRequest())

        ### task start ##
        # agent.door_open()
        self.agent.say('Hi, I will serve breakfast for you!')
        rospy.sleep(2)

        for item in self.item_list:

            has_grasped = False

            ## Try picking until an item is grasped
            while not has_grasped:
                rospy.logwarn('Go to pick_location...')
                self.agent.say('I will move to a different location. Please be careful.')
                self.agent.pose.table_search_pose(head_tilt=self.pick_table_head_angle)
                self.agent.move_abs_safe(self.pick_table)
                rospy.sleep(2)
                # Search item
                item_info = self.search_item(item)
                if item_info is None:
                    rospy.logwarn('Failed to find item to pick. Retrying...')
                    continue
                else:
                    grasping_type, table_base_xyz = item_info

                # Pick item
                rospy.logwarn('Picking item...')
                self.agent.say(f'I will pick a {item}.', show_display=True)
                self.pick_item(item, table_base_xyz)
                self.agent.pose.table_search_pose_breakfast_initial()

                # Check if grasping is successful
                has_grasped = self.check_grasp(grasping_type)
                if has_grasped:
                    rospy.loginfo(f'Successfuly grasped {item}!')
                else:
                    rospy.logwarn(f'Failed to grasp {item}! Retrying...')

            if self.picking_test_mode:
                self.agent.open_gripper()
                continue

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.say('I will move to a different location. Please be careful.')
            self.agent.move_abs_safe(self.place_table)
            # self.agent.pose.holding_pose() # 대회 당일 의자나 아래 부분에 장애물이 있을 것도 고려해야 함. 현재 고려 x.

            base_xyz = None
            if item != 'bowl':
                try:
                    rospy.sleep(1)
                    table_item_list = self.agent.yolo_module.detect_3d_safe(
                        table=self.place_table,
                        dist=self.dist_to_place_table,
                        depth=self.place_table_depth,
                        item_list=['bowl']
                    )
                    table_base_to_object_xyz = self.agent.yolo_module.find_3d_points_by_name(table_item_list, 'bowl')
                    base_xyz = self.agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type=2) # 2 == bowl
                    print('bowl base_xyz',base_xyz)
                except:
                    base_xyz = self.default_base_xyz
                    print('no bowl detected. set default base_xyz', base_xyz)

            if item in ['spam', 'mustard']:
                self.pour_item(item=item, table_base_xyz=base_xyz)

            rospy.logwarn('Placing item...')
            self.place_item(item=item, table_base_xyz=base_xyz)
