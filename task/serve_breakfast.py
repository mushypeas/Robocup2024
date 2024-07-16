import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
from hsr_agent.agent import Agent
 
################# Final (Eindhoven ver.) ################################ 

# 출발장소 : Entrance
# picking_location 좌표 : [ ]

# < Location >
# kitchen_table = kitchen_counter : [ ]
# breakfast_table = dinner table : [   ]
 
# < Dimension >
# kitchen_table = kitchen_counter -> pick_up table [   ]
# breakfast_table = dinner table -> placing table [    ]

# <Item >
# Cereal : 0.291 * 0.055 * 0.26
# Milk : 0.075 * 0.07 * 0.098 (앞) / 0.114 (뒤)
#########################################################################

  
class ServeBreakfast:

    def __init__(self, agent: Agent):
        self.agent = agent

        # !!! Test params !!!
        self.is_picking_test_mode = False
        self.is_pouring_test_mode = False
        self.is_placing_test_mode = False

        self.attemp_pouring = {
            'cornflakes': True,
            'milk': True,
        }
        # Milk 0.075 * 0.07 * 0.098 (앞) # 500ml,우유가 비대칭적인 모양임. 뒤로 갈수록 길어짐.
        #      0.075 * 0.07 * 0.114 (뒤)
        # Cornflakes 0.291 * 0.055 * 0.26

        self.item_list = ['bowl','cornflakes','milk','spoon','fork','knife']
        
        # !!! Measured Distances !!!
        self.dist_to_pick_table = 0.8
        self.dist_to_place_table = 0.8
        self.item_height = {     # 실제 object 높이는 여기서 설정
            'cornflakes': 0.28,
            'milk': 0.114,
        }

        # !!! Hard-Coded Offsets !!!    # [x, y, height] / 최대한 offset을 쓰지 않는 방향으로 하되, 실제 시험장에서는 어쩔 수 없이 사용할 수는 있음.
        self.pick_front_bias = [0.0, 0.00, 0.0] 
        self.pick_top_bias = [0.00, 0.00, 0.0]  
        self.pick_bowl_bias = [0.0, 0.00, 0.0]
        self.pick_cornflakes_bias = [0.0, 0.00, 0.0]
        self.pick_milk_bias = [0.0, 0.00, 0.0]
        self.pick_spoon_bias = [0.0, 0.00, 0.0]      


        self.pour_offsets = { # [x, y, angle] 
            'cornflakes': [0.0, -0.08, 120], # bowl 왼쪽 끝에 맞춰서 따르기
            'milk': [0.0, -0.08, 80], # bowl 왼쪽 끝에 맞춰서 따르기
        }
        self.arm_lift_height = 0.68
        self.place_offsets = { # [x, y, height], bowl을 테이블에서 약간 오른쪽에 두도록 함  / placing_table에는 순서대로 'milk, cereal, bowl, spoon'이 놓이도록 함.
            'bowl': [self.dist_to_place_table, -0.1, 0],
            'cornflakes': [0, 0.2, 0],
            'milk': [0, 0.1, 0],
            'spoon': [0, -0.16, 0]
        }

        self.pick_table = 'kitchen_counter' # kitchen_cabinet or dish_washer
        self.pick_table_depth = self.agent.table_dimension[self.pick_table][1] 
        self.pick_table_height = self.agent.table_dimension[self.pick_table][2]
        self.pick_table_head_angle = np.arctan(
            (self.pick_table_height - 1.1) / self.dist_to_pick_table # 1.1: HSR height
        )

        self.place_table = 'dinner_table'
        self.place_table_depth = self.agent.table_dimension[self.place_table][1]

    # def picking_test_mode(self, item, table_base_xyz):
    #     if item == 'bowl':
    #         # import pdb; pdb.set_trace(), 한 줄씩 체크하는 용도
    #         table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]
    #         self.agent.move_rel(0, table_base_xyz[1], wait=False)
    #         self.agent.open_gripper(wait=False)
    #         self.agent.pose.bring_bowl_pose(table=self.pick_table) 
    #         self.agent.move_rel(table_base_xyz[0], 0, wait=True)
    #         self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=self.pick_bowl_bias[2])
    #         self.agent.move_rel(table_base_xyz[0], 0, wait=True)
    #         self.agent.grasp()
    #         self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
    #         self.agent.move_rel(-0.3, 0, wait=False)

    #     elif item in ['cereal', 'milk']:
    #         table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
    #         self.agent.move_rel(-0.5, table_base_xyz[1], wait=False)
    #         self.agent.pose.bring_bowl_pose(table=self.pick_table)
    #         self.agent.pose.pick_cereal_pose(table=self.pick_table, height=self.pick_cereak_bias[2]) # cereal 추가
    #         self.agent.open_gripper(wait=False)
    #         # self.agent.pose.pick_side_pose_by_height(height=self.pick_table_height + self.pick_front_bias[2] + self.item_height[item]/2)
    #         self.agent.move_rel(table_base_xyz[0]+0.5, 0, wait=True)
    #         self.agent.grasp(wait=False)
    #         rospy.sleep(0.5) # wait for grasping manually
    #         self.agent.move_rel(-0.6, 0, wait=False)

    #     else:
    #         if item == 'spoon':
    #             self.agent.pose.bring_bowl_pose(table=self.pick_table)
    #             # self.agent.pose.pick_top_pose_by_height(height=self.pick_table_height + self.pick_top_bias[2])
    #             table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
    #             self.agent.open_gripper(wait=False)
    #             self.agent.move_rel(0, table_base_xyz[1], wait=True)
    #             self.agent.move_rel(table_base_xyz[0], 0, wait=True)
    #             self.agent.pose.pick_up_spoon_pose(table=self.pick_table, height=self.pick_spoon_bias[2])        
    #             self.agent.grasp(wait=False)
    #             rospy.sleep(0.5) # wait for grasping manually


    # def pouring_test_mode(self, item):
    #     table_base_xyz = [item_offset + bowl_offset for item_offset, bowl_offset in zip(self.pour_offsets[item], self.place_offsets['bowl'])] 
    #     self.agent.pose.spill_object_pose(self.item_height[item]/2, table=self.place_table)
    #     self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
    #     self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
    #     self.agent.pose.wrist_roll(0) # 붓기 완료

    # def placing_test_mode(self, item):
    #     if item == 'bowl': # 주석 풀기
    #         self.agent.pose.place_bowl_pose()
    #         self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True)

    #     elif item in ['cereal', 'milk']:
    #         self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True) # 옆에 두기
    #         self.agent.pose.arm_lift_object_table_down(self.item_height[item]/2, table=self.place_table)
        
    #     elif item == 'spoon':
    #         table_base_xyz = [axis + bias for axis, bias in zip(self.place_offsets[item], self.place_offsets['bowl'])]
    #         self.agent.pose.place_top_pose(0.05, table=self.place_table)
    #         self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
    #         self.agent.pose.arm_lift_object_table_down(self.place_offsets[item][2], table=self.place_table) # top_pose = 0.2
            
    #     self.agent.open_gripper()
    #     self.agent.pose.arm_lift_up(self.arm_lift_height)
    #     self.agent.move_rel(-0.6, 0, wait=False)
    #     rospy.sleep(1) # wait manually

    def search_item(self, item, picked_items):
        table_item_list = np.zeros(0)
        table_search_attempts = 0

        item_list = [item for item in self.item_list if item not in picked_items]
        if len(item_list) == 1:
            item_list.append('fork')
            item_list.append('knife')
            
        rospy.loginfo(f"Items to search: {item_list}")
        while table_item_list.size == 0:
            
            rospy.sleep(0.2) # give some time for the YOLO to update
            table_item_list = np.array(self.agent.yolo_module.detect_3d_safe(
                table='breakfast_counter',
                dist=self.dist_to_pick_table,
                depth=self.pick_table_depth,
                item_list=item_list
            ))
            table_search_attempts += 1
            if table_search_attempts == 20:
                self.agent.say('Searching is taking a while...')
            if table_search_attempts == 60:
                self.agent.say('Still searching...')
            if table_search_attempts == 100:
                self.agent.say('Oh.. please give me the item. I cannot find it.')
                self.agent.open_gripper(wait=False)
                rospy.sleep(2)
                self.agent.grasp()
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
        # pick_table = 'kitchen_counter'
        # if pick_table == 'kitchen_counter':
        # elif pick_table == 'kitchen_cabinet':
        # elif pick_table == 'dish_washer':

        if item == 'bowl':
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
            self.agent.open_gripper(wait=False)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True) 
            self.agent.grasp()
            self.agent.move_rel(-0.3, 0, wait=False)

            # picking test mode 추가
            # if self.picking_test_mode = True
            #     self.agent.pose.bring_bowl_pose(table=self.pick_table)
            #     self.agent.move_rel(0.2, table_base_xyz[1], wait=False)
            #     self.agent.open_gripper(wait=False)

        elif item in ['cornflakes', 'milk']:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.pose.pick_side_pose(table=self.pick_table)
            self.agent.open_gripper(wait=False)
            # self.agent.pose.pick_side_pose_by_height(height=self.pick_table_height + self.pick_front_bias[2] + self.item_height[item]/2)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) 
            self.agent.move_rel(-0.4, 0, wait=False)

            # picking test mode 추가
            # if self.picking_test_mode = True
            #     self.agent.pose.bring_bowl_pose(table=self.pick_table)
            #     self.agent.move_rel(0.2, table_base_xyz[1], wait=False)
            #     self.agent.open_gripper(wait=False)

        else:
            if item == 'spoon' or 'fork' or 'knife':
                # self.agent.pose.pick_top_pose_by_height(height=self.pick_table_height + self.pick_top_bias[2])
                table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
                self.agent.pose.pick_up_spoon_pose(table=self.pick_table)
                self.agent.open_gripper(wait=False)
                self.agent.move_rel(0, table_base_xyz[1], wait=True)
                self.agent.move_rel(table_base_xyz[0], 0, wait=True)       
                self.agent.grasp(wait=False)
                rospy.sleep(0.5)


            # picking test mode 추가
            # if self.picking_test_mode = True
            #     self.agent.pose.table_search_pose_breakfast_initial(table=self.pick_table)
            #     self.agent.move_rel(0.2, table_base_xyz[1], wait=False)
            #     self.agent.open_gripper(wait=False)

    def pour_item(self, item):
        table_base_xyz = [item_offset + bowl_offset for item_offset, bowl_offset in zip(self.pour_offsets[item], self.place_offsets['bowl'])]
    
        if item == 'cornflakes':
            self.agent.pose.spill_cornflakes_pose(table=self.place_table) # 이게 필요한가?
            self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
            self.agent.pose.spill_safety_cornflakes_pose() # spill 방지용 -> 살짝 내려가기
            self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
            self.agent.pose.wrist_roll(0) # 붓기 완료
            self.agent.pose.spill_cornflakes_pose(table=self.place_table) # 다시 올라오기

        elif item == 'milk':
            self.agent.pose.spill_milk_pose(table=self.place_table) # 이것도 필요한가
            self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
            self.agent.pose.spill_safety_milk_pose() # spill 방지용 -> 살짝 내려가기
            self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
            self.agent.pose.wrist_roll(0) # 붓기 완료
            self.agent.pose.spill_milk_pose(table=self.place_table) # 다시 올라오기

    def place_item(self, item):

        if item == 'bowl': 
            self.agent.pose.place_bowl_pose()
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True)

        elif item in ['cornflakes', 'milk']:
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True) # 옆에 두기
            self.agent.pose.arm_lift_object_table_down(self.item_height[item]/2, table=self.place_table)
        
        elif item == 'spoon' or 'fork' or 'knife':
            table_base_xyz = [axis + bias for axis, bias in zip(self.place_offsets[item], self.place_offsets['bowl'])]
            self.agent.pose.place_top_pose(0.05, table=self.place_table)
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.pose.arm_lift_object_table_down(self.place_offsets[item][2], table=self.place_table) # top_pose = 0.2
            
        self.agent.open_gripper()
        self.agent.pose.arm_lift_up(self.arm_lift_height)
        rospy.sleep(1) 

    
    def check_grasp(self, grasping_type):
        if grasping_type == 0:
            return self.agent.pose.check_grasp()
        else:
            return True


# 실제 동작 코드

    def run(self):
        stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_client.call(EmptyRequest())

        # if self.picking_test_mode:
        #  self.agent.open_gripper()
        #   continue

       # self.picking_test_mode(self, item, table_base_xyz)
    
        #self.pouring_test_mode(self, item)

        #self.placing_test_mode(self, item)


        ### task start ###

        # self.agent.door_open()
        # self.agent.say('Hi, I will serve breakfast for you!')
        # rospy.sleep(2)
        # self.agent.move_rel (0, 3)
        # self.agent.move_abs('picking_location')
        # self.agent.say('I will move to picking location')
 
        picked_items = []

        for item in self.item_list:

            has_grasped = False

            ## Try picking until an item is grasped
            while not has_grasped:
                rospy.logwarn('Go to pick_location...')
                self.agent.say('I am moving. Please be careful.')
                # self.agent.move_abs(self.pick_table)
                # self.agent.move_abs[8.3447, 4.1044, -0.0287]
                self.agent.move_rel(-0.2,0)
                self.agent.pose.table_search_pose_low()
                # self.agent.pose.table_search_pose_low(head_tilt=self.pick_table_head_angle)
                # self.agent.head_tilt(-10) / head_tilt 추가 조정 시 필요 코드
                # self.agent.move_abs_safe(self.pick_table)
                # rospy.sleep(2)

                # Search item
                rospy.sleep(1)
                item_info = self.search_item(item, picked_items)
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
                    picked_items.append(item)
                else:
                    rospy.logwarn(f'Failed to grasp {item}! Retrying...')

                # if self.picking_test_mode:
                # self.agent.open_gripper()
                # continue

                ## 3. Go to place_location
                rospy.logwarn('Going to place_location...')
                self.agent.say('I will move to a different location. Please be careful.')
                #self.agent.move_rel(0, 0, wait=True)           
                self.agent.move_abs_safe(self.place_table)
                # self.agent.move_rel(-0.4, 0)
                # self.agent.pose.holding_pose() # 대회 당일 의자나 아래 부분에 장애물이 있을 것도 고려해야 함. 현재 고려 x.

                if item in ['cornflakes', 'milk']:
                    self.pour_item(item=item)

                # self.agent.move_rel(-0.4,0, wait=True)
                rospy.logwarn('Placing item...')
                self.place_item(item=item)
                self.agent.pose.table_search_pose_breakfast_initial()
                self.agent.move_rel(-0.2, 0, wait=False) # kitchen_table 앞으로 안전한 이동을 위해 추가