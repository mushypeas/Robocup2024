import rospy
from utils.distancing import distancing
from std_srvs.srv import Empty, EmptyRequest
from hsr_agent.agent import Agent
import math
 
################# Final (Eindhoven ver.) ################################ 

# 출발장소 : Entrance
# picking_location 좌표 : [ ]

# < Loction >
# kitchen_table = kitchen_counter : [ ]
# breakfast_table = dinner table : [   ]
 
# < Dimension >
# kitchen_table = kitchen_counter -> pick_up table [   ]
# breakfast_table = dinner table -> placing table [    ]

# <Item >
# Cereal : 0.291 * 0.055 * 0.26
# Milk : 0.075 * 0.07 * 0.098 (앞) / 0.114 (뒤)
#########################################################################

  # pick_bowl_pose_last 에서 4cm 정도 내려가기

class ServeBreakfast:

    def __init__(self, agent: Agent):
        self.agent = agent
        # self.agent.move_rel = self.move_rel_temp

        # !!! Test params !!!
        self.attemp_pouring = {
            'cornflakes': True,
            'milk': True,
        }

        self.item_list = ['bowl','spoon', 'fork', 'knife', 'cornflakes', 'milk']
        
        # !!! Measured Distances !!!
        self.dist_to_pick_table = 0.8
        self.dist_to_place_table = 0.7
        self.item_height = {     # 실제 object 높이는 여기서 설정
            'cornflakes': 0.25,
            'milk': 0.114,   # Milk 0.075 * 0.07 * 0.098 (앞) / 0.114 (뒤) // Cornflakes 0.019 * 0.055 * 0.255
        }

        # !!! Hard-Coded Offsets !!!    # [x, y, height] / 최대한 offset을 쓰지 않는 방향으로 하되, 실제 시험장에서는 어쩔 수 없이 사용할 수는 있음.
        self.pick_front_bias = [0.0, 0.00, 0.0] 
        self.pick_top_bias = [0.00, 0.00, 0.0]  
        self.pick_bowl_bias = [0.0, 0.00, 0.0]
        self.pick_cornflakes_bias = [0.0, 0.00, 0.0]
        self.pick_milk_bias = [0.0, 0.00, 0.0]
        self.pick_spoon_bias = [0.0, 0.00, 0.0]      


        self.pour_offsets = { # [x, y, angle] 
            'cornflakes': [0.0, -0.3, 120], # bowl 왼쪽 끝에 맞춰서 따르기
            'milk': [0.0, -0.12, 80], # bowl 왼쪽 끝에 맞춰서 따르기
        }
        self.arm_lift_height = 0.68
        self.place_offsets = { # [x, y, height], bowl을 테이블에서 약간 오른쪽에 두도록 함  / placing_table에는 순서대로 'milk, cereal, bowl, spoon'이 놓이도록 함.
            'bowl': [self.dist_to_place_table, -0.1, 0],
            'cornflakes': [0, 0.2, 0],
            'milk': [0, 0.1, 0],
            'spoon': [0, -0.16, 0]
        }

        self.pick_table = 'dish_washer'

        self.pick_table_depth = self.agent.table_dimension[self.pick_table][1] 
        self.pick_table_height = self.agent.table_dimension[self.pick_table][2]
        self.pick_table_head_angle = np.arctan(
            (self.pick_table_height - 1.1) / self.dist_to_pick_table # 1.1: HSR height
        )

        self.place_table = 'dinner_table'
        self.place_table_depth = self.agent.table_dimension[self.place_table][1]

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
                table='kitchen_counter',
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

        # print('table_base_to_object_xyz', table_base_to_object_xyz)

         # 2.3 calculate dist with offset
        try: # base_xyz는 table까지의 거리 계산 후 count해서 값을 반환 
            base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)
        except:
            continue
        print('gripper_to_object_xyz', base_xyz)
        agent.move_rel(-0.50, 0) 
        

         # 4. pick (순서: bowl-> spam -> mustard -> spoon)
        if item == 'bowl':
            self.agent.move_abs('dish_washer')
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
            self.agent.move_abs('kitchen_counter')
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.pose.pick_side_pose(table=self.pick_table)
            self.agent.open_gripper(wait=False)
            # self.agent.pose.pick_side_pose_by_height(height=self.pick_table_height + self.pick_front_bias[2] + self.item_height[item]/2)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) 
            self.agent.move_rel(-0.4, 0, wait=False)

        elif item in ['milk']:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_abs('kitchen_cabinet')
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.pose.pick_side_pose(table=self.pick_table)
            # 만약, cornflakes가 horizon하게 나올 경우, 불가피하게 pose 변경 + 좌표 변경
            
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
            if item == 'spoon' or 'fork':
                # self.agent.pose.pick_top_pose_by_height(height=self.pick_table_height + self.pick_top_bias[2])
                table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
                self.agent.move_abs('dish_washer')
                self.agent.pose.pick_down_spoon_pose(table=self.pick_table)
                self.agent.open_gripper(wait=False)
                self.agent.move_rel(0, table_base_xyz[1], wait=True)
                self.agent.move_rel(table_base_xyz[0], 0, wait=True)       
                self.agent.grasp(wait=False)
                rospy.sleep(0.5)

    def pour_item(self, item):

        if item == 'cereal':
            table_base_xyz = [item_offset + bowl_offset for item_offset, bowl_offset in zip(self.pour_offsets[item], self.place_offsets['bowl'])] 
            self.agent.pose.spill_cornflakes_pose()
            self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
            self.agent.pose.spill_safety_cornflakes_pose() # (spill 방지용) 살짝 내려갔다가
            self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
            self.agent.pose.wrist_roll(0) # 붓기 완료
            self.agent.pose.spill_cornflakes_pose() # (spill 방지용) 다시 올라오기

        elif item == 'milk':
            table_base_xyz = [item_offset + bowl_offset for item_offset, bowl_offset in zip(self.pour_offsets[item], self.place_offsets['bowl'])] 
            self.agent.pose.spill_milk_pose(self.item_height[item]/2, table=self.place_table)
            self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
            self.agent.pose.spill_safety_milk_pose() # (spill 방지용) 살짝 내려갔다가
            self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
            self.agent.pose.wrist_roll(0) # 붓기 완료
            self.agent.pose.spill_milk_pose() # (spill 방지용) 다시 올라오기

    def place_item(self, item):

        if item == 'bowl': 
            self.agent.pose.place_bowl_pose()
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True)

        elif item in ['cornflakes', 'milk']:
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True) # 옆에 두기
            self.agent.pose.arm_lift_object_table_down(self.item_height[item]/2, table=self.place_table)
        
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
        #  continue


        ### task start ###

        # self.agent.door_open()
        # self.agent.say('Hi, I will serve breakfast for you!')
        # rospy.sleep(2)
        # self.agent.move_abs('hallway')
        # self.agent.say('I will move to living room')
        # self.agent.move_abs('living_room')
        # self.agent.say('I will move to kitchen')
        # self.agent.move_abs_safe('hallway')
        # self.agent.move_abs_safe('livingroom')
        # self.agent.move_abs('picking_location')
        # self.agent.say('I will move to picking location')
 
        picked_items = ['bowl']

        for item in self.item_list:

            has_grasped = False

            ## Try picking until an item is grasped
            while not has_grasped:
                rospy.logwarn('Go to pick_location...')
                self.agent.say('I am moving. Please be careful.')
                self.agent.move_abs_safe('dish_washer')
                self.agent.pose.table_search_pose_breakfast()

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

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.say('I will move to a different location. Please be careful.')
            # self.agent.move_rel(0, -1.0, wait=True)
            self.agent.move_abs_safe(self.place_table)
            self.agent.move_rel(-0.4, 0)
            # self.agent.pose.holding_pose()

            self.agent.move_rel(-0.4,0, wait=True)
            rospy.logwarn('Placing item...')
            self.place_item(item=item)
            self.agent.pose.table_search_pose_breakfast_initial()
            self.agent.move_rel(-0.6, 0.0 , wait=False) # kitchen_table 앞으로 안전한 이동을 위해 추가


        picked_items = ['spoon']

        for item in self.item_list:

            has_grasped = False

            ## Try picking until an item is grasped
            while not has_grasped:
                rospy.logwarn('Go to pick_location...')
                self.agent.say('I am moving. Please be careful.')
                self.agent.move_abs_safe('dish_washer')
                self.agent.pose.table_search_pose_breakfast()

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

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.say('I will move to a different location. Please be careful.')
            # self.agent.move_rel(0, -1.0, wait=True)
            self.agent.move_abs_safe(self.place_table)
            self.agent.move_rel(-0.4, 0)
            # self.agent.pose.holding_pose()

            self.agent.move_rel(-0.4,0, wait=True)
            rospy.logwarn('Placing item...')
            self.place_item(item=item)
            self.agent.pose.table_search_pose_breakfast_initial()
            self.agent.move_rel(-0.6, 0.0 , wait=False) # kitchen_table 앞으로 안전한 이동을 위해 추가


        picked_items = ['cornflakes']

        for item in self.item_list:

            has_grasped = False

            ## Try picking until an item is grasped
            while not has_grasped:
                rospy.logwarn('Go to pick_location...')
                self.agent.say('I am moving. Please be careful.')
                self.agent.move_abs_safe('dish_washer')
                self.agent.pose.table_search_pose_breakfast()

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

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.say('I will move to a different location. Please be careful.')
            # self.agent.move_rel(0, -1.0, wait=True)
            self.agent.move_abs_safe(self.place_table)
            self.agent.move_rel(-0.4, 0)
            # self.agent.pose.holding_pose()
            
            self.pour_item(item=item)

            self.agent.move_rel(-0.4,0, wait=True)
            rospy.logwarn('Placing item...')
            self.place_item(item=item)
            self.agent.pose.table_search_pose_breakfast_initial()
            self.agent.move_rel(-0.6, 0.0 , wait=False) # kitchen_table 앞으로 안전한 이동을 위해 추가

        # picked_items = ['milk']

        # for item in self.item_list:

        #     has_grasped = False

        #     ## Try picking until an item is grasped
        #     while not has_grasped:
        #         rospy.logwarn('Go to pick_location...')
        #         self.agent.say('I am moving. Please be careful.')
        #         self.agent.move_abs_safe('dish_washer')
        #         self.agent.pose.table_search_pose_breakfast()

        #         # Search item
        #         rospy.sleep(1)
        #         item_info = self.search_item(item, picked_items)
        #         if item_info is None:
        #             rospy.logwarn('Failed to find item to pick. Retrying...')
        #             continue
        #         else:
        #             grasping_type, table_base_xyz = item_info

        #         # Pick item
        #         rospy.logwarn('Picking item...')
        #         self.agent.say(f'I will pick a {item}.', show_display=True)
        #         self.pick_item(item, table_base_xyz)
        #         self.agent.pose.table_search_pose_breakfast_initial()

        #         # Check if grasping is successful
        #         has_grasped = self.check_grasp(grasping_type)
        #         if has_grasped:
        #             rospy.loginfo(f'Successfuly grasped {item}!')
        #             picked_items.append(item)
        #         else:
        #             rospy.logwarn(f'Failed to grasp {item}! Retrying...')

        #     ## 3. Go to place_location
        #     rospy.logwarn('Going to place_location...')
        #     self.agent.say('I will move to a different location. Please be careful.')
        #     # self.agent.move_rel(0, -1.0, wait=True)
        #     self.agent.move_abs_safe(self.place_table)
        #     self.agent.move_rel(-0.4, 0)
        #     # self.agent.pose.holding_pose()

        #     self.pour_item(item=item)

        #     self.agent.move_rel(-0.4,0, wait=True)
        #     rospy.logwarn('Placing item...')
        #     self.place_item(item=item)
        #     self.agent.pose.table_search_pose_breakfast_initial()
        #     self.agent.move_rel(-0.6, 0.0 , wait=False) # kitchen_table 앞으로 안전한 이동을 위해 추가
