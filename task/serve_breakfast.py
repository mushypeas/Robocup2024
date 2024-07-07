import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
from hsr_agent.agent import Agent
 
#  부산대학교 실험실 (10208 , 10동 208호) # 
# picking_place_pnu 좌표 : [4.3083, -1.5883, 0.0044]
# kitchen_table_pnu 좌표 : [5.254, -1.5046, -0.0351]
# breakfast_table_pnu 좌표 : [6.0959, -1.7321, 1.591]
# kitchen_table_pnu : 흰색 테이블 , dimension : [40, 80.2, 60.2] * 낮음 -> (head_tilt -15~-20으로 조정)
# breakfast_table_pnu : 나무색 테이블 , dimension : [0.735, 1.18, 0.73]
##########################################################################################
  
class ServeBreakfast:

    def __init__(self, agent: Agent):
        self.agent = agent

        # !!! Test params !!!
        self.is_picking_test_mode = True
        self.is_pouring_test_mode = False
        self.is_placing_test_mode = False

        self.attemp_pouring = {
            'cucudas': True,
            'blue_milk': True,
        }

        self.item_list = ['bowl','cucudas','blue_milk','spoon'] # In order
        
        # !!! Measured Distances !!!
        self.dist_to_pick_table = 0.93
        self.dist_to_place_table = 0.97
        self.item_height = {
            'cucudas': 0.138,
            'blue_milk': 0.13, 
        }

        # !!! Hard-Coded Offsets !!!
        self.pick_front_bias = [0.0, 0.00, 0.0]  # [x, y, height]
        self.pick_top_bias = [0.00, 0.00, 0.0]  # [x, y, height]
        self.pick_bowl_bias = [0.0, 0.00, 0.0]    # [x, y, height]
        self.pick_cucudas_bias = [0.0, 0.00, 0.0] # [x, y, height]
        self.pick_spoon_bias = [0.0, 0.00, 0.0]    # [x, y, height]     
        self.pour_offsets = { # [x, y, angle]
            'cucudas': [0.0, 0.0, 110],
            'blue_milk': [0.0, 0.0, 130],
        }
        self.arm_lift_height = 0.68
        self.place_offsets = { # [x, y, height]
            'bowl': [self.dist_to_place_table - 0.57, -0.2, 0],
            'cucudas': [0, 0.3, 0],
            'blue_milk': [0, 0.15, 0],
            'spoon': [0.25, -0.3, 0.23]
        }

        self.pick_table = 'kitchen_table_pnu' 
        self.pick_table_depth = self.agent.table_dimension[self.pick_table][1] 
        self.pick_table_height = self.agent.table_dimension[self.pick_table][2]
        self.pick_table_head_angle = np.arctan(
            (self.pick_table_height - 1.1) / self.dist_to_pick_table # 1.1: HSR height
        )

        self.place_table = 'kitchen_table_pnu' 
        self.place_table_depth = self.agent.table_dimension[self.place_table][1]

    def picking_test_mode(self, item, table_base_xyz):

        self.agent.pose.table_search_pose_breakfast_initial()

        if item == 'bowl':
            # import pdb; pdb.set_trace(), 한 줄씩 체크하는 용도
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]
            self.agent.move_rel(-0.2, table_base_xyz[1], wait=False)
            # self.agent.pose.bring_bowl_pose(table=self.pick_table) 
            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.open_gripper(wait=False)
            self.agent.move_rel(table_base_xyz[0]+0.25, 0, wait=True)
            # self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=self.pick_bowl_bias[2])
            self.agent.pose.bring_bowl_pose_low(table=self.pick_table) 
            # self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.grasp()
            self.agent.move_rel(-0.2, 0, wait=False)
            self.agent.pose.table_search_pose_breakfast_initial(table=self.pick_table)
            self.agent.pose.bring_bowl_pose_low(table=self.pick_table)
            self.agent.open_gripper(wait=False)
            self.agent.move_rel(-0.2, 0, wait=False)

        elif item in ['cucudas', 'blue_milk']:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(-0.5, table_base_xyz[1], wait=False)
            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.pose.pick_cucudas_pose(table=self.pick_table, height=self.pick_cucudas_bias[2]) # cucudas 추가
            self.agent.open_gripper(wait=False)
            # self.agent.pose.pick_side_pose_by_height(height=self.pick_table_height + self.pick_front_bias[2] + self.item_height[item]/2)
            self.agent.move_rel(table_base_xyz[0]+0.5, 0, wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) # wait for grasping manually
            self.agent.move_rel(-0.7, 0, wait=False)

        else:
            if item == 'spoon':
                self.agent.pose.bring_bowl_pose(table=self.pick_table)
                # self.agent.pose.pick_top_pose_by_height(height=self.pick_table_height + self.pick_top_bias[2])
                table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
                self.agent.open_gripper(wait=False)
                self.agent.move_rel(0, table_base_xyz[1], wait=True)
                self.agent.move_rel(table_base_xyz[0], 0, wait=True)
                self.agent.pose.pick_up_spoon_pose(table=self.pick_table, height=self.pick_spoon_bias[2])        
                self.agent.grasp(wait=False)
                rospy.sleep(0.5) # wait for grasping manually
                self.agent.pose.arm_flex(-60)

    def pouring_test_mode(self, item):
        table_base_xyz = [item_offset + bowl_offset for item_offset, bowl_offset in zip(self.pour_offsets[item], self.place_offsets['bowl'])] 
        self.agent.pose.spill_object_pose(self.item_height[item]/2, table=self.place_table)
        self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
        self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
        self.agent.pose.wrist_roll(0) # 붓기 완료

    def placing_test_mode(self, item):
        if item == 'bowl': # 주석 풀기
            self.agent.pose.place_bowl_pose()
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True)

        elif item in ['cucudas', 'blue_milk']:
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True) # 옆에 두기
            self.agent.pose.arm_lift_object_table_down(self.item_height[item]/2, table=self.place_table)
        
        elif item == 'spoon':
            table_base_xyz = [axis + bias for axis, bias in zip(self.place_offsets[item], self.place_offsets['bowl'])]
            self.agent.pose.place_top_pose(0.05, table=self.place_table)
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.pose.arm_lift_object_table_down(self.place_offsets[item][2], table=self.place_table) # top_pose = 0.2
            
        self.agent.open_gripper()
        self.agent.pose.arm_lift_up(self.arm_lift_height)
        self.agent.move_rel(-0.6, 0, wait=False)
        rospy.sleep(1) # wait manually

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
                table='breakfast_table_pnu',
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


    def pick_item(self, item, table_base_xyz): # 이 안에 test mode 추가

        if item == 'bowl':
            # import pdb; pdb.set_trace(), 한 줄씩 체크하는 용도
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]
            self.agent.move_rel(-0.2, table_base_xyz[1], wait=False)
            self.agent.open_gripper(wait=False)
            self.agent.pose.bring_bowl_pose(table=self.pick_table) 
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=self.pick_bowl_bias[2])
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.grasp()
            self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
            self.agent.move_rel(-0.4, 0, wait=False)

            # if _is_pouring_test_mode = True: 

        elif item in ['cucudas', 'blue_milk']:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(-0.5, table_base_xyz[1], wait=False)
            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.pose.pick_cucudas_pose(table=self.pick_table, height=self.pick_cucudas_bias[2]) # cucudas 추가
            self.agent.open_gripper(wait=False)
            # self.agent.pose.pick_side_pose_by_height(height=self.pick_table_height + self.pick_front_bias[2] + self.item_height[item]/2)
            self.agent.move_rel(table_base_xyz[0]+0.5, 0, wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) # wait for grasping manually
            self.agent.move_rel(-0.7, 0, wait=False)

            # if _is_pouring_test_mode = True:             

        else:
            if item == 'spoon':
                self.agent.pose.bring_bowl_pose(table=self.pick_table)
                # self.agent.pose.pick_top_pose_by_height(height=self.pick_table_height + self.pick_top_bias[2])
                table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
                self.agent.open_gripper(wait=False)
                self.agent.move_rel(0, table_base_xyz[1], wait=True)
                self.agent.move_rel(table_base_xyz[0], 0, wait=True)
                self.agent.pose.pick_up_spoon_pose(table=self.pick_table, height=self.pick_spoon_bias[2])        
                self.agent.grasp(wait=False)
                rospy.sleep(0.5) # wait for grasping manually
                self.agent.pose.arm_flex(-60)

            # if _is_pouring_test_mode = True: 

    def pour_item(self, item):

        table_base_xyz = [item_offset + bowl_offset for item_offset, bowl_offset in zip(self.pour_offsets[item], self.place_offsets['bowl'])] 
        self.agent.pose.spill_object_pose(self.item_height[item]/2, table=self.place_table)
        self.agent.move_rel(table_base_xyz[0]+self.pour_offsets[item][0], table_base_xyz[1]+self.pour_offsets[item][1], wait=True)
        self.agent.pose.wrist_roll(self.pour_offsets[item][2]) # 붓는 중
        self.agent.pose.wrist_roll(0) # 붓기 완료

            # if _is_pouring_test_mode = True: 

    def place_item(self, item):

        if item == 'bowl': # 주석 풀기
            self.agent.pose.place_bowl_pose()
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True)

            # if _is_pouring_test_mode = True: 

        elif item in ['cucudas', 'blue_milk']:
            self.agent.move_rel(self.place_offsets[item][0], self.place_offsets[item][1], wait=True) # 옆에 두기
            self.agent.pose.arm_lift_object_table_down(self.item_height[item]/2, table=self.place_table)

             # if _is_pouring_test_mode = True: 

        elif item == 'spoon':
            table_base_xyz = [axis + bias for axis, bias in zip(self.place_offsets[item], self.place_offsets['bowl'])]
            self.agent.pose.place_top_pose(0.05, table=self.place_table)
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.pose.arm_lift_object_table_down(self.place_offsets[item][2], table=self.place_table) # top_pose = 0.2

            # if _is_pouring_test_mode = True: 
            #            
        self.agent.open_gripper()
        self.agent.pose.arm_lift_up(self.arm_lift_height)
        self.agent.move_rel(-0.6, 0, wait=False)
        rospy.sleep(1) # wait manually


    
    def check_grasp(self, grasping_type):
        if grasping_type == 0:
            return self.agent.pose.check_grasp()
        else:
            return True

    
    def run(self):
        stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_client.call(EmptyRequest())

        # if self.picking_test_mode:
        #  self.agent.open_gripper()
        #   continue

        ### task start ###

        # self.agent.door_open()
        # self.agent.say('Hi, I will serve breakfast for you!')
        # rospy.sleep(2)
        # self.agent.move_abs('picking_location_pnu')
        # self.agent.say('I will move to picking location')
 
        picked_items = []

        for item in self.item_list:

            has_grasped = False

            ## Try picking until an item is grasped
            while not has_grasped:

                rospy.logwarn('Go to pick_location...')
                self.agent.say('I will move to a different location. Please be careful.')
                self.agent.move_abs(self.pick_table)
                self.agent.move_rel(-0.2,0)
                self.agent.pose.table_search_pose_low()
                # self.agent.pose.table_search_pose_low(head_tilt=self.pick_table_head_angle)
                # self.agent.head_tilt(-10) / head_tilt 추가 조정 시 필요 코드
                # self.agent.move_abs_safe(self.pick_table)
                # rospy.sleep(2)

                # Search item
                item_info = self.search_item(item, picked_items)
                if item_info is None:
                    rospy.logwarn('Failed to find item to pick. Retrying...')
                    continue
                else:
                    grasping_type, table_base_xyz = item_info

                # Test mode # picking test mode 따로 정의하지말고, pick_item 기존 def로 동일하게 진행하도록 수정
                if self.is_picking_test_mode:
                    self.picking_test_mode(item, table_base_xyz)
            
                if self.is_pouring_test_mode:
                    self.pouring_test_mode(item)

                if self.is_placing_test_mode:
                    self.placing_test_mode(item)

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

            if self.picking_test_mode:
                self.agent.open_gripper()
                continue

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.say('I will move to a different location. Please be careful.')
            self.agent.move_abs_safe(self.place_table)
            self.agent.move_rel(-0.3, 0)
            # self.agent.pose.holding_pose() # 대회 당일 의자나 아래 부분에 장애물이 있을 것도 고려해야 함. 현재 고려 x.

            if item in ['cucudas', 'blue_milk']:
                self.pour_item(item=item)

            self.agent.move_rel(-0.3,0, wait=True)
            rospy.logwarn('Placing item...')
            self.place_item(item=item)
