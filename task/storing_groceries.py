import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
import cv2
from hsr_agent.agent import Agent

class StoringGroceries:

    def __init__(self, agent: Agent):
        self.agent = agent

        # !!! Mode params !!!
        # Set everything to False for actual task
        self.cheating_mode = False

        self.ignore_arena_door = True
        self.ignore_shelf_door = False
        
        self.picking_only_mode = False
        self.place_only_mode = False
        
        # !!! Environment params !!!
        self.closed_shelf_side = 'left' # ['right', 'left']
        self.prior_categories = ['fruit', 'food', 'dish']
        self.ignore_items = ['dish', 'plate', 'yello_bag', 'blue_bag']
        self.item_list = ['mustard', 'apple', 'orange', 'blue_mug', 'blue_milk', 'pear', 'bowl']

        # !!! Measured Distances !!!
        self.dist_to_table = 0.865
        self.dist_to_shelf = 0.9
        # self.dist_to_shelf = 0.82
        self.place_dist = 0.07
        self.new_category_dist = (self.agent.table_dimension['grocery_shelf_1f'][0] * 0.9
                                - self.place_dist * 2) / 2

        # !!! Hard-Coded Offsets !!!
        self.pick_front_bias = [0.03, 0.00, -0.04] # [x, y, height]
        self.pick_top_bias = [0.03, 0, -0.015]
        self.pick_bowl_bias = [0.15, 0.04, -0.13]
        self.place_x_bias = [None, -0.40, -0.20, 0.0]

        self.open_shelf_location = 'grocery_shelf_door'

        self.pick_table = 'grocery_table'
        self.table_height = self.agent.table_dimension[self.pick_table][2]
        self.table_depth = self.agent.table_dimension[self.pick_table][1]
        self.table_head_angle = np.arctan(
            (self.table_height - 1.1) / self.dist_to_table # 1.1: HSR height
        )

        self.place_location = 'grocery_shelf'
        self.place_shelf = 'grocery_shelf_1f'
        self.shelf_depth = self.agent.table_dimension['grocery_shelf_1f'][1]
        self.shelf_width = self.agent.table_dimension['grocery_shelf_1f'][0]
        self.shelf_heights = [ # [NULL, 1F height, 2F height, 3F height, ...]
            0,
            self.agent.table_dimension['grocery_shelf_1f'][2],
            self.agent.table_dimension['grocery_shelf_2f'][2],
            self.agent.table_dimension['grocery_shelf_3f'][2],
        ]
        for i in range(1, len(self.shelf_heights)):
            if 0.5 <= self.shelf_heights[i] <= 1.1:
                self.open_shelf_floor = f'grocery_shelf_{i}f'
                rospy.loginfo(f"open_shelf_floor: {self.open_shelf_floor}")

        self.shelf_head_angle = np.arctan(
            ((self.shelf_heights[0] + self.shelf_heights[1]) / 2 - 1.1) / self.dist_to_shelf # 1.1: HSR height
        )
        print(f"shelf_head_angle: {self.shelf_head_angle}")

        self.shelf_item_dict = {}
        self.grasp_failure_count = np.zeros(1000, dtype=int)  # 1000 is an arbitraty large number

    
    def open_shelf(self, side=None):

        if side is None:
            side = self.closed_shelf_side
        self.agent.say(f'I will attept to open the {side} door.')

        # 1. reach into door
        self.agent.pose.reach_shelf_door_pose(shelf=self.open_shelf_floor, side=side)
        reach_move_x = self.dist_to_shelf - 0.5 # 0.5 is the min distance from base to gripper when in cling_shelf_door_pose
        if side == 'left':
            reach_move_y = -0.10
        elif side == 'right':
            reach_move_y = 0.10
        self.agent.move_rel(0.0, reach_move_y, wait=True)
        self.agent.move_rel(reach_move_x, 0.0, wait=True)

        # 2. open door
        self.agent.pose.cling_shelf_door_pose(shelf=self.open_shelf_floor, side=side)
        open_move_x = - self.shelf_width / 2
        if side == 'left':
            open_move_y = max(0, self.shelf_width / 2 - 0.12)
        elif side == 'right':
            open_move_y = -max(0, self.shelf_width / 2 - 0.12)
        self.agent.move_rel(open_move_x / 3, 0.0, wait=True)
        self.agent.move_rel(open_move_x / 3 * 2, open_move_y, wait=True)

        # 3. move back to searcing position
        self.agent.pose.table_search_pose(head_tilt=self.shelf_head_angle, wait_gripper=False)


    def search_shelf(self):

        self.agent.say('Examining items in shelf.')
        center_list = np.zeros(0)
        table_search_attempts = 0
        while center_list.size == 0:
            rospy.sleep(0.2) # give some time for the YOLO to update
            center_list = np.array(
                self.agent.yolo_module.detect_3d_safe(
                    table='grocery_table',
                    dist=self.dist_to_table,
                    depth=self.table_depth,
                    item_list=self.item_list
                )
            )
            table_search_attempts += 1
            if table_search_attempts == 20:
                self.agent.say('This is taking a while...')
            if table_search_attempts == 40:
                self.agent.say('Still examining...')
            if table_search_attempts == 60:
                self.agent.say('The examination never ends...')
            if table_search_attempts == 80:
                self.agent.say('I am screwed... I will give up placing...')
                break
        rospy.loginfo(f"Shelf item center list:\n{center_list}")

        # 1. make shelf_item_dict
        object_cnts_by_floor = [0 for i in range(len(self.shelf_heights)+1)]
        object_cnts_by_floor[0] = 1000 # [NULL, 1F, 2F, 3F, ...]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_name = self.agent.yolo_module.find_name_by_id(shelf_item_class_id)
            shelf_item_type = self.agent.object_type_list[self.agent.yolo_module.find_type_by_id(shelf_item_class_id)]
            
            shelf_item_floor = 0
            for i in range(len(self.shelf_heights)):
                if shelf_item_cent_z < self.shelf_heights[i+1]:
                    shelf_item_floor = i
                    break

            object_cnts_by_floor[shelf_item_floor] += 1
            self.shelf_item_dict[shelf_item_type] = {
                'name': shelf_item_name,
                'center': [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z],
                'floor': shelf_item_floor
            }
            rospy.loginfo(f"[{shelf_item_type}] {shelf_item_name} is detected at {shelf_item_floor}F")
            rospy.loginfo(f"Item center: {shelf_item_cent_x}, {shelf_item_cent_y}, {shelf_item_cent_z}")

        # 2. add new category in shelf_item_dict
        shelf_item_dict_keys = list(self.shelf_item_dict.keys())
        shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z = self.shelf_item_dict[shelf_item_dict_keys[0]]['center']
        new_category_floor = self.shelf_item_dict[shelf_item_dict_keys[0]]['floor']
        rospy.loginfo(f"New Category Floor: {new_category_floor}F")
        self.shelf_item_dict['new'] = {
            'name': '-',
            'center': [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z],
            'floor': new_category_floor,
        }

        print(f'shelf_item_dict: {self.shelf_item_dict}')


    def visualize_item(self):
        pass


    def search_item(self):
        table_item_list = np.zeros(0)
        table_search_attempts = 0
        while table_item_list.size == 0:
            rospy.sleep(0.2) # give some time for the YOLO to update
            table_item_list = np.array(
                self.agent.yolo_module.detect_3d_safe(
                    table='grocery_table',
                    dist=self.dist_to_table,
                    depth=self.table_depth,
                    item_list=self.item_list
                )
            )
            table_search_attempts += 1
            if table_search_attempts == 20:
                self.agent.say('Searching is taking a while...')
            if table_search_attempts == 40:
                self.agent.say('Still searching...')
            if table_search_attempts == 80:
                self.agent.say('The search never ends...')
            if table_search_attempts == 120:
                self.agent.say('No more available items... Finnish Storing Groceries.')
                rospy.logwarn('Finish Storing Groeceries.')
                return

        table_item_list_sorted = table_item_list[table_item_list[:, 0].argsort()]
        table_item_list_sorted = [int(i) for i in table_item_list_sorted[:, 3]]

        for table_item_id in table_item_list_sorted:
            table_item_name = self.agent.yolo_module.find_name_by_id(table_item_id)
            table_item_type = self.agent.object_type_list[self.agent.yolo_module.find_type_by_id(table_item_id)]
            grasping_type = self.agent.yolo_module.find_grasping_type_by_id(table_item_id)

            # Only grasp items of available categories that have failed less than 3 times
            if self.grasp_failure_count[table_item_id] <= 2 and\
                table_item_type in self.prior_categories and\
                table_item_name not in self.ignore_items:
                break

        rospy.loginfo(f"Table item name:  {table_item_name}")
        rospy.loginfo(f"Table item type : {table_item_type}")
        rospy.loginfo(f"Grasping type :   {grasping_type}")
        rospy.loginfo(f"grasp_failure_count: {self.grasp_failure_count[table_item_id]}")

        try:    
            table_base_to_object_xyz = self.agent.yolo_module.find_3d_points_by_name(table_item_list, table_item_name)
            table_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)  # inclined grasping type
            rospy.loginfo(f"Base to object:   {table_base_to_object_xyz}")
            rospy.loginfo(f"Table base :      {table_base_xyz}")
            return (grasping_type, table_item_id, table_item_name, table_item_type, table_base_xyz)
        except Exception as e:
            rospy.logerr(f'[ERROR] table_base_to_object_xyz: {table_base_to_object_xyz}\n{e}')
            return None
        

    def search_all_items(self):
        table_item_list = np.zeros(0)
        table_search_attempts = 0
        while table_item_list.size == 0:
            rospy.sleep(0.2) # give some time for the YOLO to update
            table_item_list = np.array(
                self.agent.yolo_module.detect_3d_safe(
                    table='grocery_table',
                    dist=self.dist_to_table,
                    depth=self.table_depth
                )
            )
            table_search_attempts += 1
            if table_search_attempts == 20:
                self.agent.say('Searching is taking a while...')
            if table_search_attempts == 40:
                self.agent.say('Still searching...')
            if table_search_attempts == 80:
                self.agent.say('The search never ends...')
            if table_search_attempts == 120:
                self.agent.say('No more available items... Finnish Storing Groceries.')
                rospy.logwarn('Finish Storing Groeceries.')
                return

        table_item_list_sorted = table_item_list[table_item_list[:, 0].argsort()]
        table_item_list_sorted = [int(i) for i in table_item_list_sorted[:, 3]]

        item_list = []
        tiny_object_list = []
        num_items = len(table_item_list_sorted)
        for i in range(num_items):
            table_item_id = table_item_list_sorted[i]
            table_item_name = self.agent.yolo_module.find_name_by_id(table_item_id)
            table_item_type = self.agent.object_type_list[self.agent.yolo_module.find_type_by_id(table_item_id)]
            item_info = {
                'id': table_item_id,
                'name': table_item_name,
                'type': table_item_type
            }
            rospy.loginfo(f"Table item name : {table_item_name}")
            rospy.loginfo(f"Table item type : {table_item_type}")
            self.visualize_item()
            self.agent.say(f'{"I spy a" if i == 0 else "and a" if i == num_items-1 else "a"} {table_item_name}, which is a {table_item_type}...')
            rospy.sleep(3)
            if table_item_name in self.agent.tiny_object_list:
                tiny_object_list.append(item_info)
            else:
                item_list.append(item_info)
        
            item_list = tiny_object_list[1:] + item_list + tiny_object_list[1:]

        return item_list


    def pick_item(self, grasping_type, table_base_xyz):

        # front
        if grasping_type == 0:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=False)
            self.agent.pose.pick_side_pose_by_height(height=self.table_height + self.pick_front_bias[2])
            self.agent.open_gripper(wait=False)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) # wait for grasping manually
            self.agent.move_rel(-0.4, 0, wait=False)

        # top
        elif grasping_type == 1:
            self.agent.pose.pick_top_pose_by_height(height=self.table_height + self.pick_top_bias[2])
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
            self.agent.open_gripper(wait=False)
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.grasp(wait=False)
            rospy.sleep(0.5) # wait for grasping manually
            self.agent.pose.arm_flex(-60)

        # bowl. # don't care plate, ...
        elif grasping_type == 2:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]

            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.open_gripper()

            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)

            self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=self.pick_bowl_bias[2])
            self.agent.grasp()
            self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
            self.agent.move_rel(-0.2, 0)


    def place_item(self, item_name, item_type):

        # Set offset dist from object in shelf
        if item_type not in self.shelf_item_dict.keys():  # for new category objects
            self.shelf_item_dict[item_type] = self.shelf_item_dict.pop('new')
            self.shelf_item_dict[item_type]['name'] = item_name
            if item_type not in self.prior_categories:
                self.prior_categories.append(item_type)
            rospy.loginfo(f'New category : {item_type} (Current item: {item_name})')
            item_floor = self.shelf_item_dict[item_type]['floor']
            self.agent.say(f'Placing a new category object, {item_name}, on shelf floor {item_floor}...')
        else:  # for known category objects
            item_floor = self.shelf_item_dict[item_type]['floor']
            rospy.loginfo(f'Known category : {item_type} (Current item: {item_name})')
            self.agent.say(f'Placing {item_name}, next to {self.shelf_item_dict[item_type]["name"]}, on shelf floor {item_floor}...')

        print(f'Type of item to place: {item_type}')
        print(f'shelf_item_dict: {self.shelf_item_dict[item_type]}')

        # Get axis of the shelf the item belongs
        try:
            shelf_item_cent_y = self.shelf_item_dict[item_type]['center'][1]
        except:
            rospy.logerr('Failed to navigate shelf...')
            self.agent.open_gripper()
            self.agent.move_rel(-0.4, 0, wait=True)
            self.agent.pose.table_search_pose()
        
        # Place item in shelf
        self.agent.pose.place_shelf_pose(f'grocery_shelf_{item_floor}f')
        if shelf_item_cent_y > 0:   # 3d horizontal cent point.
            shelf_base_to_object_xyz = [self.dist_to_shelf + self.place_x_bias[item_floor], shelf_item_cent_y - self.place_dist, 0]
        else:
            shelf_base_to_object_xyz = [self.dist_to_shelf + self.place_x_bias[item_floor], shelf_item_cent_y + self.place_dist, 0]
        shelf_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(shelf_base_to_object_xyz, 4)
        rospy.loginfo(f'shelf_base_xyz: {shelf_base_xyz}')
        self.agent.move_rel(shelf_base_xyz[0], shelf_base_xyz[1], wait=True)
        self.agent.open_gripper()
        self.agent.move_rel(-0.4, 0, wait=False)
        rospy.sleep(1) # wait manually


    def check_grasp(self, grasping_type):
        if grasping_type == 0:
            return self.agent.pose.check_grasp()
        else:
            return True


    def run(self):
        # Stop viewpoint_controller
        stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_client.call(EmptyRequest())

        self.agent.pose.table_search_pose(head_tilt=self.table_head_angle, wait_gripper=False)

        if not self.ignore_arena_door:
            self.agent.door_open()
            rospy.logwarn('Start Storing Groceries')
            self.agent.say('Start Storing Groceries')
            self.agent.move_rel(2.0, 0, wait=True) # to prevent arena door collision
        else:
            rospy.logwarn('Start Storing Groceries')
            self.agent.say('Start Storing Groceries')

        ## OPTION 1: Run Storing Groceries by exploiting the rulebook as much as possible
        if self.cheating_mode:

            # Inspect items on pick_table
            rospy.logwarn('Going to pick_location...')
            self.agent.move_abs_safe(self.pick_table)
            item_info_list = self.search_all_items()

            # Open shelf door
            rospy.logwarn('Going to open_shelf_location...')
            self.agent.move_abs_safe(self.open_shelf_location)
            self.open_shelf(side=self.closed_shelf_side)

            # Search shelf items
            rospy.logwarn('Searching items in shelf...')
            self.agent.pose.head_tilt(angle=self.shelf_head_angle)
            self.search_shelf()

            # Place items in shelf
            for item_info in item_info_list:
                self.agent.pose.pick_side_pose_by_height(height=1.0)
                self.agent.open_gripper(wait=False)

                self.agent.say(f'Please hand me the {item_info['name']}.')
                rospy.sleep(2)
                self.agent.say('three'); rospy.sleep(1)
                self.agent.say('two');   rospy.sleep(1)
                self.agent.say('one');   rospy.sleep(1)
                self.agent.grasp(wait=True)

                self.place_item(self.place_test_object_name, self.place_test_object_type)
                self.agent.move_abs_safe(self.place_location)

        ## OPTION 2: Run Storing Groceries without any gimmicky approaches
        else:
            if not self.ignore_shelf_door:
                self.agent.move_abs_safe(self.open_shelf_location)
                self.open_shelf(side=self.closed_shelf_side)
            
            if self.place_only_mode:
                self.agent.move_abs_safe(self.place_location)
                self.search_shelf()

                while True:
                    self.agent.pose.pick_side_pose_by_height(height=1.0)
                    self.agent.open_gripper(wait=False)

                    self.agent.say('Please hand me an item.')
                    rospy.sleep(2)
                    self.agent.say('three'); rospy.sleep(1)
                    self.agent.say('two');   rospy.sleep(1)
                    self.agent.say('one');   rospy.sleep(1)
                    self.agent.grasp(wait=True)

                    self.place_item(self.place_test_object_name, self.place_test_object_type)
                    self.agent.move_abs_safe(self.place_location)

            has_searched_shelf = False
            
            # Search items in shelf only once
            if not has_searched_shelf:
                rospy.logwarn('Going to place_location...')
                self.agent.move_abs_safe(self.place_location)

                rospy.logwarn('Searching items in shelf...')
                self.agent.pose.head_tilt(angle=self.shelf_head_angle)
                self.search_shelf()
                has_searched_shelf = True

            # Pick & place loop
            while True:

                has_grasped = False

                ## 2. Try picking until an item is grasped
                while not has_grasped:
                    # 2-1. Go to pick_location
                    rospy.logwarn('Going to pick_location...')
                    self.agent.pose.table_search_pose(head_tilt=self.table_head_angle)
                    self.agent.move_abs_safe(self.pick_table)

                    # 2-2. Select item to pick
                    rospy.logwarn('Searching for item to pick...')
                    self.agent.say('Searching for item to pick.')
                    item_info = self.search_item()
                    if item_info is None:
                        rospy.logwarn('Failed to find item to pick. Retrying...')
                    else:
                        grasping_type, table_item_id, table_item_name, table_item_type, table_base_xyz = item_info

                    # 2-3. Pick item
                    rospy.logwarn('Picking item...')
                    self.pick_item(grasping_type, table_base_xyz)
                    self.agent.pose.table_search_pose(head_tilt=self.table_head_angle)

                    # 2-4. Check if grasping is successful
                    has_grasped = self.check_grasp(grasping_type)
                    if has_grasped:
                        rospy.loginfo(f'Successfuly grasped {table_item_name}!')
                        self.agent.say(f'Grasped a {table_item_name}, which is a {table_item_type}!')
                    else:
                        rospy.logwarn(f'Failed to grasp {table_item_name}! Retrying...')
                        self.grasp_failure_count[table_item_id] += 1

                if self.picking_only_mode:
                    self.agent.open_gripper()
                    continue

                ## 3. Go to place_location
                rospy.logwarn('Going to place_location...')
                self.agent.move_abs_safe(self.place_location)

                ## 4. Place item
                rospy.logwarn('Placing item...')
                self.place_item(table_item_name, table_item_type)