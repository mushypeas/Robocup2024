import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
import cv2
from hsr_agent.agent import Agent

class StoringGroceries:

    def __init__(self, agent: Agent):
        self.agent = agent

        ## !!! Mode params !!!
        # Set everything to False for actual task
        self.ignore_arena_door = False
        self.ignore_shelf_door = False
        
        self.picking_only_mode = False
        self.place_only_mode = False
        
        ## !!! Environment params !!!
        self.closed_shelf_side = 'right' # ['right', 'left']
        self.open_shelf_floor = 2
        self.prior_categories = ['drink', 'fruit', 'food', 'snack']
        self.ignore_items = ['bowl', 'dish', 'plate', 'big coke', 'candle', 'soap', 'banana', 'white bag', 'yellow bag', 'sausages']
        # self.ignore_items = ['bowl', 'dish', 'plate', 'big coke', 'candle', 'soap', 'banana', 'white bag', 'yellow bag']
        self.item_list = None
        # self.item_list = ['bowl']

        ## !!! Measured Distances !!!
        self.table_dist = 0.98
        self.shelf_dist = 1.06
        self.place_dist = 0.12
        self.new_category_dist = (self.agent.table_dimension['grocery_shelf'][0][0] * 0.9
                                - self.place_dist * 2) / 2

        ## !!! Hard-Coded Offsets !!!
        self.pick_front_bias = [0.05, 1.15, -0.03] # [x, y_ratio, height]
        self.pick_top_bias = [0.1, 0, -0.015]
        self.pick_bowl_bias = [0.0, 0.00, -0.10]
        self.place_x_bias = [None, -0.33, -0.33, -0.22]

        self.open_shelf_location = 'grocery_shelf_door'
        self.open_shelf_move_y = 0.10
        self.open_shelf_move_yaw = 0.05

        self.pick_table = 'grocery_table'
        self.table_height = self.agent.table_dimension[self.pick_table][2]
        self.table_depth = self.agent.table_dimension[self.pick_table][1]
        self.table_head_angle =np.deg2rad(np.arctan(
            (self.table_height - 1.1) / self.table_dist # 1.1: HSR height
        ))
        self.place_shelf = 'grocery_shelf'
        self.place_test_object_name, self.place_test_object_type = 'orange', 'fruit'
        self.shelf_depth = self.agent.table_dimension[self.place_shelf][0][1]
        self.shelf_width = self.agent.table_dimension[self.place_shelf][0][0]
        self.shelf_heights = [shelf_dim[2] for shelf_dim in self.agent.table_dimension['grocery_shelf']]

        self.shelf_head_angle = -0.157
        print(f"table_head_angle: {self.table_head_angle}") 
        print(f"shelf_head_angle: {self.shelf_head_angle}")

        self.shelf_item_dict = {}
        self.grasp_failure_count = np.zeros(1000, dtype=int)  # 1000 is an arbitraty large number


    def open_shelf(self, side=None):

        if side is None:
            side = self.closed_shelf_side
        self.agent.say(f'I will attept to open the {side} door.')

        # 1. reach into door
        self.agent.pose.reach_shelf_door_pose(shelf=self.place_shelf, floor=self.open_shelf_floor, side=side)
        reach_move_x = self.shelf_dist - 0.57 # 0.52 is the min distance from base to gripper when in cling_shelf_door_pose
        if side == 'left':
            reach_move_y = -self.open_shelf_move_y
            open_move_yaw = self.open_shelf_move_yaw
        elif side == 'right':
            reach_move_y = self.open_shelf_move_y
            open_move_yaw = -self.open_shelf_move_yaw
        self.agent.move_rel(0.0, reach_move_y, wait=True)
        self.agent.move_rel(reach_move_x, 0, wait=True)

        # 2. open door
        self.agent.pose.cling_shelf_door_pose(shelf=self.place_shelf, floor=self.open_shelf_floor, side=side)
        open_move_x = - self.shelf_width / 2
        if side == 'left':
            open_move_y = max(0, self.shelf_width / 2 - 0.12)
        elif side == 'right':
            open_move_y = -max(0, self.shelf_width / 2 - 0.12)
        self.agent.move_rel(open_move_x, open_move_y, open_move_yaw, wait=True)

        # 3. move back to searcing position
        self.agent.pose.table_search_pose(head_tilt=self.shelf_head_angle, wait_gripper=True)


    def search_shelf(self):
        center_list = np.array(
            self.agent.yolo_module.detect_3d_safer(
                height=self.shelf_heights[1],
                dist=self.shelf_dist,
                depth=self.shelf_depth
            )
        )
        rospy.loginfo(f"Shelf item center list:\n{center_list}")

        # 1. make shelf_item_dict
        object_cnts_by_floor = [0 for i in range(len(self.shelf_heights))]
        object_cnts_by_floor[0] = 1000 # [NULL, 1F, 2F, 3F, ...]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_name = self.agent.yolo_module.find_name_by_id(shelf_item_class_id)
            shelf_item_type = self.agent.object_type_list[self.agent.yolo_module.find_type_by_id(shelf_item_class_id)]
            
            shelf_item_floor = 0
            for i in range(len(self.shelf_heights)):
                shelf_item_floor = i
                if shelf_item_cent_z < self.shelf_heights[i] - 0.05: # 0.05 is the margin for short items
                    shelf_item_floor = i-1
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
        new_category_floor = np.argmin(object_cnts_by_floor)
        if object_cnts_by_floor[new_category_floor] == 0:
            new_shelf_item_cent_x = self.shelf_dist
            new_shelf_item_cent_y = 0
            new_shelf_item_cent_z = self.shelf_heights[new_category_floor] + 0.05
        else:
            shelf_item_dict_keys = list(self.shelf_item_dict.keys())
            for key in shelf_item_dict_keys:
                if self.shelf_item_dict[key]['floor'] == new_category_floor:
                    new_shelf_item_cent_x, new_shelf_item_cent_y, new_shelf_item_cent_z = self.shelf_item_dict[key]['center']
                    if shelf_item_cent_y > 0:
                        new_shelf_item_cent_y -= self.new_category_dist
                    else:
                        new_shelf_item_cent_y += self.new_category_dist
                    break

        rospy.loginfo(f"New Category Floor: {new_category_floor}F")
        self.shelf_item_dict['new'] = {
            'name': '-',
            'center': [new_shelf_item_cent_x, new_shelf_item_cent_y, new_shelf_item_cent_z],
            'floor': new_category_floor,
        }

        for key in self.shelf_item_dict.keys():
            rospy.loginfo(f"shelf_item_dict[{key}]: {self.shelf_item_dict[key]}")


    def visualize_item(self):
        pass


    def search_item(self):
        table_item_list = np.array(
            self.agent.yolo_module.detect_3d_safer(
                height=self.table_height,
                dist=self.table_dist,
                depth=self.table_depth,
                item_list=self.item_list,
                ignore_items=self.ignore_items,
            )
        )
        table_item_list_sorted = table_item_list[table_item_list[:, 0].argsort()]
        table_item_list_sorted = [int(i) for i in table_item_list_sorted[:, 3]]

        for table_item_id in table_item_list_sorted:
            table_item_name = self.agent.yolo_module.find_name_by_id(table_item_id)
            table_item_type = self.agent.object_type_list[self.agent.yolo_module.find_type_by_id(table_item_id)]
            grasping_type = self.agent.yolo_module.find_grasping_type_by_id(table_item_id)

            # Only grasp items of available categories that have failed less than 2 times
            if self.grasp_failure_count[table_item_id] <= 1 and\
                table_item_type in self.prior_categories:
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


    def pick_item(self, grasping_type, table_base_xyz):

        # front
        if grasping_type == 0:
            table_base_xyz[0] += self.pick_front_bias[0]
            table_base_xyz[1] *= self.pick_front_bias[1] # The HSR tends to undershoot the distance for some reason, hence giving the bias as a y axis multiplier
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
            self.agent.move_rel(-0.4, 0, wait=True)


        # bowl. # don't care plate, ...
        elif grasping_type == 2:
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_bowl_bias)]

            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.open_gripper()

            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)

            self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=self.pick_bowl_bias[2])
            self.agent.grasp()
            self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
            self.agent.move_rel(-0.4, 0, wait=False)


    def place_item(self, item_name, item_type):
        rospy.logdebug(f'prior_categories: {self.prior_categories}')
        # Set offset dist from object in shelf
        if item_type not in self.shelf_item_dict.keys():  # for new category objects
            self.shelf_item_dict[item_type] = self.shelf_item_dict.pop('new')
            self.shelf_item_dict[item_type]['name'] = item_name
            self.prior_categories = self.shelf_item_dict.keys()
            rospy.loginfo(f'New category : {item_type} (Current item: {item_name})')
            item_floor = self.shelf_item_dict[item_type]['floor']
            self.agent.say(f'Placing a new category object, {item_name}, on shelf floor {item_floor}...')
        else:  # for known category objects
            item_floor = self.shelf_item_dict[item_type]['floor']
            rospy.loginfo(f'Known category : {item_type} (Current item: {item_name})')
            self.agent.say(f'Placing {item_name},\n next to {self.shelf_item_dict[item_type]["name"]},\n on shelf floor {item_floor}...', show_display=True)

        # Get axis of the shelf the item belongs
        try:
            shelf_item_cent_y = self.shelf_item_dict[item_type]['center'][1]
        except:
            rospy.logerr('Failed to navigate shelf...')
            self.agent.open_gripper()
            self.agent.move_rel(-0.4, 0, wait=True)
            self.agent.pose.table_search_pose()
        
        # Place item in shelf
        self.agent.pose.place_shelf_pose(shelf=self.place_shelf, floor=item_floor)
        if shelf_item_cent_y > 0:   # 3d horizontal cent point.
            shelf_base_to_object_xyz = [self.shelf_dist + self.place_x_bias[item_floor], shelf_item_cent_y - self.place_dist, 0]
        else:
            shelf_base_to_object_xyz = [self.shelf_dist + self.place_x_bias[item_floor], shelf_item_cent_y + self.place_dist, 0]
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

        ## 0. Preparation
        stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
        stop_client.call(EmptyRequest())

        self.agent.pose.table_search_pose(head_tilt=self.shelf_head_angle)
        if not self.ignore_arena_door:
            self.agent.door_open()
            rospy.logwarn('Start Storing Groceries')
            self.agent.say('Start Storing Groceries', show_display=True)
            # self.agent.move_rel(2.0, 0, wait=True) # Advance 2 meters
            self.agent.move_abs_safe('living_living_1')
            self.agent.move_abs_safe('living_living_2')

        else:
            rospy.logwarn('Start Storing Groceries')
            self.agent.say('Start Storing Groceries', show_display=True)
            self.agent.move_abs_safe('living_living')

        ## 1. Open shelf door
        if not self.ignore_shelf_door:
            self.agent.say(f'Please close the \n {self.closed_shelf_side} shelf door.')
            self.agent.move_abs_safe(self.open_shelf_location)
            self.open_shelf(side=self.closed_shelf_side)
            self.agent.say(f'Please remove all \n big and tiny objects \n on the table.')
            rospy.sleep(1)

        rospy.logwarn('Going to place_location...')
        self.agent.move_abs_safe(self.place_shelf)

        ## 2. Search shelf
        rospy.logwarn('Examining items in shelf...')
        self.agent.say('Examining items in shelf.', show_display=True)
        rospy.sleep(2) # give some time for the YOLO to update
        self.search_shelf()
        
        if self.place_only_mode:
            self.agent.move_abs_safe(self.place_shelf)
            self.search_shelf()

            while True:
                self.agent.pose.pick_side_pose_by_height(height=1.0)
                self.agent.open_gripper(wait=True)

                self.agent.say('Please hand me an item.', show_display=True)
                rospy.sleep(2)
                self.agent.say('three'); rospy.sleep(1)
                self.agent.say('two');   rospy.sleep(1)
                self.agent.say('one');   rospy.sleep(1)
                self.agent.grasp(wait=True)

                self.place_item(self.place_test_object_name, self.place_test_object_type)
                self.agent.move_abs_safe(self.place_shelf)

        while True:

            ## 3. Try picking until an item is grasped
            has_grasped = False
            while not has_grasped:
                # 3-1. Go to pick_location
                rospy.logwarn('Going to pick_location...')
                self.agent.pose.table_search_pose(head_tilt=self.table_head_angle)
                self.agent.move_abs_safe(self.pick_table)

                # 3-2. Select item to pick
                rospy.logwarn('Searching for item to pick...')
                self.agent.say('Searching for item to pick.', show_display=True)
                rospy.sleep(1) # give some time for the YOLO to update
                item_info = self.search_item()
                if item_info is None:
                    rospy.logwarn('Failed to find item to pick. Retrying...')
                else:
                    grasping_type, table_item_id, table_item_name, table_item_type, table_base_xyz = item_info

                # 3-3. Pick item
                rospy.logwarn('Picking item...')
                self.agent.say(f'I will pick a {table_item_name},\nwhich is a {table_item_type}.', show_display=True)
                self.pick_item(grasping_type, table_base_xyz)
                self.agent.pose.table_search_pose(head_tilt=self.table_head_angle)

                # 3-4. Check if grasping is successful
                rospy.sleep(1)
                has_grasped = self.check_grasp(grasping_type)
                if has_grasped:
                    rospy.loginfo(f'Successfuly grasped {table_item_name}!')
                    self.agent.say(f'Picked a {table_item_name},\nwhich is a {table_item_type}!', show_display=True)
                else:
                    rospy.logwarn(f'Failed to grasp {table_item_name}! Retrying...')
                    self.grasp_failure_count[table_item_id] += 1

            if self.picking_only_mode:
                self.agent.open_gripper()
                continue

            ## 4. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.move_abs_safe(self.place_shelf)

            ## 5. Place item
            rospy.logwarn('Placing item...')
            self.place_item(table_item_name, table_item_type)