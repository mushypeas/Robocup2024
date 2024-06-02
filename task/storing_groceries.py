import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
import cv2
from utils.distancing import distancing, distancing_horizontal
from hsr_agent.agent import Agent

class StoringGroceries:

    def __init__(self, agent: Agent):
        self.agent = agent

        # Test params
        # Set everything to False for actual task
        self.ignore_door = True
        self.picking_test_mode = False
        self.place_test_mode = False
        self.place_test_object_name = 'apple'
        self.place_test_object_type = 'fruit'

        self.prior_categories = ['fruit', 'food', 'snack']
        # self.item_list = ['tomato_soup', 'plum', 'apple', 'spam', 'mustard', 'knife', 'spoon', 'tennis_ball']
        self.item_list = None

        # !!! Measured Distances !!!
        self.dist_to_table = 0.85
        self.dist_to_shelf = 1.22
        self.place_dist = 0.07
        self.new_category_dist = (self.shelf_width * 0.9 - self.place_dist * 2) / 2

        # !!! Hard-Coded Offsets !!!
        self.pick_front_bias = [0.03, 0.00, -0.03] # [x, y, height]
        self.pick_top_bias = [0.03, 0, -0.015]
        self.pick_bowl_bias = [0.15, 0.04, -0.13]
        self.place_x_bias = [None, -0.20, -0.10, 0.0]

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
        self.shelf_height = [ # [NULL, 1F height, 2F height, 3F height, ...]
            0,
            self.agent.table_dimension['grocery_shelf_1f'][2],
            self.agent.table_dimension['grocery_shelf_2f'][2],
            self.agent.table_dimension['grocery_shelf_3f'][2],
        ]
        self.shelf_head_angle = np.arctan(
            ((self.shelf_height[0] + self.shelf_height[1]) / 2 - 1.1) / self.dist_to_shelf # 1.1: HSR height
        )
        self.shelf_item_dict = {}

        # [BONUS] shelf_open
        self.shelf_open = False
        self.shelf_arm_height = 0.59

        self.grasp_failure_count = np.zeros(1000, dtype=int)  # 1000 is an arbitraty large number


    def detect_shelf_open_point(self):

        # params for scanning shelf

        y_pixel_range = [150, 450] # show width pixel between 100~450 only.
        z_range = [self.shelf_height[1] - 0.05, self.shelf_height[1] + 0.05]
        cv2.imwrite('shelf.jpg', self.agent.rgb_img[:, y_pixel_range[0]:y_pixel_range[1]])
        ###########################
        try:
            _pc = self.agent.yolo_module.pc.reshape(480, 640)
            pc_np = np.array(_pc.tolist())[:, :, :3]
            # 1. filtering y. using 2D image pixel
            pc_y_filtered = pc_np[:, y_pixel_range[0]:y_pixel_range[1], :]
            pc_y_filtered = pc_y_filtered.reshape(-1, 3)

            base_pc = self.agent.axis_transform.tf_camera_to_base(pc_y_filtered, multi_dimention=True)
            # 2. filtering z(height). using z_range
            z_filtered_pc = base_pc[np.where((z_range[0] <= base_pc[:, 2])
                                        & (z_range[1] >= base_pc[:, 2]))]
            min_x_dist = np.nanmin(z_filtered_pc[:, 0])
            print('min_x_dist', min_x_dist)
            # 3. filtering x(front). min_x +0.1 ~ min_x + 0.5 == background of shelf
            xz_filtered_pc = z_filtered_pc[np.where((min_x_dist+0.1 <= z_filtered_pc[:, 0])
                                            & (min_x_dist+0.5 >= z_filtered_pc[:, 0]))]
            min_y_dist = np.nanmax(xz_filtered_pc[:, 1])
            print('min_y_dist', min_y_dist)
            print('xyz', (min_x_dist, min_y_dist, self.shelf_height[1]))
        except:
            return None

        return (min_x_dist, min_y_dist)


    def open_shelf(self):

        # self.agent.move_rel(0.2, 0)
        self.agent.grasp()
        base_axis = self.detect_shelf_open_point(self.shelf_height[1])

        # HSR is able to open the shelf 
        if base_axis is not None:
            self.agent.move_rel(0, base_axis[1] - 0.25, wait=True)
            self.agent.pose.pick_side_pose_by_height(height=self.shelf_arm_geight)
            self.agent.move_rel(base_axis[0] - 0.53, 0, wait=True)            # tuning
            self.agent.pose.open_shelf_pose1_by_height(height=self.shelf_arm_geight)
            self.agent.pose.open_shelf_pose2_by_height(height=self.shelf_arm_geight)

            self.agent.move_rel(-0.1, 0, wait=True)
            self.agent.move_rel(-0.1, 0.23, wait=True)
        # HSR is not able to open the shelf 
        else:
            self.agent.move_rel(-0.1, 0, wait=True)
            self.agent.say('please open the shelf.')
            rospy.sleep(2)
            self.agent.say('five');  rospy.sleep(1)
            self.agent.say('four');  rospy.sleep(1)
            self.agent.say('three'); rospy.sleep(1)
            self.agent.say('two');   rospy.sleep(1)
            self.agent.say('one');   rospy.sleep(1)

        # move back to searcing position
        self.agent.move_abs_safe(self.place_location)
        self.agent.pose.table_search_pose(head_tilt=self.shelf_head_angle)


    def search_shelf(self):

        self.agent.say('Examining items in shelf.')
        center_list = np.zeros(0)
        table_search_attempts = 0
        while center_list.size == 0:
            rospy.sleep(0.2) # give some time for the YOLO to update
            center_list = np.array(self.agent.yolo_module.detect_3d_safe('grocery_table', dist=self.dist_to_table, depth=self.table_depth, item_list=self.item_list))
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
        object_cnts_by_floor = [0 for i in range(len(self.shelf_height)+1)]
        object_cnts_by_floor[0] = 0 # [NULL, 1F, 2F, 3F, ...]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_name = self.agent.yolo_module.find_name_by_id(shelf_item_class_id)
            shelf_item_type = self.agent.object_types[self.agent.yolo_module.find_type_by_id(shelf_item_class_id)]
            
            shelf_item_floor = 0
            for i in range(len(self.shelf_height)):
                if shelf_item_cent_z < self.shelf_height[i]:
                    shelf_item_floor = i
                    break
            if shelf_item_floor == 0:
                shelf_item_floor = len(self.shelf_height)

            object_cnts_by_floor[shelf_item_floor] += 1
            self.shelf_item_dict[shelf_item_type] = {
                'name': shelf_item_name,
                'center': [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z],
                'floor': shelf_item_floor
            }
            rospy.loginfo(f"[{shelf_item_type}] {shelf_item_name} is detected at {shelf_item_floor}F")
            rospy.loginfo(f"Item center: {shelf_item_cent_x}, {shelf_item_cent_y}, {shelf_item_cent_z}")

        # 2. add new category in shelf_item_dict
        new_category_floor = np.argmin(np.array(object_cnts_by_floor))
        for item in self.shelf_item_dict.values():
            if item['floor'] == new_category_floor:
                shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z = item['center']
                shelf_item_cent_y = shelf_item_cent_y - self.new_category_dist if shelf_item_cent_y > 0 else shelf_item_cent_y + self.new_category_dist
                self.shelf_item_dict['new'] = {
                    'name': '-',
                    'center': [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z],
                    'floor': new_category_floor,
                }
                break

        print(f'shelf_item_dict: {self.shelf_item_dict}')


    def search_item(self):
        table_item_list = np.zeros(0)
        table_search_attempts = 0
        while table_item_list.size == 0:
            rospy.sleep(0.2) # give some time for the YOLO to update
            table_item_list = np.array(self.agent.yolo_module.detect_3d_safe(
                table='grocery_table',
                dist=self.dist_to_table,
                depth=self.table_depth,
                item_list=self.item_list
            ))
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
            table_item_type = self.agent.object_types[self.agent.yolo_module.find_type_by_id(table_item_id)]
            grasping_type = self.agent.yolo_module.find_grasping_type_by_id(table_item_id)

            # Only grasp items of available categories that have failed less than 3 times
            if self.grasp_failure_count[table_item_id] <= 2 and\
                table_item_type in self.prior_categories:
                break

        rospy.loginfo(f"Table item :      {table_item_name}")
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
            self.agent.say(f'Placing a new category object, {item_name}, which is a {item_type}, on shelf floor {item_floor}...')
        else:  # for known category objects
            item_floor = self.shelf_item_dict[item_type]['floor']
            rospy.loginfo(f'Known category : {item_type} (Current item: {item_name})')
            self.agent.say(f'Placing {item_name}, which is a {item_type}, next to {self.shelf_item_dict[item_type]["name"]}...')

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

        if not self.ignore_door:
            self.agent.door_open()

        rospy.logwarn('Start Storing Groceries')
        self.agent.say('Start Storing Groceries')

        self.agent.pose.table_search_pose(head_tilt=self.table_head_angle, wait_gripper=False)
        
        if self.place_test_mode:
            self.agent.move_abs_safe(self.place_location)
            self.search_shelf()
            
            self.agent.pose.pick_side_pose_by_height(height=1.0)
            self.agent.open_gripper(wait=False)

            self.agent.say('Please hand me an item.')
            rospy.sleep(3)
            self.agent.say('three'); rospy.sleep(1)
            self.agent.say('two');   rospy.sleep(1)
            self.agent.say('one');   rospy.sleep(1)
            self.agent.grasp(wait=True)

            # place
            self.place_item(self.place_test_object_name, self.place_test_object_type)
            return

        has_searched_shelf = False
        
        # Pick & place loop
        while True:

            has_grasped = False

            ## 2. Try picking until an item is grasped
            while not has_grasped:
                # 2-1. Go to pick_location
                rospy.logwarn('Go to pick_location...')
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
                self.agent.pose.table_search_pose(head_tilt=self.shelf_head_angle, wait_gripper=False)

                # 2-4. Check if grasping is successful
                has_grasped = self.check_grasp(grasping_type)
                if has_grasped:
                    rospy.loginfo(f'Successfuly grasped {table_item_name}!')
                    self.agent.say(f'Grasped a {table_item_name}, which is a {table_item_type}!')
                else:
                    rospy.logwarn(f'Failed to grasp {table_item_name}! Retrying...')
                    self.grasp_failure_count[table_item_id] += 1

            if self.picking_test_mode:
                self.agent.open_gripper()
                continue

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')
            self.agent.move_abs_safe(self.place_location)

            # Search items in shelf only once
            if not has_searched_shelf:
                rospy.logwarn('Searching items in shelf...')
                self.search_shelf()
                has_searched_shelf = True

            ## 4. Place item
            rospy.logwarn('Placing item...')
            self.place_item(table_item_name, table_item_type)