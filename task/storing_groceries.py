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

        self.prior_categories = ['fruit', 'food', 'snack']
        self.ignored_items = ['plate']


        # !!! Hard coding offset !!!
        self.pick_front_bias = [0.04, 0, 0.915] # [x, y, height_ratio]
        self.pick_top_bias = [-0.01, 0, -0.015]
        self.pick_bowl_bias = [0.15, 0.04, 0]
        self.place_x_bias = [None, -0.20, -0.15, 0.0]
        self.gripper_to_shelf_dist = 0.9
        self.dist_to_table = 0.85
        self.dist_to_shelf = 1.22
        self.default_place_dist = 0.07
        self.new_place_dist = 0.25

        self.pick_table = 'grocery_table'
        self.table_height = self.agent.table_dimension['grocery_table'][2]
        self.table_depth = self.agent.table_dimension['grocery_table'][1]

        self.place_location = 'grocery_shelf'
        self.place_shelf = 'grocery_shelf_1f'
        self.shelf_depth = self.agent.table_dimension['grocery_shelf_1f'][1]
        self.shelf_width = self.agent.table_dimension['grocery_shelf_1f'][0]
        self.shelf_0_1_height_threshold = self.agent.table_dimension['grocery_shelf_1f'][2]
        self.shelf_1_2_height_threshold = self.agent.table_dimension['grocery_shelf_2f'][2]
        self.shelf_2_3_height_threshold = self.agent.table_dimension['grocery_shelf_3f'][2]
        self.shelf_head_angle = np.arctan(
            ((self.shelf_0_1_height_threshold + self.shelf_1_2_height_threshold) / 2 - 1.1) / self.dist_to_shelf # 1.0: HSR height
        )


        # [BONUS] shelf_open
        self.shelf_open = False
        self.shelf_arm_height = 0.59

        self.grasp_failure_count = np.zeros(1000, dtype=int)  # 1000 is an arbitraty large number


    def detect_shelf_open_point(self):

        # params for scanning shelf

        y_pixel_range = [150, 450] # show width pixel between 100~450 only.
        z_range = [self.shelf_1_2_height_threshold - 0.05, self.shelf_1_2_height_threshold + 0.05]
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
            print('xyz', (min_x_dist, min_y_dist, self.shelf_1_2_height_threshold))
        except:
            return None

        return (min_x_dist, min_y_dist)


    def open_shelf(self):

        # self.agent.move_rel(0.2, 0)
        self.agent.grasp()
        base_axis = self.detect_shelf_open_point(self.shelf_1_2_height_threshold)

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
            center_list = np.array(self.agent.yolo_module.detect_3d_safe('grocery_table', dist=self.dist_to_table, depth=self.table_depth))
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
        shelf_item_dict = {}

        # 1. make shelf_item_dict
        object_cnts_by_floor = [1000, 0, 0] # [NULL, 1F, 2F, 3F]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_name = self.agent.yolo_module.find_name_by_id(shelf_item_class_id)
            shelf_item_type = self.agent.object_types[self.agent.yolo_module.find_type_by_id(shelf_item_class_id)]
            # 1F
            if shelf_item_cent_z < self.shelf_1_2_height_threshold:
                shelf_item_floor = 1
            # 2F
            elif shelf_item_cent_z < self.shelf_2_3_height_threshold:
                shelf_item_floor = 2
            # # 3F
            # else:
            #     shelf_item_floor = 3

            object_cnts_by_floor[shelf_item_floor] += 1
            shelf_item_dict[shelf_item_type] = {
                'name': shelf_item_name,
                'center': [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z],
                'floor': shelf_item_floor
            }
            rospy.loginfo(f"[{shelf_item_type}] {shelf_item_name} is detected at {shelf_item_floor}F")
            rospy.loginfo(f"Item center: {shelf_item_cent_x}, {shelf_item_cent_y}, {shelf_item_cent_z}")

        # 2. add new category in shelf_item_dict
        new_category_floor = np.argmin(np.array(object_cnts_by_floor))

        shelf_item_dict['new'] = {
            'name': shelf_item_name,
            'center': [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z],
            'floor': new_category_floor,
        }

        print(f'shelf_item_dict: {shelf_item_dict}')

        return shelf_item_dict


    def pick_item(self, grasping_type, item_name, table_base_xyz):

        # front
        if grasping_type == 0:
            self.agent.pose.pick_side_pose_by_height(height=self.table_height * self.pick_front_bias[2])
            self.agent.open_gripper()
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=True)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            # self.agent.grasp()
            # self.agent.pose.pick_side_pose_by_height(height=self.table_height + 0.6)

        # top
        elif grasping_type == 1:
            self.agent.pose.pick_top_pose_by_height(height=self.table_height + self.pick_top_bias[2])
            self.agent.open_gripper()
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_top_bias)]
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.grasp()
            self.agent.pose.arm_flex(-60)
            self.agent.move_rel(-0.17, 0)

        # bowl. # don't care plate, ...
        elif item_name == 'bowl':
            self.agent.pose.bring_bowl_pose(table=self.pick_table)
            self.agent.open_gripper()

            self.agent.move_rel(table_base_xyz[0] + 0.15, table_base_xyz[1] + 0.04, wait=True)

            self.agent.pose.pick_bowl_max_pose(table=self.pick_table, height=-0.13) # modified from -0.1 to -0.13
            self.agent.grasp()
            rospy.sleep(0.5)

            self.agent.pose.pick_up_bowl_pose(table=self.pick_table)
            self.agent.move_rel(-0.2, 0)


    def place_item(self, shelf_item_dict, table_item_name, table_item_type):

        # Set offset dist from object in shelf
        if table_item_type not in shelf_item_dict.keys():  # for new category
            item_type = 'new'
            place_object_offset_dist = self.new_place_dist
            print(f'New category : {item_type} (Current item: {table_item_name})')
        else:
            item_type = table_item_type
            place_object_offset_dist = self.default_place_dist
            print(f'Known category : {item_type} (Current item: {table_item_name})')

        print(f'Type of item to place: {item_type}')
        print(f'shelf_item_dict: {shelf_item_dict[item_type]}')

        # Get axis of the shelf the item belongs
        try:
            shelf_item_cent_y, shelf_item_cent_z = shelf_item_dict[item_type]['center'][1], shelf_item_dict[item_type]['center'][2]
        except:
            rospy.logerr('Failed to navigate shelf...')
            self.agent.open_gripper()
            self.agent.move_rel(-0.4, 0, wait=True)
            self.agent.pose.table_search_pose()
            return
        
        # Place item in shelf
        item_floor = shelf_item_dict[item_type]['floor']
        self.agent.pose.place_shelf_pose(f'grocery_shelf_{item_floor}f')
        if shelf_item_cent_y > 0:   # 3d horizontal cent point.
            shelf_base_to_object_xyz = [self.dist_to_shelf + self.place_x_bias[item_floor], shelf_item_cent_y - place_object_offset_dist, 0]
        else:
            shelf_base_to_object_xyz = [self.dist_to_shelf + self.place_x_bias[item_floor], shelf_item_cent_y + place_object_offset_dist, 0]
        shelf_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(shelf_base_to_object_xyz, 4)
        print(f'shelf_base_xyz: {shelf_base_xyz}')
        self.agent.move_rel(shelf_base_xyz[0], shelf_base_xyz[1], wait=True)
        self.agent.open_gripper()
        self.agent.move_rel(-0.4, 0, wait=True)
        self.agent.pose.table_search_pose()


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
            self.agent.move_rel(2.0, 0, wait=True)
        else:
            rospy.logwarn('Start Storing Groceries')
            self.agent.say('Start Storing Groceries')
            
        self.agent.pose.table_search_pose(head_tilt=self.shelf_head_angle)
        
        if not self.picking_test_mode:
            self.agent.move_abs_safe(self.place_location, thresh=0.05, angle=30)
            self.agent.pose.head_tilt(self.shelf_head_angle)
            rospy.sleep(1)
            
            ## 0. open shelf if closed
            if self.shelf_open:
                self.open_shelf()
                rospy.sleep(1)

            ## 1. search objects in shelf
            shelf_item_dict = self.search_shelf()

            if self.place_test_mode:
                dist_to_table = distancing(self.agent.yolo_module.pc, self.pick_table, dist=self.gripper_to_shelf_dist)
                # shelf_y = distancing_horizontal(self.agent.yolo_module.pc, place_table)
                self.agent.move_rel(dist_to_table, 0)

                # place
                table_item_name = 'orange'
                table_item_type = 'fruit'
                self.place_item(shelf_item_dict, table_item_name, table_item_type)
                return

        # Pick & place loop
        while True:

            has_grasped = False

            ## 2. Try picking until an item is grasped
            while not has_grasped:
                # 2-1. Go to pick_location
                rospy.logwarn('Go to pick_location...')
                self.agent.pose.table_search_pose()
                self.agent.move_abs_safe(self.pick_table)

                # 2-2. Select item to pick
                rospy.logwarn('Searching for item to pick...')
                self.agent.say('Searching for item to pick.')

                table_item_list = np.zeros(0)
                table_search_attempts = 0
                while table_item_list.size == 0:
                    rospy.sleep(0.2) # give some time for the YOLO to update
                    table_item_list = np.array(self.agent.yolo_module.detect_3d_safe('grocery_table', dist=self.dist_to_table, depth=self.table_depth))
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
                        table_item_type in self.prior_categories and\
                        table_item_name not in self.ignored_items:
                        break

                # 2-3. Find item name & type
                rospy.loginfo(f"Table item :      {table_item_name}")
                rospy.loginfo(f"Table item type : {table_item_type}")
                rospy.loginfo(f"Grasping type :   {grasping_type}")
                rospy.loginfo(f"grasp_failure_count: {self.grasp_failure_count[table_item_id]}")

                try:    
                    table_base_to_object_xyz = self.agent.yolo_module.find_3d_points_by_name(table_item_list, table_item_name)
                    table_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)  # inclined grasping type
                    rospy.loginfo(f"Base to object:   {table_base_to_object_xyz}")
                    rospy.loginfo(f"Table base :      {table_base_xyz}")
                except Exception as e:
                    rospy.logerr(f'[ERROR] table_base_to_object_xyz: {table_base_to_object_xyz}\n{e}')
                    continue

                # 2-4. Pick item
                rospy.logwarn('Picking item...')
                self.pick_item(grasping_type, table_item_name, table_base_xyz)
                self.agent.pose.table_search_pose()

                # 2-5. Check if grasping is successful
                has_grasped = self.check_grasp(grasping_type)
                if has_grasped:
                    rospy.loginfo(f'Successfuly grasped {table_item_name}!')
                    self.agent.say(f'Grasped a {table_item_name}, which is a {table_item_type}!')
                else:
                    rospy.logwarn(f'Failed to grasp {table_item_name}! Retrying...')
                    self.grasp_failure_count[table_item_id] += 1

            # Don't go to shelf if picking_test_mode
            if self.picking_test_mode:
                self.agent.open_gripper()
                continue

            ## 3. Go to place_location
            rospy.logwarn('Going to place_location...')

            self.agent.move_abs_safe(self.place_location)
            # shelf_y = distancing_horizontal(self.agent.yolo_module.pc, place_table)

            ## 4. Place item
            rospy.logwarn('Placing item...')
            self.place_item(shelf_item_dict, table_item_name, table_item_type)