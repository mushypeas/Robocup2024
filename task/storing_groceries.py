import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
import cv2
from utils.distancing import distancing, distancing_horizontal
from hsr_agent.agent import Agent

class StoringGroceries:

    def __init__(self, agent: Agent):
        self.agent = agent

        # test params
        # Set everything to False for actual task
        self.ignore_door = True
        self.picking_test_mode = False
        self.place_test_mode = False
        self.available_categories = ['fruit', 'food']
        self.pick_table = 'grocery_table'
        self.pick_location = 'grocery_table'
        self.place_table = 'shelf_1f'
        self.place_location = 'shelf_front'

        self.shelf_1_2_height_threshold = 1.15
        self.shelf_2_3_height_threshold = 1.5
        self.shelf_head_angle = 15

        # hard coding offset
        self.pick_front_bias = [0, 0.01, 0]
        self.pick_top_bias = [0.25, -0.01, 0]
        self.gripper_to_shelf_x = 0.9
        self.default_offset_dist = 0.05
        self.new_category_offset_dist = 0.3
        self.dist_to_grocery_table = 1

        # [BONUS] shelf_open
        self.shelf_open = False
        self.shelf_arm_height = 0.59

        self.not_using_check_grasp_list = ['milk', 'bubble_tea']  # items that can't be detected by check_grasp
        self.grasp_failure_count = np.zeros(1000)


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
        self.agent.pose.move_pose()
        self.agent.pose.head_tilt(self.shelf_head_angle)
        rospy.sleep(1)
        self.agent.move_rel(0.2, 0, wait=True)


    def search_shelf(self):
        dist_to_shelf = distancing(self.agent.yolo_module.pc, self.place_table, dist=self.gripper_to_shelf_x)
        center_list = self.agent.yolo_module.detect_3d('shelf_1f', dist=dist_to_shelf)
        print(f"Shelf item center list:\n{center_list}")
        shelf_item_dict = {}

        # 1. make shelf_item_dict
        object_cnts_by_floor = [0, 0, 0] # [1F, 2F, 3F]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_type = self.agent.yolo_module.find_type_by_id(shelf_item_class_id)
            shelf_item_dict[shelf_item_type] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            print("0-1. shelf item & type: ", self.agent.yolo_module.find_name_by_id(shelf_item_class_id),
                    self.agent.object_types[shelf_item_type], "\n")
            # 1F
            if shelf_item_cent_z < self.shelf_1_2_height_threshold:
                object_cnts_by_floor[0] += 1
            # 2F
            elif shelf_item_cent_z < self.shelf_2_3_height_threshold:
                object_cnts_by_floor[1] += 1
            # 3F
            else:
                object_cnts_by_floor[2] += 1

        # 2. add new category in shelf_item_dict
        # new category_id = -1
        new_category_floor = np.argmin(np.array(object_cnts_by_floor))
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            # 1F
            if new_category_floor == 0:
                if shelf_item_cent_z < self.shelf_1_2_height_threshold:
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            # 2F
            elif new_category_floor == 1:
                if shelf_item_cent_z < self.shelf_2_3_height_threshold:
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            # 3F
            else:
                if shelf_item_cent_z > self.shelf_2_3_height_threshold:
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]

        print(f'shelf_item_dict: {shelf_item_dict}')

        return shelf_item_dict


    def pick_item(self, grasping_type, table_base_xyz):

        # front
        if grasping_type == 0:
            self.agent.pose.pick_side_pose('grocery_table_pose2')
            self.agent.open_gripper()
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(0, table_base_xyz[1], wait=True)
            self.agent.move_rel(table_base_xyz[0] + 0.05, 0, wait=True)
            self.agent.grasp()
            self.agent.pose.pick_side_pose('grocery_table_pose1')

        # top
        elif grasping_type == 1:
            self.agent.pose.pick_top_pose(table='grocery_table_pose')
            self.agent.open_gripper()
            table_base_xyz = [axis + bias for axis, bias in zip(table_base_xyz, self.pick_front_bias)]
            self.agent.move_rel(table_base_xyz[0], table_base_xyz[1], wait=True)
            self.agent.pose.arm_lift_top_table_down(height=-0.015, table='grocery_table_pose') # modify
            self.agent.grasp()
            self.agent.pose.arm_flex(-60)
            self.agent.move_rel(-0.17, 0)

            # self.agent.pose.pick_top_pose(table='grocery_table_pose')
            # self.agent.open_gripper()
            # self.agent.move_rel(0, table_base_xyz[1], wait=True)
            # self.agent.move_rel(0.05, 0, wait=True)
            # self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            # self.agent.pose.arm_lift_top_table_down(height=-0.017, table='grocery_table_pose')  # -0.015
            # self.agent.grasp()
            # rospy.sleep(0.5)
            # self.agent.pose.arm_lift_top_table_down(height=0.1, table='grocery_table_pose')

        # bowl. # don't care plate, ...
        else:
            self.agent.pose.pick_bowl_pose(table='grocery_table_pose')
            self.agent.open_gripper()
            self.agent.move_rel(0, table_base_xyz[1], wait=True)
            self.agent.move_rel(table_base_xyz[0], 0, wait=True)
            self.agent.pose.arm_lift_top_table_down(height=0.01, table='grocery_table_pose')
            self.agent.grasp()
            self.agent.pose.arm_lift_top_table_down(height=0.1, table='grocery_table_pose')


    def place_item(self, shelf_item_dict, table_item_name, table_item_type):

        item_type = self.agent.yolo_module.find_type_by_id(self.agent.yolo_module.find_id_by_name(table_item_name))

        # Offset dist between objects
        if item_type not in shelf_item_dict.keys():  # for new category
            item_type = -1
            place_object_offset_dist = self.new_category_offset_dist  # 0.3
            print(f'New category : {self.agent.object_types[table_item_type]} (Current item: {table_item_name})')
        else:
            place_object_offset_dist = self.default_offset_dist  # 0.05
            print(f'Known category : {self.agent.object_types[table_item_type]} (Current item: {table_item_name})')

        print(f'Type of item to place: {table_item_name}')
        print(f'shelf_item_dict: {shelf_item_dict}')

        # Get axis of the shelf the item belongs
        try:
            shelf_item_cent_y, shelf_item_cent_z = shelf_item_dict[item_type][1], shelf_item_dict[item_type][2]
        except:
            print('[ERROR] Failed to navigate shelf...')
            rospy.sleep(1)
            self.agent.pose.place_shelf_pose('shelf_2f')
            shelf_base_xyz = [0.6, -0.1]
            self.agent.move_rel(shelf_base_xyz[0] - 0.08 + 0.33, shelf_base_xyz[1], wait=True)
            self.agent.open_gripper()
            self.agent.move_rel(-0.4, 0)
            self.agent.pose.move_pose()
            self.agent.grasp()
            return
        
        if shelf_item_cent_z > self.shelf_2_3_height_threshold:
            self.agent.pose.place_shelf_pose('shelf_3f')
        elif shelf_item_cent_z > self.shelf_1_2_height_threshold:
            self.agent.pose.place_shelf_pose('shelf_2f')
        else:
            self.agent.pose.place_side_pose('shelf_1f')

        if shelf_item_cent_y > 0:   # 3d horizontal cent point.
            shelf_base_to_object_xyz = [self.gripper_to_shelf_x, shelf_item_cent_y - place_object_offset_dist, 0]
        else:
            shelf_base_to_object_xyz = [self.gripper_to_shelf_x, shelf_item_cent_y + place_object_offset_dist, 0]
        print(f'shelf_base_to_object_xyz: {shelf_base_to_object_xyz}')
        shelf_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(shelf_base_to_object_xyz, 4)
        print(f'shelf_base_xyz: {shelf_base_xyz}')

        # Place item at 1F
        if shelf_item_cent_z < self.shelf_1_2_height_threshold:
            self.agent.move_rel(shelf_base_xyz[0] - 0.15, shelf_base_xyz[1], wait=True)  # 2f pose - 1f pose = 0.18
        # Place item at 2F of 3F
        else:
            self.agent.move_rel(shelf_base_xyz[0] + 0.05, shelf_base_xyz[1], wait=True)
        self.agent.open_gripper()
        self.agent.move_rel(-0.4, 0)
        # self.agent.grasp()
        self.agent.pose.move_pose()


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
            self.agent.move_rel(1.0, 0, wait=True)
            self.agent.move_rel(1.0, 0, wait=True)
        else:
            rospy.logwarn('Start Storing Groceries')
            self.agent.say('Start Storing Groceries')
            
        if not self.picking_test_mode:
            self.agent.move_abs_safe(self.place_location)
            self.agent.pose.move_pose()

            self.agent.move_abs_safe(self.place_location, thresh=0.05, angle=30)
            self.agent.pose.head_tilt(self.shelf_head_angle)
            rospy.sleep(1)
            
            # shelf_y = distancing_horizontal(self.agent.yolo_module.pc, place_table)
            # self.agent.move_rel(0, shelf_y - 0.08, wait=True)

            ## 0. open shelf if closed
            if self.shelf_open:
                self.open_shelf()
                rospy.sleep(1)

            ## 1. search objects in shelf
            shelf_item_dict = self.search_shelf()

            if self.place_test_mode:
                dist_to_table = distancing(self.agent.yolo_module.pc, self.pick_table, dist=self.gripper_to_shelf_x)
                # shelf_y = distancing_horizontal(self.agent.yolo_module.pc, place_table)
                self.agent.move_rel(dist_to_table, 0)

                # place
                table_item_name = 'orange'
                table_item_type = 3
                self.place_item(shelf_item_dict, table_item_name, table_item_type)
                return

        # Pick & place loop
        while True:

            has_grasped = False

            ## 2. Try picking until an item is grasped
            while not has_grasped:
                # 2-1. Go to pick_location
                rospy.logwarn('Go to pick_location...')
                rospy.sleep(0.5)
                self.agent.move_abs_safe(self.pick_location)
                dist_to_table = distancing(self.agent.yolo_module.pc, self.pick_table, dist=0.9)
                self.agent.move_rel(dist_to_table, 0)
                self.agent.pose.table_search_pose()

                # 2-2. Select item to pick
                rospy.logwarn('Searching for item to pick...')
                rospy.sleep(0.5)
                start_table_item_list = np.array(self.agent.yolo_module.detect_3d('grocery_table', dist=self.dist_to_grocery_table))
                if start_table_item_list.size == 0:    # if array is empty(nothing detected)
                    continue
                # # >>> choose object without exception
                # table_item_list_sorted = start_table_item_list[start_table_item_list[:, 0].argsort()]
                # table_item_id = [int(i) for i in table_item_list_sorted[:, 3]][0]

                # # 2-3. Find item name & type
                # table_item_name = self.agent.yolo_module.find_name_by_id(table_item_id)
                # table_item_type = self.agent.yolo_module.find_type_by_id(table_item_id)
                # grasping_type = self.agent.yolo_module.find_grasping_type_by_id(table_item_id)
                # # <<< choose object without exception

                # >>> choose object with exception
                table_item_list_sorted = start_table_item_list[start_table_item_list[:, 0].argsort()]
                table_item_list_sorted = [int(i) for i in table_item_list_sorted[:, 3]]

                # 2-3. Find item name & type
                # only grasp items of available categories that have failed less than 3 times
                for table_item_id in table_item_list_sorted:
                    table_item_name = self.agent.yolo_module.find_name_by_id(table_item_id)
                    table_item_type = self.agent.yolo_module.find_type_by_id(table_item_id)
                    grasping_type = self.agent.yolo_module.find_grasping_type_by_id(table_item_id)
                    if self.grasp_failure_count[table_item_id] > 2 or\
                        self.agent.object_types[table_item_type] not in self.available_categories:
                        continue
                # <<< choose object with exception

                # 2-3. Find item name & type
                print(f"Table item :      {table_item_name}")
                print(f"Table item type : {self.agent.object_types[table_item_type]}")

                try:
                    table_item_list = self.agent.yolo_module.detect_3d('grocery_table', dist=self.dist_to_grocery_table)
                    table_base_to_object_xyz = self.agent.yolo_module.find_3d_points_by_name(table_item_list, table_item_name)
                    table_base_xyz = self.agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)  # inclined grasping type
                    print(f"Table base :      {table_base_xyz}")
                except Exception as e:
                    print(f'[ERROR] table_base_to_object_xyz: {table_base_to_object_xyz}\n{e}')
                    continue

                # 2-4. Pick item
                rospy.logwarn('Picking item...')
                print(f'grasping_type: {grasping_type}')
                self.pick_item(grasping_type, table_base_xyz)

                self.agent.move_rel(-0.2, 0)
                self.agent.pose.move_pose()

                # 2-5. Check if grasping is successful
                has_grasped = self.check_grasp(grasping_type)
                if has_grasped:
                    rospy.loginfo(f'Successfuly grasped {table_item_name}!')
                    self.agent.say(f'Grasped a {table_item_name}, which is a {self.agent.object_types[table_item_type]}!')
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
            dist_to_table = distancing(self.agent.yolo_module.pc, self.place_table, dist=self.gripper_to_shelf_x)
            # shelf_y = distancing_horizontal(self.agent.yolo_module.pc, place_table)
            self.agent.move_rel(dist_to_table, 0)

            ## 4. Place item
            rospy.logwarn('Placing item...')
            self.place_item(shelf_item_dict, table_item_name, table_item_type)