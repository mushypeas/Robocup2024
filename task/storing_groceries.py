import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest
import cv2
from utils.distancing import distancing, distancing_horizontal

def detect_shelf_open_point(agent, shelf_1_2_height_threshold):

    # params for scanning shelf

    y_pixel_range = [150, 450] # show width pixel between 100~450 only.
    z_range = [shelf_1_2_height_threshold - 0.05, shelf_1_2_height_threshold + 0.05]
    cv2.imwrite('shelf.jpg', agent.rgb_img[:, y_pixel_range[0]:y_pixel_range[1]])
    ###########################
    try:
        _pc = agent.yolo_module.pc.reshape(480, 640)
        pc_np = np.array(_pc.tolist())[:, :, :3]
        # 1. filtering y. using 2D image pixel
        pc_y_filtered = pc_np[:, y_pixel_range[0]:y_pixel_range[1], :]
        pc_y_filtered = pc_y_filtered.reshape(-1, 3)

        base_pc = agent.axis_transform.tf_camera_to_base(pc_y_filtered, multi_dimention=True)
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
        print('xyz', (min_x_dist, min_y_dist, shelf_1_2_height_threshold))
    except:
        return None

    return (min_x_dist, min_y_dist, shelf_1_2_height_threshold)


def storing_groceries(agent):
    ### task params #################
    pick_table = 'grocery_table'
    place_table = 'shelf_1f'
    place_location = 'shelf_front'
    pick_location = 'grocery_table'


    shelf_1_2_height_threshold = 0.65
    shelf_2_3_height_threshold = 1.0
    # hard coding offset
    gripper_to_shelf_x = 0.9
    default_offset_dist = 0.05
    new_category_offset_dist = 0.3
    dist_to_grocery_table = 0.75

    # [BONUS] shelf_open
    shelf_open = False
    arm_lift_height_shelf_open = 0.59

    # safe_flag = 0
    not_using_check_grasp_list = ['milk', 'bubble_tea']  # items that can't be detected by check_grasp
    ##################################


    stop_client = rospy.ServiceProxy('/viewpoint_controller/stop', Empty)
    stop_client.call(EmptyRequest())

    agent.door_open()
    agent.say('start storing groceries')
    agent.move_rel(1.0, 0, wait=True)
    agent.move_rel(1.0, 0, wait=True)
    agent.move_rel(1.0, 0, wait=True)
    # agent.move_abs_safe(place_location)

    picking_test_mode = False
    place_test_mode = False
    agent.pose.move_pose()
    if place_test_mode:
        agent.open_gripper()
        agent.pose.head_tilt(-10)
        rospy.sleep(1)
        center_list = agent.yolo_module.detect_3d('shelf_1f')
        print("0. Shelf item center list\n", center_list)
        shelf_item_dict = {}
        # 0-1. make shelf_item_dict
        object_cnts_by_floor = [0, 0, 0]  # [1F, 2F, 3F]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_type = agent.yolo_module.find_type_by_id(shelf_item_class_id)
            shelf_item_dict[shelf_item_type] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            print("0-1. shelf item & type: ", agent.yolo_module.find_name_by_id(shelf_item_class_id),
                  agent.object_types[shelf_item_type], "\n")
            if shelf_item_cent_z < shelf_1_2_height_threshold:  # 1F
                object_cnts_by_floor[0] += 1
            elif shelf_item_cent_z < shelf_2_3_height_threshold:  # 2F
                object_cnts_by_floor[1] += 1
            else:  # 3F
                object_cnts_by_floor[2] += 1

        # 0-2. new category add in shelf_item_dict
        # new category_id = -1
        new_category_floor = np.argmin(np.array(object_cnts_by_floor))
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            if new_category_floor == 0:  # 1F
                if shelf_item_cent_z < shelf_1_2_height_threshold:
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            elif new_category_floor == 1:  # 2F
                if shelf_item_cent_z < shelf_2_3_height_threshold:  # 2F
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            else:  # 3F
                if shelf_item_cent_z > shelf_2_3_height_threshold:  # 3F
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
        print('shelf_item_dict', shelf_item_dict)
        dist_to_table = distancing(agent.yolo_module.pc, pick_table, dist=gripper_to_shelf_x)
        # shelf_y = distancing_horizontal(agent.yolo_module.pc, place_table)
        agent.move_rel(dist_to_table, 0)

        # 4. place
        table_item_name = 'orange'
        table_item_type = 3
        item_type = agent.yolo_module.find_type_by_id(agent.yolo_module.find_id_by_name(table_item_name))
        # 4-1. offset dist between objects
        if item_type not in shelf_item_dict.keys():  # for new category
            item_type = -1
            place_object_offset_dist = new_category_offset_dist  # 0.3
            print('4-1. New category :', table_item_name, agent.object_types[table_item_type])
        else:
            place_object_offset_dist = default_offset_dist  # 0.05
            print('4-1. known category :', table_item_name, agent.object_types[table_item_type])
        print('item_type', item_type)
        print('shelf_item_dict', shelf_item_dict)
        shelf_item_cent_y, shelf_item_cent_z = shelf_item_dict[item_type][1], shelf_item_dict[item_type][2]
        return


    if not picking_test_mode:

        agent.move_abs_safe('grocery_bypass', thresh=0.05, angle=30)
        agent.move_abs_safe(place_location, thresh=0.05, angle=30)
        agent.pose.head_tilt(-10)
        rospy.sleep(1)
        #
        # shelf_y = distancing_horizontal(agent.yolo_module.pc, place_table)
        # agent.move_rel(0, shelf_y - 0.08, wait=True)

        if shelf_open:
            # agent.move_rel(0.2, 0)
            agent.grasp()
            base_xyz = detect_shelf_open_point(agent, shelf_1_2_height_threshold)
            if base_xyz is None:
                agent.move_rel(-0.1, 0, wait=True)
                agent.say('please open the shelf.')
                rospy.sleep(2)
                agent.say('five'); rospy.sleep(1)
                agent.say('four'); rospy.sleep(1)
                agent.say('three'); rospy.sleep(1)
                agent.say('two'); rospy.sleep(1)
                agent.say('one'); rospy.sleep(1)
            else:
                agent.move_rel(0, base_xyz[1]-0.25, wait=True)
                agent.pose.pick_side_pose_by_height(height=arm_lift_height_shelf_open)
                agent.move_rel(base_xyz[0] - 0.53, 0, wait=True)            # tuning
                agent.pose.open_shelf_pose1_by_height(height=arm_lift_height_shelf_open)
                agent.pose.open_shelf_pose2_by_height(height=arm_lift_height_shelf_open)

                agent.move_rel(-0.1, 0, wait=True)
                agent.move_rel(-0.1, 0.23, wait=True)
            # 0. search objects in shelf
            agent.move_abs_safe(place_location)
            agent.pose.move_pose()
            agent.pose.head_tilt(-10)
            rospy.sleep(1)
            # shelf_y = distancing_horizontal(agent.yolo_module.pc, place_table)
            agent.move_rel(0.2, 0, wait=True)

        rospy.sleep(1)
        center_list = agent.yolo_module.detect_3d('shelf_1f')
        print("0. Shelf item center list\n", center_list)
        shelf_item_dict = {}
        # 0-1. make shelf_item_dict
        object_cnts_by_floor = [0, 0, 0] # [1F, 2F, 3F]
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            shelf_item_type = agent.yolo_module.find_type_by_id(shelf_item_class_id)
            shelf_item_dict[shelf_item_type] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            print("0-1. shelf item & type: ", agent.yolo_module.find_name_by_id(shelf_item_class_id),
                  agent.object_types[shelf_item_type], "\n")
            if shelf_item_cent_z < shelf_1_2_height_threshold: # 1F
                object_cnts_by_floor[0] += 1
            elif shelf_item_cent_z < shelf_2_3_height_threshold: # 2F
                object_cnts_by_floor[1] += 1
            else: # 3F
                object_cnts_by_floor[2] += 1

        # 0-2. new category add in shelf_item_dict
        # new category_id = -1
        new_category_floor = np.argmin(np.array(object_cnts_by_floor))
        for shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z, shelf_item_class_id in center_list:
            if new_category_floor == 0: # 1F
                if shelf_item_cent_z < shelf_1_2_height_threshold:
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            elif new_category_floor == 1:                       # 2F
                if shelf_item_cent_z < shelf_2_3_height_threshold: # 2F
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
            else:                       # 3F
                if shelf_item_cent_z > shelf_2_3_height_threshold: # 3F
                    shelf_item_dict[-1] = [shelf_item_cent_x, shelf_item_cent_y, shelf_item_cent_z]
        print('shelf_item_dict', shelf_item_dict)

    # 0-3. go pick_place
    while True:

        # 1. go pick_place
        rospy.sleep(1)
        agent.move_abs_safe(pick_location)
        agent.move_rel(0.9 - dist_to_grocery_table, 0)
        # dist_to_table = distancing(agent.yolo_module.pc, pick_table, dist=0.9)
        # agent.move_rel(dist_to_table, 0)
        agent.pose.table_search_pose()

        # select item to pick
        rospy.sleep(1)
        start_table_item_list = np.array(agent.yolo_module.detect_3d('grocery_table', dist=dist_to_grocery_table))
        if start_table_item_list.size == 0:    # if array is empty(nothing detected)
            continue
        table_item_list_sorted = start_table_item_list[start_table_item_list[:, 0].argsort()]
        table_item_id = [int(i) for i in table_item_list_sorted[:, 3]][0]

        # 2. find item name & type
        # if safe_flag == 0:
        table_item_name = agent.yolo_module.find_name_by_id(table_item_id)
        table_item_type = agent.yolo_module.find_type_by_id(table_item_id)
        grasping_type = agent.yolo_module.find_grasping_type_by_id(table_item_id)
        # else:
        #     table_item_name = agent.yolo_module.find_name_by_id(goal_item_id)
        #     table_item_type = agent.yolo_module.find_type_by_id(goal_item_id)
        #     grasping_type = agent.yolo_module.find_grasping_type_by_id(table_item_id)
        print("1. table item & type : ", table_item_name, agent.object_types[table_item_type], "\n")

        try:
            table_item_list = agent.yolo_module.detect_3d('grocery_table', dist=dist_to_grocery_table)
            table_base_to_object_xyz = agent.yolo_module.find_3d_points_by_name(table_item_list, table_item_name)
            table_base_xyz = agent.yolo_module.calculate_dist_to_pick(table_base_to_object_xyz, grasping_type)  # inclined grasping type
            print(table_item_name, table_base_xyz)
        except Exception as e:
            print('table_base_to_object_xyz', table_base_to_object_xyz)
            print('[error] line 165. ', e)
            continue

        # if safe_flag == 0:
        # agent.move_rel(0, table_base_xyz[1], wait=True)
            # safe_flag += 1
            # goal_item_id = table_item_id
            # continue

        # safe_flag = 0
        ### pick
        # agent.move_rel(-0.15, 0)
        print('grasping_type', grasping_type)
        if grasping_type == 0: # front
            agent.pose.pick_side_pose('grocery_table_pose1')
            agent.open_gripper()
            print("2. gripper opened")
            agent.move_rel(0, table_base_xyz[1], wait=True)
            print('table_base_xyz', table_base_xyz)
            agent.move_rel(table_base_xyz[0], 0, wait=True)
            agent.pose.pick_side_pose('grocery_table_pose2')
            agent.grasp()
            agent.pose.pick_side_pose('grocery_table_pose1')
        elif grasping_type == 1: # top
            agent.pose.pick_top_pose(table='grocery_table_pose')
            agent.open_gripper()
            agent.move_rel(0, table_base_xyz[1], wait=True)
            agent.move_rel(0.05, 0, wait=True)
            agent.move_rel(table_base_xyz[0], 0, wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.017, table='grocery_table_pose')  # -0.015
            agent.grasp()
            rospy.sleep(0.5)
            agent.pose.arm_lift_top_table_down(height=0.1, table='grocery_table_pose')
        else: # bowl. # don't care plate, ...
            agent.pose.pick_bowl_pose(table='grocery_table_pose')
            agent.open_gripper()
            agent.move_rel(0, table_base_xyz[1], wait=True)
            agent.move_rel(table_base_xyz[0], 0, wait=True)
            agent.pose.arm_lift_top_table_down(height=0.01, table='grocery_table_pose')
            agent.grasp()
            agent.pose.arm_lift_top_table_down(height=0.1, table='grocery_table_pose')


        agent.move_rel(-0.2, 0)
        agent.pose.move_pose()

        # 3. move for place
        ##### placing
        # 3.1
        # is_picked = agent.pose.check_grasp()  # modify this for tab
        # print(f"2.4 grasp value of item : {is_picked}")

        # if table_item_name not in not_using_check_grasp_list:   # if the object can be detected by check_grasp
        #     if is_picked:   # if grasping target object successed
        #         agent.say(f'I picked the {table_item_name}! hooray~')
        #     else: # if grasping target object failed -> continue
        #         agent.say(f'I didn\'t pick the {table_item_name}! boo hoo hoo')
        #         continue
        if picking_test_mode:
            agent.open_gripper()
            continue
        agent.move_abs_safe(place_location)

        dist_to_table = distancing(agent.yolo_module.pc, pick_table, dist=gripper_to_shelf_x)
        # shelf_y = distancing_horizontal(agent.yolo_module.pc, place_table)
        agent.move_rel(dist_to_table, 0)

        # 4. place
        item_type = agent.yolo_module.find_type_by_id(agent.yolo_module.find_id_by_name(table_item_name))
        # 4-1. offset dist between objects
        if item_type not in shelf_item_dict.keys(): # for new category
            item_type = -1
            place_object_offset_dist = new_category_offset_dist # 0.3
            print('4-1. New category :', table_item_name, agent.object_types[table_item_type])
        else:
            place_object_offset_dist = default_offset_dist  # 0.05
            print('4-1. known category :', table_item_name, agent.object_types[table_item_type])
        print('shelf_item_dict', shelf_item_dict)
        try:
            shelf_item_cent_y, shelf_item_cent_z = shelf_item_dict[item_type][1], shelf_item_dict[item_type][2]
        except:
            print('erro in 262')
            rospy.sleep(1)
            agent.pose.place_shelf_pose('shelf_2f')
            shelf_base_xyz = [0.6, -0.1]
            agent.move_rel(shelf_base_xyz[0] - 0.08 + 0.33, shelf_base_xyz[1], wait=True)
            agent.open_gripper()
            agent.move_rel(-0.4, 0)
            agent.pose.move_pose()
            continue
        if shelf_item_cent_z > shelf_2_3_height_threshold:
            agent.pose.place_shelf_pose('shelf_3f')
        elif shelf_item_cent_z > shelf_1_2_height_threshold:
            agent.pose.place_shelf_pose('shelf_2f')
        else:
            agent.pose.place_side_pose('shelf_1f')

        if shelf_item_cent_y > 0:   # 3d horizontal cent point.
            shelf_base_to_object_xyz = [gripper_to_shelf_x, shelf_item_cent_y - place_object_offset_dist, 0]
        else:
            shelf_base_to_object_xyz = [gripper_to_shelf_x, shelf_item_cent_y + place_object_offset_dist, 0]
        print('shelf_base_to_object_xyz', shelf_base_to_object_xyz)
        shelf_base_xyz = agent.yolo_module.calculate_dist_to_pick(shelf_base_to_object_xyz, 4)
        print('shelf_base_xyz', shelf_base_xyz)
        # agent.move_rel(0, shelf_base_xyz[1], wait=True)
        # agent.move_rel(shelf_base_xyz[0], 0, wait=True)

        if shelf_item_cent_z < shelf_1_2_height_threshold: # 1F
            agent.move_rel(shelf_base_xyz[0] - 0.08 - 0.18 + 0.33, shelf_base_xyz[1], wait=True)  # 2f pose - 1f pose = 0.18
        else: # 2F & 3F
            agent.move_rel(shelf_base_xyz[0] - 0.08 + 0.33, shelf_base_xyz[1], wait=True)
        agent.open_gripper()
        agent.move_rel(-0.4, 0)
        agent.pose.move_pose()