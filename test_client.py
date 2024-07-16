import math

import rospy
from hsr_agent.agent import Agent
from utils.axis_transform import Axis_transform
from utils.depth_to_pc import Depth2PC
import numpy as np
from collections import deque
import cv2
import time

if __name__ == '__main__':
    rospy.init_node('test_client_hsr')
    axis_transform = Axis_transform()
    agent = Agent()
    while True:
        command = input('task : ')
        if command == 'start_1':
            agent.move_abs('start_1')
        elif command == 'p1':
            agent.pose.p1()
        elif command == 'inspection':
            agent.move_abs_safe('insp_target')
        elif command == 'zero':
            agent.move_abs('zero')
        elif command == 'dev_front':
            agent.move_abs('dev_front')
        elif command == 'table_front':
            agent.move_abs('table_front')
        elif command == "seat_scan":
            agent.move_abs_safe('seat_scan')
        elif command == 'table_side':
            agent.move_abs('table_side')
        elif command == 'gt':
            agent.move_abs('grocery_table')
        elif command == 'gs':
            agent.move_abs('grocery_shelf')
        elif command == 'gsd':
            agent.move_abs('grocery_shelf_door')
        elif command == 'bin' :
            agent.move_abs('bin')
        elif command == 'sofa_view':
            agent.move_abs('sofa_view')
        elif command == 'door_handle':
            agent.move_abs('door_handle')
        elif command == 'door_bypass':
            agent.move_abs('door_bypass')
        elif command == 'grocery_table':
            agent.move_abs('grocery_table')
        elif command == 'cloth_scan':
            agent.move_abs_safe('cloth_scan')
        elif command == 'dishwasher':
            agent.move_abs('dishwasher')
        elif command == 'breakfast_test':
            agent.move_abs_safe('breakfast_table')
        elif command == 'table_front_test':
            agent.move_abs_safe('table_front')
        elif command == 'table_side_test':
            agent.move_abs_safe('table_side')
        elif command == 'breakfast_table':
            agent.move_abs('breakfast_table')

        ### Pose commands
        elif command == 'point_seat_pose':
            agent.pose.point_seat_pose()
        elif command == 'place_cutlery_pose':
            agent.pose.place_cutlery_pose()
        elif command == 'move_pose':
            agent.pose.move_pose()
        elif command == 'move_pose_vertical':
            agent.pose.move_pose(vertical=True)
        elif command == 'dish_washer_ready_pose':
            agent.pose.dish_washer_ready_pose()
        elif command == 'neutral_pose':
            agent.pose.neutral_pose()
        elif command == 'neutral_pose_vertical':
            agent.pose.neutral_pose(vertical=True)
        elif command == 'pick_side_pose':
            agent.pose.pick_side_pose(table='kitchen_table')
        elif command == 'pick_side_pose_breakfast_table':
            agent.pose.pick_side_pose(table='breakfast_table')
        elif command == 'pick_side_pose_by_height':
            agent.pose.pick_side_pose_by_height(height=0.59)
        elif command == 'place_side_shelf_pose':
            agent.pose.place_side_pose('shelf_1f')
        elif command == 'reach_left':
            agent.pose.reach_shelf_door_pose(shelf='grocery_shelf', floor=2, side='left')
        elif command == 'reach_right':
            agent.pose.reach_shelf_door_pose(shelf='grocery_shelf', floor=2, side='right')
        elif command == 'cling_left':
            agent.pose.cling_shelf_door_pose(shelf='grocery_shelf', floor=2, side='left')
        elif command == 'cling_right':
            agent.pose.cling_shelf_door_pose(shelf='grocery_shelf', floor=2, side='right')
        elif command == 'pick_top_pose':
            agent.pose.pick_top_pose(table='breakfast_table')
        elif command == 'pick_top_pose_breakfast_table':
            agent.pose.pick_top_pose_last(table='breakfast_table')
        elif command == 'pick_plate_pose':
            agent.pose.pick_plate_pose(table='kitchen_table')
        elif command == 'pick_dish_pose':
            agent.pose.pick_dish_pose(table='kitchen_table')
        elif command == 'pick_dish_pose_fold':
            agent.pose.pick_dish_pose_fold(table='kitchen_table')
        elif command == 'pick_bowl_pose_breakfast_table':
            agent.pose.pick_bowl_pose(table='breakfast_table')
        elif command == 'plate_dishwasher_pose':
            agent.pose.plate_dishwasher_pose()
        elif command == 'pick_bowl_pose_last':
            agent.pose.pick_bowl_pose_last(table='breakfast_table')
        elif command == 'pick_bowl_max_pose':
            agent.pose.pick_bowl_max_pose(table='kitchen_table')
        elif command == 'pick_up_bowl_pose':
            agent.pose.pick_up_bowl_pose(table='kitchen_table')
        elif command == 'place_bowl_pose':
            agent.pose.place_bowl_pose(table='kitchen_table')
        elif command == 'pick_plate_pose_fold':
            agent.pose.pick_plate_pose_fold(table='kitchen_table')
        elif command == 'place_object_pose':
            object = input("Give the name of the object: ")
            agent.pose.place_object_pose(table='dishwasher', item=object)
        elif command == 'put_plate_dish_washer':
            agent.pose.put_plate_dish_washer(table='dishwasher')
        elif command == 'bring_bowl_pose':
            agent.pose.bring_bowl_pose(table='kitchen_table')
        elif command == 'spill_cereal_pose':
            object_height = 0.14  # [m]
            agent.pose.spill_object_pose(object_height, table='final_kitchen_table')
        elif command == 'pick_shelf_low_pose':
            agent.pose.pick_shelf_low_pose('pantry')
        elif command == 'pick_side_pose':
            agent.pose.pick_side_pose('kitchen_table')
        elif command == 'door_open_pose':
            agent.pose.door_open_pose()
        elif command == 'table_search_pose' or command == 'ts':
            agent.pose.table_search_pose()
        elif command == 'table_search_pose_breakfast':
            agent.pose.table_search_pose_breakfast()
        elif command == 'table_search_go_pose':
            agent.pose.table_search_go_pose()
        elif command == 'pick_object_side_pose':
            agent.pose.pick_object_side_pose(0.06, table='breakfast_table')
        elif command == 'place_shelf_pose_1f':
            agent.pose.place_shelf_pose('shelf_1f')
        elif command == 'place_shelf_pose_2f':
            agent.pose.place_shelf_pose('shelf_2f')
        elif command == 'arm_lift_object_table_down_spoon':
            agent.pose.place_top_pose(0.05, table='kitchen_table')
            agent.pose.arm_lift_object_table_down(0.23, table='kitchen_table')
        elif command == 'arm_lift_object_table_down_wine':
            object_height = 0.1
            agent.pose.arm_lift_object_table_down(object_height, table='final_kitchen_table')
        elif command == 'pick_side_inclined_pose':
            table_name = input("Give me the table name : ")
            agent.pose.pick_side_inclined_pose(table_name)
        elif command == 'table_search_pose_high':
            agent.pose.table_search_pose_high()
        elif command == 'arm_lift_up':
            length = input("Give the arm_lift_up in length(0 ~ 0.69) : ")
            agent.pose.arm_lift_up(float(length))
        elif command == 'head_tilt':
            angle = input("Give the head tilt angle in degrees(-90 ~ 30) : ")
            agent.pose.head_tilt(angle)
        elif command == 'wrist_roll':
            angle = input("Give the wrist roll angle in degrees(-110 ~ 210) : ")
            agent.pose.wrist_roll(angle)
        elif command == 'head_pan':
            angle = input("Give the head pan angle in degrees(-90 ~ 30) : ")
            agent.pose.head_pan(angle)
        elif command == 'wrist_flex':
            angle = input("Give the wrist flex angle in degrees(-110 ~ 70) : ")
            agent.pose.wrist_flex(angle)
        elif command == 'arm_flex':
            angle = input("Give the arm flex angle in degrees  (-120 ~ 0): ")
            agent.pose.arm_flex(angle)
        elif command == 'arm_roll':
            angle = input("Give the arm roll angle in degrees (-110 ~ 210) : ")
            agent.pose.arm_roll(angle)
        elif command == 'arm_lift_object_table_down_cereal':
            cereal_height = 0.32  # [m] 
            object_height = cereal_height / 2
            agent.pose.arm_lift_object_table_down(object_height, table='kitchen_table')
        elif command == 'arm_lift_top_table_down':
            height = 0.02
            agent.pose.arm_lift_top_table_down(height=height, table='breakfast_table')
        elif command == 'handle_down':
            agent.pose.door_handle_down_pose()
        elif command == 'detect_3d':
             print(agent.yolo_module.detect_3d('final_kitchen_table'))
        elif command == 'yolo_bbox':
            print(agent.yolo_module.yolo_bbox)
        elif command == 'find_type_by_id':
            table_item_type = agent.yolo_module.find_type_by_id(0)
            print('table_item_type', table_item_type)
        elif command == 'find_id_by_name':
            table_item_type = agent.yolo_module.find_id_by_name('jello_red')
            print('table_item_type', table_item_type)
        elif command == 'yolo_bbox_item':
            name = input("Item name : ")
            _, item_id, itemtype, grasping_type = agent.yolo_module.find_object_info_by_name(name)
            bbox = agent.yolo_module.find_box_by_name(name)
            print(bbox)
            target_object_pc = agent.get_target_object_pc(bbox)
            print(target_object_pc)
            base_to_object_xyz = axis_transform.tf_camera_to_base(target_object_pc)
            print(base_to_object_xyz)
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_object_xyz, grasping_type)
            print(base_xyz)
        elif command == 'yolo_item_3d_points':
            print("Give 'shelf_1f' if HSR is in front of the shelf.")
            table = input("Table name : ")
            print(agent.yolo_module.detect_3d(table))
        elif command == 'distancing':
            from utils.distancing import distancing, distancing_horizontal
            print(distancing(agent.yolo_module.pc, 'kitchen_table'))
        elif command == 'q' or command == 'quit':
            break
        elif command == 'tts':
            sentence = input('sentence : ')
            agent.say(sentence)
        elif command == 'tts_sentence':
            sentence = 'bowl'
            agent.say('please hand me the '+str(sentence))
        elif command == 'stt':
            result = agent.stt(5.)
            rospy.loginfo(f'STT Result: {result}')
        elif command == 'l': # open gripper
            agent.open_gripper()
        elif command == 'p': # pick
            agent.grasp()
        elif command == 'weak': # pick
            agent.grasp(weak=True)
        elif command == 'grasp_degree':
            radian = float(input('radian : '))
            agent.grasp_degree(radian)
        elif command == 'w':
            agent.move_rel(.3, 0, wait=True)
        elif command == 's':
            agent.move_rel(-0.3, 0, wait=True)
        elif command == 'a':
            agent.move_rel(0, 0.3, wait=True)
        elif command == 'd':
            agent.move_rel(0, -0.3, wait=True)
        elif command == 'z':
            agent.move_rel(0, 0, yaw=1.57)
        elif command == 'c':
            agent.move_rel(0, 0, yaw=-1.57)
        elif command == 'ww':
            agent.move_rel(0.1, 0,wait=True)
        elif command == 'www':
            agent.move_rel(0.03, 0)
        elif command == 'ss':
            agent.move_rel(-0.1, 0)
        elif command == 'aa':
            agent.move_rel(0, 0.1)
        elif command == 'aaa':
            agent.move_rel(0, 0.03)
        elif command == 'dd':
            agent.move_rel(0, -0.1)
        elif command == 'ddd':
            agent.move_rel(0, -0.03)
        elif command == 'zz':
            agent.move_rel(0, 0, yaw=0.2)
        elif command == 'zzz':
            agent.move_rel(0, 0, yaw=math.radians(5))
        elif command == 'cc':
            agent.move_rel(0, 0, yaw=-0.2)
        elif command == 'ccc':
            agent.move_rel(0, 0, yaw=-math.radians(5))
        elif command == 'show_image':
            file_name = input('file_name : ')
            agent.head_show_image(file_name=file_name)
        elif command == 'head_show_text':
            show_text = input('head_show_text : ')
            agent.head_show_text(show_text=show_text)
        elif command == 'getp':
            agent.get_pose()
        elif command == 'check_grasp':
            agent.pose.check_grasp()
        elif command == 'pick_shelf_pose':
            agent.pose.pick_shelf_pose('pantry_2f')
        elif command == 'check_bowl_grasp_threshold':
            agent.open_gripper()
            agent.pose.arm_lift_up(float(0.63))

            total_value_list = []
            total_bool_list = []
            for i in range(10):
                temp_value_list = []
                temp_bool_list = []
                for j in range(5):
                    agent.grasp()
                    rospy.sleep(0.5)

                    rad_value = agent.pose.check_grasp_radian()
                    is_grasp = agent.pose.check_grasp()
                    temp_value_list.append(rad_value)
                    temp_bool_list.append(is_grasp)

                    agent.open_gripper()
                    rospy.sleep(0.5)

                total_value_list.append(temp_value_list)
                total_bool_list.append(temp_bool_list)

                arm_height = float(0.63) + float(i+1)*0.01
                print('arm_height', arm_height)

                agent.pose.arm_lift_up(arm_height)
                rospy.sleep(1)

            print('total_value_list')
            for i in range(10):
                arm_height = float(0.63) + float(i) * 0.01
                print(f'{arm_height}: {total_value_list[i]}')
            print()

            print('total_bool_list')
            for i in range(10):
                arm_height = float(0.63) + float(i) * 0.01
                print(f'{arm_height}: {total_bool_list[i]}')


        elif command == 'bag_inspection_pose':
            agent.pose.bag_inspection_pose()
        elif command == 'handme':
            agent.pose.hand_me_bag()
        elif command == 'movego':
            agent.pose.move_to_go()
        elif command == 'armtd':
            agent.pose.arm_lift_top_down(.33)
        elif command == 'pickbag':
            agent.pose.pick_up_bag(.33)
        elif command == 'head_pan_tilt':
            joints = input('head_pan_joint, head_tilt_joint: ')
            joints = joints.split(' ')
            agent.pose.head_pan_tilt(int(joints[0]), int(joints[1]))
        elif command == 'detect_edges':
            agent.detect_edges()
        elif command == 'rgbd_tiny_object':
            d2pc = Depth2PC()
            #Todo: normalize하고 실시간으로 장애물이 어떻게 표시되는지 히트맵으로 보자...
            while not rospy.is_shutdown():

                # _pc = agent.pc.reshape(480, 640)
                # pc_np = np.nan_to_num(np.array(_pc.tolist())[240+120: 240+220, 150:490, :-1], nan=0.0).reshape(-1, 3)
                pc_np = np.nan_to_num(d2pc.pc_for_bbox(320, 410, 340, 100), nan=0.0)
                # print("pc np", pc_np.min(), pc_np.max(), pc_np.mean())
                cropped_img = agent.rgb_img[240+120: 240+220, 150:490]
                # cv2.imshow('cropped', agent.rgb_img[240:, :])
                # cv2.waitKey(1)
                points_by_base_link = axis_transform.tf_camera_to_base(pc_np, multi_dimention=True)
                depth_base_link = axis_transform.transform_coordinate_array( \
                    'head_rgbd_sensor_link', 'base_link', pc_np).reshape(100, 340, -1)
                print("final", depth_base_link.shape)
                z_no_nan = np.nan_to_num(depth_base_link[:, :, 2], nan=0.0).reshape(-1)
                z_no_nan[z_no_nan > .15] = z_no_nan.min()
                z_no_nan = z_no_nan.reshape(100, 340)
                print("z no nan", z_no_nan.mean(), z_no_nan.max())
                heatmap = (z_no_nan - z_no_nan.min()) / (z_no_nan.max() - z_no_nan.min())
                heatmap = cv2.GaussianBlur(heatmap, (0, 0), sigmaX=4)
                heatmap = (cv2.applyColorMap((heatmap * 255).astype(np.uint8), cv2.COLORMAP_RAINBOW) * .5 + cropped_img * .5).astype(np.uint8)
                cv2.imshow('dd', heatmap)
                cv2.waitKey(1)
                # print(depth_base_link)

        elif command =='bar':
            agent.move_abs_safe('bar')

        elif command == 'rest_move':
            agent.pose.restaurant_move_pose()
        elif command == 'rest_give':
            agent.pose.restaurant_give_pose()
        elif command == "getj":
            print(agent.pose.joint_value)

        elif command == 'move_rel':
            rel = input('x, y, rot: ')
            rel = rel.split(' ')
            agent.move_rel(float(rel[0]), float(rel[1]), float(rel[2]), wait = True)
        elif command == 'move_abs':
            position = input("position: ")
            agent.move_abs(position, wait=False)
        elif command == 'abs_safe':
            position = input("position: ")
            timeout = float(input("timeout: "))
            thresh = float(input("thresh: "))
            agent.move_abs_safe(position, thresh=thresh, timeout=timeout)
        elif command == 'abs_coord_safe':
            coordinate = input("coordinate (write with space ex 2 3 0.1):")
            coordinate = coordinate.split(' ')
            coordinate = [float(i) for i in coordinate]
            print("coordinate : ", coordinate)
            timeout = float(input("timeout: "))
            x, y, yaw = agent.get_pose()
            agent.move_abs_coordinate_safe(coordinate, timeout=timeout)
        elif command == 'test_obstacle_ahead':
            while not rospy.is_shutdown():
                agent.inspection_obstacle_ahead()
                rospy.sleep(.5)
        elif command == 'distancing_horizontal':
            from utils.distancing import distancing_horizontal
            pick_table = 'dishwasher'
            dist_y_center = distancing_horizontal(agent.yolo_module.pc, pick_table)
            agent.move_rel(0, dist_y_center, wait=True)
            print('dist_y_center', dist_y_center)
        elif command == 'check_grasp':
            print(agent.pose.check_grasp())
        elif command == 'wait_false':
            agent.move_rel(1.0,0,0,wait=False)
            # rospy.sleep(1)
            agent.say('move!')
            # rospy.sleep(1)
        elif command == 'depth_test':
            timeout = 5
            start_time = time.time()
            while time.time() - start_time <= timeout:
                _pc = agent.pc.reshape(480, 640)
                pc_np = np.array(_pc.tolist())[:, :, :3].reshape(-1, 3)
                points_by_base_link = agent.axis_transform.tf_camera_to_base(pc_np, multi_dimention=True)[:, 2]
                print("min", np.nanmin(points_by_base_link), "max", np.nanmax(points_by_base_link))
        elif command == 'detect_3d_unseen':
            agent.yolo_module.detect_3d_unseen('desk')[0]

        elif command == 'gpsr_test':
            from task.gpsr_repo.gpsr_test import gpsr_test
            gpsr_test(agent)

        else:
            print('invalid command')


# shoe_scan [1.4227, 1.1802, -1.5106]
# door pose [-2.439925046798405, 3.741186220829006, 1.6161519211842417]
# pick pose  [0.3110857689895185, -0.009397325880122043, 0.016862118362033503]

