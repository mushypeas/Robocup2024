import os
is_sim = 'localhost' in os.environ['ROS_MASTER_URI']

import rospy
import numpy as np
import utils.parse_gpsr_command
import time
from utils.gpsr_configs import *
from utils.distancing import distancing
from datetime import datetime
from module.human_attribute.qrcode import decoder_loop

##################### TODO
# calibration params
shelf_grasp_x_offset = 0.25
shelf_grasp_x_offset_ni = 0.05
shelf_place_x_offset = 0.35
table_grasp_x_offset = 0.01
table_place_x_offset = 0.10

shelf_grasp_z_offset = 0.115


# from sg
gripper_to_shelf_x = 0.9
###########################

top_grasping_objects = ['fork', 'spoon', 'knife']

command_keyboard_mode = False

def get_obj_by_pos(agent, prop, obj_list, target_cat):
    if prop == "left most":
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 1])[::-1]]  # ascend
    elif prop == 'right most':
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 1])]  # descend
    else:
        print("Current object property : ", prop)
        print("Object property not available. Quitting the program...")

    if target_cat != '':
        for t in agent.object_types:
            if t in target_cat:
                target_cat_name = t  # converting into type name in global config
                break
        # id of objects in target category
        yolo_obj_list = np.array(agent.object_list)
        obj_in_cat_id = yolo_obj_list[np.where(yolo_obj_list[:, 2].astype(int) ==
                                               agent.object_types.index(target_cat_name))][:, 1].astype(int)
        for obj in obj_list_sorted:
            if obj[3] in obj_in_cat_id:
                print(prop, "object : ", agent.yolo_module.find_name_by_id(obj[3]))
                return obj
    else:
        print(prop, "object :", agent.yolo_module.find_name_by_id(obj_list_sorted[0][3]))
        return obj_list_sorted[0]


def get_obj_by_size(agent, prop, bbox_list, obj_list, target_cat, task='pick'):
    bbox_list[:, 3] = np.multiply(bbox_list[:, 2], bbox_list[:, 3])
    if prop == 'biggest' or prop == 'largest' or prop == 'heaviest':
        bbox_list_sorted = bbox_list[np.argsort(bbox_list[:, 3])[::-1]]
    elif prop == 'smallest' or prop == 'lightest':
        bbox_list_sorted = bbox_list[np.argsort(bbox_list[:, 3])]
    elif prop == 'thinnest':
        bbox_list_sorted = bbox_list[np.argsort(bbox_list[:, 2])]
    else:
        print("Current object property : ", prop)
        agent.say("object property not available")
        return [-1]


    if target_cat != '':
        for t in agent.object_types:
            if t in target_cat:
                target_cat_name = t  # converting into type name in global config
                break
                # id of objects in target category
        yolo_obj_list = np.array(agent.object_list)
        obj_in_cat_id = yolo_obj_list[np.where(yolo_obj_list[:, 2].astype(int) ==
                                               agent.object_types.index(target_cat_name))][:, 1].astype(int)
        for obj in bbox_list_sorted:
            if obj[4] in obj_in_cat_id:
                print(prop, "object : ", agent.yolo_module.find_name_by_id(obj[4]))
                if task == 'pick':
                    return obj_list[obj_list[:, 3] == obj[4]][0]
                else:
                    return agent.yolo_module.find_name_by_id(obj[4])
    else:
        obj_id = bbox_list_sorted[0][4]
        print(prop, "object : ", agent.yolo_module.find_name_by_id(obj_id))
        if task == 'pick':
            return obj_list[obj_list[:, 3] == obj_id][0]
        else:
            return agent.yolo_module.find_name_by_id(obj_id)

def get_obj_by_ref(agent, ref, ref_prop, obj_list):
    if ref_prop == 'at the left of':
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 1])]
        ref_idx = int(np.where(obj_list_sorted[:, 3] == agent.yolo_module.find_id_by_name(ref))[0])
        for idx in range(ref_idx+1, len(obj_list_sorted)):
            if abs(obj_list_sorted[idx][2] - obj_list_sorted[ref_idx][2]) < 0.15:
                obj_idx = idx
                break
        obj_3d_points = obj_list_sorted[obj_idx]
        print(ref_prop, "object : ", agent.yolo_module.find_name_by_id(obj_3d_points[3]))
        return obj_3d_points
    elif ref_prop == 'at the right of':
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 1])]
        ref_idx = int(np.where(obj_list_sorted[:, 3] == agent.yolo_module.find_id_by_name(ref))[0])
        for idx in range(ref_idx)[::-1]:
            if abs(obj_list_sorted[idx][2] - obj_list_sorted[ref_idx][2]) < 0.15:
                obj_idx = idx
                break
        obj_3d_points = obj_list_sorted[obj_idx]
        print(ref_prop, "object : ", agent.yolo_module.find_name_by_id(obj_3d_points[3]))
        return obj_3d_points
    elif ref_prop == 'on top of' or ref_prop == 'above':
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 2])]
        ref_idx = int(np.where(obj_list_sorted[:, 3] == agent.yolo_module.find_id_by_name(ref))[0])
        for idx in range(ref_idx + 1, len(obj_list_sorted)):
            if abs(obj_list_sorted[idx][1] - obj_list_sorted[ref_idx][1]) < 0.3:
                obj_idx = idx
        obj_3d_points = obj_list_sorted[obj_idx]
        print(ref_prop, "object : ", agent.yolo_module.find_name_by_id(obj_3d_points[3]))
        return obj_3d_points
    elif ref_prop == 'under':
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 2])]
        ref_idx = int(np.where(obj_list_sorted[:, 3] == agent.yolo_module.find_id_by_name(ref))[0])
        for idx in range(ref_idx)[::-1]:
            if abs(obj_list_sorted[idx][1] - obj_list_sorted[ref_idx][1]) < 0.3:
                obj_idx = idx
        obj_3d_points = obj_list_sorted[obj_idx]
        print(ref_prop, "object : ", agent.yolo_module.find_name_by_id(obj_3d_points[3]))
        return obj_3d_points
    elif ref_prop == 'behind':
        obj_list_sorted = obj_list[np.argsort(obj_list[:, 0])]
        print(ref)
        print(agent.yolo_module.find_id_by_name(ref))
        ref_idx = int(np.where(obj_list_sorted[:, 3] == agent.yolo_module.find_id_by_name(ref))[0])
        for idx in range(ref_idx + 1, len(obj_list_sorted)):
            if abs(obj_list_sorted[idx][1] - obj_list_sorted[ref_idx][1]) < 0.3 and \
                    abs(obj_list_sorted[idx][2] - obj_list_sorted[ref_idx][2]) < 0.15:
                obj_idx = idx
        obj_3d_points = obj_list_sorted[obj_idx]
        print(ref_prop, "object : ", agent.yolo_module.find_name_by_id(obj_3d_points[3]))
        return obj_3d_points
    else:
        print("Current relative property to reference object : ", ref_prop)
        agent.say("Relative property not available.")
        return [-1]

def pick_function(agent, object, object_place, base_to_obj):
    if object == 'plate':
        grasping_type = 3
        base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
        # TODO: modify
        plate_radius = 0.08
        base_to_arm_dist = 0.5

        dist_to_base_x = distancing(agent.yolo_module.pc, object_place, raw=True)
        plate_xyz = base_xyz
        dist_plate_x = plate_xyz[0] - (dist_to_base_x - base_to_arm_dist) + plate_radius
        print('dist_plate_x', dist_plate_x)

    shelf_high_layer = ['shelf_2f', 'shelf_3f', 'shelf_4f', 'storage_rack_2f', 'storage-rack_3f',
                        'pantry_2f', 'pantry_3f', 'pantry_4f']
    shelf_low_layer = ['shelf_1f', 'storage_rack_1f', 'pantry_1f']
    # TODO: what if spoon in shelf?
    if object_place == 'shelf':
        if base_to_obj[2] < 0.6:
            object_place = 'shelf_1f'
        elif base_to_obj[2] < 0.9:
            object_place = 'shelf_2f'
        elif base_to_obj[2] < 1.2:
            object_place = 'shelf_3f'
        else:
            object_place = 'shelf_4f'
    elif object_place == 'pantry':
        if base_to_obj[2] < 0.6:
            object_place = 'pantry_1f'
        elif base_to_obj[2] < 0.9:
            object_place = 'pantry_2f'
        elif base_to_obj[2] < 1.2:
            object_place = 'pantry_3f'
        else:
            object_place = 'pantry_4f'
    elif object_place == 'storage_rack':
        if base_to_obj[2] < 0.6:
            object_place = 'storage_rack_1f'
        elif base_to_obj[2] < 1.03:
            object_place = 'storage_rack_2f'
        else:
            object_place = 'storage_rack_3f'

    print(object_place)
    if object_place in shelf_high_layer:
        grasping_type = 4
        base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
        agent.pose.pick_shelf_pose(object_place, shelf_grasp_z_offset)
        agent.open_gripper()
        agent.move_rel(0, base_xyz[1], wait=True)
        agent.move_rel(base_xyz[0], 0, wait=True)
        agent.grasp()
        rospy.sleep(1)
        agent.move_rel(-0.5, 0, wait=True)
    elif object_place in shelf_low_layer:
        grasping_type = 0
        base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
        agent.pose.pick_shelf_low_pose(object_place)
        agent.open_gripper()
        agent.move_rel(0, base_xyz[1], wait=True)
        agent.move_rel(base_xyz[0] + shelf_grasp_x_offset_ni, 0, wait=True)
        agent.grasp()
        rospy.sleep(1)
        agent.move_rel(-0.5, 0, wait=True)
    else:
        agent.move_rel(-0.15, 0, wait=True)
        if object in top_grasping_objects:
            grasping_type = 1
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
            # from ctt
            agent.pose.pick_top_pose(table=object_place)
            agent.open_gripper()
            agent.move_rel(base_xyz[0] + 0.35, base_xyz[1] - 0.01, wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.005, table=object_place)
            agent.grasp()
            rospy.sleep(1)
            agent.pose.arm_flex(-60)
            agent.move_rel(-0.4, 0)
        elif object == 'bowl':
            grasping_type = 2
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
            # from ctt
            agent.pose.pick_bowl_pose(table=object_place)
            agent.open_gripper()
            agent.move_rel(base_xyz[0] + 0.15, base_xyz[1], wait=True)
            agent.pose.pick_bowl_max_pose(table=object_place, height=-0.12)
            agent.grasp()
            rospy.sleep(1)
            rospy.sleep(0.5)
            agent.pose.pick_up_bowl_pose(table=object_place)
            agent.move_rel(-0.4, 0)
        elif object == 'plate':
            # from ctt
            agent.pose.pick_plate_pose(table=object_place)
            agent.open_gripper()
            agent.move_rel(plate_xyz[0] + 0.18, plate_xyz[1], wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.025, table=object_place)
            agent.move_rel(-dist_plate_x - 0.06, 0, wait=True)
            agent.pose.arm_lift_top_table_down(height=0.05, table=object_place)
            agent.move_rel(-0.18, 0, wait=True)
            agent.pose.pick_plate_pose_fold(table=object_place)
            agent.move_rel(0.04, 0, wait=True)  # slightly move forward
            agent.grasp()
            rospy.sleep(1)
        else:
            grasping_type = 0
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
            agent.pose.pick_side_pose(object_place)
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0] + 0.15 + table_grasp_x_offset, 0, wait=True)
            agent.grasp()
            rospy.sleep(1)
            agent.move_rel(-0.4, 0, wait=True)

def manipulation_main(agent, command_dict):
    """
    command_dict keys:
    1) action: what action robot should take
    2) target_pl: place to put object
    3) target_obj: object to manipulate
    4) target_cat: category to manipulate(when object is not specified)  ex: bring 'fruits'
    5) ref: object which has relative position to target object
    6) ref_prop: property that object has relative to reference object
    7) prop: target object property
    8) obj_pl: place where object exists currently
    9) person: person
    """

    #################### for object names ###################

    command_dict['target_obj'] = command_dict['target_obj'].replace(' ', '_')
    command_dict['target_obj'] = change_object_names(command_dict['target_obj'])
    command_dict['target_cat'] = change_object_types(command_dict['target_cat'])

    print(command_dict['target_obj'])
    print(command_dict['target_cat'])

    cur_pl = None
    if command_dict['target_pl'] == 'here':
        cur_pl = agent.get_pose()
        print(cur_pl)

    if command_dict['obj_pl'] == 'here':
        if not command_keyboard_mode:
            is_correct_place = False
            for i in range(3):
                rospy.sleep(2)
                agent.say('Please tell me where the object is\nonly by word\nafter the ding sound')
                rospy.sleep(9)
                placement, _ = agent.stt(7.)
                placement = placement.lower().split()
                # TODO: test
                agent.say("The placement you said is\n{0}".format(placement[0]))
                if len(placement) <= 2:
                    if [placement[0]] in locations:
                        command_dict['obj_pl'] = placement
                        is_correct_place = True
                        break
                    else:
                        if i < 2:
                            agent.say('please give me the\ncorrect name of the placement', show_display=True)
                            rospy.sleep(4)
                else:
                    if i < 2:
                        agent.say('please give me only\nthe name, not by sentence', show_display=True)
                        rospy.sleep(4)
            command_dict['obj_pl'] = ' '.join(placement)
        else:
            command_dict['obj_pl'] = input("Give me the name of the placement : ")

    agent.pose.move_pose()

    command_dict['obj_pl'] = command_dict['obj_pl'].replace(' ', '_')
    if command_dict['obj_pl'] == 'dishwasher':
        command_dict['obj_pl'] = 'dishwasher_gpsr'

    # TODO: test
    if command_dict['obj_pl'] in list(agent.location_map.keys()):
        placements = agent.location_map[command_dict['obj_pl']]
        for p in placements:
            if p in list(agent.table_dimension.keys()):
                agent.move_abs_safe(p)
                rospy.sleep(2)
                if p in ['shelf', 'storage_rack', 'pantry']:
                    agent.head_tilt(-10)
                    rospy.sleep(2)
                    obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))
                    ishere = False
                    for obj in obj_list:
                        if obj[3] == agent.yolo_module.find_id_by_name(command_dict['target_obj']):
                            ishere = True
                            command_dict['obj_pl'] = p
                            break
                    if ishere:
                        break
                    if not ishere:
                        agent.head_tilt(10)
                        rospy.sleep(2)
                        obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))
                        ishere = False
                        for obj in obj_list:
                            if obj[3] == agent.yolo_module.find_id_by_name(command_dict['target_obj']):
                                ishere = True
                                command_dict['obj_pl'] = p
                        if ishere:
                            break
                else:
                    if agent.table_dimension[p][2] > 0.8:
                        agent.pose.table_search_pose_high()
                    else:
                        agent.pose.table_search_pose()
                    rospy.sleep(2)
                    obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))
                    ishere = False
                    for obj in obj_list:
                        if obj[3] == agent.yolo_module.find_id_by_name(command_dict['target_obj']):
                            ishere = True
                            command_dict['obj_pl'] = p
                            break
                    if ishere:
                        break

    if command_dict['obj_pl'] in ['shelf', 'storage_rack', 'pantry']:
        agent.move_abs_safe(command_dict['obj_pl'])
        # dist_initial_forward = distancing(agent.yolo_module.pc, command_dict['obj_pl'], dist=0.9)
        # agent.move_rel(dist_initial_forward, 0, wait=True)
        agent.pose.head_tilt(10)
        rospy.sleep(2)
        obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))

        if command_dict['prop'] in ['right most', 'left most']:
            base_to_obj = get_obj_by_pos(agent, command_dict['prop'], obj_list, command_dict['target_cat'])
        elif command_dict['prop'] in ['biggest', 'largest', 'smallest', 'heaviest', 'lightest', 'thinnest']:
            bbox_list = agent.yolo_module.yolo_bbox
            base_to_obj = get_obj_by_size(agent, command_dict['prop'], bbox_list, obj_list,
                                          command_dict['target_cat'])
        elif command_dict['ref'] != '':
            base_to_obj = get_obj_by_ref(agent, command_dict['ref'], command_dict['ref_prop'], obj_list)
            if command_dict['ref_prop'] == 'behind':
                base_to_obj_ref = agent.yolo_module.find_3d_points_by_name(obj_list, command_dict['ref'])
                base_xyz_ref = agent.yolo_module.calculate_dist_to_pick(base_to_obj_ref, 0)
        else:
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, command_dict['target_obj'])
            print(base_to_obj)
        if base_to_obj is None:
            agent.pose.head_tilt(-10)
            rospy.sleep(2)
            obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))
            if command_dict['prop'] in ['right most', 'left most']:
                base_to_obj = get_obj_by_pos(agent, command_dict['prop'], obj_list, command_dict['target_cat'])
            elif command_dict['prop'] in ['biggest', 'largest', 'smallest', 'heaviest', 'lightest', 'thinnest']:
                bbox_list = agent.yolo_module.yolo_bbox
                base_to_obj = get_obj_by_size(agent, command_dict['prop'], bbox_list, obj_list,
                                              command_dict['target_cat'])
            elif command_dict['ref'] != '':
                base_to_obj = get_obj_by_ref(agent, command_dict['ref'], command_dict['ref_prop'], obj_list)
                if command_dict['ref_prop'] == 'behind':
                    base_to_obj_ref = agent.yolo_module.find_3d_points_by_name(obj_list, command_dict['ref'])
                    base_xyz_ref = agent.yolo_module.calculate_dist_to_pick(base_to_obj_ref, 0)
            else:
                base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, command_dict['target_obj'])
    else:
        agent.move_abs_safe(command_dict['obj_pl'])
        # dist_initial_forward = distancing(agent.yolo_module.pc, command_dict['obj_pl'])
        # agent.move_rel(dist_initial_forward, 0, wait=True)
        if agent.table_dimension[command_dict['obj_pl']][2] > 0.8:
            agent.pose.table_search_pose_high()
        else:
            agent.pose.table_search_pose()
        rospy.sleep(2)

        obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))

        if command_dict['prop'] in ['right most', 'left most']:
            base_to_obj = get_obj_by_pos(agent, command_dict['prop'], obj_list, command_dict['target_cat'])
        elif command_dict['prop'] in ['biggest', 'largest', 'smallest', 'heaviest', 'lightest', 'thinnest']:
            bbox_list = agent.yolo_module.yolo_bbox
            base_to_obj = get_obj_by_size(agent, command_dict['prop'], bbox_list, obj_list, command_dict['target_cat'])
        elif command_dict['ref'] != '':
            base_to_obj = get_obj_by_ref(agent, command_dict['ref'], command_dict['ref_prop'], obj_list)
            if command_dict['ref_prop'] == 'behind':
                base_to_obj_ref = agent.yolo_module.find_3d_points_by_name(obj_list, command_dict['ref'])
                base_xyz_ref = agent.yolo_module.calculate_dist_to_pick(base_to_obj_ref, 0)
        else:
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, command_dict['target_obj'])
        print(base_to_obj)
        if len(base_to_obj) == 1:
            agent.say("Wrong command\nStop doing the task")
            return

    if command_dict['ref'] != '' and command_dict['ref_prop'] == 'behind':
        pick_function(agent, command_dict['ref'], command_dict['obj_pl'], base_to_obj_ref)
        agent.move_rel(0, 0.1)
        agent.open_gripper()
        agent.move_rel(-0.5, 0, wait=True)
        agent.pose.move_pose()
        agent.move_abs_safe(command_dict['obj_pl'])

    pick_function(agent, command_dict['target_obj'], command_dict['obj_pl'], base_to_obj)
    agent.pose.move_pose()

    if cur_pl is None:
        command_dict['target_pl'] = command_dict['target_pl'].replace(' ', '_')
        if command_dict['target_pl'] == 'dishwasher':
            command_dict['target_pl'] = 'dishwasher_gpsr'

        agent.move_abs_safe(command_dict['target_pl'])
        if len(command_dict['person']) != 0:
            answer = ''
            is_here = False
            for i in range(3):
                agent.say('{0}\nPlease come here\nso I can hand you\nthe object'.format(command_dict['person']),
                          show_display=True)
                rospy.sleep(5)
                agent.say('Please say yes\nif you are\nin front of me\nafter the sound', show_display=True)
                rospy.sleep(4)
                answer, _ = agent.stt(5.)
                if 'yes' in answer:
                    is_here = True
                    break
            if not is_here:
                agent.say("I will assume that\nyou are here", show_display=True)
                rospy.sleep(4)
            agent.say('Please move back\nBecause I will\nstretch out my arm', show_display=True)
            rospy.sleep(5)
            agent.pose.place_side_pose('kitchen_table')
            agent.say('I will open my gripper\nplease receive it', show_display=True)
            rospy.sleep(5)
            agent.open_gripper()
        else:
            # TODO: bowl, plate, spoon etc place
            if command_dict['target_pl'] in ['shelf', 'pantry', 'cabinet']:
                place_location = command_dict['target_pl'] + '_' + '2f'
                agent.pose.place_shelf_pose(place_location)
                shelf_base_to_object_xyz = [gripper_to_shelf_x, 0, 0]
                shelf_base_xyz = agent.yolo_module.calculate_dist_to_pick(shelf_base_to_object_xyz, 4)
                agent.move_rel(shelf_base_xyz[0] + 0.05, shelf_base_xyz[1], wait=True)
                agent.open_gripper()
                agent.move_rel(-0.5, 0)
                agent.pose.move_pose()
            else:
                if agent.table_dimension[command_dict['target_pl']][2] > 0.8:
                    agent.pose.table_search_pose_high()
                else:
                    pass
                if command_dict['target_pl'] is not 'kitchen_table':
                    dist_to_table = distancing(agent.yolo_module.pc, command_dict['target_pl'])
                    agent.move_rel(dist_to_table, 0, wait=True)
                else:
                    pass
                agent.move_rel(-0.2, 0, wait=True)
                rospy.sleep(2)
                agent.pose.place_side_pose(command_dict['target_pl'])
                rospy.sleep(2)
                agent.move_rel(0.2 + table_place_x_offset, 0, wait=True)
                rospy.sleep(2)
                agent.open_gripper()
                agent.move_rel(-0.5, 0, wait=True)
                agent.pose.move_pose()
    else:
        agent.move_abs_by_point(cur_pl)
        agent.pose.place_side_pose('kitchen_table')
        agent.say('I will open my gripper\nplease receive it', show_display=True)
        rospy.sleep(5)
        agent.open_gripper()

    agent.say("Task done!")
    rospy.sleep(5)
    agent.say('Wait a second\nto give me a new command', show_display=True)
    rospy.sleep(7)

def manipulation_other(agent, command_dict):
    if command_dict['action'] == 'bring_bag':
        # agent.say("You have to hand me the bag", show_display=True)
        # rospy.sleep(5)
        # agent.say('I will stretch my arm\nplease stay back', show_display=True)
        # rospy.sleep(5)
        # agent.pose.place_side_pose('kitchen_table')
        # rospy.sleep(3)
        # agent.say('Please hang the bag\non my arm', show_display=True)
        # rospy.sleep(10)
        # agent.grasp()
        # agent.pose.neutral_pose()
        command_dict['target_pl'] = command_dict['target_pl'].replace(' ', '_')
        if command_dict['target_pl'] == 'dishwasher':
            command_dict['target_pl'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['target_pl'])
    elif command_dict['action'] == 'take_out_trash':
        agent.say("You have to hand me the trash", show_display=True)
        rospy.sleep(5)
        agent.say('I wll stretch my arm\nplease stay back', show_display=True)
        rospy.sleep(5)
        agent.pose.place_side_pose('kitchen_table')
        rospy.sleep(3)
        agent.open_gripper()
        agent.say('Please put the trash\nbetween my gripper\nso that I can grab it', show_display=True)
        rospy.sleep(10)
        agent.grasp()
        rospy.sleep(3)
        agent.pose.neutral_pose()
        agent.move_abs_safe('exit')
        rospy.sleep(3)
        agent.pose.place_side_pose('kitchen_table')
        rospy.sleep(2)
        agent.open_gripper()
        rospy.sleep(3)

    agent.say("Task done!")
    rospy.sleep(5)
    agent.say('Wait a second\nto give me a new command', show_display=True)
    rospy.sleep(7)



def hri_main(agent, command_dict):
    if command_dict['action'] in ['follow', 'guide']:
        command_dict['first_place'] = command_dict['first_place'].replace(' ', '_')
        command_dict['final_place'] = command_dict['final_place'].replace(' ', '_')
        if command_dict['first_place'] == 'dishwasher':
            command_dict['first_place'] = 'dishwasher_gpsr'
        if command_dict['final_place'] == 'dishwasher':
            command_dict['final_place'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['first_place'])
        rospy.sleep(3)
        answer = ''
        is_ready = False
        for i in range(3):
            agent.say("{0},\nplease come here".format(command_dict['person']), show_display=True)
            rospy.sleep(5)
            if command_dict['action'] == 'follow':
                agent.say('Say I am ready\nwhen you are ready\nto guide me\nafter the sound', show_display=True)
            elif command_dict['action'] == 'guide':
                agent.say('Say I am ready\nwhen you are ready\nto follow me\nafter the sound', show_display=True)
            rospy.sleep(7)
            answer, _ = agent.stt(5.)
            if 'ready' in answer:
                is_ready = True
                break
        if not is_ready:
            agent.say("I will assume that\nyou are ready", show_display=True)
            rospy.sleep(4)
        agent.move_abs_safe(command_dict['final_place'])

    elif command_dict['action'] == 'tell_name':
        command_dict['first_place'] = command_dict['first_place'].replace(' ', '_')
        if command_dict['first_place'] == 'dishwasher':
            command_dict['first_place'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['first_place'])
        rospy.sleep(3)
        answer = ''
        is_here = False
        for i in range(3):
            agent.say("please come here", show_display=True)
            rospy.sleep(5)
            agent.say('Say yes if you are here\nafter the sound', show_display=True)
            rospy.sleep(5)
            answer, _ = agent.stt(5.)
            if 'ready' in answer:
                is_here = True
                break
        if not is_here:
            agent.say("I will assume that\nyou are here", show_display=True)
            rospy.sleep(4)

        is_correct = False
        for i in range(3):
            agent.say('Say your name\nafter the sound', show_display=True)
            rospy.sleep(4)
            name, _ = agent.stt(5.)
            agent.say('Is this your name? {0}\nSay yes if right\nor no if not right\nafter the sound'.format(name),
                      show_display=True)
            rospy.sleep(7)
            answer, _ = agent.stt(5.)
            if 'yes' in answer:
                is_correct = True
                break
        if not is_correct:
            agent.say("I cannot recognize\nyour name\precisely", show_display=True)
            rospy.sleep(4)
            agent.say("I will keep going\nas I understood", show_display=True)
            rospy.sleep(4)
        agent.move_abs_safe('gpsr_start')
        rospy.sleep(3)
        agent.say('The name of the person is')
        rospy.sleep(2)
        agent.say(name)
        rospy.sleep(5)

    elif command_dict['action'] == 'introduce':
        command_dict['first_place'] = command_dict['first_place'].replace(' ', '_')
        command_dict['final_place'] = command_dict['final_place'].replace(' ', '_')
        if command_dict['first_place'] == 'dishwasher':
            command_dict['first_place'] = 'dishwasher_gpsr'
        if command_dict['final_place'] == 'dishwasher':
            command_dict['final_place'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['first_place'])
        rospy.sleep(3)
        answer = ''
        is_here = False
        for i in range(3):
            agent.say("{0},\nplease come here".format(command_dict['person']), show_display=True)
            rospy.sleep(5)
            agent.say('Say I am ready\nwhen you are ready\nto follow me\nafter the sound', show_display=True)
            rospy.sleep(7)
            answer, _ = agent.stt(5.)
            if 'ready' in answer:
                is_here = True
                break
        if not is_here:
            agent.say('I will assume that\nyou are here', show_display=True)
            rospy.sleep(4)
        agent.say('Follow me', show_display=True)
        rospy.sleep(3)
        agent.move_abs_safe(command_dict['final_place'])
        rospy.sleep(5)
        agent.say("{0}\nPlease come here".format(command_dict['subject']))
        rospy.sleep(7)
        agent.say('Hi, {0}'.format(command_dict['subject']), show_display=True)
        rospy.sleep(7)
        agent.say('This is {0}'.format(command_dict['person']), show_display=True)
        rospy.sleep(7)
        agent.say('Have a nice time')
        rospy.sleep(5)

    elif command_dict['action'] == 'ask_to_leave':
        command_dict['first_place'] = command_dict['first_place'].replace(' ', '_')
        if command_dict['first_place'] == 'dishwasher':
            command_dict['first_place'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['first_place'])
        rospy.sleep(3)
        answer = ''
        is_here = False
        for i in range(3):
            agent.say("{0}\nplease come here".format(command_dict['person']), show_display=True)
            rospy.sleep(5)
            agent.say('Say yes if you are here\nafter the sound', show_display=True)
            rospy.sleep(5)
            answer, _ = agent.stt(5.)
            if 'yes' in answer:
                is_here = True
                break
        if not is_here:
            agent.say('I will assume that\nyou are here', show_display=True)
            rospy.sleep(4)
        agent.say('Please leave here')
        rospy.sleep(5)

    else:
        command_dict['first_place'] = command_dict['first_place'].replace(' ', '_')
        if command_dict['first_place'] == 'dishwasher':
            command_dict['first_place'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['first_place'])
        rospy.sleep(3)
        answer = ''
        is_here = False
        if not command_keyboard_mode:
            for i in range(3):
                agent.say('{0}\nplease come here'.format(command_dict['subject']), show_display=True)
                rospy.sleep(5)
                agent.say('Say yes if you are here\nafter the sound')
                rospy.sleep(5)
                answer, _ = agent.stt(5.)
                if 'yes' in answer:
                    is_here = True
                    break
            if not is_here:
                agent.say('I will assume that\nyou are here', show_display=True)
                rospy.sleep(4)

        if int(command_dict['action']) == 0:
            agent.say('Hi\nmy name is\nTidyboy', show_display=True)
            rospy.sleep(3)
            agent.say('I stayed in Korea', show_display=True)
            rospy.sleep(5)
        elif int(command_dict['action']) == 1:
            now_time = str(datetime.now().time())
            agent.say('The time now is', show_display=True)
            rospy.sleep(3)
            agent.say(now_time[:2], show_display=True)
            rospy.sleep(2)
            agent.say(now_time[3:5], show_display=True)
            rospy.sleep(5)
        elif int(command_dict['action']) == 2:
            agent.say('Today is\nFriday', show_display=True)  # TODO: check if right..
            rospy.sleep(5)
        elif int(command_dict['action']) == 3:
            agent.say('Tomorrow is\nSaturday', show_display=True)  # TODO: check if right..
            rospy.sleep(5)
        elif int(command_dict['action']) == 4:
            agent.say('Our team name is\nTidyboy', show_display=True)
            rospy.sleep(5)
        elif int(command_dict['action']) == 5:
            agent.say('Our team is from\nKorea', show_display=True)
            rospy.sleep(5)
        elif int(command_dict['action']) == 6:
            agent.say('Our team belongs to\nSeoul National University\nand Pusan National University', show_display=True)
            rospy.sleep(5)
        elif int(command_dict['action']) in [7, 8]:  # TODO: check if right..
            agent.say('Today is\nJuly 7th')
            rospy.sleep(5)
        elif int(command_dict['action']) == 9:
            agent.say('Guess why did the boy\nthrow the butter\nout of his window', show_display=True)
            rospy.sleep(8)
            agent.say('Because...\nhe wanted to see\na butterfly!', show_display=True)
            rospy.sleep(6)
            agent.say('ha ha ha')
            rospy.sleep(5)
        elif int(command_dict['action']) == 10:
            command = command_dict['full_command']
            if command_keyboard_mode:
                question = input('question : ')
                question_list = question.lower().split()
            else:
                for i in range(3):
                    agent.say('Please ask me a question\nafter the ding sound', show_display=True)
                    rospy.sleep(5)
                    question, _ = agent.stt(15)
                    question_list = question.lower().split()
                    newline_question = ""
                    for c in range(len(question_list) // 3 + 1):
                        newline_question += ' '.join(question_list[c * 3: (c + 1) * 3])
                        if c != len(question_list) // 3:
                            newline_question += '\n'
                    is_correct_q = False
                    agent.say("Is this your question? You said")
                    rospy.sleep(5)
                    agent.say(newline_question, show_display=True)
                    rospy.sleep(10)
                    agent.say('Please say yes\nif this is right\nor say no\nif this is wrong\nafter the ding sound')
                    rospy.sleep(10)
                    answer, _ = agent.stt(5.)
                    if 'yes' in answer:
                        is_correct_q = True
                        break

                if not is_correct_q:
                    agent.say('Cannot recognize question precisely\nI will answer as I understood', show_display=True)
                    rospy.sleep(7)

            common_words_num = []
            for q in questions:
                common_words_num.append(len(set(q.lower().split()).intersection(set(question_list))))
            right_question_idx = np.argmax(np.array(common_words_num))
            answer = answers[right_question_idx]
            answer_list = answer.split()
            newline_answer = ""
            for c in range(len(answer_list) // 3 + 1):
                newline_answer += ' '.join(answer_list[c * 3: (c + 1) * 3])
                if c != len(answer_list) // 3:
                    newline_answer += '\n'
            agent.say(newline_answer, show_display=True)
            print(newline_answer)
            rospy.sleep(15)

    agent.say("Task done!")
    rospy.sleep(5)
    agent.say('Wait a second\nto give me a new command', show_display=True)
    rospy.sleep(7)

def find_object_main(agent, command_dict):
    command_dict['target_obj'] = command_dict['target_obj'].replace(' ', '_')
    command_dict['target_obj'] = change_object_names(command_dict['target_obj'])
    command_dict['target_cat'] = change_object_types(command_dict['target_cat'])

    print(command_dict['target_obj'])
    print(command_dict['target_cat'])

    if command_dict['action'] in ['find', 'find_three']:
        placements = agent.location_map[command_dict['obj_pl']]
        count = 0
        for p in placements:
            if p in list(agent.table_dimension.keys()):
                agent.pose.move_pose()
                agent.move_abs_safe(p)
                rospy.sleep(2)
                if p in ['shelf', 'storage_rack', 'pantry']:
                    agent.pose.head_tilt(10)
                    rospy.sleep(2)
                    bbox_list = agent.yolo_module.yolo_bbox
                    for bbox in bbox_list:
                        if command_dict['target_cat'] != '':
                            bbox_type = agent.yolo_module.find_type_by_id(bbox[4])
                            bbox_type = agent.object_types[bbox_type]
                            bbox_name = agent.yolo_module.find_name_by_id(bbox[4])
                            if bbox_type in command_dict['target_cat']:
                                agent.say("{0} is here".format(bbox_name), show_display=True)
                                print("{0} is here".format(bbox_name))
                                count += 1
                                rospy.sleep(1)
                        else:
                            if command_dict['target_obj'] == agent.yolo_module.find_name_by_id(bbox[4]):
                                agent.say("{0} is here".format(command_dict['target_obj']), show_display=True)
                                print("{0} is here".format(command_dict['target_obj']))
                                count += 1
                                rospy.sleep(1)
                    agent.pose.head_tilt(-10)
                    rospy.sleep(2)
                    bbox_list = agent.yolo_module.yolo_bbox
                    for bbox in bbox_list:
                        if command_dict['target_cat'] != '':
                            bbox_type = agent.yolo_module.find_type_by_id(bbox[4])
                            bbox_type = agent.object_types[bbox_type]
                            bbox_name = agent.yolo_module.find_name_by_id(bbox[4])
                            if bbox_type in command_dict['target_cat']:
                                agent.say("{0} is here".format(bbox_name), show_display=True)
                                print("{0} is here".format(bbox_name))
                                count += 1
                                rospy.sleep(1)
                        else:
                            if command_dict['target_obj'] == agent.yolo_module.find_name_by_id(bbox[4]):
                                agent.say("{0} is here".format(command_dict['target_obj']), show_display=True)
                                print("{0} is here".format(command_dict['target_obj']))
                                count += 1
                                rospy.sleep(1)
                else:
                    if agent.table_dimension[p][2] > 0.8:
                        agent.pose.table_search_pose_high()
                    else:
                        agent.pose.table_search_pose()
                    rospy.sleep(2)
                    bbox_list = agent.yolo_module.yolo_bbox
                    for bbox in bbox_list:
                        if command_dict['target_cat'] != '':
                            bbox_type = agent.yolo_module.find_type_by_id(bbox[4])
                            bbox_type = agent.object_types[bbox_type]
                            bbox_name = agent.yolo_module.find_name_by_id(bbox[4])
                            if bbox_type in command_dict['target_cat']:
                                agent.say("{0} is here".format(bbox_name), show_display=True)
                                print("{0} is here".format(bbox_name))
                                count += 1
                                rospy.sleep(1)
                        else:
                            if command_dict['target_obj'] == agent.yolo_module.find_name_by_id(bbox[4]):
                                agent.say("{0} is here".format(command_dict['target_obj']), show_display=True)
                                print("{0} is here".format(command_dict['target_obj']))
                                count += 1
                                rospy.sleep(1)
                if command_dict['action'] == 'find':
                    if count >= 1:
                        break
                else:
                    if count >= 3:
                        break

    elif command_dict['action'] in ['three_what_prop', 'what_prop']:
        command_dict['obj_pl'] = command_dict['obj_pl'].replace(' ', '_')
        if command_dict['obj_pl'] == 'dishwasher':
            command_dict['obj_pl'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['obj_pl'])
        rospy.sleep(2)
        if command_dict['obj_pl'] in ['shelf', 'storage_rack', 'pantry']:
            agent.pose.head_tilt(10)
        else:
            if agent.table_dimension[command_dict['obj_pl']][2] > 0.8:
                agent.pose.table_search_pose_high()
            else:
                agent.pose.table_search_pose()
        rospy.sleep(2)

        if command_dict['action'] == 'what_prop':
            iter = 1
        else:
            iter = 3
        obj_list = np.array(agent.yolo_module.detect_3d(command_dict['obj_pl']))
        bbox_list = agent.yolo_module.yolo_bbox
        for i in range(iter):
            got_object = get_obj_by_size(agent, command_dict['prop'], bbox_list, obj_list, command_dict['target_cat'],
                                         task='find')
            if command_dict['target_cat'] != '':
                agent.say("The {0} {1} is {2}".format(command_dict['prop'], command_dict['target_cat'], got_object),
                          show_display=True)
                print("The {0} {1} is {2}".format(command_dict['prop'], command_dict['target_cat'], got_object))
                rospy.sleep(3)
            else:
                agent.say("The {0} object is {1}".format(command_dict['prop'], got_object), show_display=True)
                print("The {0} object is {1}".format(command_dict['prop'], got_object))
                rospy.sleep(3)
            for idx, obj in enumerate(obj_list):
                if obj[3] == agent.yolo_module.find_id_by_name(got_object):
                    obj_list = np.delete(obj_list, idx, 0)
            for idx, bbox in enumerate(bbox_list):
                if bbox[4] == agent.yolo_module.find_id_by_name(got_object):
                    bbox_list = np.delete(bbox_list, idx, 0)

    else:
        command_dict['obj_pl'] = command_dict['obj_pl'].replace(' ', '_')
        if command_dict['obj_pl'] == 'dishwasher':
            command_dict['obj_pl'] = 'dishwasher_gpsr'
        agent.move_abs_safe(command_dict['obj_pl'])
        rospy.sleep(2)
        if command_dict['obj_pl'] in ['shelf', 'storage_rack', 'pantry']:
            if len(command_dict['target_cat']) == 0:
                count = 0
                agent.pose.head_tilt(10)
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    if bbox[4] == agent.yolo_module.find_id_by_name(command_dict['target_obj']):
                        count += 1
                rospy.sleep(2)
                agent.pose.head_tilt(-10)
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    if bbox[4] == agent.yolo_module.find_id_by_name(command_dict['target_obj']):
                        count += 1
                rospy.sleep(2)
                agent.say(
                    'There are {0} {1}\nat the {2}'.format(count, command_dict['target_obj'], command_dict['obj_pl']),
                    show_display=True)
                print('There are {0} {1}\nat the {2}'.format(count, command_dict['target_obj'], command_dict['obj_pl']))
            else:
                count = 0
                agent.pose.head_tilt(10)
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    bbox_type = agent.yolo_module.find_type_by_id(bbox[4])
                    bbox_type = agent.object_types[bbox_type]
                    if bbox_type in command_dict['target_cat']:
                        count += 1
                rospy.sleep(2)
                agent.pose.head_tilt(-10)
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    bbox_type = agent.yolo_module.find_type_by_id(bbox[4])
                    bbox_type = agent.object_types[bbox_type]
                    if bbox_type in command_dict['target_cat']:
                        count += 1
                agent.say('There are {0} {1}\nat the {2}'.format(count, command_dict['target_cat'], command_dict['obj_pl']),
                          show_display=True)
                print('There are {0} {1}\nat the {2}'.format(count, command_dict['target_cat'], command_dict['obj_pl']))
        else:
            if agent.table_dimension[command_dict['obj_pl']][2] > 0.8:
                agent.pose.table_search_pose_high()
            else:
                agent.pose.table_search_pose()
            rospy.sleep(2)

            if len(command_dict['target_cat']) == 0:
                count = 0
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    if bbox[4] == agent.yolo_module.find_id_by_name(command_dict['target_obj']):
                        count += 1
                agent.say('There are {0} {1}\nat the {2}'.format(count, command_dict['target_obj'], command_dict['obj_pl']),
                          show_display=True)
                print('There are {0} {1}\nat the {2}'.format(count, command_dict['target_obj'], command_dict['obj_pl']))
            else:
                count = 0
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    bbox_type = agent.yolo_module.find_type_by_id(bbox[4])
                    bbox_type = agent.object_types[bbox_type]
                    if bbox_type in command_dict['target_cat']:
                        count += 1
                agent.say('There are {0} {1}\nat the {2}'.format(count, command_dict['target_cat'], command_dict['obj_pl']),
                          show_display=True)
                print('There are {0} {1}\nat the {2}'.format(count, command_dict['target_cat'], command_dict['obj_pl']))
    rospy.sleep(2)
    agent.say("Task done!")
    rospy.sleep(5)
    agent.say('Wait a second\nto give me a new command', show_display=True)
    rospy.sleep(7)


def parse_command(command, command_type):
    common_words_num = []
    for cmd_type in command_type:
        common_words_num.append(len(set(cmd_type).intersection(set(command))))
    right_command_type_idx = np.argmax(np.array(common_words_num))
    if right_command_type_idx == 31:
        if 'person' in command:
            pass
        else:
            right_command_type_idx = 34
    elif right_command_type_idx == 19:
        if len(command) > 10:
            pass
        else:
            right_command_type_idx = 34
    return right_command_type_idx

def change_object_names(name):
    if name == 'cleanser':
        name = 'scrub'
    elif name == 'tennisball':
        name = 'tennis_ball'
    elif name == 'soccerball':
        name = 'soccer_ball'
    elif name == 'baseball':
        name = 'base_ball'
    elif name in ['rubiks_cube', 'rubikscube']:
        name = 'cube'
    elif name == 'cup':
        name = 'mug'
    elif name in ['redwine', 'red_wine']:
        name = 'wine'
    elif name == 'cola':
        name = 'coke'
    elif name == 'juicepack':
        name = 'juice_pack'
    elif name == 'iced_tea':
        name = 'ice_tea'
    elif name == 'tuna':
        name = 'tuna_can'
    elif name == 'strawberry_jello':
        name = 'jello_red'
    elif name == 'chocolate_jello':
        name = 'jello_black'
    elif name in ['coffeegrounds', 'coffee_grounds']:
        name = 'coffee_can'
    elif name in ['cornflakes', 'corn_flakes']:
        name = 'cereal_red'
    elif name in ['cheezeit', 'cheeze']:
        name = 'cracker'

    return name

def change_object_types(type):
    if type == 'cleaning supplies':
        type = 'cleaning'
    elif type == 'toys':
        type = 'toy'
    elif type == 'dishes':
        type = 'dish'
    elif type == 'drinks':
        type = 'drink'
    elif type == 'fruits':
        type = 'fruit'
    elif type == 'snacks':
        type = 'snack'

    return type


def gpsr(agent):
    is_test = False
    if is_test:
        test_task = input("Give me the task to test : ")
        if test_task == 'pick_shelf':
            agent.move_abs('shelf')
            dist_initial_forward = distancing(agent.yolo_module.pc, 'shelf', dist=0.9)
            agent.move_rel(dist_initial_forward, 0, wait=True)
            agent.pose.head_tilt(10)
            rospy.sleep(2)
            obj_list = np.array(agent.yolo_module.detect_3d('shelf'))
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, 'ice tea')
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, 0)

            if base_xyz[2] < 1:
                agent.pose.pick_shelf_pose('shelf_1f', shelf_grasp_z_offset)
            else:
                agent.pose.pick_shelf_pose('shelf_2f', shelf_grasp_z_offset)

            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0] + shelf_grasp_x_offset, 0, wait=True)
            agent.move_rel(-0.5, 0, wait=True)
            agent.pose.move_pose()

        elif test_task == 'pick_table':
            agent.move_abs('kitchen_table')
            dist_initial_forward = distancing(agent.yolo_module.pc, 'kitchen_table')
            agent.move_rel(dist_initial_forward, 0, wait=True)
            agent.pose.table_search_pose()

            obj_list = np.array(agent.yolo_module.detect_3d('kitchen_table'))
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, 'apple')
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, 0)

            agent.move_rel(-0.15, 0, wait=True)
            agent.pose.pick_side_pose('kitchen_table')
            agent.open_gripper()
            agent.move_rel(0, base_xyz[1], wait=True)
            agent.move_rel(base_xyz[0] + 0.15 + table_grasp_x_offset, 0, wait=True)
            agent.grasp()
            agent.move_rel(-0.3, 0, wait=True)
            agent.pose.move_pose()

        elif test_task == 'pick_table_spoon':
            agent.move_abs('kitchen_table')
            dist_initial_forward = distancing(agent.yolo_module.pc, 'kitchen_table')
            agent.move_rel(dist_initial_forward, 0, wait=True)
            agent.pose.table_search_pose()

            obj_list = np.array(agent.yolo_module.detect_3d('kitchen_table'))
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, 'fork')
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, 0)

            agent.move_rel(-0.15, 0, wait=True)
            agent.pose.pick_top_pose(table='kitchen_table')
            agent.open_gripper()
            agent.move_rel(base_xyz[0] + 0.35, base_xyz[1] - 0.01, wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.005, table='kitchen_table')
            agent.grasp()
            agent.pose.arm_flex(-60)
            agent.move_rel(-0.4, 0)
            agent.pose.move_pose()

        elif test_task == 'pick_table_bowl':
            agent.move_abs('kitchen_table')
            dist_initial_forward = distancing(agent.yolo_module.pc, 'kitchen_table')
            agent.move_rel(dist_initial_forward, 0, wait=True)
            agent.pose.table_search_pose()

            obj_list = np.array(agent.yolo_module.detect_3d('kitchen_table'))
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, 'bowl')
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, 0)

            agent.move_rel(-0.15, 0, wait=True)
            agent.pose.bring_bowl_pose(table='kitchen_table')
            agent.open_gripper()
            agent.move_rel(base_xyz[0] + 0.25, base_xyz[1] + 0.04, wait=True)
            agent.pose.pick_bowl_max_pose(table='kitchen_table', height=-0.12)
            agent.grasp()
            rospy.sleep(0.5)
            agent.pose.pick_up_bowl_pose(table='kitchen_table')
            agent.move_rel(-0.4, 0)
            agent.pose.move_pose()

        elif test_task == 'pick_table_plate':
            agent.move_abs('kitchen_table')
            dist_initial_forward = distancing(agent.yolo_module.pc, 'kitchen_table')
            agent.move_rel(dist_initial_forward, 0, wait=True)
            agent.pose.table_search_pose()

            obj_list = np.array(agent.yolo_module.detect_3d('kitchen_table'))
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, 'plate')
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, 0)

            plate_radius = 0.08
            base_to_arm_dist = 0.5

            dist_to_base_x = distancing(agent.yolo_module.pc, 'kitchen_table', raw=True)
            plate_xyz = base_xyz
            dist_plate_x = plate_xyz[0] - (dist_to_base_x - base_to_arm_dist) + plate_radius
            print('dist_plate_x', dist_plate_x)

            agent.move_rel(-0.15, 0, wait=True)

            agent.pose.pick_plate_pose(table='kitchen_table')
            agent.open_gripper()
            agent.move_rel(plate_xyz[0] + 0.18, plate_xyz[1], wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.025, table='kitchen_table')
            agent.move_rel(-dist_plate_x - 0.06, 0, wait=True)
            agent.pose.arm_lift_top_table_down(height=0.05, table='kitchen_table')
            agent.move_rel(-0.18, 0, wait=True)
            agent.pose.pick_plate_pose_fold(table='kitchen_table')
            agent.move_rel(0.04, 0, wait=True)  # slightly move forward
            agent.grasp()
            agent.pose.move_pose()

        elif test_task == 'place_shelf':
            agent.move_abs('shelf')
            agent.pose.place_shelf_pose('shelf_1f')
            shelf_base_to_object_xyz = [gripper_to_shelf_x, 0, 0]
            shelf_base_xyz = agent.yolo_module.calculate_dist_to_pick(shelf_base_to_object_xyz, 4)
            agent.move_rel(shelf_base_xyz[0], shelf_base_xyz[1], wait=True)
            agent.open_gripper()
            agent.move_rel(-0.5, 0)
            agent.pose.move_pose()

        elif test_task == 'place_table':
            agent.move_abs('kitchen_table')
            dist_to_table = distancing(agent.yolo_module.pc, 'kitchen_table')
            agent.move_rel(dist_to_table, 0, wait=True)
            agent.move_rel(-0.2, 0, wait=True)
            rospy.sleep(2)
            agent.pose.place_side_pose('kitchen_table')
            rospy.sleep(2)
            agent.move_rel(0.2 + table_place_x_offset, 0, wait=True)
            rospy.sleep(2)
            agent.open_gripper()
            agent.move_rel(-0.5, 0, wait=True)
            agent.pose.move_pose()

    agent.door_open()
    agent.say('start gpsr')
    rospy.sleep(3)

    command_count = 0
    while True:
        agent.pose.move_pose()
        agent.move_abs_safe('gpsr_start')
        agent.pose.head_tilt(10)
        if not command_keyboard_mode:
            for j in range(5):
                agent.say('please give me the \ncommand after the \nding sound', show_display=True)
                rospy.sleep(5)
                command = agent.stt(10.)
                raw_command, _ = command
                print(raw_command)
                if len(raw_command) == 0:
                    continue
                command_list = raw_command.split()
                newline_command = ""
                for c in range(len(command_list) // 3 + 1):
                    newline_command += ' '.join(command_list[c * 3: (c + 1) * 3])
                    if c != len(command_list) // 3:
                        newline_command += '\n'
                rospy.sleep(3)
                agent.say('is this your command? you said')
                rospy.sleep(5)
                agent.say(newline_command, show_display=True)
                rospy.sleep(7)
                agent.say('please say yes or no\nafter the ding sound', show_display=True)
                rospy.sleep(4)
                answer, _ = agent.stt(5.)
                if 'yes' in answer:
                    agent.say('now i will carry out the task')
                    break
                elif 'no' in answer:
                    continue
                else:
                    while 'yes' not in answer and 'no' not in answer:
                        agent.say('please answer me\nonly by yes or no\nafter the ding sound', show_display=True)
                        rospy.sleep(5)
                        answer, _ = agent.stt(5.)
                    if 'yes' in answer:
                        agent.say('now i will carry out the task')
                        rospy.sleep(3)
                        break
        else:
            raw_command = input("Please give me a command : ")

        raw_command = raw_command.lower().split()
        right_command_type_idx = parse_command(raw_command, command_type)
        print(right_command_type_idx)

        impossible_commands = [9, 21, 22, 23, 24]
        manipulation_commands = list(range(14))
        hri_commands = list(range(14, 33))
        find_object_commands = list(range(33, 38))

        if right_command_type_idx in impossible_commands:
            if not command_keyboard_mode:
                rospy.sleep(2)
                # agent.say('sorry\nI have no ability to carry out the task\nplease do it by your self', show_display=True)
                # rospy.sleep(7)
                # answer = ''
                # while 'done' not in answer:
                #     agent.say('please complete the task by yourself')
                #     rospy.sleep(4)
                #     agent.say('please say "I am done"\nwhen you are done\nafter the ding sound', show_display=True)
                #     rospy.sleep(6)
                #     answer, _ = agent.stt(10.)
                # command_count += 1
                places = []
                for word in raw_command:
                    if [word] in rooms or [word] in locations:
                        places.append(word)

                first_place = ''
                if len(places) == 0:
                    pass
                else:
                    if len(places) > 1:
                        if [places[1]] in rooms_sub_words or [places[1]] in locations_sub_words:
                            first_place = places[0] + '_' + places[1]
                        else:
                            first_place = places[0]
                    else:
                        first_place = places[0]
                    rospy.sleep(3)
                    agent.move_abs_safe(first_place)
                    rospy.sleep(3)

                agent.say("sorry\nI cannot do the task\nanymore", show_display=True)
                rospy.sleep(5)
                agent.say("Please do the remaining task\nby yourself", show_display=True)
                rospy.sleep(5)
                answer = ''
                is_done = False
                for i in range(3):
                    agent.say('please complete the task by yourself')
                    rospy.sleep(4)
                    agent.say('please say "I am done"\nwhen you are done\nafter the ding sound', show_display=True)
                    rospy.sleep(6)
                    answer, _ = agent.stt(10.)
                    if 'done' in answer:
                        is_done = True
                        break
                if not is_done:
                    agent.say("I will assume that\nYou have done the task", show_display=True)
                    rospy.sleep(5)
                agent.say('task done')
                rospy.sleep(5)
                command_count += 1
            else:
                print("This command is impossible.")
                command_count += 1
        else:
            command_dict = utils.parse_gpsr_command.make_dict(right_command_type_idx, raw_command)
            try:
                condition_1 = len(command_dict['target_obj']) == 0 and len(command_dict['target_cat']) == 0 and \
                              'object' not in raw_command
                if right_command_type_idx in manipulation_commands:
                    condition_2 = 'object' in raw_command and 'most' not in raw_command and len(command_dict['ref']) == 0
                    if condition_1 or condition_2:
                        print('using qr..')
                        pass  # TODO: qr
                elif right_command_type_idx in find_object_commands:
                    if condition_1:
                        print('using qr..')
                        pass  # TODO: QR
            except:
                pass
            places = []
            for word in raw_command:
                if [word] in rooms or [word] in locations:
                    places.append(word)
            if 'bookshelf' in places or 'potted plant' in places:
                agent.say("sorry, I cannot do\nthis task", show_display=True)
                rospy.sleep(5)
                continue
            try:
                if right_command_type_idx in manipulation_commands:
                    if right_command_type_idx in [4, 10]:
                        manipulation_other(agent, command_dict)
                    else:
                        manipulation_main(agent, command_dict)
                elif right_command_type_idx in hri_commands:
                    hri_main(agent, command_dict)
                elif right_command_type_idx in find_object_commands:
                    find_object_main(agent, command_dict)
                command_count += 1
            except Exception as e:
                print(e)
                rospy.sleep(2)
                agent.say("I cannot understand the command\nor I cannot carry out the task\n", show_display=True)
                rospy.sleep(7)
                agent.say("give me another command", show_display=True)
                rospy.sleep(2)
                continue
            # if right_command_type_idx in manipulation_commands:
            #     if right_command_type_idx in [4, 10]:
            #         manipulation_other(agent, command_dict)
            #     else:
            #         manipulation_main(agent, command_dict)
            # elif right_command_type_idx in hri_commands:
            #     hri_main(agent, command_dict)
            # elif right_command_type_idx in find_object_commands:
            #     find_object_main(agent, command_dict)
            # command_count += 1

    # for test
    # command = input("Please give me a command : ").lower().split()

    # command_without_comma = []
    # for word in command:
    #     if ',' in word:
    #         word = word.replace(',', '')
    #     command_without_comma.append(word)
    #
    # right_command_type_idx = parse_command(command_without_comma, command_type)
    # command_dict = utils.manipulation_parse.make_dict(right_command_type_idx, command_without_comma)
    # manipulation_main(agent, command_dict)