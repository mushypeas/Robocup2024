import os
is_sim = 'localhost' in os.environ['ROS_MASTER_URI']
import rospy
import numpy as np
from utils.gpsr_configs import *


shelf_grasp_x_offset = 0.15
shelf_grasp_x_offset_ni = 0.05
shelf_place_x_offset = 0.35
table_grasp_x_offset = 0.01
table_place_x_offset = 0.10

shelf_grasp_z_offset = 0.115

top_grasping_objects = ['fork', 'spoon', 'knife']

text_mode = False

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

def judge_available_command(command):
    polite = ['could', 'you', 'please', 'robot']
    new_command = []
    for word in command:
        if word not in polite:
            new_command.append(word)
    command = new_command
    print(command)
    find_object = ['find', 'locate', 'spot', 'pinpoint', 'look', 'for', 'in', 'the']
    if len(set(command).intersection(set(find_object))) >= 3:
        if len(set(command) - set(find_object)) <= 4:
            return 'find_object'

    rooms = ['living', 'bedroom', 'study', 'kitchen']
    if command[0] in ['guide', 'escort', 'take', 'lead', 'accompany', 'conduct']:
        if len(set(command).intersection(set(rooms))) >= 1:
            return 'guide_person'

    if command[0] == 'bring' and command[1] == 'the':
        return 'bring_object'

    return 'impossible'

def find_object(agent, command):
    target_obj = []
    for word in command:
        if [word] in objects:
            target_obj.append(word)
    target_obj = '_'.join(target_obj)
    target_obj = change_object_names(target_obj)

    target_pl = []  # room to go
    for word in command:
        if [word] in rooms:
            target_pl.append(word)
    target_pl = '_'.join(target_pl)
    obj_pl = ''  # where object is

    print('target object : ', target_obj)
    print('target place : ', target_pl)

    placements = agent.location_map[target_pl]
    is_here = False
    # TODO: dishwasher
    for p in placements:
        if p in list(agent.table_dimension.keys()):
            agent.pose.move_pose()
            agent.move_abs(p)
            rospy.sleep(2)
            if p in ['shelf', 'storage_rack', 'pantry']:
                agent.pose.head_tilt(10)
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    if target_obj == agent.yolo_module.find_name_by_id(bbox[4]):
                        target_obj = target_obj.replace('_', ' ')
                        print(target_obj)
                        agent.say("{0} is here".format(target_obj), show_display=True)
                        print("{0} is here".format(target_obj))
                        is_here = True
                        break
                if is_here:
                    obj_pl = p
                    break
                agent.pose.head_tilt(-10)
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    if target_obj == agent.yolo_module.find_name_by_id(bbox[4]):
                        target_obj = target_obj.replace('_', ' ')
                        agent.say("{0} is here".format(target_obj), show_display=True)
                        print("{0} is here".format(target_obj))
                        is_here = True
                        break
                if is_here:
                    obj_pl = p
                    break
            else:
                if agent.table_dimension[p][2] >= 0.8:
                    agent.pose.table_search_pose_high()
                else:
                    agent.pose.table_search_pose()
                rospy.sleep(2)
                bbox_list = agent.yolo_module.yolo_bbox
                for bbox in bbox_list:
                    if target_obj == agent.yolo_module.find_name_by_id(bbox[4]):
                        target_obj = target_obj.replace('_', ' ')
                        agent.say("{0} is here".format(target_obj), show_display=True)
                        print("{0} is here".format(target_obj))
                        is_here = True
                        break
                if is_here:
                    obj_pl = p
                    break

    agent.move_abs('gpsr_start')
    if len(obj_pl) == 0:
        target_obj = target_obj.replace('_', ' ')
        agent.say("I cannot find the {0}".format(target_obj))
        rospy.sleep(5)
    else:
        obj_pl = obj_pl.replace('_', ' ')
        agent.say("{0} is at the {1}".format(target_obj, obj_pl))
        rospy.sleep(5)

    agent.say('task done')
    rospy.sleep(3)


def guide_person(agent, command):
    for idx, word in enumerate(command):
        if word == 'in':
            second_idx = idx
    target_person = command[2: second_idx]
    target_person = ' '.join(target_person)

    places = []
    for word in command:
        if [word] in rooms or word == 'exit':
            places.append(word)
    print(places)

    if len(places) == 2:
        first_place = places[0]
        second_place = places[1]
    elif len(places) == 3:
        if places[1] == 'room':
            first_place = places[0] + '_' + 'room'
            second_place = places[2]
        else:
            first_place = places[0]
            second_place = places[1] + '_' + 'room'
    else:
        first_place = places[0] + '_' + places[1]
        second_place = places[2] + '_' + places[3]

    print(first_place)
    print(second_place)
    agent.move_abs(first_place)
    for i in range(3):
        agent.say("{0}\nplease come here\nand wait".format(target_person), show_display=True)
        rospy.sleep(10)
        agent.say("Say 'I am ready'\nwhen you are ready\nto follow me\nafter the ding sound", show_display=True)
        rospy.sleep(8)
        answer, _ = agent.stt(5.)
        if 'ready' in answer:
            break
    agent.move_abs(second_place)
    agent.say('task done')
    rospy.sleep(3)


def bring_object(agent, command):
    target_obj = []
    for word in command:
        if [word] in objects:
            target_obj.append(word)
    target_obj = '_'.join(target_obj)
    target_obj = change_object_names(target_obj)
    print(target_obj)

    the_idxs = []
    for idx, word in enumerate(command):
        if word == 'the':
            the_idxs.append(idx)
        elif word == 'in':
            second_idx = idx
    first_idx = the_idxs[1]
    print(first_idx)
    print(second_idx)
    target_person = command[first_idx + 1: second_idx]
    target_person = ' '.join(target_person)

    place = []
    for word in command:
        if [word] in rooms:
            place.append(word)
    second_place = '_'.join(place)
    print(second_place)

    if not text_mode:
        is_correct_place = False
        for i in range(5):
            rospy.sleep(2)
            agent.say('Please tell me\nwhere the object is\nonly by word\nafter the ding sound', show_display=True)
            rospy.sleep(8)
            placement, _ = agent.stt(7.)
            agent.say("The placement you said is\n{0}".format(placement), show_display=True)
            rospy.sleep(4)
            agent.say('Is this right?\nplease say yes or no\nafter the ding sound', show_display=True)
            rospy.sleep(5)
            answer, _ = agent.stt(5)
            if 'no' in answer:
                continue
            placement = placement.lower().split()
            if len(placement) <= 2:
                if [placement[0]] in rooms:
                    agent.say('please give me\nthe name of the precise place,\nnot just room')
                    rospy.sleep(4)
                    continue
                else:
                    if [placement[0]] in locations:
                        first_place = '_'.join(placement)
                        is_correct_place = True
                        break
                    else:
                        if i < 4:
                            agent.say('please give me the\ncorrect name of the placement', show_display=True)
                            rospy.sleep(4)
            else:
                if i < 4:
                    agent.say('please give me\nonly the name,\nnot by sentence', show_display=True)
                    rospy.sleep(4)

        if not is_correct_place:
            agent.say('I cannot understand\nyour word\nsorry', show_display=True)
            rospy.sleep(4)
            agent.say('please wait to give me\nanother command', show_display=True)
            rospy.sleep(6)
    else:
        first_place = input("give me the placement")
        first_place = first_place.split()
        first_place = '_'.join(first_place)


    agent.move_abs(first_place)

    if first_place in ['shelf', 'storage_rack', 'pantry']:
        agent.pose.head_tilt(10)
        rospy.sleep(2)
        obj_list = np.array(agent.yolo_module.detect_3d(first_place))
        base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, target_obj)
        print(base_to_obj)

        if base_to_obj is None:
            agent.pose.head_tilt(-10)
            rospy.sleep(2)
            obj_list = np.array(agent.yolo_module.detect_3d(first_place))
            base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, target_obj)
    else:
        if agent.table_dimension[first_place][2] >= 0.8:
            agent.pose.table_search_pose_high()
        else:
            agent.pose.table_search_pose()
        rospy.sleep(2)

        obj_list = np.array(agent.yolo_module.detect_3d(first_place))
        base_to_obj = agent.yolo_module.find_3d_points_by_name(obj_list, target_obj)
        print(base_to_obj)

    pick_function(agent, target_obj, first_place, base_to_obj)
    agent.pose.move_pose()

    agent.move_abs(second_place)

    if not text_mode:
        for i in range(3):
            agent.say('{0}\nplease come here\nand wait'.format(target_person), show_display=True)
            rospy.sleep(7)
            agent.say("please say 'yes'\nwhen you are here\nafter the ding sound", show_display=True)
            rospy.sleep(7)
            answer, _ = agent.stt(5)
            if 'yes' in answer:
                break
            if i == 2:
                agent.say("I will assume that\nyou are here", show_display=True)
                rospy.sleep(4)
    else:
        print('{0} please come here\nand wait'.format(target_person))

    agent.say('Please move back\nBecause I will\nstretch out my arm', show_display=True)
    rospy.sleep(5)
    agent.pose.place_side_pose('kitchen_table')
    agent.say('I will open my gripper\nplease receive it', show_display=True)
    rospy.sleep(5)
    agent.open_gripper()

    agent.say('task done')
    rospy.sleep(3)

def pick_function(agent, object, object_place, base_to_obj):
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
        # TODO: clean the table
        if object in top_grasping_objects:
            grasping_type = 1
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
            # from ctt
            agent.pose.pick_top_pose(table=object_place)
            agent.open_gripper()
            agent.move_rel(base_xyz[0] + 0.25, base_xyz[1] - 0.01, wait=True)
            agent.pose.arm_lift_top_table_down(height=-0.005, table=object_place)
            agent.grasp()
            rospy.sleep(1)
            agent.pose.arm_flex(-60)
            agent.move_rel(-0.4, 0)
        # TODO: clean the table
        elif object == 'bowl':
            grasping_type = 2
            base_xyz = agent.yolo_module.calculate_dist_to_pick(base_to_obj, grasping_type)
            # from ctt
            agent.pose.pick_bowl_pose(table=object_place)
            agent.open_gripper()
            agent.move_rel(base_xyz[0] + 0.15, base_xyz[1] + 0.04, wait=True)
            agent.pose.pick_bowl_max_pose(table=object_place, height=-0.1)
            agent.grasp()
            rospy.sleep(1)
            rospy.sleep(0.5)
            agent.pose.pick_up_bowl_pose(table=object_place)
            agent.move_rel(-0.4, 0)
        # TODO: clean the table
        elif object == 'plate':
            agent.move_rel(-0.2, 0)
            agent.say("please hand me the plate", show_display=True)
            rospy.sleep(2)
            agent.pose.pick_plate_pose_fold(object_place)
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


def egpsr(agent):
    agent.door_open()

    agent.say('start egpsr')
    rospy.sleep(3)

    while True:
        agent.pose.move_pose()
        agent.move_abs('gpsr_start')
        if not text_mode:
            for j in range(5):
                agent.pose.head_tilt(10)
                agent.say('please give me the \ncommand after the \nding sound', show_display=True)
                rospy.sleep(5)
                command = agent.stt(13.)
                raw_command, _ = command
                print(raw_command)
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
                agent.say('please say yes or no, after the ding sound')
                rospy.sleep(5)
                answer, _ = agent.stt(5.)
                if 'yes' in answer:
                    agent.say('now i will carry out the task')
                    break
                elif 'no' in answer:
                    continue
                else:
                    while 'yes' not in answer and 'no' not in answer:
                        agent.say('please answer me only by yes or no, after the ding sound')
                        rospy.sleep(5)
                        answer, _ = agent.stt(5.)
                    if 'yes' in answer:
                        agent.say('now i will carry out the task')
                        break
        else:
            raw_command = input("Please give me a command : ").lower().split()

        if not text_mode:
            raw_command = raw_command.lower().split()
        command_type = judge_available_command(raw_command)
        print(command_type)
        polite = ['could', 'you', 'please', 'robot']
        new_command = []
        for word in raw_command:
            if word not in polite:
                new_command.append(word)
        raw_command = new_command
        try:
            if command_type == 'find_object':
                find_object(agent, raw_command)
            elif command_type == 'guide_person':
                guide_person(agent, raw_command)
            elif command_type == 'bring_object':
                bring_object(agent, raw_command)
            else:
                places = []
                for word in raw_command:
                    if [word] in rooms or [word] in locations:
                        places.append(word)
                first_place = ''
                if len(places) == 0:
                    pass
                else:
                    put_idx = 100
                    for idx, word in enumerate(raw_command):
                        if word in ['put', 'place', 'leave', 'set']:
                            put_idx = idx
                        if word == places[0]:
                            place_idx = idx
                    if put_idx < place_idx:
                        pass
                    else:
                        if len(places) > 1:
                            if [places[1]] in rooms_sub_words or [places[1]] in locations_sub_words:
                                    first_place = places[0] + '_' + places[1]
                            else:
                                first_place = places[0]
                        else:
                            first_place = places[0]
                        if first_place == 'bookshelf':
                            pass
                        else:
                            rospy.sleep(3)
                            agent.move_abs(first_place)
                            rospy.sleep(3)

                print('sorry\nI cannot do the task\nanymore')
                agent.say("sorry\nI cannot do the task\nanymore", show_display=True)
                rospy.sleep(5)
                agent.say("Please do the remaining task\nby yourself", show_display=True)
                rospy.sleep(5)
                if not text_mode:
                    answer = ''
                    for i in range(3):
                        agent.say('please complete the task by yourself')
                        rospy.sleep(4)
                        agent.say('please say "I am done"\nwhen you are done\nafter the ding sound', show_display=True)
                        rospy.sleep(6)
                        answer, _ = agent.stt(10.)
                        if 'done' in answer:
                            break
                        if i == 2:
                            agent.say("I will assume\nthat you have done\nthe task", show_display=True)
                            rospy.sleep(5)

                agent.say('task done')


                rospy.sleep(5)
        except:
            pass