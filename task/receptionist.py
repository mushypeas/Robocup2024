import numpy as np
import pandas as pd
import rospy
from module.human_attribute.extract_attribute import Attribute
from module.human_attribute.check_seat import CheckSeat
from module.human_attribute.qrcode import decoder_loop
from module.human_attribute.face_attribute.face_attr import FaceAttribute
from playsound import playsound
# from module.human_attribute.xtion import Xtion_camera

#1. STT test
#2. cloth guideline display
#3. sofa scan->turn around-> stand behind me+ name


# open door X
# def door_handle_detection(agent):
#     depth_img = agent.depth_for_pc
#     # print(depth_img[290])

#     depth_img[np.where(depth_img == 0)] = 1500
#     door_min = np.min(depth_img[250:330, 200:500])
#     print(door_min)
#     print(np.max(depth_img[250:330, 200:500]))

#     handle = []
#     for y in range(220, 350):
#         for x in range(200, 500):
#             if depth_img[y][x] < door_min + 20:
#                 handle.append([x, y])
#     print(len(handle))
#     # for idx in handle:
#     #     cv2.circle(rgb_img, idx, 1, (255, 255, 0))

#     most_left = np.argmin(handle, axis=0)[0]
#     most_right = np.argmax(handle, axis=0)[0]
#     center = [(most_left)]

#     return handle


def receptionist(agent):
    ### task params #################
    # door_position = 'door_handle'
    door_position = 'cloth_scan'
    door_bypass_position = 'door_bypass'
    cloth_position = 'cloth_scan'
    scan_position = 'seat_scan'
    scan_bypass_position = 'seat_scan_bypass'
    start_position = 'start'
    # start_position = 'start_receptionist' # AIIS

    open_door_mode = False
    calibration_mode = False

    cloth_threshold = 0.15
    face_threshold = 40 #38
    face_threshold2 = 38 #35

    ##TODO
    sofa_range = [130, 580]
    door_range = 100
    # most important to make sure that the non-sofa people and sofa people aren't at the same view.
    # calibrate the distance to make sure the upper condition and then calibrate the head pan angle.
    # closer the better to the sofa to cut the sofa side view


    #150cm setting
    sofa_point_angle = 20
    chair_point_angle = 60
    head_pan_angle = [chair_point_angle-8, sofa_point_angle-3, -sofa_point_angle, -chair_point_angle]

    # 150cm setting, sofa side view is visible
    # head_pan_angle = [60, 25, 0, -25, -60]
    # point_seat_angle = 25

    # 90cm~100cm setting currently the best setting
    # head_pan_angle = [50, 25, 0, -25, -50]
    # point_seat_angle = 25


    face_list = pd.read_csv("./module/human_attribute/face_detection/data.csv")
    name_list = ['adel', 'angel', 'axel', 'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone']
    drink_list = ['red wine', 'juice pack', 'cola', 'tropical juice', 'milk', 'iced tea', 'orange juice']

    name_host = 'john'
    drink_host = 'milk'
    if calibration_mode:
        name1 = 'juno'
        drink1 = 'lemonade'
        name2 = 'matilda'
        drink2 = 'mango milk'
        clothes = 'white no pattern woman spring vest no sleeves'
        gender = ['male']
        age = '20-29'

    # cap = Xtion_camera()
    attr = Attribute(cloth_threshold, calibration_mode)
    cs = CheckSeat(face_threshold, face_threshold2, sofa_range,door_range, head_pan_angle, sofa_point_angle, calibration_mode)
    face_attr = FaceAttribute()


    ### main scenario ########################
    agent.pose.move_pose()
    agent.move_abs_safe(start_position)
    agent.say('start receptionist')
    input('#################### Debug 1 ####################')

    # open door X
    # if open_door_mode:
    #     # 1. go to door position
    #     agent.grasp()
    #     agent.pose.move_pose()
    #     # agent.move_abs_safe(door_bypass_position)
    #     agent.move_abs_safe(door_position)
    #     agent.pose.head_tilt(10)

    #     # 2. open the door
    #     # handle_xyz = door_handle_detection(agent)
    #     rospy.sleep(1)
    #     agent.open_gripper()
    #     agent.pose.door_open_pose()
    #     # agent.move_abs_safe(pick_position)
    #     agent.grasp()
    #     agent.pose.door_handle_down_pose() # 10cm position add
    #     agent.move_rel(-0.5, 0, wait=True)
    #     rospy.sleep(1)
    #     agent.open_gripper()
    #     agent.pose.move_pose()
    #     agent.grasp()
    #     # # agent.open_gripper()

    # 3. meet the first guest

    # 3-1. Get first guest information
    agent.pose.move_pose()
    agent.move_abs_safe(cloth_position)
    rospy.sleep(8)

    agent.pose.head_tilt(10)
    # agent.say('Hello, I am tidy boy.\n Please stand to the guideline', show_display=True)
    agent.say('Hello, I am tidy boy.\n Please look at my face and\n stand to the guideline', show_display=True)
    rospy.sleep(3.5)

    try:
        gender, age = face_attr.face_attribute(agent)
    except:
        gender = ['male']
        age = '20-29'
    print('receptionist gender,age: ', gender, age)
    rospy.sleep(1)
    input('#################### Debug 2 ####################')

    agent.pose.head_tilt(-7.5)
    if calibration_mode:
        attr.cloth_extract_calibration_mode(agent)
    clothes, hair_color = attr.scan_human(agent)
    print('receptionist clothes: ', clothes)
    agent.pose.head_tilt(10)
    input('#################### Debug 3 ####################')

    # First guest STT
    # qr_check = True
    qr_check = False
    agent.say('I will ask your \nname and drink.', show_display=True)
    rospy.sleep(2.5)
    if not calibration_mode:
        name1, drink1 = '_', '_'
        for _ in range(10):
            agent.say('Come very close to me\n and answer after \nthe ring sound', show_display=True)
            rospy.sleep(4)

            name1, drink1 = '_', '_'
            
            if name1.lower() not in name_list:
                agent.say('What is your name?\n Say only name word.', show_display=True)
                rospy.sleep(3)
                first_raw_name = agent.stt(3, mode='name')
                # first_raw_name = agent.stt(7, mode=None)
                name1, _ = first_raw_name
                print('receptionist name1: ', name1)
                if name1 == '':
                    agent.say('Sorry, voice not recognized.', show_display=True)
                    rospy.sleep(1.5)
                    continue
    
            if drink1.lower() not in drink_list:
                agent.say('What is your favorite drink?\n Say only drink word.', show_display=True)
                rospy.sleep(3.5)
                first_raw_drink = agent.stt(3, mode='drink')
                # first_raw_drink = agent.stt(7, mode=None)
                drink1, _ = first_raw_drink
                print('receptionist drink1: ', drink1)
                if drink1 == '':
                    agent.say('Sorry, voice not recognized.', show_display=True)
                    rospy.sleep(1.2)
                    continue

            agent.say(f'Is this correct?\n ({name1}, {drink1}) \nsay yes or no', show_display=True)
            rospy.sleep(4.5)
    
            answer, _ = agent.stt(3, mode='yesno')
            if 'yes' in answer:
                qr_check = False
                break
            elif 'no' in answer:
                continue
            else:
                agent.say('Answer only by \nyes or no', show_display=True)
                rospy.sleep(2.5)
                answer, _ = agent.stt(3, mode='yesno')
                if 'yes' in answer:
                    qr_check = False
                    break
                elif 'no' in answer:
                    continue

    if qr_check:
        agent.say('Sorry.\nShow me the QR code', show_display=True)
        rospy.sleep(1.2)
        # read qrcode
        qr_str = decoder_loop(agent)
        data = qr_str.split(',')
        name1, drink1 = data[0], data[1]

    input('#################### Debug 4 ####################')

    # 4. offer the seat
    # agent.pose.move_pose()
    agent.move_abs_safe(scan_bypass_position)
    agent.pose.head_tilt(10)

    # agent.say(f'Hi, {name1}.\n Please follow me. \nI will find the seat for you', show_display=True)
    agent.say(f'{name1}.\n Stand in this direction\n and wait until I find your seat', show_display=True)
    rospy.sleep(5.5)

    agent.move_abs_safe(scan_position)
    agent.pose.head_tilt(0)

    agent.say('Searching empty seat.')
    first_seat_info = cs.check_empty_seat(agent)
    first_2b_seated = cs.seat_available(first_seat_info)
    host_seated = cs.host_seat(first_seat_info)
    print('First Seat info:', first_seat_info, 'First guest Seat:', first_2b_seated, 'Host Seat:', host_seated)

    cs.point_seat(agent, first_2b_seated)
    agent.say(name1+'. Please sit down there', show_display=True)
    rospy.sleep(1)

    # 5. Introduce each other
    agent.pose.move_pose()
    agent.move_abs_safe(scan_position)
    cs.gaze_seat(agent, host_seated)
    agent.say(name_host)
    rospy.sleep(0.6)
    cs.gaze_seat(agent, first_2b_seated)
    agent.say('This is ' + name1 + '.', show_display=True)
    rospy.sleep(1.3)
    cs.gaze_seat(agent, host_seated)
    agent.say(f'{name1}\'s favorite drink\n is ' + drink1, show_display=True)
    rospy.sleep(1.8)

    cs.gaze_seat(agent, first_2b_seated)
    agent.say(name1)
    rospy.sleep(0.6)
    cs.gaze_seat(agent, host_seated)
    agent.say(f'This is {name_host}.', show_display=True)
    rospy.sleep(1.3)
    cs.gaze_seat(agent, first_2b_seated)
    agent.say(f'{name_host}\'s favorite drink\n is {drink_host}', show_display=True)
    rospy.sleep(1.8)

    # 6. meet the second guest
    # 6-1. go to door position
    agent.pose.move_pose()
    agent.move_abs_safe(cloth_position)
    rospy.sleep(8)

    if open_door_mode:
        # 6-2. open the door
        # handle_xyz = door_handle_detection(agent)
        rospy.sleep(1)
        agent.pose.head_tilt(10)
        agent.open_gripper()
        agent.pose.door_open_pose()
        # agent.move_abs_safe(pick_position)
        agent.grasp()
        agent.pose.door_handle_down_pose() # 10cm position add
        agent.move_rel(-0.5, 0, wait=True)
        rospy.sleep(1)
        agent.open_gripper()
        agent.pose.move_pose()
        agent.grasp()

    # 6-3. get second guest information
    agent.pose.head_tilt(10)
    agent.say('Hello.', show_display=True)
    rospy.sleep(0.8)

    # Second guest STT
    # qr_check = True
    qr_check = False
    agent.say('I will ask your \nname and drink.', show_display=True)
    rospy.sleep(2.5)
    if not calibration_mode:
        name2, drink2 = '_', '_'
        for j in range(2):
            agent.say('Come very close to me\n and answer after \nthe ring sound', show_display=True)
            rospy.sleep(4)
    
            if name2.lower() not in name_list:
                agent.say('What is your name?\n Say only drink word.', show_display=True)
                rospy.sleep(3)
                second_raw_name = agent.stt(3)
                name2, _ = second_raw_name
                if name2 == '':
                    agent.say('Sorry, voice not recognized.', show_display=True)
                    rospy.sleep(1.5)
                    continue
                print(name2)
    
            if drink2.lower() not in drink_list:
                agent.say('What is your favorite drink?\n Say only drink word.', show_display=True)
                rospy.sleep(3.5)
                second_raw_drink = agent.stt(3)
                drink2, _ = second_raw_drink
                if drink2 == '':
                    agent.say('Sorry, voice not recognized.', show_display=True)
                    rospy.sleep(1.2)
                    continue
                print(drink2)
    
            agent.say(f'Is this correct?\n ({name2}, {drink2}) \nsay yes or no', show_display=True)
            rospy.sleep(4.5)
    
            answer, _ = agent.stt(3)
            if 'yes' in answer:
                qr_check = False
                break
            elif 'no' in answer:
                continue
            else:
                agent.say('Answer only by \nyes or no', show_display=True)
                rospy.sleep(2.5)
                answer, _ = agent.stt(3)
                if 'yes' in answer:
                    qr_check = False
                    break
                elif 'no' in answer:
                    continue

    if qr_check:
        rospy.sleep(1)
        agent.say('Show me the QR code', show_display=True)
        rospy.sleep(1.2)
        # read qrcode
        qr_str = decoder_loop(agent)
        data = qr_str.split(',')
        name2, drink2 = data[0], data[1]

    # agent.say(f'Hi, {name2}.\n Please follow me. \nI will find the seat for you', show_display=True)
    # agent.say(f'{name2}.\n I will find the seat for you', show_display=True)
    # rospy.sleep(3.5)
    agent.move_abs_safe(scan_bypass_position)

    # agent.say(f'Hi, {name1}.\n Please follow me. \nI will find the seat for you', show_display=True)
    agent.pose.head_tilt(10)
    agent.say(f'{name2}.\n Stand in this direction\n and wait until I find your seat', show_display=True)
    rospy.sleep(5.5)

    # 7. offer the seat for the second guest
    # agent.pose.move_pose()
    # agent.move_abs_safe(scan_bypass_position)
    # agent.pose.head_tilt(10)
    # agent.say('Do not move\n until I find your seat.', show_display=True)
    # rospy.sleep(3)
    agent.move_abs_safe(scan_position)
    agent.pose.head_tilt(0)

    # 7-1. check the existing people first
    agent.say('Searching empty seat.')
    second_seat_info = cs.check_empty_seat(agent)
    second_2b_seated = cs.seat_available(second_seat_info)
    host_seated, first_seated = cs.crowd_seat(second_seat_info)
    print('Seat info:', second_seat_info, 'Host Seat:', host_seated, 'First guest Seat:', first_seated, 'Second guest Seat:', second_2b_seated)

    # 8. Introduce each other
    agent.pose.move_pose()
    agent.move_abs_safe(scan_position)
    cs.gaze_seat(agent, host_seated, first_seated)
    agent.say('Hi everyone.', show_display=True)
    rospy.sleep(0.8)
    agent.pose.head_pan(100)
    agent.say('This is ' + name2 + '.', show_display=True)
    rospy.sleep(1.3)
    cs.gaze_seat(agent, host_seated, first_seated)
    agent.say(f'{name2}\'s favorite drink\n is ' + drink2, show_display=True)
    rospy.sleep(1.8)

    agent.pose.head_pan(100)
    agent.say(name2)
    rospy.sleep(0.6)
    cs.gaze_seat(agent, host_seated)
    agent.say(f'This is {name_host}', show_display=True)
    rospy.sleep(1.3)
    cs.gaze_seat(agent, first_seated)
    agent.say(f'and {name1}.', show_display=True)
    rospy.sleep(0.8)
    agent.pose.head_pan(100)
    agent.say(f'{name_host}\'s favorite drink\n is {drink_host}', show_display=True)
    rospy.sleep(2)

    # cs.gaze_seat(agent, first_seated)
    agent.say(f'and {name1}\'s favorite drink\n is ' + drink1, show_display=True)
    rospy.sleep(2.2)

    # gender, age
    age, clothes = attr.parsing(age[0], clothes)


    ########################################################################################3
    # with nametag
    agent.say(f'{name1} is {gender[0]} and \naged around the {age}.\n Also {name1} is wearing\n {clothes},\n and has a nametag.\n ', show_display=True)

    # without nametag
    agent.say(f'{name1} is {gender[0]} and \naged around the {age}.\n Also {name1} is wearing\n {clothes}.\n ', show_display=True)

    ########################################################################################3


    rospy.sleep(10)

    cs.point_seat(agent, second_2b_seated)
    agent.say(name2+' Please sit down there', show_display=True)
    rospy.sleep(1)

    agent.pose.move_pose()
    agent.pose.head_tilt(10)
    agent.say('Thank you', show_display=True)





