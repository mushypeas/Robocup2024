import cv2
import rospy
from xtion import Xtion_camera
import time

import numpy as np
import pandas as pd
import rospy
from module.human_attribute.extract_attribute import Attribute
from module.human_attribute.check_seat import CheckSeat
from module.human_attribute.qrcode import decoder_loop
from module.human_attribute.face_attribute.face_attr import FaceAttribute
from playsound import playsound

### task params #################
# door_position = 'door_handle'
door_position = 'cloth_scan'
door_bypass_position = 'door_bypass'
cloth_position = 'cloth_scan'
scan_position = 'seat_scan'
scan_bypass_position = 'seat_scan_bypass'

open_door_mode = False
calibration_mode = False

cloth_threshold = 0.15
face_threshold = 35
sofa_range = [160, 560]

# most important to make sure that the non-sofa people and sofa people aren't at the same view.
# calibrate the distance to make sure the upper condition and then calibrate the head pan angle.
# closer the better to the sofa to cut the sofa side view

# 150cm setting, sofa side view is visible
head_pan_angle = [40, 20, 0, -20, -40]
point_seat_angle = 20

# 90cm~100cm setting currently the best setting
# head_pan_angle = [50, 25, 0, -25, -50]
# point_seat_angle = 25


face_list = pd.read_csv("./module/human_attribute/face_detection/data.csv")
name_list = ['Sophia', 'Isabella', 'Emma', 'Olivia', 'Ava', 'Emily', 'Abigail', 'Madison', 'Mia', 'Chloe', 'Alex',
             'Charlie', 'Elizabeth', 'Francis', 'Jennifer', 'Linda', 'Mary', 'Patricia', 'Robin', 'Skylar', 'James',
             'John', 'Robert', 'Michael', 'William', 'David', 'Richard', 'Charles', 'Joseph', 'Thomas']
drink_list = ['Chocolate Drink', 'Coke', 'Grape Juice', 'Orange Juice', 'Apple Juice', 'Lemon Juice', 'Water',
              'Sprite', 'Iso Drink', 'Milk', 'Red Spritzer', 'Sparkling Water']

name_host = 'Host'
drink_host = 'Orange Juice'
if calibration_mode:
    name1 = 'Juno'
    drink1 = 'Lemonade'
    name2 = 'Matilda'
    drink2 = 'Mango milk'
    clothes = 'white no pattern woman spring vest no sleeves '

# cap = Xtion_camera()
attr = Attribute(cloth_threshold, calibration_mode)
sofa_range = [130, 580]
door_range = 140
cs = CheckSeat(face_threshold, face_threshold, sofa_range, door_range, head_pan_angle, point_seat_angle, calibration_mode)
face_attr = FaceAttribute()
face_list = pd.read_csv("./module/human_attribute/face_detection/data.csv")

rospy.init_node('xtion')
webcam = Xtion_camera()

attr.cloth_extract_calibration_mode(webcam)
cs.check_calibration_mode(webcam, 40, [140, 600])

#
# time.sleep(5)
# while(True):
#     frame = webcam.rgb_img
#     print(frame)
#     cv2.imshow("test", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
# cv2.destroyAllWindows()