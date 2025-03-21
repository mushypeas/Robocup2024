import cv2
import numpy as np
from module.human_attribute.face_detection.face_cropper import FaceCropper
from module.human_attribute.face_detection.face_id import FaceId
from PIL import Image
import rospy
import math

# import rospy
# from module.human_attribute.xtion import Xtion_camera
# rospy.init_node('xtion')
# cap = Xtion_camera()


class CheckSeat():
    # def __init__(self, face_threshold, face_threshold2, sofa_range, door_range, head_pan_angle, point_seat_angle, calibration_mode):
    # def __init__(self, face_threshold, face_threshold2, head_pan_angle, calibration_mode):
    def __init__(self, face_threshold, face_threshold2, face_threshold3, head_pan_angle, calibration_mode):

        self.fc = FaceCropper(min_face_detector_confidence=0.1)
        self.face_id = FaceId()
        self.face_threshold = face_threshold
        self.face_threshold2 = face_threshold2
        self.face_threshold3 = face_threshold3
        # self.sofa_range = sofa_range
        # self.door_range = door_range

        # self.sofa_width = self.sofa_range[1] - self.sofa_range[0]
        self.head_pan_angle = head_pan_angle
        # self.point_seat_angle = point_seat_angle
        self.host_face_data = None
        self.calibration_mode = calibration_mode

        self.face_save_idx = 0

    def check_empty_seat(self, agent, check_face=False):
        # seat_info = np.full((6, 2), -1)  # [if seated, who]
        seat_info = np.full((len(self.head_pan_angle), 2), -1)

        # width = agent.rgb_img.shape[1]
        user_location_list = []
        user_face_data_list = []

        #################### left view
        # agent.pose.head_pan(self.head_pan_angle[0])
        # agent.pose.head_pan(45) # 0609 # PNU
        #################### 2024 Eindhoven # OK
        agent.pose.head_pan(50) # OK
        rospy.sleep(1)

        if self.calibration_mode:
            # self.check_calibration_mode(agent, self.face_threshold, [140, 620])
            # self.check_calibration_mode(agent, self.face_threshold, [200, 560]) # 0514
            # self.check_calibration_mode(agent, self.face_threshold, [180, 600]) # 0609
            # self.check_calibration_mode(agent, self.face_threshold, [160, 520]) # PNU
            #################### 2024 Eindhoven # OK 3 LEFT SEATS
            self.check_calibration_mode(agent, self.face_threshold, [10, 600])
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [140, 620])
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [200, 560]) # 0514
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [180, 600]) # 0609
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [160, 520]) # PNU
        #################### 2024 Eindhoven # OK 3 LEFT SEATS
        user_locations, user_face_data = self.check(agent, self.face_threshold, [10, 600])
        if user_locations != None:
            user_location_list.extend(user_locations)
            user_face_data_list.extend(user_face_data)


            #################### 2024 Eindhoven # 3 LEFT SEATS
            for user in user_locations:
                if 10 <= user[0] < 200:
                    seat_info[0][0] = 1
                elif 200 <= user[0] < 360:
                    seat_info[1][0] = 1
                elif 360 <= user[0] < 600:
                    seat_info[2][0] = 1
        #################### left view

        #################### middle view
        agent.pose.head_pan(0)
        rospy.sleep(1)

        if self.calibration_mode:
            # self.check_calibration_mode(agent, self.face_threshold, [130, 580])
            # self.check_calibration_mode(agent, self.face_threshold, [100, 500]) # 0514
            # self.check_calibration_mode(agent, self.face_threshold2, [150, 550]) # 0609
            # self.check_calibration_mode(agent, self.face_threshold2, [100, 540]) # PNU
            #################### 2024 Eindhoven # OK
            self.check_calibration_mode(agent, self.face_threshold2, [60, 590])
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [130, 580])
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [100, 500]) # 0514
        # user_locations, user_face_data = self.check(agent, self.face_threshold2, [150, 550]) # 0609
        # user_locations, user_face_data = self.check(agent, self.face_threshold2, [100, 540]) # PNU
        #################### 2024 Eindhoven # OK
        user_locations, user_face_data = self.check(agent, self.face_threshold2, [60, 590])
        if user_locations != None:
            user_location_list.extend(user_locations)
            user_face_data_list.extend(user_face_data)

            #################### 2024 Eindhoven # OK
            for user in user_locations:
                if 60 <= user[0] < 320:
                    seat_info[3][0] = 1
                elif 320 <= user[0] < 590:
                    seat_info[4][0] = 1

        #################### middle view


        #################### right view (1)
        # agent.pose.head_pan(self.head_pan_angle[-1])
        # agent.pose.head_pan(self.head_pan_angle[-2])
        # agent.pose.head_pan(-45) # 0609
        #################### 2024 Eindhoven # OK 
        agent.pose.head_pan(-55)
        rospy.sleep(1)

        if self.calibration_mode:
            # self.check_calibration_mode(agent, self.face_threshold2, [20, 620])
            # self.check_calibration_mode(agent, self.face_threshold2, [80, 560])
            # self.check_calibration_mode(agent, self.face_threshold, [30, 560])
            #################### 2024 Eindhoven # OK
            self.check_calibration_mode(agent, self.face_threshold3, [60, 610])
        # user_locations, user_face_data = self.check(agent, self.face_threshold2, [20, 620])
        # user_locations, user_face_data = self.check(agent, self.face_threshold2, [80, 560])
        # user_locations, user_face_data = self.check(agent, self.face_threshold, [30, 560])
        #################### 2024 Eindhoven # OK
        user_locations, user_face_data = self.check(agent, self.face_threshold3, [60, 610])
        if user_locations is not None:
            user_location_list.extend(user_locations)
            user_face_data_list.extend(user_face_data)


            #################### 2024 Eindhoven # TEMPOARILY OK
            # for user in user_locations: # NO LEFT SEATS
            #     if 100 <= user[0] < 540:
            #         seat_info[2][0] = 1
            #     # elif 170 <= user[0] < 330:
            #     #     seat_info[5][0] = 1
            for user in user_locations: # 
                if 60 <= user[0] < 370:
                    seat_info[5][0] = 1
                elif 370 <= user[0] < 610:
                    seat_info[6][0] = 1
        #################### right view (1)

        
        # #################### right view (2)
        # #################### 2024 Eindhoven # OK
        # agent.pose.head_pan(-105)
        # rospy.sleep(1)

        # if self.calibration_mode:
        #     #################### 2024 Eindhoven # OK
        #     self.check_calibration_mode(agent, self.face_threshold3, [120, 490])
        # #################### 2024 Eindhoven # OK
        # user_locations, user_face_data = self.check(agent, self.face_threshold3, [100, 490])
        # if user_locations is not None:
        #     user_location_list.extend(user_locations)
        #     user_face_data_list.extend(user_face_data)

        #     #################### 2024 Eindhoven # OK
        #     for user in user_locations: # 
        #         if 120 <= user[0] < 490:
        #             seat_info[7][0] = 1
        #         # elif 370 <= user[0] < 600:
        #         #     seat_info[8][0] = 1
        # #################### right view (2)
            


        # # save host face image
        # if self.host_face_data is None and len(user_face_data_list):
        #     self.host_face_data = user_face_data_list[0]

        # identify seated people
        if self.host_face_data is not None:
            host_idx = self.face_id.check_host(self.host_face_data, user_face_data_list)
        else:
            host_idx = 0
            if len(user_face_data_list):
                self.host_face_data = user_face_data_list[0]
            else:
                print('!!!fucked up!!! open dummy data')
                # when no face detected, open image from ./face_detection/face_img_debug using PIL.Image
                self.host_face_data = Image.open(f"./face_detection/face_img_debug/0.png")
        print('check_seat check_empty_seat host_idx: ', host_idx)

        user_searched_idx = 0
        for seat in seat_info:
            if seat[0] == 1:
                if host_idx == user_searched_idx:
                    seat[1] = 0
                else:
                    seat[1] = 1
                user_searched_idx += 1
        print('check_seat check_empty_seat seat_info: ', seat_info)
        return seat_info

    def check(self, agent, face_threshold, view_range):
        img = agent.rgb_img
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        faces, locations = self.fc.get_faces_locations(imgRGB, remove_background=False)  ##facebox_rectangle in get_face_debug

        print("check_seat check len(faces): ", len(faces))
        print("check_seat check len(locations): ", len(locations))

        guest_faces, guest_locations = [], []

        face_candidate, locations_candidate = [], []
        if len(faces) != 0:
            # 그냥 tts 지워버릴까? ㅇㅇ 지워버림 -> 그냥 시나리오를 바꿈
            # agent.say("Person detected.\n Put your face forward\n and look at me.", show_display=True)
            rospy.sleep(2.5)
            for i in range(6):
                img = agent.rgb_img
                temp_faces, temp_locations = self.fc.get_faces_locations(cv2.cvtColor(agent.rgb_img, cv2.COLOR_BGR2RGB), remove_background=False)
                face_candidate.append(temp_faces)
                locations_candidate.append(temp_locations)
                rospy.sleep(0.4)
            for i in range(6):
                if len(face_candidate[5 - i]) != 0:
                    faces = face_candidate[5 - i]
                    locations = locations_candidate[5 - i]
            # agent.say("Thank you")

            for f, face_box in zip(faces, locations):
                # 기존 코드 (view range 사용 안함)
                # if (f.shape[0] + f.shape[1]) * 0.5 > face_threshold:# and view_range[0] <= face_box[0] and face_box[0] <= view_range[1]:
                # 0514
                if (f.shape[0] + f.shape[1]) * 0.5 > face_threshold and view_range[0] <= face_box.xmin*640 and (face_box.xmin+face_box.width)*640 <= view_range[1]:
                    guest_faces.append(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
                    box_info = [(round(face_box.xmin * img.shape[1]), round(face_box.ymin * img.shape[0])),
                                (round((face_box.xmin + face_box.width) * img.shape[1]), round((face_box.ymin + face_box.height) * img.shape[0]))]
                    guest_locations.append([int((box_info[0][0] + box_info[1][0]) / 2), int((box_info[0][1] + box_info[1][1]) / 2)])

                    cv2.imwrite(f"/home/tidy/Robocup2024/module/human_attribute/face_detection/face_img_debug/{self.face_save_idx}.png", cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
                    self.face_save_idx += 1
                else:
                    print("Check face threshold: ", (f.shape[0] + f.shape[1]) * 0.5 > face_threshold)
                    print("Check face location left: ", view_range[0] <= face_box.xmin*640)
                    print("Check face location right: ", (face_box.xmin+face_box.width)*640 <= view_range[1])

            try:
                print('check_seat check guest_locations(1): ', guest_locations)
                guest_locations = np.array(guest_locations)
                idx_sorted = guest_locations.argsort(axis=0)[:, 0]
                guest_locations = guest_locations[idx_sorted]
                print('check_seat check guest_locations(2): ', guest_locations)

                return list(guest_locations), [Image.fromarray(guest_faces[idx]) for idx in idx_sorted]
            except:
                print("No face! While taking photo.")
                return None, None
        else:
            print("No face!")
            return None, None

    def seat_available(self, seat_info):
        # # idx 2, 3
        # sofa_seat_count = 0
        # for i in range(2, 4):
        #     if seat_info[i][0] == 1:
        #         sofa_seat_count += 1
        # print('check_seat seat_available sofa_seat_count: ', sofa_seat_count)
        # if sofa_seat_count == 2:
        #     return 5
        # elif sofa_seat_count == 1:
        #     if seat_info[2][0] == 1:
        #         return 3
        #     else:
        #         return 2
        # else:
        #     return 2
        
        # 0609
        # 일단 가운데부터, 양 옆 사람 없는 자리 고름
        # for i in [2, 3, 1, 4, 0, 5, 6]:
        # for i in [2, 3, 1, 4, 0]:
        #################### 2024 Eindhoven # OK
        # for i in [1, 0, 2]: # NO LEFT SEAT
        # for i in [3, 2, 4, 1, 0]: # ADDITIONAL TWO LEFT SEATS
        # for i in [4, 3, 5]:
            
        #     if seat_info[i][0] != 1:
        #         if i == 0:
        #             if seat_info[1][0] != 1:
        #                 return i
        #             else:
        #                 continue
        #         elif i==len(seat_info)-1:
        #             if seat_info[len(seat_info)-2][0] != 1:
        #                 return i
        #             else:
        #                 continue
        #         else:
        #             if seat_info[i-1][0] != 1 and seat_info[i+1][0] != 1:
        #                 return i
        #             else:
        #                 continue
        # 안되면 그냥 왼쪽부터 빈자리로
        # for i in range(len(seat_info)):
        for i in [4, 3, 5]:
            if seat_info[i][0] == -1:
                return i

    def host_seat(self, seat_info):
        for idx, seat in enumerate(seat_info):
            if seat[1] == 0:
                return idx

    def crowd_seat(self, seat_info):
        host_seat_idx = 2
        first_seat_idx = 3
        for idx, seat in enumerate(seat_info):
            if seat[1] == 0:
                host_seat_idx = idx
            if seat[1] == 1:
                first_seat_idx = idx

        return host_seat_idx, first_seat_idx

    def point_seat(self, agent, seat_idx):
        agent.pose.head_pan(0)

        # 기존코드
        # if seat_idx <= 1:
        #     agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[0]))
        # elif seat_idx <= 2:
        #     agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[1]))
        # elif seat_idx <= 3:
        #     agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[2]))
        # else:
        #     agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[3]))
        # 0514
        agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[seat_idx]))
        # seat_idx -= 2
        # if 0 <= seat_idx <= 4:
        #     print('---------------------------------', -self.point_seat_angle * seat_idx + self.point_seat_angle * 2)
        #     agent.move_rel(0, 0, yaw=math.radians(-self.point_seat_angle * seat_idx + self.point_seat_angle * 2))
        # elif seat_idx < 0:
        #     agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[0]))
        # elif seat_idx > 4:
        #     agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[6]))
        # else:
        #     agent.move_rel(0, 0, 0)

        agent.pose.point_seat_pose()

    def gaze_seat(self, agent, seat_idx, guest_seat_idx=None):
        if guest_seat_idx is None:
            # 기존 코드
            # try:
            #     if seat_idx <= 1:
            #         agent.pose.head_pan_tilt(self.head_pan_angle[0], 0)
            #     elif seat_idx <= 2:
            #         agent.pose.head_pan_tilt(self.head_pan_angle[1], 0)
            #     elif seat_idx <= 3:
            #         agent.pose.head_pan_tilt(self.head_pan_angle[2], 0)
            #     else:
            #         agent.pose.head_pan_tilt(self.head_pan_angle[3], 0)
            #     # elif seat_idx <= 4:
            #     #     agent.pose.head_pan_tilt(self.head_pan_angle[3], 0)
            #     # elif seat_idx <= 5:
            #     #     agent.pose.head_pan_tilt(self.head_pan_angle[4], 0)
            #     # elif seat_idx <= 6:
            #     #     agent.pose.head_pan_tilt(self.head_pan_angle[5], 0)
            #     # elif seat_idx <= 7:
            #     #     agent.pose.head_pan_tilt(self.head_pan_angle[6], 0)
            # except:
            #     agent.pose.head_pan_tilt(self.head_pan_angle[1], 0)

            # 0514
            agent.pose.head_pan_tilt(self.head_pan_angle[seat_idx], 0)

        else:
            try:
                # 기존 코드 주석처리
                # if seat_idx <= 1:
                #     seat_idx = 0
                # elif seat_idx <= 2:
                #     seat_idx = 1
                # elif seat_idx <= 3:
                #     seat_idx = 2
                # else:
                #     seat_idx = 3
                # # elif seat_idx <= 5:
                # #     seat_idx = 4
                # # elif seat_idx <= 6:
                # #     seat_idx = 5
                # # elif seat_idx <= 7:
                # #     seat_idx = 6

                # if guest_seat_idx <= 1:
                #     guest_seat_idx = 0
                # elif guest_seat_idx <= 2:
                #     guest_seat_idx = 1
                # elif guest_seat_idx <= 3:
                #     guest_seat_idx = 2
                # else:
                #     guest_seat_idx = 3
                # # elif guest_seat_idx <= 4:
                # #     guest_seat_idx = 3
                # # elif guest_seat_idx <= 5:
                # #     guest_seat_idx = 4
                # # elif guest_seat_idx <= 6:
                # #     guest_seat_idx = 5
                # # elif guest_seat_idx <= 7:
                # #     guest_seat_idx = 6

                agent.pose.head_pan_tilt((self.head_pan_angle[seat_idx] + self.head_pan_angle[guest_seat_idx]) / 2.0, 0)
            except:
                # 기존 코드
                # agent.pose.head_pan_tilt(self.head_pan_angle[1], 0)
                # 0514
                agent.pose.head_pan_tilt(self.head_pan_angle[seat_idx], 0)

    def check_calibration_mode(self, agent, face_threshold, view_range):
        while True:
            guest_faces, guest_locations = [], []
            img = agent.rgb_img
            faces, locations = self.fc.get_faces_locations(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), remove_background=False)

            for face_img, face_box in zip(faces, locations):
                if (face_img.shape[0] + face_img.shape[1]) * 0.5:# > face_threshold and view_range[0] <= face_box[0] and face_box[0] <= view_range[1]:
                    print(view_range, face_box)
                    guest_faces.append(cv2.cvtColor(face_img, cv2.COLOR_RGB2BGR))

                    box_info = [(round(face_box.xmin * img.shape[1]), round(face_box.ymin * img.shape[0])),
                                (round((face_box.xmin + face_box.width) * img.shape[1]),
                                 round((face_box.ymin + face_box.height) * img.shape[0]))]
                    guest_locations.append([int((box_info[0][0] + box_info[1][0]) / 2), int((box_info[0][1] + box_info[1][1]) / 2)])

            face_crop_box_list = []
            depth_list = []
            for idx, face_img in enumerate(guest_faces):
                cv2.imshow(f'{idx + 1}: Cropped Face', face_img)
                cv2.circle(img, guest_locations[idx], 4, (0, 0, 255), -1)
                face_crop_box_list.append((face_img.shape[0] + face_img.shape[1]) * 0.5)
                depth_list.append(agent.depth_image[guest_locations[idx][1], guest_locations[idx][0]])
            print("crop face box size:", face_crop_box_list, "Threshold:", self.face_threshold, "|| Depth:", depth_list)

            # cv2.line(img, (self.sofa_range[0], 0), (self.sofa_range[0], 479), (0, 255, 0), 2)
            # cv2.line(img, (self.sofa_range[1], 0), (self.sofa_range[1], 479), (0, 255, 0), 2)

            # sofa seat (sofa half line)
            # cv2.line(img, (int(self.sofa_width / 2) + self.sofa_range[0], 0), (int(self.sofa_width / 2) + self.sofa_range[0], 479), (255, 255, 0), 2)

            # cv2.line(img, (int(self.sofa_width / 5) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5) + self.sofa_range[0], 479), (255, 255, 0), 2)
            # cv2.line(img, (int(self.sofa_width / 5 * 2) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5 * 2) + self.sofa_range[0], 479), (255, 255, 0), 2)
            # cv2.line(img, (int(self.sofa_width / 5 * 3) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5 * 3) + self.sofa_range[0], 479), (255, 255, 0), 2)
            # cv2.line(img, (int(self.sofa_width / 5 * 4) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5 * 4) + self.sofa_range[0], 479), (255, 255, 0), 2)

            # half line (image)
            cv2.line(img, (img.shape[1] // 2, 0), (img.shape[1] // 2, 479), (0, 0, 255), 2)

            cv2.imshow('full img', img)
            key = cv2.waitKey(10)
            if key == ord('q'):
                cv2.destroyAllWindows()
                break




