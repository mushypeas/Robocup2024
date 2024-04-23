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
    def __init__(self, face_threshold, face_threshold2, sofa_range, door_range, head_pan_angle, point_seat_angle, calibration_mode):

        self.fc = FaceCropper()
        self.face_id = FaceId()
        self.face_threshold = face_threshold
        self.face_threshold2 = face_threshold2
        self.sofa_range = sofa_range
        self.door_range = door_range

        self.sofa_width = self.sofa_range[1] - self.sofa_range[0]
        self.head_pan_angle = head_pan_angle
        self.point_seat_angle = point_seat_angle
        self.host_face_data = None
        self.calibration_mode = calibration_mode

        self.face_save_idx = 0

    def check_empty_seat(self, agent, check_face=False):
        seat_info = np.full((6, 2), -1)  # [if seated, who]
        width = agent.rgb_img.shape[1]
        user_location_list = []
        user_face_data_list = []

        # left -> sofa -> right
        # sofa range

        # left view
        print("########## check_empty_seat left view ##########")
        agent.pose.head_pan(self.head_pan_angle[0])
        rospy.sleep(1)

        if self.calibration_mode:
            self.check_calibration_mode(agent, self.face_threshold, [140, 620])
        user_locations, user_face_data = self.check(agent, self.face_threshold, [140, 620])
        if user_locations != None:
            user_location_list.extend(user_locations)
            user_face_data_list.extend(user_face_data)

            for user in user_locations:
                if self.door_range <= user[0] < width / 2:
                    seat_info[0][0] = 1
                elif width / 2 <= user[0] < width:
                    seat_info[1][0] = 1

        # sofa view
        print("########## check_empty_seat sofa view ##########")
        agent.pose.head_pan(0)
        rospy.sleep(1)

        if self.calibration_mode:
            self.check_calibration_mode(agent, self.face_threshold, [130, 580])
        user_locations, user_face_data = self.check(agent, self.face_threshold, [130, 580])
        if user_locations != None:
            user_location_list.extend(user_locations)
            user_face_data_list.extend(user_face_data)

            for user in user_locations:
                if self.sofa_range[0] <= user[0] < self.sofa_width / 2 + self.sofa_range[0]:
                    seat_info[2][0] = 1
                else:
                    seat_info[3][0] = 1

                # if self.sofa_range[0] <= user[0] < self.sofa_width / 5 + self.sofa_range[0]:
                #     seat_info[2][0] = 1
                # elif self.sofa_width / 5 + self.sofa_range[0] <= user[0] < self.sofa_width / 5 * 2 + self.sofa_range[0]:
                #     seat_info[3][0] = 1
                # elif self.sofa_width / 5 * 2 + self.sofa_range[0] <= user[0] < self.sofa_width / 5 * 3 + self.sofa_range[0]:
                #     seat_info[4][0] = 1
                # elif self.sofa_width / 5 * 3 + self.sofa_range[0] <= user[0] < self.sofa_width / 5 * 4 + self.sofa_range[0]:
                #     seat_info[5][0] = 1
                # elif self.sofa_width / 5 * 4 + self.sofa_range[0] <= user[0] < self.sofa_range[1]:
                #     seat_info[6][0] = 1

        # right view
        print("########## check_empty_seat right view ##########")
        agent.pose.head_pan(self.head_pan_angle[-1])
        rospy.sleep(1)

        if self.calibration_mode:
            self.check_calibration_mode(agent, self.face_threshold2, [20, 620])
        user_locations, user_face_data = self.check(agent, self.face_threshold2, [20, 620])
        if user_locations is not None:
            user_location_list.extend(user_locations)
            user_face_data_list.extend(user_face_data)

            for user in user_locations:
                if 0 <= user[0] < width / 2:
                    seat_info[4][0] = 1
                elif width / 2 <= user[0] < width:
                    seat_info[5][0] = 1

        # save host face image
        if self.host_face_data is None and len(user_face_data_list):
            self.host_face_data = user_face_data_list[0]

        # identify seated people
        if self.host_face_data is not None:
            host_idx = self.face_id.check_host(self.host_face_data, user_face_data_list)
        else:
            host_idx = 0
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
            agent.say("Person detected.\n Put your face forward\n and look at me.", show_display=True)
            rospy.sleep(2.5)
            for i in range(6):
                img = agent.rgb_img
                temp_faces, temp_locations = self.fc.get_faces_locations(cv2.cvtColor(agent.rgb_img, cv2.COLOR_BGR2RGB), remove_background=False)
                face_candidate.append(temp_faces)
                locations_candidate.append(temp_locations)
                rospy.sleep(0.4)
            print("check_seat check person detected len(face_candidate): ", len(face_candidate))
            print("check_seat check len(locations_candidate): ", len(locations_candidate))
            for i in range(6):
                if len(face_candidate[5 - i]) != 0:
                    faces = face_candidate[5 - i]
                    locations = locations_candidate[5 - i]
            agent.say("Thank you")

            for f, face_box in zip(faces, locations):
                if (f.shape[0] + f.shape[1]) * 0.5 > face_threshold:# and view_range[0] <= face_box[0] and face_box[0] <= view_range[1]:
                    print("Check seat face size:", (f.shape[0] + f.shape[1]) * 0.5)
                    print("Check seat face location:", face_box)
                    guest_faces.append(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
                    box_info = [(round(face_box.xmin * img.shape[1]), round(face_box.ymin * img.shape[0])),
                                (round((face_box.xmin + face_box.width) * img.shape[1]), round((face_box.ymin + face_box.height) * img.shape[0]))]
                    guest_locations.append([int((box_info[0][0] + box_info[1][0]) / 2), int((box_info[0][1] + box_info[1][1]) / 2)])

                    cv2.imwrite(f"/home/tidy/Robocup2024/module/human_attribute/face_detection/face_img_debug/{self.face_save_idx}.png", cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
                    self.face_save_idx += 1

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
        # idx 2, 3
        sofa_seat_count = 0
        for i in range(2, 4):
            if seat_info[i][0] == 1:
                sofa_seat_count += 1
        print('check_seat seat_available sofa_seat_count: ', sofa_seat_count)
        if sofa_seat_count == 2:
            return 5
        elif sofa_seat_count == 1:
            if seat_info[2][0] == 1:
                return 3
            else:
                return 2
        else:
            return 2

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

        if seat_idx <= 1:
            agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[0]))
        elif seat_idx <= 2:
            agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[1]))
        elif seat_idx <= 3:
            agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[2]))
        else:
            agent.move_rel(0, 0, yaw=math.radians(self.head_pan_angle[3]))
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
            try:
                if seat_idx <= 1:
                    agent.pose.head_pan_tilt(self.head_pan_angle[0], 0)
                elif seat_idx <= 2:
                    agent.pose.head_pan_tilt(self.head_pan_angle[1], 0)
                elif seat_idx <= 3:
                    agent.pose.head_pan_tilt(self.head_pan_angle[2], 0)
                else:
                    agent.pose.head_pan_tilt(self.head_pan_angle[3], 0)
                # elif seat_idx <= 4:
                #     agent.pose.head_pan_tilt(self.head_pan_angle[3], 0)
                # elif seat_idx <= 5:
                #     agent.pose.head_pan_tilt(self.head_pan_angle[4], 0)
                # elif seat_idx <= 6:
                #     agent.pose.head_pan_tilt(self.head_pan_angle[5], 0)
                # elif seat_idx <= 7:
                #     agent.pose.head_pan_tilt(self.head_pan_angle[6], 0)
            except:
                agent.pose.head_pan_tilt(self.head_pan_angle[1], 0)
        else:
            try:
                if seat_idx <= 1:
                    seat_idx = 0
                elif seat_idx <= 2:
                    seat_idx = 1
                elif seat_idx <= 3:
                    seat_idx = 2
                else:
                    seat_idx = 3
                # elif seat_idx <= 5:
                #     seat_idx = 4
                # elif seat_idx <= 6:
                #     seat_idx = 5
                # elif seat_idx <= 7:
                #     seat_idx = 6

                if guest_seat_idx <= 1:
                    guest_seat_idx = 0
                elif guest_seat_idx <= 2:
                    guest_seat_idx = 1
                elif guest_seat_idx <= 3:
                    guest_seat_idx = 2
                else:
                    guest_seat_idx = 3
                # elif guest_seat_idx <= 4:
                #     guest_seat_idx = 3
                # elif guest_seat_idx <= 5:
                #     guest_seat_idx = 4
                # elif guest_seat_idx <= 6:
                #     guest_seat_idx = 5
                # elif guest_seat_idx <= 7:
                #     guest_seat_idx = 6

                agent.pose.head_pan_tilt((self.head_pan_angle[seat_idx] + self.head_pan_angle[guest_seat_idx]) / 2.0, 0)
            except:
                agent.pose.head_pan_tilt(self.head_pan_angle[1], 0)

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

            cv2.line(img, (self.sofa_range[0], 0), (self.sofa_range[0], 479), (0, 255, 0), 2)
            cv2.line(img, (self.sofa_range[1], 0), (self.sofa_range[1], 479), (0, 255, 0), 2)

            # sofa seat
            cv2.line(img, (int(self.sofa_width / 2) + self.sofa_range[0], 0), (int(self.sofa_width / 2) + self.sofa_range[0], 479), (255, 255, 0), 2)

            # cv2.line(img, (int(self.sofa_width / 5) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5) + self.sofa_range[0], 479), (255, 255, 0), 2)
            # cv2.line(img, (int(self.sofa_width / 5 * 2) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5 * 2) + self.sofa_range[0], 479), (255, 255, 0), 2)
            # cv2.line(img, (int(self.sofa_width / 5 * 3) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5 * 3) + self.sofa_range[0], 479), (255, 255, 0), 2)
            # cv2.line(img, (int(self.sofa_width / 5 * 4) + self.sofa_range[0], 0),
            #          (int(self.sofa_width / 5 * 4) + self.sofa_range[0], 479), (255, 255, 0), 2)

            # half line
            cv2.line(img, (img.shape[1] // 2, 0), (img.shape[1] // 2, 479), (0, 0, 255), 2)

            cv2.imshow('full img', img)
            key = cv2.waitKey(10)
            if key == ord('q'):
                cv2.destroyAllWindows()
                break




