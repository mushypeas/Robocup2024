import cv2
import mediapipe as mp
import time
import face_cropper
from face_id import FaceId
from PIL import Image
import numpy as np
import rospy
from module.human_attribute.xtion import Xtion_camera
from hsr_agent.agent import Agent

# rospy.init_node('xtion')
# cap = Xtion_camera()
rospy.init_node('main_client_hsr', disable_signals=True)
agent = Agent()

#
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


pTime = 0

mpDraw = mp.solutions.drawing_utils
mpFaceMesh = mp.solutions.face_mesh
faceMesh = mpFaceMesh.FaceMesh(max_num_faces=1)
drawSpec = mpDraw.DrawingSpec(thickness=1, circle_radius=1)

fc = face_cropper.FaceCropper()

start = time.time()
idx = 0
face_id = FaceId()
while True:
    img = agent.rgb_img
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = faceMesh.process(imgRGB)
    faces = fc.get_faces(imgRGB, remove_background=False)
    if len(faces) != 0:
        print(len(faces))
    if results.multi_face_landmarks:
        for faceLms in results.multi_face_landmarks:
            mpDraw.draw_landmarks(img, faceLms, mpFaceMesh.FACEMESH_CONTOURS, #FACEMESH_CONTOURS, FACEMESH_TESSELATION
                                  drawSpec, drawSpec)
            # for id, lm in enumerate(faceLms.landmark):
            #     #print(lm)
            #     ih, iw, ic = img.shape
            #     x, y = int(lm.x*iw), int(lm.y*ih)
            #     # x, y 크롭하세요!
            #     #print(x, y)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
    cv2.imshow("Image", img)

    ################################# 추가
    cropped_face_shape = []
    for idx, face_rgb in enumerate(faces):
        cropped_face_shape.append((face_rgb.shape[0] + face_rgb.shape[1]) * 0.5)
        cv2.imshow(f'{idx + 1}: Cropped Face', cv2.cvtColor(face_rgb, cv2.COLOR_RGB2BGR))
    # try:
    #     biggest_face_idx = cropped_face_shape.index(max(cropped_face_shape))
    #     crop_img = cv2.cvtColor(faces[biggest_face_idx], cv2.COLOR_RGB2BGR)
    #     cv2.imshow('Cropped Face', crop_img)
    #     if time.time() - start > 1 and (crop_img.shape[0] > 40 and crop_img.shape[1] > 40):
    #         crop_img = Image.fromarray(crop_img)
    #         checked_id, user_name = face_id.check_id(crop_img)
    #         print(user_name, checked_id)
    #     else:
    #         print('come to the camera more closer')
    # except:
    #     # print("No face!")
    #     pass

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cv2.destroyAllWindows()