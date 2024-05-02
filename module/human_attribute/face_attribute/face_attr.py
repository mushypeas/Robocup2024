import torch
import argparse
import cv2
import numpy as np
from PIL import Image
from tqdm import tqdm
from pathlib import Path
from torchvision import transforms as T
from typing import Union

from module.human_attribute.easyface.attributes.models import *
from module.human_attribute.easyface.utils.visualize import show_image
from module.human_attribute.easyface.utils.io import WebcamStream, VideoReader, VideoWriter, FPS
from module.human_attribute.face_attribute.detect_align import FaceDetectAlign
import rospy
import os

class FaceAttribute():
    def __init__(self) -> None:
        os.makedirs('./module/human_attribute/face_attribute/faces', exist_ok=True)
        parser = argparse.ArgumentParser()
        parser.add_argument('--source', type=str, default='./module/human_attribute/faces/test.jpg')
        parser.add_argument('--model', type=str, default='FairFace')
        parser.add_argument('--checkpoint', type=str,
                            default='./module/human_attribute/face_attribute/attr_models/res34_fairface.pth')
        parser.add_argument('--det_model', type=str, default='RetinaFace')
        parser.add_argument('--det_checkpoint', type=str,
                            default='./module/human_attribute/face_attribute/attr_models/mobilenet0.25_Final.pth')
        self.args = vars(parser.parse_args())

        self.img_path = './module/human_attribute/face_attribute/faces/test.jpg'

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.gender_labels = ['Male', 'Female']
        self.race_labels = ['White', 'Black', 'Latino Hispanic', 'East Asian', 'Southeast Asian', 'Indian',
                            'Middle Eastern']
        self.age_labels = ['0-2', '3-9', '10-19', '20-29', '30-39', '40-49', '50-59', '60-69', '70+']

        self.model = eval('FairFace')(len(self.gender_labels) + len(self.race_labels) + len(self.age_labels))
        self.model.load_state_dict(torch.load('./module/human_attribute/face_attribute/attr_models/res34_fairface.pth', map_location='cpu'))
        self.model = self.model.to(self.device)
        self.model.eval()
        self.align = FaceDetectAlign('RetinaFace', './module/human_attribute/face_attribute/attr_models/mobilenet0.25_Final.pth')

        self.preprocess = T.Compose([
            T.Resize((224, 224)),
            T.Lambda(lambda x: x / 255),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

    def visualize(self, image, dets, races, genders, ages):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        boxes = dets[:, :4].astype(int)

        for box, race, gender, age in zip(boxes, races, genders, ages):
            cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
            cv2.rectangle(image, (box[0], box[3] + 5), (box[2] + 20, box[3] + 50), (255, 255, 255), -1)
            cv2.putText(image, gender, (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.4, (0, 0, 0),
                        lineType=cv2.LINE_AA)
            cv2.putText(image, race, (box[0], box[3] + 30), cv2.FONT_HERSHEY_DUPLEX, 0.4, (0, 0, 0),
                        lineType=cv2.LINE_AA)
            cv2.putText(image, age, (box[0], box[3] + 45), cv2.FONT_HERSHEY_DUPLEX, 0.4, (0, 0, 0),
                        lineType=cv2.LINE_AA)
        return image

    def postprocess(self, preds: torch.Tensor):
        race_logits, gender_logits, age_logits = preds[:, :7].softmax(dim=1), preds[:, 7:9].softmax(dim=1), preds[:,
                                                                                                            9:18].softmax(
            dim=1)
        race_preds = torch.argmax(race_logits, dim=1)
        gender_preds = torch.argmax(gender_logits, dim=1)
        age_preds = torch.argmax(age_logits, dim=1)
        return [self.race_labels[idx] for idx in race_preds], [self.gender_labels[idx] for idx in gender_preds], [
            self.age_labels[idx] for idx in age_preds]

    def face_attribute(self, agent):
        # agent.say("Please look at me \nfor three seconds", show_display=True)
        # agent.say("Please look at me", show_display=True)
        # rospy.sleep(2)
        guest_img = agent.rgb_img
        cv2.imwrite(self.img_path, guest_img)
        faces, dets, image = self.align.detect_and_align_faces(self.img_path, (112, 112))
        if faces is None:
            print('face_attr.face_attribute align.detect_and_align_faces: faces is None !!!')
            return None

        pfaces = self.preprocess(faces.permute(0, 3, 1, 2)).to(self.device)

        with torch.inference_mode():
            preds = self.model(pfaces).detach().cpu()
        races, genders, ages = self.postprocess(preds)
        print('face_attr.face_attribute rages,genders,ages: ', races, genders, ages)

        image = self.visualize(image[0], dets[0], races, genders, ages)
        # cv2.imshow("visualize_image", image) #################### Debug 2 ####################
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return genders, ages
