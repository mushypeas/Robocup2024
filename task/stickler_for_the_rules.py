import rospy
import time
import math
import sys
import cv2
import numpy as np
import mediapipe as mp
import open_clip
from PIL import Image
import torch
# from module.CLIP.clip_detection import CLIPDetector, CLIPDetectorConfig
from utils.marker_maker import MarkerMaker
from utils.axis_transform import Axis_transform
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

sys.path.append('.')

class ShoeDetection:
    def __init__(self, agent, axis_transform):
        self.agent = agent
        self.axis_transform = axis_transform
        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray,
                         self._knee_pose_callback)
        self.knee_list = None

        # Initialize the CLIP model
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.clip_model, _, self.preprocess = open_clip.create_model_and_transforms('ViT-B/32')
        state_dict = torch.load('module/CLIP/openfashionclip.pt', map_location=self.device)
        self.clip_model.load_state_dict(state_dict['CLIP'])
        self.clip_model = self.clip_model.eval().requires_grad_(False).to(self.device)
        self.tokenizer = open_clip.get_tokenizer('ViT-B-32')

    def _knee_pose_callback(self, data):
        self.knee_list = np.reshape(data.data, (-1, 2))
        visualize_mode = False
        if visualize_mode:
            for x, y in self.knee_list:
                cv2.circle(self.agent.rgb_img, (x, y), 2,
                           (255, 0, 0), -1, cv2.LINE_AA)
            cv2.imshow('hsr_vision', self.agent.rgb_img)
            cv2.waitKey(1)  # 1 millisecond


    def find_shoes(self):
        # Prompt for CLIP model
        prompt = "a photo of a"
        text_inputs = ["human who is wearing shoes", "barefoot person"]
        text_inputs = [prompt + " " + t for t in text_inputs]
        tokenized_prompt = self.tokenizer(text_inputs).to(self.device)

        # Preprocess image
        image = self.agent.rgb_img
        img = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        img = self.preprocess(img).unsqueeze(0).to(self.device)

        # Encode image and text features
        with torch.no_grad():
            image_features = self.clip_model.encode_image(img)
            text_features = self.clip_model.encode_text(tokenized_prompt)
            image_features /= image_features.norm(dim=-1, keepdim=True)
            text_features /= text_features.norm(dim=-1, keepdim=True)

            # Calculate text probabilities
            text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)

        # Convert probabilities to percentages
        text_probs_percent = text_probs * 100
        text_probs_percent_np = text_probs_percent.cpu().numpy()

        # Get the numeric value of the first probability
        first_prob_percent = text_probs_percent_np[0][0]

        # Print the formatted probabilities
        formatted_probs = ["{:.2f}%".format(value) for value in text_probs_percent_np[0]]
        print("Labels probabilities in percentage:", formatted_probs)

        # Determine if shoes are detected by comparing the numeric value
        if first_prob_percent > 65:
            return True
        return False


    def detect(self):
        rospy.sleep(1)
        head_tilt_list = [-20, -40]

        for tilt in head_tilt_list:
            self.agent.pose.head_tilt(tilt)
            if self.find_shoes():
                return True
            rospy.sleep(1)

        return False


    def clarify_violated_rule(self):
        if len(self.knee_list) > 0:
            for x, y in self.knee_list:
                move_human_infront(
                    self.agent, self.axis_transform, y, x, coord=False)

            # clarify what rule is being broken
            self.agent.pose.head_tilt(20)
            self.agent.say('Hello!', show_display=True)
            rospy.sleep(1)
            self.agent.say(
                'Sorry but\n all guests should take off\n their shoes at the entrance.', show_display=True)
            rospy.sleep(5)
                

    def ask_to_action(self, entrance):
        rospy.sleep(1)
        # take the offender to the entrance
        self.agent.say('Allow me to guide you\nto the entrance.',
                       show_display=True)
        rospy.sleep(3)
        self.agent.say('Please follow me!', show_display=True)
        rospy.sleep(2)
        self.agent.move_abs_safe(entrance)

        # ask to take off their shoes
        self.agent.pose.head_tilt(20)
        self.agent.say('Please take off your shoes\n here at the enterance', show_display=True)
        rospy.sleep(3)

        self.agent.say('I will wait ten seconds\nfor you to take off your shoes', show_display=True)
        rospy.sleep(13)

        rebelion_count = 0
        while rebelion_count < 3:
            self.agent.pose.head_tilt(-40)
            self.agent.say('Are you finished?\nLet me see your feet', show_display=True)
            rospy.sleep(4)

            if self.find_shoes():
                self.agent.pose.head_tilt(20)
                self.agent.say('You are still wearing shoes!', show_display=True)
                rospy.sleep(2)
                self.agent.say(f'I will wait five more seconds\nfor you to take off your shoes', show_display=True)
                rospy.sleep(8)
                rebelion_count += 1
            else:
                self.agent.pose.head_tilt(20)
                self.agent.say('Thank you!\nEnjoy your party', show_display=True)
                rospy.sleep(2.5)

        self.agent.pose.head_tilt(20)
        self.agent.say('I give up.\nEnjoy your party', show_display=True)
        rospy.sleep(2.5)

