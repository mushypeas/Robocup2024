import rospy
from datetime import datetime, timedelta

from gpsr_cmds import *
from gpsr_followup import *
from gpsr_parser import *
from gpsr_utils import *
from gpsr_config import *
from gpsr_follow import *
from gpsr_model_class import *

from PIL import Image
import cv2
import torch

from std_msgs.msg import Float32MultiArray

import torch
import torch.nn.functional as F

import Levenshtein
import math
import time
import subprocess

objects_data = readData(objects_file_path)

class GPSR:
    def __init__(self, agent):
        self.agent = agent
        self.task_finished_count = 0

        self.loc_list = list(ABS_POSITION.keys())
        self.rooms_list = rooms_list
        self.names_list = names_list

        self.gesture_person_list = gesture_person_list
        self.pose_person_list = pose_person_list
        self.gesture_person_plural_list = gesture_person_plural_list
        self.pose_person_plural_list = pose_person_plural_list
        self.person_info_list = person_info_list
        self.object_comp_list = object_comp_list

        self.talk_list = talk_list
        self.question_list = question_list
        self.color_list = color_list
        self.clothe_list = clothe_list
        self.clothes_list = clothes_list

        print('objdata', objects_data)
        
        self.object_names, self.object_categories_plural, self.object_categories_singular = parseObjects(objects_data)
        print("object_names", self.object_names)
        print("object_categories_plural", self.object_categories_plural)
        print("object_categories_singular", self.object_categories_singular)
        
        self.category2objDict, self.categoryPlur2Sing, self.categorySing2Plur = extractCategoryToObj(objects_data)
        print("category2objDict", self.category2objDict)
        print("categoryPlur2Sing", self.categoryPlur2Sing)
        print("categorySing2Plur", self.categorySing2Plur)

        self.kpts_sub = rospy.Subscriber('human_pose_and_bbox', Float32MultiArray, self.hkpts_cb)

        self.cmdNameTocmdFunc = {
            "goToLoc": goToLoc,
            "takeObjFromPlcmt": takeObjFromPlcmt,
            "findPrsInRoom": findPrsInRoom,
            "findObjInRoom": findObjInRoom,
            "meetPrsAtBeac": meetPrsAtBeac,
            "countObjOnPlcmt": countObjOnPlcmt,
            "countPrsInRoom": countPrsInRoom,
            "tellPrsInfoInLoc": tellPrsInfoInLoc,
            "talkInfoToGestPrsInRoom": talkInfoToGestPrsInRoom,
            "answerToGestPrsInRoom": answerToGestPrsInRoom,
            "followNameFromBeacToRoom": followNameFromBeacToRoom,
            "guideNameFromBeacToBeac": guideNameFromBeacToBeac,
            "guidePrsFromBeacToBeac": guidePrsFromBeacToBeac,
            "guideClothPrsFromBeacToBeac": guideClothPrsFromBeacToBeac,
            "bringMeObjFromPlcmt": bringMeObjFromPlcmt,
            "tellCatPropOnPlcmt": tellCatPropOnPlcmt,
            "greetClothDscInRm": greetClothDscInRm,
            "greetNameInRm": greetNameInRm,
            "meetNameAtLocThenFindInRm": meetNameAtLocThenFindInRm,
            "countClothPrsInRoom": countClothPrsInRoom,
            "tellPrsInfoAtLocToPrsAtLoc": tellPrsInfoAtLocToPrsAtLoc,
            "followPrsAtLoc": followPrsAtLoc
        }
        
        self.followupNameTofollowupFunc = {
            "findObj": findObj,
            "findPrs": findPrs,
            "meetName": meetName,
            "placeObjOnPlcmt": placeObjOnPlcmt,
            "deliverObjToMe": deliverObjToMe,
            "deliverObjToPrsInRoom": deliverObjToPrsInRoom,
            "deliverObjToNameAtBeac": deliverObjToNameAtBeac,
            "talkInfo": talkInfo,
            "answerQuestion": answerQuestion,
            "followPrs": followPrs,
            "followPrsToRoom": followPrsToRoom,
            "guidePrsToBeacon": guidePrsToBeacon,
            "takeObj": takeObj
        }
        
        # CLIP
        # self.clip_model, self.preprocess, self.tokenizer, self.device = init_clip()

        # 학습된 모델 로드
        self.pose_model = PoseClassifier(51)
        self.pose_model.load_state_dict(torch.load('module/yolov7-pose-estimation/pose_classifier_1.pth'))
        self.pose_model.eval()

        self.gest_model = GestClassifier(51)
        self.gest_model.load_state_dict(torch.load('module/yolov7-pose-estimation/gest_classifier_1.pth'))
        self.gest_model.eval()

        self.return_point = ABS_POSITION['gpsr_instruction_point']

##############################################################
##############################################################
##############################################################
##############################################################
##############################################################
    def follow(self):
        f = GPSRFollow(self.agent, self)

        self.say("come close")
        rospy.sleep(2)

        moved_time = time.time()

        moved_count = 0

        while not rospy.is_shutdown():
            if time.time() - moved_time < main_period:
                continue

            moved_count += 1

            if moved_count > 30:
                self.say("I follow you enough")
                rospy.sleep(1.5)
                return

            try:
                candidates = f.candidates
            except AttributeError:
                continue

            print("candidates: ", candidates)

            if not candidates:
                ## TODO (better) ##
                ### If there is no candidate, what should we do? ###
                ### go back, turn around, etc... ###
                f.move_rel(0, 0, yaw=math.pi / 2)
                moved_time = time.time()
                continue

            best_interval = f.get_best_candidate(candidates)

            while not best_interval:
                best_interval = f.get_best_candidate(candidates)
            
            f.move_best_interval(best_interval)

            moved_time = time.time()

    def followToLoc(self, loc):
        self.follow()
##############################################################
##############################################################
##############################################################
##############################################################
##############################################################
    # CALLBACKS
    # human keypoints callback
    def hkpts_cb(self, msg):
        self.human_keypoints = msg.data

    ### HELP Functions ###
    def get_yolo_bbox(self, category=None):
        yolo_bbox = self.agent.yolo_module.yolo_bbox

        print("original_yolo_bbox", yolo_bbox)

        if category and category != 'object':
            categoryItems = self.category2objDict[category]
            print("categoryItems", categoryItems)
            yolo_bbox = [obj for obj in yolo_bbox if self.agent.yolo_module.find_name_by_id(obj[4]) in categoryItems]
        
        print("final_yolo_bbox", yolo_bbox)

        return yolo_bbox

    def move(self, loc):
        print("GPSR Move Start")
        self.agent.move_abs(loc)
        print(f"[MOVE] HSR moved to {loc}")

    def move_rel(self, x, y, yaw=0):
        print("GPSR Move_rel Start")
        self.agent.move_rel(x, y, yaw=yaw)
        print(f"[MOVE] HSR moved to relative position ({x}, {y}, {yaw})")
        
    def guide(self, loc):
        print("GPSR Guide Start")
        self.say("Please follow me")
        rospy.sleep(1)
        self.agent.move_abs(loc)
        self.say("Bye bye.")
        rospy.sleep(1)

    def pickCat(self, cat):
        yolo_bbox = self.get_yolo_bbox(cat)

        maware_count = 0
        
        while yolo_bbox == []:
            self.move_rel(0, 0, 1)
            rospy.sleep(1)
            maware_count += 1

            if maware_count > 6:
                self.say(f"Sorry, I can't find any {cat}, can you give me a {cat}?")
                rospy.sleep(4)
                self.agent.open_gripper()
                rospy.sleep(5)
                self.agent.grasp()

                return
        
        obj_id = yolo_bbox[0][4]
        obj = self.objIdToName(obj_id)
        self.pick(obj)

    def pick(self, obj):
        self.say(f"GIVE {obj} to me")
        rospy.sleep(3)
        self.agent.open_gripper()
        self.say("3")
        rospy.sleep(1)
        self.say("2")
        rospy.sleep(1)
        self.say("1")
        rospy.sleep(1)
        self.agent.grasp()

        print(f"[PICK] {obj} is picked up")

    def place(self, loc):
        self.agent.pose.neutral_pose()
        self.say(f"I want to place it to {loc}, please help me")
        rospy.sleep(3)
        self.say("3")
        rospy.sleep(1)
        self.say("2")
        rospy.sleep(1)
        self.say("1")
        rospy.sleep(1)
        self.agent.open_gripper()

    def deliver(self):
        self.say("take this.")
        rospy.sleep(1.5)
        self.say("3")
        rospy.sleep(1)
        self.say("2")
        rospy.sleep(1)
        self.say("1")
        rospy.sleep(1)
        self.agent.pose.neutral_pose()
        self.agent.open_gripper()

    def say(self, text, show_display=True):
        self.agent.say(text, show_display=show_display)

    def img(self):
        return Image.fromarray(cv2.cvtColor(self.agent.rgb_img, cv2.COLOR_RGB2BGR))
        
    def hear(self, len=5.):
        userSpoken = self.agent.stt(len)
        return userSpoken
        
    def talk(self, talk):
        if talk not in self.talk_list:
            talk = self.cluster(talk, self.talk_list)

        if talk == 'something about yourself':
            self.say('I am a robot designed to help people in their daily lives')
            
        elif talk == 'the time':
            self.say(f'The time is {datetime.now().time()}')
            
        elif talk == 'what day is today':
            self.say(f'Today is {datetime.now().date()}')
            
        elif talk == 'what day is tomorrow':
            self.say(f'Tomorrow is {datetime.now().date() + timedelta(days=1)}')
        
        elif talk == 'your teams name':
            self.say('My team name is Tidy Boy')

    def exeFollowup(self, followup):
        followupName, params = ultimateFollowupParser(followup)
        followUpFunc = self.followupNameTofollowupFunc[followupName]
        followUpFunc(self, params)

    def cmdError(self):
        self.say("Sorry, Error in the command")
        rospy.sleep(2.5)

    def quiz(self): 
        self.say("Ask question after the ding sound")
        rospy.sleep(3)
        userSpeech = self.hear(7.)
        additionalPrompt = "Answer easy and short as you can, less than 20 words."
        try:
            robotAns = chat(userSpeech + additionalPrompt)
        except Exception as e:
            print(e)
            print("Error in quiz")
            robotAns = "Question is hard! Good luck."

        self.say(robotAns)
        rospy.sleep(7)

    def cluster(self, word, arr):
        if word in arr:
            return word

        prompt = f"What is the closest pronounciation to the {word} between these words? Only focus on the pronounciation similarities."

        for i, w in enumerate(arr):
            prompt += f"{i+1}. {w} "

        prompt += "you must answer only one number, the number of the word. Do not say the word and alphabet"
        try:
            ans = chat(prompt)
            return arr[int(ans.split('.')[0])-1]
        
        except Exception as e:
            print(e)
            print("Error in clustering with API")

            distances = [(Levenshtein.distance(word, a), a) for a in arr]
            closest_match = min(distances, key=lambda x: x[0])
            return closest_match[1]
            
    def getName(self):
        self.say("Please say your name after ding sound")
        rospy.sleep(3)
        userName = self.hear()
        userName = self.cluster(userName, self.names_list)
        return userName

    def getPose(self, getAll=False):
        self.agent.pose.head_tilt(gpsr_human_head_tilt)
        rospy.sleep(0.5)

        num_features = 55  # 4 (XYXY) + 51 (keypoints)
        combined_data = self.human_keypoints
        print(len(combined_data))
        split_data = [combined_data[i:i + num_features] for i in range(0, len(combined_data), num_features)]

        human_poses = []

        for data in split_data:
            print(data)
            bbox = data[:4]  # XYXY 좌표
            keypoints = data[4:]  # 키포인트 데이터

            input_tensor = torch.tensor(keypoints, dtype=torch.float32).unsqueeze(0)  # Add batch dimension

            # Perform the inference
            with torch.no_grad():
                output = self.pose_model(input_tensor)

            # Apply softmax to get probabilities
            probabilities = F.softmax(output, dim=1)
            
            # Print the probabilities
            print("Probabilities:", probabilities.data)

            # Get the highest probability and its index
            confidence, predicted_label = torch.max(probabilities, 1)
            
            confidence = confidence.item()
            predicted = predicted_label.item()

            pose_label = ''
            if predicted == 0:
                pose_label = 'sitting person'
            elif predicted == 1:
                pose_label = 'standing person'
            elif predicted == 2:
                pose_label = 'lying person'

            human_poses.append((pose_label, confidence, bbox))

        print('human_poses', human_poses)


        if getAll:
            return human_poses
        
        if len(human_poses) == 0:
            return "no person"
        
        return human_poses[0][0]
    
    def getGest(self, getAll=False):
        self.agent.pose.head_tilt(gpsr_human_head_tilt)
        rospy.sleep(0.5)

        num_features = 55  # 4 (XYXY) + 51 (keypoints)
        combined_data = self.human_keypoints
        print(len(combined_data))
        split_data = [combined_data[i:i + num_features] for i in range(0, len(combined_data), num_features)]

        human_gests = []

        for data in split_data:
            print(data)
            bbox = data[:4]  # XYXY 좌표
            keypoints = data[4:]  # 키포인트 데이터
            input_tensor = torch.tensor(keypoints, dtype=torch.float32).unsqueeze(0)  # Add batch dimension

            # Perform the inference
            with torch.no_grad():
                output = self.gest_model(input_tensor)
            
            # Apply softmax to get probabilities
            probabilities = F.softmax(output, dim=1)
            
            # Print the probabilities
            print("Probabilities:", probabilities.data)

            # Get the highest probability and its index
            confidence, predicted_label = torch.max(probabilities, 1)
            
            confidence = confidence.item()
            predicted = predicted_label.item()

            gest_label = ''
            if predicted == 0:
                gest_label = 'person raising their left arm'
            elif predicted == 1:
                gest_label = 'person raising their right arm'
            elif predicted == 2:
                gest_label = 'person pointing to the right'
            elif predicted == 3:
                gest_label = 'person pointing to the left'

            human_gests.append((gest_label, confidence, bbox))

        print('human_gests', human_gests)

        if getAll:
            return human_gests
        
        if len(human_gests) == 0:
            return "no person"
        
        return human_gests[0][0]
        
    def getCloth(self):
        noPersonCount = 0
        self.agent.pose.head_tilt(gpsr_human_head_tilt)

        while True:
            image = self.img()
            personCount = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device)

            if noPersonCount > 20:
                print("No person detected, finish getGest")
                return "no person"
            
            if personCount[0] == "no person":
                noPersonCount += 1
                print("No person detected", noPersonCount)
                continue

            print(f"Person detected: {personCount[0]}")
            feature = detectColorCloth(image, self.clip_model, self.preprocess, self.tokenizer, self.device)
            break

        return feature
        
    def identify(self, type='default', pose=None, gest=None):
        maware_count = 0
        while True:
            ### Check identify Type
            if type == 'default':
                if self.human_keypoints:
                    break
            
            if type == 'pose':
                human_poses = self.getPose(getAll=True)
                if pose in human_poses:
                    break
                
            if type == 'gest':
                human_gests = self.getGest(getAll=True)
                if gest in human_gests:
                    break
                
                
            ### To many maware? finish
            if maware_count == 6:
                break
            
            ### Move to find human
            if maware_count % 3 == 0:
                self.move_rel(0, 0, math.pi/8)
            elif maware_count % 3 == 1:
                self.move_rel(0, 0, -math.pi/4)
            elif maware_count % 3 == 2:
                self.move_rel(0, 0, math.pi/8)
                
            maware_count += 1
            rospy.sleep(1)
                
        self.say("I found you")
        rospy.sleep(1)

    # 이름을 가진 사람 앞에서 멈추기
    def identifyByName(self, name):
        self.say(f"{name}, please come closer to me.")
        rospy.sleep(3)
        self.say("three")
        rospy.sleep(1)
        self.say("two")
        rospy.sleep(1)
        self.say("one")
        rospy.sleep(1)
        # [TODO] Implement how the name can be identified

    # 어떤 제스처나 포즈를 가진 사람 앞에서 멈추기
    def identifyByGestPose(self, gestPosePers):
        if gestPosePers in self.pose_person_list:
            self.identify(type='pose', pose=gestPosePers)

        # Gesture
        elif gestPosePers in self.gesture_person_list:
            self.identify(type='gest', gest=gestPosePers)

        else:
            rospy.logwarn("No such gesture or pose")
            self.identify()

        self.say(f"I found a person who is {gestPosePers}. Let's go.")
        rospy.sleep(3)

    # 어떤 포즈나 제스쳐 취하고 있는 사람 수 세기
    def countGestPosePers(self, gestPosePers):
        self.say("every guys, please come in my sight.")
        rospy.sleep(4)       
        
        if gestPosePers in self.pose_person_list:
            Pers = self.getPose(getAll=True)
            posePers = [per for per in Pers if per[0] == gestPosePers]
            return len(posePers)

        elif gestPosePers in self.gesture_person_list:
            Pers = self.getGest(getAll=True)
            gestPers = [per for per in Pers if per[0] == gestPosePers]
            return len(gestPers)
        
        else:
            rospy.logwarn("No such gesture or pose")
            return 0
        
    # 옷으로 찾기
    def identifyByClothing(self, Clothes):
        self.say(f"who wear {Clothes}, please come closer to me.")
        rospy.sleep(4)

        self.identify()
    
    # 어떤 색의 옷을 입고 있는 사람 수 세기
    def countColorClothesPers(self, colorClothes):
        image = self.img()
        personCount, _ = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device, type="colorCloth", key=colorClothes)

        return personCount
    
    def getHumanAttribute(self):
        return self.getCloth()
        
    # getHumanAttribute로 가져온 humanAttribute에 해당하는 사람을 찾기 위해 쓰임
    def identifyByHumanAttribute(self, humanAttribute):
        self.identifyByClothing(humanAttribute)
    
    def objIdToName(self, id):
        return self.agent.yolo_module.find_name_by_id(id)
    
    def findBiggestObjId(self, yolo_bbox):
        ObjIdArea = [(objInfo[4], objInfo[2] * objInfo[3]) for objInfo in yolo_bbox]
        return max(ObjIdArea, key=lambda x: x[1])[0]
    
    def findSmallestObjId(self, yolo_bbox):
        ObjIdArea = [(objInfo[4], objInfo[2] * objInfo[3]) for objInfo in yolo_bbox]
        return min(ObjIdArea, key=lambda x: x[1])[0]
    
    def findThinnestObjId(self, yolo_bbox):
        objIdThinLen = [(objInfo[4], min(objInfo[2], objInfo[3])) for objInfo in yolo_bbox]
        return min(objIdThinLen, key=lambda x: x[1])[0]
    
    def findHeaviestObjId(self, yolo_bbox):
        # [TODO] Implement how the heaviest object can be found
        return self.findBiggestObjId(yolo_bbox)
    
    def findLightestObjId(self, yolo_bbox):
        # [TODO] Implement how the lightest object can be found
        return self.findSmallestObjId(yolo_bbox)
    
    

# MAIN
def gpsr(agent):
    yolov10_path = "/home/tidy/Robocup2024/yolov10.sh"
    yolov7_pose_path = "/home/tidy/Robocup2024/yolov7_pose.sh"
    
    yolov10_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {yolov10_path}; exec bash']
    yolov7_pose_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {yolov7_pose_path}; exec bash']

    # yolov10_process = subprocess.Popen(yolov10_command)
    yolov7_pose_process = subprocess.Popen(yolov7_pose_command)

    g = GPSR(agent)

    while g.task_finished_count < task_iteration:

        g.move('gpsr_instruction_point')

        # Get input with STT
        agent.say("Give a command after the ding sound.")
        rospy.sleep(2.5)

        inputText = g.hear(7.)

        agent.say(f"Given Command is {inputText}")

        try:            
            # parse InputText 
            cmdName, params = ultimateParser(inputText)

            cmdName = g.cluster(cmdName, g.cmdNameTocmdFunc.keys())
            cmdFunc = g.cmdNameTocmdFunc[cmdName]
            
            cmdFunc(g, params)
        
        except:
            g.say("Sorry. Another command please.")
            rospy.sleep(2)
            pass