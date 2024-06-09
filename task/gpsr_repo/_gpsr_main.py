import rospy
from datetime import datetime, timedelta

from gpsr_cmds import *
from gpsr_followup import *
from gpsr_parser import *
from gpsr_utils import *
from gpsr_clip import *
from gpsr_config import *
from gpsr_follow import *

from PIL import Image
import cv2

from std_msgs.msg import Int16MultiArray

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



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
        

        self.object_names, self.object_categories_plural, self.object_categories_singular = parseObjects(objects_data)
        
        self.category2objDict, self.categoryPlur2Sing, self.categorySing2Plur = extractCategoryToObj(objects_data)

        rospy.Subscriber('/snu/openpose/knee', Int16MultiArray, self._knee_pose_callback)

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
        self.clip_model, self.preprocess, self.tokenizer, self.device = init_clip()

        # FOLLOW
        # self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # self.following = False
        # self.person_distance = None
        # self.person_angle = None

    # CALLBACKS
    def _knee_pose_callback(self, msg):
        rospy.loginfo(msg.data)

    ### HELP Functions ###
        
    def follow(self):
        bag_search_limit_time = 15
        goal_radius = 0.5
        pose_save_time_period = 3
        start_location = self.agent.get_pose(print_option=False)
        bag_height = 0.25
        stop_rotate_velocity = 1.2 #1.2
        try_bag_picking = False #True
        try_bytetrack = False
        map_mode = False
        stt_option = False #True
        yolo_success = True
        tilt_angle = 20
        

        # Capture target
        demotrack_pub = rospy.Publisher('/snu/demotrack', String, queue_size=10)

        human_reid_and_follower = HumanReidAndFollower(init_bbox=[320 - 100, 240 - 50, 320 + 100, 240 + 50],
                                                    frame_shape=(480, 640),
                                                    stop_thres=.4,
                                                    linear_max=.3,
                                                    angular_max=.2,
                                                    tilt_angle=tilt_angle)
        human_following = HumanFollowing(self.agent, human_reid_and_follower, start_location, goal_radius, stop_rotate_velocity, tilt_angle, stt_option)

        while not rospy.is_shutdown():
            end_following = human_following.follow_human(time.time(), pose_save_time_period)
            if end_following:
                human_following.show_byte_track_image = False
                stop_client = rospy.ServiceProxy('/viewpoint_controller/start', Empty)
                print("VIEWPOINT CONTROLLER ON")
                stop_client.call(EmptyRequest())
                break

    def followToLoc(self, loc):
        self.follow()

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
        self.say(f"Please follow me")
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
        # find object
        # [TODO] Implement how the object can be picked up
        if False:
            self.agent.pose.pick_side_pose('grocery_table_pose2')
            self.agent.open_gripper()
            self.agent.move_rel(0, 0.5, wait=True)
            self.agent.move_rel(0.05, 0, wait=True)
            self.agent.grasp()
            self.agent.pose.pick_side_pose('grocery_table_pose1')

        else:
            self.agent.say(f"GIVE {obj} to me")
            rospy.sleep(3)
            self.agent.open_gripper()
            rospy.sleep(5)
            self.agent.grasp()

        print(f"[PICK] {obj} is picked up")

    def place(self, loc):
        self.agent.pose.neutral_pose()
        self.agent.open_gripper()

    def deliver(self):
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
        robotAns = chat(userSpeech + additionalPrompt)
        self.say(robotAns)
        rospy.sleep(7)
        # [TODO] improve how the quiz can be answered

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
            print("Error in clustering")

            ## TODO 단어 거리 하드코딩
            return word
            
    # TODO
    def getName(self):
        self.say("Please say your name after ding sound")
        rospy.sleep(3)
        userName = self.hear()
        userName = self.cluster(userName, self.names_list)
        return userName
    
    def getPose(self):
        noPersonCount = 0
        self.agent.pose.head_tilt(0)
        while True:
            image = self.img()
            personCount = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device)

            if noPersonCount > 20:
                print("No person detected, finish getPose")
                return "no person"

            if personCount[0] == "no person":
                noPersonCount += 1
                print("No person detected", noPersonCount)
                continue

            print(f"Person detected: {personCount[0]}")
            feature, _ = detectPose(image, self.clip_model, self.preprocess, self.tokenizer, self.device)
            break

        return feature
    
    def getGest(self):
        noPersonCount = 0
        self.agent.pose.head_tilt(5)
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
            feature, _ = detectGest(image, self.clip_model, self.preprocess, self.tokenizer, self.device)
            if feature == "no pose":
                print("No pose detected")
                continue
            
            break

        return feature
    
    def getCloth(self):
        noPersonCount = 0
        self.agent.pose.head_tilt(5)

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
        

    def identify(self):
        noPersonCount = 0
        maxPersonCount = 7

        while True:
            self.agent.pose.head_tilt(5)
            image = self.img()
            personCount = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device)

            if noPersonCount > maxPersonCount:
                print("No person detected, finish getGest")
                self.move_rel(0, 0, 1.2)
                rospy.sleep(1)
                noPersonCount = 0
            
            if personCount[0] == "no person":
                noPersonCount += 1
                print("No person detected", noPersonCount)
                continue

            print(f"Person detected: {personCount[0]}")

            for i in range(maxPersonCount):
                image = self.img()
                personCount = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device)

                if personCount[0] != "no person":
                    break

            if i == maxPersonCount - 1:
                continue
            
            self.say("I found you")
            rospy.sleep(1)
            break

    # 어떤 제스처나 포즈를 가진 사람 앞에서 멈추기
    def identifyByGestPose(self, gestPosePers):
        self.identify()
        return

        self.agent.pose.head_tilt(5)
        poseCount = 0

        if gestPosePers in ['standing', 'lying', 'sitting']:
            print("Identify by pose")

            while True:
                feature = self.getPose()
                print("feature", feature)

                if feature != gestPosePers:
                    print(f"No {gestPosePers} detected")
                    self.move_rel(0, 0, 0.5)
                    rospy.sleep(1)
                    continue

                if poseCount == 2:
                    break

                else:
                    poseCount += 1

        # Gesture
        else:
            print("Identify by gesture")

            while True:
                feature = self.getGest()
                print("feature", feature)

        
                if feature != gestPosePers:
                    print(f"No {gestPosePers} detected")
                    self.move_rel(0, 0, 0.5)
                    rospy.sleep(1)
                    continue

                if poseCount == 2:
                    break

                else:
                    poseCount += 1

        self.say(f"I found a person who is {gestPosePers}. Let's go.")
        rospy.sleep(3)
    
    # 옷으로 찾기
    def identifyByClothing(self, Clothes):
        self.say(f"who wear {Clothes}, please come closer to me.")
        self.agent.pose.head_tilt(5)
        
        rospy.sleep(4)

        self.identify()

    # 어떤 포즈나 제스쳐 취하고 있는 사람 수 세기
    def countGestPosePers(self, gestPosePers):
        if gestPosePers in ['standing', 'lying', 'sitting']:
            image = self.img()
            personCount, _ = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device, type="pose", key=gestPosePers)

        else:
            image = self.img()
            personCount, _ = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device, type="gest", key=gestPosePers)
        
        return personCount
    
    # 어떤 색의 옷을 입고 있는 사람 수 세기
    def countColorClothesPers(self, colorClothes):
        image = self.img()
        personCount, _ = detectPersonCount(image, self.clip_model, self.preprocess, self.tokenizer, self.device, type="colorCloth", key=colorClothes)

        return personCount
    
    def getHumanAttribute(self):
        # [TODO] Implement how the human attributes can be extracted
        return None
    
    # getHumanAttribute로 가져온 humanAttribute에 해당하는 사람을 찾기 위해 쓰임
    def identifyByHumanAttribute(self, humanAttribute):
        # [TODO] Implement how the human attributes can be identified
        pass
    
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
    g = GPSR(agent)

    # inputText = "Bring me an apple from the desk" #bringMeOjbFromPlcmt
    # inputText = "Tell me how many drinks there are on the desk" #countObjOnPlcmt
    # inputText = "Tell me what is the biggest food on the desk" #tellCatPropOnPlcmt
    # inputText = "Tell me what is the biggest object on the table" #tellObjPropOnPlcmt
    # inputText = "Answer the quiz of the person raising their left arm in the kitchen" #answerToGestPrsInRoom
    # inputText = "Tell me how many people in the kitchen are wearing white t shirts" #countClothPrsInRoom
    # inputText = "Tell me how many lying persons are in the kitchen" #countPrsInRoom
    # inputText = "Find a food in the kitchen then grasp it and put it on the bed" #findObjInRoom
    #############################  DONE  #############################
    # inputText = "Follow Angel from the desk lamp to the office" #followNameFromBeacToRoom
    # inputText = "Follow the standing person in the bedroom" #followPrsAtLoc
    # inputText = "Go to the bedroom then find a food and get it and bring it to the waving person in the kitchen" #goToLoc
    # inputText = "Introduce yourself to the person wearing an orange coat in the bedroom and answer a quiz" #greetClothDscInRm
    # inputText = "Say hello to Jules in the living_room and tell the time" #greetNameInRm
    # inputText = "Take the person wearing a white shirt from the living_room to the kitchen_table" #guideClothPrsFromBeacToBeac
    # inputText = "Lead Paris from the lamp to the kitchen" #guideNameFromBeacToBeac
    # inputText = "Lead the person raising their right arm from the bookshelf to the office" #guidePrsFromBeacToBeac
    # inputText = "Meet Charlie at the shelf then find them in the living room" #meetNameAtLocThenFindInRm
    # inputText = "Meet Jules in the living room and follow them" #meetPrsAtBeac
    # inputText = "Take a cola from the desk and put it on the sofa" #takeObjFromPlcmt
    # inputText = "Tell the day of the week to the person pointing to the left in the kitchen" #talkInfoToGestPrsInRoom
    # inputText = "Tell the name of the person at the kitchen to the person at the desk" #tellPrsInfoAtLocToPrsAtLoc
    # inputText = "Tell me the name of the person at the trashbin" #tellPrsInfoInLoc

    while g.task_finished_count < task_iteration:

        g.move('gpsr_instruction_point')

        # Get input with STT
        agent.say("Give a command after the ding sound.")
        rospy.sleep(2.5)

        inputText = g.hear(7.)
        agent.say(f"Given Command is {inputText}")

            
        # parse InputText 
        cmdName, params = ultimateParser(inputText)

        cmdName = g.cluster(cmdName, g.cmdNameTocmdFunc.keys())
        
        cmdFunc = g.cmdNameTocmdFunc[cmdName]
        cmdFunc(g, params)

        g.move('gpsr_instruction_point')