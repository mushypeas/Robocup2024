import rospy
import json
from datetime import datetime, timedelta

from gpsr_cmds import *
from gpsr_followup import *
from gpsr_parser import *
from gpsr_utils import *

objects_file_path = 'task/gpsr_repo/object.md'
objects_data = readData(objects_file_path)

def followup(cmd):
    print(cmd)
    
class GPSR:
    def __init__(self, agent):
        self.agent = agent
        
        self.cmdNameTocmdFunc = {
            "goToLoc": goToLoc,
            "takeObjFromPlcmt": takeObjFromPlcmt,
            "findPrsInRoom": findPrsInRoom,
            "findObjInRoom": findObjInRoom,
            "meetPrsAtBeac": meetPrsAtBeac,
            "countObjOnPlcmt": countObjOnPlcmt,
            "countPrsInRoom": countPrsInRoom,
            "tellPrsInfoInLoc": tellPrsInfoInLoc,
            "tellObjPropOnPlcmt": tellObjPropOnPlcmt,
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
            # "findObj": findObj,
            # "findPrs": findPrs,
            # "meetName": meetName,
            # "placeObjOnPlcmt": placeObjOnPlcmt,
            "deliverObjToMe": deliverObjToMe,
            # "deliverObjToPrsInRoom": deliverObjToPrsInRoom,
            # "deliverObjToNameAtBeac": deliverObjToNameAtBeac,
            # "talkInfo": talkInfo,
            # "answerQuestion": answerQuestion,
            # "followPrs": followPrs,
            # "followPrsToRoom": followPrsToRoom,
            # "guidePrsToBeacon": guidePrsToBeacon,
            # "takeObj": takeObj
        }

        self.object_names, self.object_categories_plural, self.object_categories_singular = parseObjects(objects_data)
        self.category2objDict, self.categoryPlur2Sing, self.categorySing2Plur = extractCategoryToObj(objects_data)

    ### HELP Functions ###
        
    ## TODO : Implement yolo id2name & name2id
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
        
    def guide(self, loc):
        print("GPSR Guide Start")
        self.say("I will guide you to the location")
        self.agent.move_abs(loc)

    def pick(self, obj):
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
        self.agent.open_gripper()
        self.agent.pose.neutral_pose()
        
    def deliver(self):
        self.agent.open_gripper()
        self.agent.pose.neutral_pose()

    def say(self, text):
        self.agent.say(text)
        
    def talk(self, talk):
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
            
    def quiz(self):
        # [TODO] Implement how the quiz can be answered
        pass
            
    # TODO
    def getName(self):
        # [TODO] Implement how the name can be extracted
        return None
    
    # TODO
    def getPose(self):
        # [TODO] Implement how the pose can be extracted
        return None
    
    def getGest(self):
        # [TODO] Implement how the gesture can be extracted
        return None
    
    def getHumanAttribute(self):
        # [TODO] Implement how the human attributes can be extracted
        return None
        
    # 사람 앞에서 멈추기
    def identifyByName(self, name):
        # [TODO] Implement how the name can be identified
        pass
        
    def identifyByGestPose(self, gestPosePers):
        # [TODO] Implement how the pose can be identified
        pass
    
    def identifyByHumanAttribute(self, humanAttribute):
        # [TODO] Implement how the human attributes can be identified
        pass
    
    def identifyByClothing(self, Clothes):
        # [TODO] Implement how the clothes can be identified
        pass
    
    def countGestPosePers(self, gestPosePers):
        # [TODO] Implement how the pose can be counted
        return None
    
    def countColorClothesPers(self, colorClothes):
        # [TODO] Implement how the color of clothes can be counted
        return None
    
    def follow(self):
        # [TODO] Implement how the person can be followed
        pass
    
    def followToLoc(self, loc):
        # [TODO] Implement how the person can be followed to the location
        pass
    
    def extractLocFrominRoomatLoc(self, inRoom_atLoc):
        # [TODO] Implement how the location can be extracted
        return None
    
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
        return None
    
    def findLightestObjId(self, yolo_bbox):
        # [TODO] Implement how the lightest object can be found
        return None
    
    def exeFollowup(self, followup):
        followupName, params = ultimateFollowupParser(followup)
        followUpFunc = self.followupNameTofollowupFunc[followupName]
        followUpFunc(self, params)
    

# MAIN
def gpsr(agent):
    g = GPSR(agent)
    # TODO : goto the instruction loc

    # # Get input with STT
    # agent.say("I'm ready to receive a command")
    # rospy.sleep(4)

    # inputText, _ = agent.stt(10.)
    # agent.say(f"Given Command is {inputText}")

    inputText = "bring me a peach from the table"
    
    # parse InputText 
    cmdName, params = ultimateParser(inputText)
    
    cmdFunc = g.cmdNameTocmdFunc[cmdName]
    cmdFunc(g, params)

    # TODO : repeat 3 times, return to the instruction loc