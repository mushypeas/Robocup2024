import sys
sys.path.append('task/gpsr_repo/')

from gpsr_cmds import *
from gpsr_followup import *
from gpsr_parser import *
from gpsr_utils import *

import rospy
import json

objects_file_path = 'task/gpsr_repo/object.md'
objects_data = read_data(objects_file_path)

def followup(cmd):
    print(cmd)
    
class GPSR:
    def __init__(self, agent):
        self.agent = agent
        
        self.cmdName2cmdFunc = {
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
        
        self.followupName2followUpFunc = {
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

        self.object_names, self.object_categories_plural, self.object_categories_singular = parse_objects(objects_data)
        self.category2objDict, self.categoryPlur2Sing, self.categorySing2Plur = extractCategory2obj(objects_data)

    ### HELP Functions ###
        
    ## TODO : Implement yolo id2name & name2id
    def get_yolo_bbox(self, category=None):
        yolo_bbox = self.agent.yolo_module.yolo_bbox

        if category:
            categoryItems = self.category2objDict[category]
            print("categoryItems", categoryItems)
            yolo_bbox = [obj for obj in yolo_bbox if self.agent.yolo_module.find_name_by_id(obj[4]) in categoryItems]
        
        print("yolo_bbox", yolo_bbox)

        return yolo_bbox

    def move(self, loc):
        self.agent.move_abs(loc)
        print(f"[MOVE] HSR moved to {loc}")

    def pick(self, obj, loc=None):
        if loc:
            self.agent.move_abs(loc)
            rospy.sleep(2)

        # [TODO] Implement how the object can be picked up
        if False:
            pass

        else:
            self.agent.say(f"GIVE {obj} to me")
            rospy.sleep(3)
            self.agent.open_gripper()
            rospy.sleep(5)
            self.agent.grasp()

        print(f"[PICK] {obj} is picked up")

    def place(self):
        self.agent.open_gripper()

    def say(self, text):
        self.agent.say(text)

cmdName2cmdFunc = {
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

# MAIN
def gpsr(agent):
    g = GPSR(agent)
    # TODO : goto the instruction loc

    # # Get input with STT
    # agent.say("I'm ready to receive a command")
    # rospy.sleep(4)

    # inputText, _ = agent.stt(10.)
    # agent.say(f"Given Command is {inputText}")
    
    inputText = "Give me a baseball from the bedside table"
    
    # parse InputText 
    cmdName, params = ultimateParser(inputText)
    
    cmdFunc = cmdName2cmdFunc[cmdName]
    cmdFunc(agent, params)

    # TODO : repeat 3 times, return to the instruction loc