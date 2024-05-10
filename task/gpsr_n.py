import sys
sys.path.append('task/gpsr_repo/')

from gpsr_cmds import *
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
        self.verbType2verb = {
        "{takeVerb}": ["take", "get", "grasp", "fetch"],
        "{placeVerb}": ["put", "place"],
        "{deliverVerb}": ["bring", "give", "deliver"],
        "{bringVerb}": ["bring", "give"],
        "{goVerb}": ["go", "navigate"],
        "{findVerb}": ["find", "locate", "look for"],
        "{talkVerb}": ["tell", "say"],
        "{answerVerb}": ["answer"],
        "{meetVerb}": ["meet"],
        "{tellVerb}": ["tell"],
        "{greetVerb}": ["greet", "salute", "say hello to", "introduce yourself to"],
        "{countVerb}": ["tell me how many"],
        "{followVerb}": ["follow"],
        "{guideVerb}": ["guide", "escort", "take", "lead"],
        # "{rememberVerb}": ["meet", "contact", "get to know", "get acquainted with"],
        # "{describeVerb}": ["tell me how", "describe"],
        # "{offerVerb}": ["offer"],
        # "{accompanyVerb}": ["accompany"]
    }
        self.verbType2cmdName = {
        "{takeVerb}": ["takeObjFromPlcmt"],
        "{placeVerb}": [],
        "{deliverVerb}": [],
        "{bringVerb}": ["bringMeObjFromPlcmt"],
        "{goVerb}": ["goToLoc"],
        "{findVerb}": ["findPrsInRoom", "findObjInRoom"],
        "{talkVerb}": ["talkInfoToGestPrsInRoom"],
        "{answerVerb}": ["answerToGestPrsInRoom"],
        "{meetVerb}": ["meetPrsAtBeac", "meetNameAtLocThenFindInRm"],
        "{tellVerb}": ["tellPrsInfoInLoc", "tellObjPropOnPlcmt", "tellCatPropOnPlcmt"],
        "{greetVerb}": ["greetClothDscInRm", "greetNameInRm"],
        "{countVerb}": ["countObjOnPlcmt", "countPrsInRoom", "countClothPrsInRoom"],
        "{followVerb}": ["followNameFromBeacToRoom", "followPrsAtLoc"],
        "{guideVerb}": ["guideNameFromBeacToBeac", "guidePrsFromBeacToBeac", "guideClothPrsFromBeacToBeac"],
        # "remember": [],
        # "describe": [],
        # "offer": [],
        # "accompany": []
    }
        self.cmdName2cmdStr = {
        "goToLoc": "{goVerb} {toLocPrep} the {loc_room} then {followup}",
        "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
        "findPrsInRoom": "{findVerb} a {gestPers_posePers} {inLocPrep} the {room} and {followup}",
        "findObjInRoom": "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then {followup}",
        "meetPrsAtBeac": "{meetVerb} {name} {inLocPrep} the {room} and {followup}",
        "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
        "countPrsInRoom": "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}",
        "tellPrsInfoInLoc": "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}",
        "tellObjPropOnPlcmt": "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}",
        "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
        "answerToGestPrsInRoom": "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}",
        "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}",
        "guideNameFromBeacToBeac": "{guideVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
        "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
        "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
        "bringMeObjFromPlcmt": "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}",
        "tellCatPropOnPlcmt": "{tellVerb} me what is the {objComp} {singCat} {onLocPrep} the {plcmtLoc}",
        "greetClothDscInRm": "{greetVerb} the person wearing {art} {colorClothe} {inLocPrep} the {room} and {followup}",
        "greetNameInRm": "{greetVerb} {name} {inLocPrep} the {room} and {followup}",
        "meetNameAtLocThenFindInRm": "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}",
        "countClothPrsInRoom": "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}",
        "tellPrsInfoAtLocToPrsAtLoc": "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}",
        "followPrsAtLoc": "{followVerb} the {gestPers_posePers} {inRoom_atLoc}"
    }
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

        # verbType2followUpStr
        # followUpStr2followUp
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

    # agent.say("I'm ready to receive a command")
    # rospy.sleep(4)

    # inputText, _ = agent.stt(10.)
    # cmdName, params = ultimateParser(inputText)

    # agent.say(f"Given Command is {cmdName}, Given Parameters {params}")
    # cmdFunc = cmdName2cmdFunc[cmdName]

    # cmdFunc(agent, params)

    print('gpsr start')

    m = int(input("Enter the mode: "))

    if m == 1:
        countObjOnPlcmt(g, {})
    if m == 2:
        tellObjPropOnPlcmt(g, {})
    if m == 3:
        tellCatPropOnPlcmt(g, {})
    if m == 4:
        bringMeObjFromPlcmt(g, {})

    # TODO : repeat 3 times, return to the instruction loc