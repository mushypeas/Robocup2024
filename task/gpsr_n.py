import rospy
import openai
import json
import re
import warnings

# Read data from file
def read_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    return data

# Parse object.md
def parse_objects(data):
    parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
    parsed_objects = [objects for objects in parsed_objects if objects != 'Objectname']
    parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
    parsed_objects = [objects.strip() for objects in parsed_objects]

    parsed_categories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
    parsed_categories = [category.strip() for category in parsed_categories]
    parsed_categories = [category.replace('(', '').replace(')', '').split() for category in parsed_categories]
    parsed_categories_plural = [category[0] for category in parsed_categories]
    parsed_categories_plural = [category.replace("_", " ") for category in parsed_categories_plural]
    parsed_categories_singular = [category[1] for category in parsed_categories]
    parsed_categories_singular = [category.replace("_", " ") for category in parsed_categories_singular]

    if parsed_objects or parsed_categories:
        return parsed_objects, parsed_categories_plural, parsed_categories_singular
    else:
        warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
        return []

# Make Category to Object Dictionary
def extractCategory2obj(markdown_content):
    category_pattern = re.compile(r'\# Class (\w+) \((\w+)\)')
    object_pattern = re.compile(r'\| (\w+)  \|')

    objects_dict = {}
    current_category = None

    for line in markdown_content.split('\n'):
        category_match = category_pattern.match(line)
        if category_match:
            current_category = category_match.group(2)
            objects_dict[current_category] = []

        object_match = object_pattern.match(line)
        if object_match and current_category:
            object_name = object_match.group(1)
            objects_dict[current_category].append(object_name)

    return objects_dict

objects_file_path = 'task/gpsr_repo/object.md'
objects_data = read_data(objects_file_path)
object_names, object_categories_plural, object_categories_singular = parse_objects(objects_data)
category2objDict = extractCategory2obj(objects_data)



def followup(cmd):
    print(cmd)


### HELP Functions ###
def get_yolo_bbox(agent, category=None):
    yolo_bbox = agent.yolo_module.yolo_bbox

    if category:
        print(category)
        print(category2objDict)
        categoryItems = category2objDict[category]
        yolo_bbox = [obj for obj in yolo_bbox if agent.yolo_module.find_name_by_id(obj[4]) in categoryItems]

    while len(yolo_bbox) == 0:
        print("No objects detected")
        rospy.sleep(1)
        yolo_bbox = agent.yolo_module.yolo_bbox

    return yolo_bbox

def move_gpsr(agent, loc):
    agent.move_abs(loc)
    rospy.sleep(2)  
    print(f"[MOVE] HSR moved to {loc}")

def pick(obj):
    # [TODO] Implement how the object can be picked up
    print(f"[PICK] {obj} is picked up")

def place(obj, loc):
    # [TODO] Implement how the object can be placed at the location
    print(f"[PLACE] {obj} is placed at {loc}")

### HRI and People Perception Commands ###
# "goToLoc": "{goVerb} {toLocPrep} the {loc_room} then {followup}",
def goToLoc(agent, params):
    # Go to the storage rack then look for a dish and take it and bring it to me
    # Go to the bedroom then find a food and get it and bring it to the waving person in the kitchen
    # Go to the living room then find a fruit and get it and give it to me
    # Go to the kitchen then look for a cleaning supply and grasp it and bring it to the standing person in the kitchen
    
    # [0] Extract parameters
    goVerb, toLocPrep, loc_room, cmd = params['goVerb'], params['toLocPrep'], params['loc_room'], params['followup']
    
    # [1] Move to the specified room
    print(f"I'm moving to the {loc_room}")
    agent.say(f"I'm moving to the {loc_room}")
    move_gpsr(agent, loc_room)

    # [2] Handling the follow-up action
    followup(cmd)

# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def findPrsInRoom(agent, params):
    # Find a person pointing to the left in the bedroom and take them to the dishwasher
    # Find a lying person in the bedroom and lead them to the potted plant
    # Locate a waving person in the office and answer a quiz
    
    # [0] Extract parameters
    find, human, room, cmd = params['findVerb'], params['gestPers_posePers'], params['room'], params['followup']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person with that pose in the room
    print(f"[FIND] {find} a {human} in the {room}")

    # [2-1] find waving person
    # [2-2] find person raising their left/right arm
    # [2-3] find person pointing to the left/right
    # [2-4] find sitting/standing/lying person

    # [3] Follow-up command
    followup(cmd)

# "meetPrsAtBeac": "{meetVerb} {name} {inLocPrep} the {room}",
def meetPrsAtBeac(agent, params):
    # Meet Jules in the living room and follow them
    # Meet Angel in the kitchen and take them to the shelf
    # Meet Simone in the living room and follow them
    # Meet Axel in the office and follow them to the bedroom
    
    # [0] Extract parameters
    name, room, cmd = params['name'], params['room'], params['followup']  

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] find the person
    # face recognition?
    print(f"[FIND] {name}")

    # [3] Generate the followup comman
    followup(cmd)

# "countPrsInRoom": "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}",
def countPrsInRoom(agent, params):
    # Tell me how many waving persons are in the living room
    # Tell me how many persons pointing to the right are in the kitchen
    # Tell me how many persons pointing to the left are in the bedroom
    # Tell me how many lying persons are in the living room
    
    # [0] Extract parameters
    count, human, room = params['countVerb'], params['gestPers_posePers'], params['room']

    # [1] move to the specified room
    move_gpsr(agent, room)

    # [2] Count the number of persons in the room
    print(f"[COUNT] {count} {human} in the {room}")

# "tellPrsInfoInLoc": "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}",
def tellPrsInfoInLoc(agent, params):
    # Tell me the name of the person at the trashbin
    # Tell me the pose of the person at the storage rack
    # Tell me the gesture of the person in the office
    # Tell me the name of the person at the cabinet

    # params= {'tellVerb': 'Tell', 'persInfo': 'gesture', 'inRoom_atLoc': 'at the trashbin'}    
    # [0] Extract parameters
    tellVerb, persInfo, loc = params['tellVerb'], params['persInfo'], params['inRoom_atLoc']
    
    # [1] Move to the specifed room
    print(f"I'm moving to the {loc}")

    # [2] Find the person in the room
    print(f"Humans are detected in the {loc}")

    # [3] Get the information of the person
    if persInfo == 'name':
        # Face Detection
        pass
    elif persInfo == 'pose' or persInfo == 'gesture':
        # OpenPose
        print(f"[INFO] {tellVerb} me the gesture of the person {loc}")

# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def talkInfoToGestPrsInRoom(agent, params):
    # Tell the day of the week to the person pointing to the left in the office
    # Tell something about yourself to the person raising their right arm in the kitchen
    # Tell the time to the person raising their right arm in the living room
    # Tell your teams affiliation to the person pointing to the right in the bathroom
    
    # params = {'talkVerb': 'Tell', 'talk': 'the day of the week', 'talkPrep': 'to', 'gestPers': 'person pointing to the left', 'inLocPrep': 'in', 'room': 'office'}
    # [0] Extract parameters
    talkVerb, talk, talkPrep, gestPers, inLocPrep, room = params['talkVerb'], params['talk'], params['talkPrep'], params['gestPers'], params['inLocPrep'], params['room']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person in the room
    # [TODO] Gesture Detection with OpenPose
    print(f"[FIND] {gestPers}")

    # [3] Talk to the person
    # [TODO] Speech Synthesis
    #     talk_list = ["something about yourself", "the time", "what day is today", "what day is tomorrow", "your teams name",
                #  "your teams country", "your teams affiliation", "the day of the week", "the day of the month"]
    print(f"[TALK] {talkVerb} {talk} {talkPrep} {gestPers} {inLocPrep} the {room}")

# "answerToGestPrsInRoom": "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}",
def answerToGestPrsInRoom(agent, params):
    # Answer the question of the person raising their right arm in the bedroom
    # Answer the quiz of the person raising their left arm in the kitchen
    # Answer the quiz of the person raising their left arm in the living room
    # Answer the question of the person raising their left arm in the bedroom

    # params = {'answerVerb': 'Answer', 'question': 'quiz', 'ofPrsPrep': 'of', 'gestPers': 'person raising their left arm', 'inLocPrep': 'in', 'room': 'kitchen'}
    # [0] Extract parameters
    answerVerb, question, ofPrsPrep, gestPers, inLocPrep, room = params['answerVerb'], params['question'], params['ofPrsPrep'], params['gestPers'], params['inLocPrep'], params['room']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person in the room
    # [TODO] Gesture Detection with OpenPose
    print(f"[FIND] {gestPers}")

    # [3] Answer the question/quiz?
    # 어떻게 할지 감이 안잡히네 어떤 quiz일지, 어떤 question일지

# "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}",
def followNameFromBeacToRoom(agent, params):
    # Follow Angel from the desk lamp to the office
    # Follow Morgan from the bookshelf to the bathroom
    # Follow Angel from the side tables to the kitchen
    # Follow Angel from the tv stand to the living room
    params = {'followVerb': 'Follow', 'name': 'Angel', 'fromLocPrep': 'from', 'loc': 'desk lamp', 'toLocPrep': 'to', 'room': 'office'}

    # [0] Extract parameters
    follow, name, loc, room = params['followVerb'], params['name'], params['loc'], params['room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)
    
    # [2] Find the person in the location
    print(f"[FIND] {name} in the {loc}")

    # [3] Make the person to follow HSR to the room 
    # follow 

# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guideNameFromBeacToBeac(agent, params):
    # Lead Paris from the lamp to the kitchen
    # Escort Charlie from the entrance to the bathroom
    # Lead Angel from the storage rack to the chairs
    # Take Paris from the coatrack to the living room
    
    params = {'guideVerb': 'Lead', 'name': 'Paris', 'fromLocPrep': 'from', 'loc': 'lamp', 'toLocPrep': 'to', 'loc_room': 'kitchen'}
    # [0] Extract parameters
    guide, name, loc, room = params['guideVerb'], params['name'], params['loc'], params['loc_room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] {name} in the {loc}")

    # [3] Make the person to follow HSR to the room
    # follow

# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guidePrsFromBeacToBeac(agent, params):
    # Lead the person raising their right arm from the bookshelf to the office
    # Escort the lying person from the sink to the shelf
    # Escort the standing person from the chairs to the lamp
    # Guide the waving person from the trashbin to the living room

    params = {'guideVerb': 'Lead', 'gestPers_posePers': 'person raising their right arm', 'fromLocPrep': 'from', 'loc': 'bookshelf', 'toLocPrep': 'to', 'loc_room': 'office'}

    # [0] Extract parameters
    guide, gestPers_posePers, loc, room = params['guideVerb'], params['gestPers_posePers'], params['loc'], params['loc_room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] {gestPers_posePers} in the {loc}")

    # [3] Make the person to follow HSR to the room
    # follow

# "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guideClothPrsFromBeacToBeac(agent, params):
    # Take the person wearing a white shirt from the entrance to the trashbin
    # Escort the person wearing a yellow shirt from the pantry to the kitchen
    # Lead the person wearing a yellow t shirt from the kitchen table to the sofa
    # Escort the person wearing a blue sweater from the kitchen table to the office
    # Take the person wearing a gray sweater from the storage rack to the living room
    params = {'guideVerb': 'Take', 'colorClothe': 'white shirt', 'fromLocPrep': 'from', 'loc': 'entrance', 'toLocPrep': 'to', 'loc_room': 'trashbin'}

    # [0] Extract parameters
    guide, color, loc, room = params['guideVerb'], params['colorClothe'], params['loc'], params['loc_room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] the person wearing a {color} in the {loc}")

    # [3] Make the person to follow HSR to the room
    # follow

# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def greetClothDscInRm(agent, params):
    # Salute the person wearing a blue t shirt in the living room and follow them to the kitchen
    # Introduce yourself to the person wearing an orange coat in the bedroom and answer a quiz
    # Greet the person wearing a blue t shirt in the bedroom and answer a question
    # Introduce yourself to the person wearing a gray t shirt in the kitchen and say something about yourself
    # params = {'greetVerb': 'Salute', 'art': 'a', 'colorClothe': 'blue t shirt', 'inLocPrep': 'in', 'room': 'living room', 'followup': 'follow them to the kitchen'}

    # [0] Extract parameters
    greet, art, color, room, cmd = params['greetVerb'], params['art'], params['colorClothe'], params['room'], params['followup']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person in the room
    print(f"[FIND] the person wearing {art} {color} in the {room}")

    # [3] Generate the followup command
    followup(cmd)

# "greetNameInRm": "{greetVerb} {name} {inLocPrep} the {room} and {followup}",
def greetNameInRm(agent, params):
    # Salute Morgan in the living room and follow them to the storage rack
    # Say hello to Jules in the living room and tell the time
    # Say hello to Robin in the office and say your teams country
    # Greet Angel in the living room and say your teams name

    # params = {'greetVerb': 'Salute', 'name': 'Morgan', 'inLocPrep': 'in', 'room': 'living room', 'followup': 'follow them to the storage rack'}
    # [0] Extract parameters
    greet, name, room, cmd = params['greetVerb'], params['name'], params['room'], params['followup']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2][TODO] Find the person in the room
    print(f"[FIND] {name}")

    # [3] Generate the followup command
    # [3-1][TODO] Team Country, Name, Time...
    followup(cmd)

# "meetNameAtLocThenFindInRm": "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}",
def meetNameAtLocThenFindInRm(agent, params):
    # Meet Charlie at the shelf then find them in the living room
    # Meet Jane at the desk lamp then locate them in the office
    # Meet Robin at the tv stand then find them in the office
    # Meet Adel at the side tables then look for them in the living room
    
    # [0] Extract parameters
    name, loc, room = params['name'], params['loc'], params['room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] {name} in the {loc}")

    # [3][TODO] Make human locate to the room
    # print('please follow me')

    # params = {'name': Robin, 'loc': 'tv stand', 'room': 'office'}

# "countClothPrsInRoom": "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}",
def countClothPrsInRoom(agent, params):
    # Tell me how many people in the kitchen are wearing red jackets
    # Tell me how many people in the living room are wearing black jackets
    # Tell me how many people in the bathroom are wearing white jackets
    
    # [0] Extract parameters
    room, color = params['room'], params['colorClothes']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Check the number of people wearing the specified color
    # [TODO] Color Detection, Clothes Detection
    count = 0
    print(f"[COUNT] {count} people in the {room} are wearing {color}")

    # params = {countVerb: 'Tell me how many', room: 'kitchen', colorClothes: 'red jackets'}

# "tellPrsInfoAtLocToPrsAtLoc": "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}",
def tellPrsInfoAtLocToPrsAtLoc(agent, params):
    # Tell the name of the person at the potted plant to the person at the lamp
    # Tell the gesture of the person at the entrance to the person at the desk
    # Tell the name of the person at the desk to the person at the sink
    # Tell the pose of the person at the entrance to the person at the trashbin

    # params = {'tellVerb': 'Tell', 'persInfo': 'name', 'loc': 'potted plant', 'loc2': 'lamp'}
    # [0] Extract parameters
    tellVerb, persInfo, loc, loc2 = params['tellVerb'], params['persInfo'], params['loc'], params['loc2']

    # [1] Move to the specified room
    move_gpsr(agent, loc)

    # [2] Find the person in the room
    # [2-1] name with FaceDetection
    # [2-2] pose, gesture with OpenPose
    pass 

# "followPrsAtLoc": "{followVerb} the {gestPers_posePers} {inRoom_atLoc}",
def followPrsAtLoc(agent, params):
    # Follow the person raising their right arm at the armchair
    # Follow the standing person in the bedroom
    # Follow the sitting person at the dishwasher
    # Follow the person pointing to the left in the bedroom

    # [0] Extract parameters
    human, room = params['gestPers_posePers'], params['inRoom_atLoc']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2][TODO] Find the person with that pose in the room
    
    print(f"[FIND] {human}")

    # params = {'followVerb': 'Follow', 'gestPers_posePers': 'person pointing to the left', 'inRoom_atLoc': 'in the bedroom'}

### Object Manipulation and Perception Commands ###
# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def takeObjFromPlcmt(agent, params):
    # Fetch a dish from the refrigerator / and deliver it to the lying person in the bedroom
    # Get a drink from the storage rack / and deliver it to the person pointing to the right in the office
    # Get a cleaning supply from the tv stand / and bring it to me
    # Take a cola from the desk / and put it on the sofa

    # params = {'takeVerb': 'Get', 'obj_singCat': 'drink', 'fromLocPrep': 'from', 'plcmtLoc': 'storage rack', 'followup': 'deliver it to the person pointing to the right in the office'}
    # takeObjFromPlcmt(params)
    
    # [0] Extract parameters
    takeVerb, obj, loc, cmd = params['takeVerb'], params['obj_singCat'], params['plcmtLoc'], params['followup']
    # print(takeVerb, '\n', obj, '\n', fromLocPrep, '\n', plcmtLoc, '\n', cmd)

    # [1] Move to the specified space
    move_gpsr(agent, loc)

    # [2] Take the object from the specified location
    print(f"[TAKE] {takeVerb} the {obj} from the {loc}")

    # [3] Follow-up command
    followup(cmd)

# "findObjInRoom": "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then {followup}",
def findObjInRoom(agent, params):
    
    # params = {'findVerb': 'Find', 'art': 'a', 'obj_singCat': 'food', 'inLocPrep': 'in', 'room': 'kitchen', 'followup': 'take it and bring it to me'}
    
    # [0] Extract parameters
    findVerb, art, obj, room = params['findVerb'], params['art'], params['obj_singCat'], params['room']
    cmd = params['followup']
    
    # [1] Move to the specified room
    print(f"I'm moving to the {room}")
    move_gpsr(agent, room)
    
    # [2] Find the object in the room
    print(f"Let me find {art} {obj} in the {room}")
    
    found = False
    detected_objects = agent.yolo_module.yolo_bbox
    # detected_objects = [{'name': 'food', 'bbox': (10, 20, 30, 40)}, 
    #                     {'name': 'drink', 'bbox': (50, 60, 70, 80)}]
    
    for obj_data in detected_objects:
        obj_name, obj_bbox = obj_data['name'], obj_data['bbox']
        if obj_name == obj:
            print(f"{obj} is found at {obj_bbox}")
            found = True
            break
    
    # [3] Do follow-up action
    if found:
        followup(cmd)
        # Here you would add the code to manipulate the object as described in `followup`
        # This might involve additional robot movement or manipulator actions
    else:
        print(f"{obj} is not found in the {room}")

# "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
def countObjOnPlcmt(agent, params):
    # params = {'countVerb': 'tell me how many', 'plurCat': 'drinks', 'onLocPrep': 'on', 'plcmtLoc': 'sofa'}

    # Tell me how many drinks there are on the sofa
    # Tell me how many drinks there are on the sofa
    # Tell me how many cleaning supplies there are on the bedside table
    # Tell me how many cleaning supplies there are on the shelf
    # Tell me how many snacks there are on the tv stand
    # Tell me how many dishes there are on the kitchen table
    
    # [0] Extract parameters
    countVerb, plurCat, onLocPrep, plcmtLoc = params['countVerb'], params['plurCat'], params['onLocPrep'], params['plcmtLoc']
    
    # [1] Find the object in the room
    print(f"Let me find {plurCat} in the {plcmtLoc}")

    found = False
    # detected_objects = agent.yolo_module.yolo_bbox
    if len(agent.yolo_module.yolo_bbox) != 0:
        found = True
        pass
    else:
        print("No objects detected")
        return False

# "tellObjPropOnPlcmt": "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}",
def tellObjPropOnPlcmt(agent, params):    
    # [0] Extract parameters
    tell, comp, place = params['tellVerb'], params['objComp'], params['plcmtLoc']

    # [1] Move to the specified space
    agent.move_abs(place)

    # [2] Find the objects in the room
    print(f"[FIND] {tell} me what is the {comp} object {place}")

    yolo_bbox = get_yolo_bbox(agent)
    
    # [3] biggest, largest

    ObjIdArea = [(objInfo[4], objInfo[2] * objInfo[3]) for objInfo in yolo_bbox]
    objIdThinLen = [(objInfo[4], min(objInfo[2], objInfo[3])) for objInfo in yolo_bbox]

    if comp in ['biggest', 'largest']:
        targetObjId = max(ObjIdArea, key=lambda x: x[1])[0]
        targetObjName = agent.yolo_module.find_name_by_id(targetObjId)
        
    elif comp in ['smallest']:
        targetObjId = min(ObjIdArea, key=lambda x: x[1])[0]
        targetObjName = agent.yolo_module.find_name_by_id(targetObjId)

    elif comp in ['thinnest']:
        targetObjId = min(objIdThinLen, key=lambda x: x[1])[0]
        targetObjName = agent.yolo_module.find_name_by_id(targetObjId)

    ### TODO ###
    # elif comp in ['heaviest', 'lightest']:
    # define weight dictionary according to objId

    robotOutput = f"The {comp} object is {targetObjName}"
    agent.say(robotOutput)
    
# "tellCatPropOnPlcmt": "{tellVerb} me what is the {objComp} {singCat} {onLocPrep} the {plcmtLoc}",
def tellCatPropOnPlcmt(agent, params):
    
    # [0] Extract parameters
    tell, comp, cat, onLocPrep, loc = params['tellVerb'], params['objComp'], params['singCat'], params['onLocPrep'], params['plcmtLoc']

    # [1] Move to the specified space
    agent.move_abs(loc)

    # [2] Find the objects in the room
    yolo_bbox = get_yolo_bbox(agent, cat)

    # [3] Tell the information
    ObjIdArea = [(objInfo[4], objInfo[2] * objInfo[3]) for objInfo in yolo_bbox]
    objIdThinLen = [(objInfo[4], min(objInfo[2], objInfo[3])) for objInfo in yolo_bbox]

    if comp in ['biggest', 'largest']:
        targetObjId = max(ObjIdArea, key=lambda x: x[1])[0]
        targetObjName = agent.yolo_module.find_name_by_id(targetObjId)
        
    elif comp in ['smallest']:
        targetObjId = min(ObjIdArea, key=lambda x: x[1])[0]
        targetObjName = agent.yolo_module.find_name_by_id(targetObjId)

    elif comp in ['thinnest']:
        targetObjId = min(objIdThinLen, key=lambda x: x[1])[0]
        targetObjName = agent.yolo_module.find_name_by_id(targetObjId)

    ### TODO ###
    # elif comp in ['heaviest', 'lightest']:
    # define weight dictionary according to objId

    robotOutput = f"The {comp} {cat} is {targetObjName}"
    agent.say(robotOutput)


### TODO NOW ###
# "bringMeObjFromPlcmt": "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}",
# Give me a strawberry jello from the desk
# Bring me an apple from the refrigerator
# Give me an iced tea from the bedside table
# Give me a baseball from the bedside table
def bringMeObjFromPlcmt(agent, params):
    params = {'bringVerb': 'Give', 'art': 'an', 'obj': 'apple', 'fromLocPrep': 'from', 'plcmtLoc': 'refrigerator'}

    # [0] Extract parameters
    bring, art, obj, loc = params['bringVerb'], params['art'], params['obj'], params['plcmtLoc']

    # [1] Move to the specified location
    agent.move_abs(loc)

    # Try picking
    # Ask Human to give an item

    # [2] Find the object in the room
    print(f"[FIND] {obj} in the {loc}")

    # [3] Give the object to the human
    print(f"[GIVE] {bring} {art} {obj} from the {loc}")


verbType2verb = {
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

verbType2cmdName = {
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

cmdName2cmdStr = {
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

# verbType2followUpStr
# followUpStr2followUp

class NoAppropriateVerbError(Exception):
   """
   This exception is raised when no appropriate verb is found for a given command.
   
   Attributes:
       command (str): The command for which no appropriate verb was found.
       message (str): An explanatory message about the exception.
   """

   def __init__(self, command, message="No appropriate verb found for the given command."):
       self.command = command
       self.message = message
       super().__init__(self.message)

   def __str__(self):
       return f"{self.message} Command: {self.command}"

# LOAD gpsr_config.json
def load_config(config_file):
    with open(config_file) as f:
        config = json.load(f)
    return config

# CHAT w/ gpt-4
def chat(prompt):
    gpsr_config = load_config('gpsr_repo/gpsr_config.json')
    openai.api_key = gpsr_config['openai_api_key']
    model_engine = "gpt-4"

    response = openai.ChatCompletion.create(
        model=model_engine,
        messages=[{"role": "user", "content": prompt}],
        max_tokens=1024,
        n=1,
        stop=None,
        temperature=0.7,
    )

    return response.choices[0].message.content

# ULTIMATE Text Parser
def ultimateParser(inputText):
    '''Ultimate parser for the inputText. It uses GPT-4 to parse the inputText.'''
    splitedInputText = inputText.split()
    mainVerb = splitedInputText[0]

    for verbType in verbType2verb:
        if mainVerb.lower() in verbType2verb[verbType]:
            candidateCmdStr = dict([cmdEntries for cmdEntries in cmdName2cmdStr.items() if cmdEntries[1].split()[0] == verbType])

            prompt = f'dict of {{cmdName: parameter}}: {candidateCmdStr}\n'
            prompt += f'inputText: {inputText}\n'
            prompt += 'return which cmdName the inputText is, and the parameters in ({})\n'
            prompt += 'you should only write with format: cmdName, {"parameterName": "parameterValue"}'
            
            gptAnswer = chat(prompt)

            splitIndex = gptAnswer.find(', ')
            cmdName = gptAnswer[:splitIndex]
            params = json.loads(gptAnswer[splitIndex+2:])

            ### TODO ###
            ### Catch Error and Retry

            print("[Parser] cmdName:", cmdName)
            print("[Parser] params:", params)

            return cmdName, params
            
    else: 
        raise NoAppropriateVerbError("No Appropriate Verb")
    
def nogadaParser(inputText):
    '''Handcrafted parser for the inputText'''
    ### TODO ###
    ### Make Handcrafted Parser
    pass

# MAIN
def gpsr(agent):

    # agent.say("I'm ready to receive a command")
    # rospy.sleep(4)

    # inputText, _ = agent.stt(10.)
    # cmdName, params = ultimateParser(inputText)

    # agent.say(f"Given Command is {cmdName}, Given Parameters {params}")
    # cmdFunc = cmdName2cmdFunc[cmdName]

    # cmdFunc(agent, params)

    tellCatPropOnPlcmt(agent, {})