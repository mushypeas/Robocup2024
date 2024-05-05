import openai
import json

# HRI and People Perception Commands
# "goToLoc": "{goVerb} {toLocPrep} the {loc_room} then {followup}",
def goToLoc(agent, params):
    goVerb = params['goVerb']
    toLocPrep = params['toLocPrep']
    loc_room = params['loc_room']
    followup = params['followup']

# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def findPrsInRoom(agent, params):
    findVerb = params['findVerb']
    gestPers_posePers = params['gestPers_posePers']
    inLocPrep = params['inLocPrep']
    room = params['room']
    followup = params['followup']
    pass

# "meetPrsAtBeac": "{meetVerb} {name} {inLocPrep} the {room}",
def meetPrsAtBeac(agent, params):
    meetVerb = params['meetVerb']
    name = params['name']
    inLocPrep = params['inLocPrep']
    room = params['room']
    followup = params['followup']
    pass

# "countPrsInRoom": "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}",
def countPrsInRoom(agent, params):
    countVerb = params['countVerb']
    gestPersPlur_posePersPlur = params['gestPersPlur_posePersPlur']
    inLocPrep = params['inLocPrep']
    room = params['room']
    pass

# "tellPrsInfoInLoc": "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}",
def tellPrsInfoInLoc(agent, params):
    tellVerb = params['tellVerb']
    persInfo = params['persInfo']
    inRoom_atLoc = params['inRoom_atLoc']
    pass

# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def talkInfoToGestPrsInRoom(agent, params):
    talkVerb = params['talkVerb']
    talk = params['talk']
    talkPrep = params['talkPrep']
    gestPers = params['gestPers']
    inLocPrep = params['inLocPrep']
    room = params['room']
    pass

# "answerToGestPrsInRoom": "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}",
def answerToGestPrsInRoom(agent, params):
    answerVerb = params['answerVerb']
    question = params['question']
    ofPrsPrep = params['ofPrsPrep']
    gestPers = params['gestPers']
    inLocPrep = params['inLocPrep']
    room = params['room']
    pass

# "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}",
def followNameFromBeacToRoom(agent, params):
    followVerb = params['followVerb']
    name = params['name']
    fromLocPrep = params['fromLocPrep']
    loc = params['loc']
    toLocPrep = params['toLocPrep']
    room = params['room']
    pass

# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guideNameFromBeacToBeac(agent, params):
    guideVerb = params['guideVerb']
    name = params['name']
    fromLocPrep = params['fromLocPrep']
    loc = params['loc']
    toLocPrep = params['toLocPrep']
    loc_room = params['loc_room']
    pass

# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guidePrsFromBeacToBeac(agent, params):
    guideVerb = params['guideVerb']
    gestPers_posePers = params['gestPers_posePers']
    fromLocPrep = params['fromLocPrep']
    loc = params['loc']
    toLocPrep = params['toLocPrep']
    loc_room = params['loc_room']
    pass

# "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guideClothPrsFromBeacToBeac(agent, params):
    guideVerb = params['guideVerb']
    colorClothe = params['colorClothe']
    fromLocPrep = params['fromLocPrep']
    loc = params['loc']
    toLocPrep = params['toLocPrep']
    loc_room = params['loc_room']
    pass

# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def greetClothDscInRm(agent, params):
    greetVerb = params['greetVrb']
    colorClothe = params['colorClothe']
    inLocPrep = params['inLocPrep']
    room = params['room']
    followup = params['followup']
    pass

# "greetNameInRm": "{greetVerb} {name} {inLocPrep} the {room} and {followup}",
def greetNameInRm(agent, params):
    greetVerb = params['greetVrb']
    name = params['name']
    inLocPrep = params['inLocPrep']
    room = params['room']
    followup = params['followup']
    pass

# "meetNameAtLocThenFindInRm": "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}",
def meetNameAtLocThenFindInRm(agent, params):
    meetVerb = params['meetVerb']
    name = params['name']
    atLocPrep = params['atLocPrep']
    loc = params['loc']
    findVerb = params['findVerb']
    inLocPrep = params['inLocPrep']
    room = params['room']
    pass

# "countClothPrsInRoom": "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}",
def countClothPrsInRoom(agent, params):
    countVerb = params['countVerb']
    inLocPrep = params['inLocPrep']
    room = params['room']
    colorClothes = params['colorClothes']
    pass

# "tellPrsInfoAtLocToPrsAtLoc": "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}",
def tellPrsInfoAtLocToPrsAtLoc(agent, params):
    tellVerb = params['tellVerb']
    persInfo = params['persInfo']
    atLocPrep = params['atLocPrep']
    loc = params['loc']
    loc2 = params['loc2']
    pass

# "followPrsAtLoc": "{followVerb} the {gestPers_posePers} {inRoom_atLoc}",
def followPrsAtLoc(agent, params):
    followVerb = params['followVerb']
    gestPers_posePers = params['gestPers_posePers']
    inRoom_atLoc = params['inRoom_atLoc']
    pass

# Object Manipulation and Perception Commands
# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def takeObjFromPlcmt(agent, params):
    takeVerb = params['takeVerb']
    art = params['art']
    obj_singCat = params['obj_singCat']
    fromLocPrep = params['fromLocPrep']
    plcmtLoc = params['plcmtLoc']
    followup = params['followup']
    pass

# "findObjInRoom": "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then {followup}",
def findObjInRoom(agent, params):
    findVerb = params['findVerb']
    art = params['art']
    obj_singCat = params['obj_singCat']
    inLocPrep = params['inLocPrep']
    room = params['room']
    followup = params['followup']
    pass

# "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
def countObjOnPlcmt(agent, params):
    countVerb = params['countVerb']
    plurCat = params['plurCat']
    onLocPrep = params['onLocPrep']
    plcmtLoc = params['plcmtLoc']
    pass

# "tellObjPropOnPlcmt": "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}",
def tellObjPropOnPlcmt(agent, params):
    tellVerb = params['tellVerb']
    objComp = params['objComp']
    singCat = params['singCat']
    onLocPrep = params['onLocPrep']
    plcmtLoc = params['plcmtLoc']
    pass

# "bringMeObjFromPlcmt": "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}",
def bringMeObjFromPlcmt(agent, params):
    bringVerb = params['bringVerb']
    art = params['art']
    obj = params['obj']
    fromLocPrep = params['fromLocPrep']
    plcmtLoc = params['plcmtLoc']
    pass

# "tellCatPropOnPlcmt": "{tellVerb} me what is the {objComp} {singCat} {onLocPrep} the {plcmtLoc}",
def tellCatPropOnPlcmt(agent, params):
    tellVerb = params['tellVerb']
    objComp = params['objComp']
    singCat = params['singCat']
    onLocPrep = params['onLocPrep']
    plcmtLoc = params['plcmtLoc']
    pass

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
    gpsr_config = load_config('gpsr_config.json')
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
    # inputText = input()
    inputText = "Locate a food in the office then take it and place it on the sofa"
    cmdName, params = ultimateParser(inputText)
    cmdFunc = cmdName2cmdFunc[cmdName]

    cmdFunc(agent, params)

if __name__ == "__main__":
    gpsr('1')
