import openai
import json

# HRI and People Perception Commands
def goToLoc(agent, params):
    pass

def findPrsInRoom(agent, params):
    pass

def meetPrsAtBeac(agent, params):
    pass

def countPrsInRoom(agent, params):
    pass

def tellPrsInfoInLoc(agent, params):
    pass

def talkInfoToGestPrsInRoom(agent, params):
    pass

def answerToGestPrsInRoom(agent, params):
    pass

def followNameFromBeacToRoom(agent, params):
    pass

def guideNameFromBeacToBeac(agent, params):
    pass

def guidePrsFromBeacToBeac(agent, params):
    pass

def guideClothPrsFromBeacToBeac(agent, params):
    pass

def greetClothDscInRm(agent, params):
    pass

def greetNameInRm(agent, params):
    pass

def meetNameAtLocThenFindInRm(agent, params):
    pass

def countClothPrsInRoom(agent, params):
    pass

def tellPrsInfoAtLocToPrsAtLoc(agent, params):
    pass

def followPrsAtLoc(agent, params):
    pass

# Object Manipulation and Perception Commands

def takeObjFromPlcmt(agent, params):
    pass

def findObjInRoom(agent, params):
    pass

def countObjOnPlcmt(agent, params):
    pass

def tellObjPropOnPlcmt(agent, params):
    pass

def bringMeObjFromPlcmt():
    pass

def tellCatPropOnPlcmt():
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
