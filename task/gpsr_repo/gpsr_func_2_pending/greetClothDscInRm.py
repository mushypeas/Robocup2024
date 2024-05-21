# "greetClothDscInRm": "{greetVerb} the person wearing {art} {colorClothe} {inLocPrep} the {room} and {followup}"
def greetClothDscInRm(g, params):
    # Salute the person wearing a blue t shirt in the living room and follow them to the kitchen
    # Introduce yourself to the person wearing an orange coat in the bedroom and answer a quiz
    # Greet the person wearing a blue t shirt in the bedroom and answer a question
    # Introduce yourself to the person wearing a gray t shirt in the kitchen and say something about yourself
    # params = {'greetVerb': 'Salute', 'art': 'a', 'colorClothe': 'blue t shirt', 'inLocPrep': 'in', 'room': 'living room', 'followup': 'follow them to the kitchen'}
    print("Start greetClothDscInRm")

    # [0] Extract parameters
    colorClothe = params['colorClothe']
    room = params['room']
    followup = params['followup']
    
    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person in the room
    g.identifyByClothing(colorClothe)
    g.say("Hello")

    # [3] Generate the followup command
    g.exeFollowup(followup)