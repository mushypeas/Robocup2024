# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def greetClothDscInRm(g, params):
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