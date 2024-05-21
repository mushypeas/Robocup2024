# "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guideClothPrsFromBeacToBeac(g, params):
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