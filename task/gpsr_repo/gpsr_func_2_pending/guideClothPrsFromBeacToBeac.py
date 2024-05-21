# "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
def guideClothPrsFromBeacToBeac(g, params):
    # Take the person wearing a white shirt from the entrance to the trashbin
    # Escort the person wearing a yellow shirt from the pantry to the kitchen
    # Lead the person wearing a yellow t shirt from the kitchen table to the sofa
    # Escort the person wearing a blue sweater from the kitchen table to the office
    # Take the person wearing a gray sweater from the storage rack to the living room
    print("Start guideClothPrsFromBeacToBeac")
    
    # [0] Extract parameters
    colorClothe = params['colorClothe']
    loc = params['loc']
    loc2 = params['loc_room']
    
    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByClothing(colorClothe)
    g.say(f"Please follow me to the {loc2}")
    
    # [3] Make the person to follow HSR to the room
    g.guide(loc2)
    g.say(f"You have arrived at the {loc2}")