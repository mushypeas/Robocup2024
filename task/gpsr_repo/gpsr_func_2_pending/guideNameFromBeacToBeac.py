# "guideNameFromBeacToBeac": "{guideVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
def guideNameFromBeacToBeac(g, params):
    # Lead Paris from the lamp to the kitchen
    # Escort Charlie from the entrance to the bathroom
    # Lead Angel from the storage rack to the chairs
    # Take Paris from the coatrack to the living room
    print("Start guideNameFromBeacToBeac")
    
    # [0] Extract parameters
    name = params['name']
    loc = params['loc']
    loc2 = params['loc_room']
    
    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByName(name)
    g.say(f"Please follow me to the {loc2}")
    
    # [3] Make the person to follow HSR to the room
    g.guide(loc2)
    g.say(f"You have arrived at the {loc2}")