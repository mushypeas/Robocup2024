# "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}"
def followNameFromBeacToRoom(g, params):
    # Follow Angel from the desk lamp to the office
    # Follow Morgan from the bookshelf to the bathroom
    # Follow Angel from the side tables to the kitchen
    # Follow Angel from the tv stand to the living room
    print("Start followNameFromBeacToRoom")

    # [0] Extract parameters
    name = params['name']
    loc = params['loc']
    room = params['room']

    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByName(name)
    g.say("Hello")

    # [3] follow the person to the room 
    g.followToLoc(room)