# "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}",
def followNameFromBeacToRoom(g, params):
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