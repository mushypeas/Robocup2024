# "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}"
def followNameFromBeacToRoom(g, params):
    # Follow Angel from the desk lamp to the office
    # Follow Morgan from the bookshelf to the bathroom
    # Follow Angel from the side tables to the kitchen
    # Follow Angel from the tv stand to the living room
    print("Start followNameFromBeacToRoom")

    # [0] Extract parameters
    try:
        name = params['name']
        loc = params['loc']
        room = params['room']
    except:
        g.cmdError()
        return

    name = g.cluster(name, g.names_list)
    loc = g.cluster(loc, g.loc_list)
    room = g.cluster(room, g.rooms_list)

    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByName(name)
    g.say("Hello")

    # [3] follow the person to the room 
    g.followToLoc(room)

    g.task_finished_count += 1