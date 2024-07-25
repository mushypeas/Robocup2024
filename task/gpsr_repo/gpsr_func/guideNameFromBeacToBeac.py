# "guideNameFromBeacToBeac": "{guideVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
def guideNameFromBeacToBeac(g, params):
    # Lead Paris from the lamp to the kitchen
    # Escort Charlie from the entrance to the bathroom
    # Lead Angel from the storage rack to the chairs
    # Take Paris from the coatrack to the living room
    print("Start guideNameFromBeacToBeac")
    
    # [0] Extract parameters
    try:
        name = params['name']
        loc = params['loc']
        loc2 = params['loc_room']

    except Exception as e:
        print(e)
        g.cmdError()
        return
    
    name = g.cluster(name, g.names_list)
    loc = g.cluster(loc, g.loc_list)
    loc2 = g.cluster(loc2, g.loc_list)
    
    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByName(name)
    g.say(f"Please follow me \n to the {loc2}")
    
    # [3] Make the person to follow HSR to the room
    g.guide(loc2)
    g.say(f"You have arrived \n at the {loc2}")

    g.task_finished_count += 1