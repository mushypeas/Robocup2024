# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guideNameFromBeacToBeac(g, params):
    # Lead Paris from the lamp to the kitchen
    # Escort Charlie from the entrance to the bathroom
    # Lead Angel from the storage rack to the chairs
    # Take Paris from the coatrack to the living room
    
    params = {'guideVerb': 'Lead', 'name': 'Paris', 'fromLocPrep': 'from', 'loc': 'lamp', 'toLocPrep': 'to', 'loc_room': 'kitchen'}
    # [0] Extract parameters
    guide, name, loc, room = params['guideVerb'], params['name'], params['loc'], params['loc_room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] {name} in the {loc}")

    # [3] Make the person to follow HSR to the room
    # follow