# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guidePrsFromBeacToBeac(g, params):
    # Lead the person raising their right arm from the bookshelf to the office
    # Escort the lying person from the sink to the shelf
    # Escort the standing person from the chairs to the lamp
    # Guide the waving person from the trashbin to the living room

    params = {'guideVerb': 'Lead', 'gestPers_posePers': 'person raising their right arm', 'fromLocPrep': 'from', 'loc': 'bookshelf', 'toLocPrep': 'to', 'loc_room': 'office'}

    # [0] Extract parameters
    guide, gestPers_posePers, loc, room = params['guideVerb'], params['gestPers_posePers'], params['loc'], params['loc_room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] {gestPers_posePers} in the {loc}")

    # [3] Make the person to follow HSR to the room
    # follow