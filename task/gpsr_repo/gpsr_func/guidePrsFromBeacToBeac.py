# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guidePrsFromBeacToBeac(g, params):
    # Lead the person raising their right arm from the bookshelf to the office
    # Escort the lying person from the sink to the shelf
    # Escort the standing person from the chairs to the lamp
    # Guide the waving person from the trashbin to the living room
    print("Start guidePrsFromBeacToBeac")
    
    # [0] Extract parameters
    gestPosePers = params['gestPers_posePers']
    loc = params['loc']
    loc2 = params['loc_room']
    
    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByGestPose(gestPosePers)
    g.say(f"Please follow me to the {loc2}")
    
    # [3] Make the person to follow HSR to the room
    g.guide(loc2)
    g.say(f"You have arrived at the {loc2}")

    g.task_finished_count += 1