# "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
def guidePrsFromBeacToBeac(g, params):
    # Lead the person raising their right arm from the bookshelf to the office
    # Escort the lying person from the sink to the shelf
    # Escort the standing person from the chairs to the lamp
    # Guide the waving person from the trashbin to the living room
    print("Start guidePrsFromBeacToBeac")
    
    # [0] Extract parameters
    try:
        gestPosePers = params['gestPers_posePers']
        loc = params['loc']
        loc2 = params['loc_room']

    except Exception as e:
        print(e)
        g.cmdError()
        return

    gestPosePers = g.cluster(gestPosePers, g.gesture_person_list + g.pose_person_list)
    loc = g.cluster(loc, g.loc_list)
    loc2 = g.cluster(loc2, g.loc_list)
    
    # [1] Move to the specified location
    g.move(loc)
    
    # [2] Find the person in the location
    g.identifyByGestPose(gestPosePers)
    
    # [3] Make the person to follow HSR to the room
    g.guide(loc2)

    g.task_finished_count += 1