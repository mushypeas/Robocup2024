# "findObjInRoom": "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then {takeVerb} it and {followup}"
def findObjInRoom(g, params):
    # Find a drink in the living room then grasp it and put it on the bed
    # Locate a tropical juice in the living room then take it and give it to me
    # Look for a red wine in the kitchen then grasp it and place it on the dishwasher
    # Look for a drink in the kitchen then fetch it and deliver it to the sitting person in the kitchen
    print("Start findObjInRoom")
    
    # [0] Extract parameters
    try:
        obj = params["obj_singCat"]
        room = params["room"]
        followup = params["followup"]
    
    except KeyError:
        g.cmdError()
        return
    
    obj = g.cluster(obj, g.object_names + g.object_categories_singular)

    room = g.cluster(room, g.rooms_list)
    
    # [1] Move to the specified room
    g.move(room)
    
    # [2] Find the object in the room and pick it
    if obj in g.object_categories_singular:
        g.pickCat(obj)

    else:
        g.pick(obj)
    
    # [3] Follow-up command
    g.exeFollowup(followup)

    g.task_finished_count += 1