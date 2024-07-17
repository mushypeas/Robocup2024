# "{findVerb} a {gestPers_posePers} {inLocPrep} the {room} and {followup}",
def findPrsInRoom(g, params):
    # Find a person pointing to the left in the bedroom and take them to the dishwasher
    # Find a lying person in the bedroom and lead them to the potted plant
    # Locate a waving person in the office and answer a quiz
    print("Start findPrsInRoom")
    
    # [0] Extract parameters
    try:
        gestPosePers = params["gestPers_posePers"]
        room = params["room"]
        followup = params["followup"]
    except:
        g.cmdError()
        return

    gestPosePers = g.cluster(gestPosePers, g.gesture_person_list + g.pose_person_list)
    room = g.cluster(room, g.rooms_list)

    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person with that pose in the room
    g.identifyByGestPose(gestPosePers)

    # [3] Follow-up command
    g.exeFollowup(followup)

    g.task_finished_count += 1