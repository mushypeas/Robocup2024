# "{findVerb} a {gestPers_posePers} {inLocPrep} the {room} and {followup}",
def findPrsInRoom(g, params):
    # Find a person pointing to the left in the bedroom and take them to the dishwasher
    # Find a lying person in the bedroom and lead them to the potted plant
    # Locate a waving person in the office and answer a quiz
    print("Start findPrsInRoom")
    
    # [0] Extract parameters
    gestPosePers = params["gestPers_posePers"]
    room = params["room"]
    followup = params["followup"]

    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person with that pose in the room
    g.identifyByGestPose(gestPosePers)

    # [3] Follow-up command
    g.exeFollowup(followup)