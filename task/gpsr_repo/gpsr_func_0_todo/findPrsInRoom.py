# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def findPrsInRoom(g, params):
    # Find a person pointing to the left in the bedroom and take them to the dishwasher
    # Find a lying person in the bedroom and lead them to the potted plant
    # Locate a waving person in the office and answer a quiz
    
    # [0] Extract parameters
    find, human, room, cmd = params['findVerb'], params['gestPers_posePers'], params['room'], params['followup']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person with that pose in the room
    print(f"[FIND] {find} a {human} in the {room}")

    # [2-1] find waving person
    # [2-2] find person raising their left/right arm
    # [2-3] find person pointing to the left/right
    # [2-4] find sitting/standing/lying person

    # [3] Follow-up command
    followup(cmd)