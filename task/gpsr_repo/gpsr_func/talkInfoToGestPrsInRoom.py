# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def talkInfoToGestPrsInRoom(g, params):
    # Tell the day of the week to the person pointing to the left in the office
    # Tell something about yourself to the person raising their right arm in the kitchen
    # Tell the time to the person raising their right arm in the living room
    # Tell your teams affiliation to the person pointing to the right in the bathroom
    print("Start talkInfoToGestPrsInRoom")
    
    # [0] Extract parameters
    talk = params['talk']
    gestPers = params['gestPers']
    room = params['room']

    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person in the room
    g.identifyByGestPose(gestPers)

    # [3] Talk to the person
    g.talk(talk)

    g.task_finished_count += 1