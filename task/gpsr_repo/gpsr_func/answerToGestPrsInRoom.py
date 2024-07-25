# "answerToGestPrsInRoom": "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}",
def answerToGestPrsInRoom(g, params):
    # Answer the question of the person raising their right arm in the bedroom
    # Answer the quiz of the person raising their left arm in the kitchen
    # Answer the quiz of the person raising their left arm in the living room
    # Answer the question of the person raising their left arm in the bedroom
    print("Start answerToGestPrsInRoom")

    # [0] Extract parameters
    try:
        gestPers = params["gestPers"]
        room = params["room"]

    except KeyError as e:
        g.cmdError()
        return
    
    gestPers = g.cluster(gestPers, g.gesture_person_list)
    room = g.cluster(room, g.rooms_list)
    
    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person in the room
    g.identifyByGestPose(gestPers)
    
    # [3] Answer the question/quiz?
    g.quiz()

    g.task_finished_count += 1