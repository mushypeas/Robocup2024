# "answerToGestPrsInRoom": "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}",
def answerToGestPrsInRoom(g, params):
    # Answer the question of the person raising their right arm in the bedroom
    # Answer the quiz of the person raising their left arm in the kitchen
    # Answer the quiz of the person raising their left arm in the living room
    # Answer the question of the person raising their left arm in the bedroom
    print("Start answerToGestPrsInRoom")

    # [0] Extract parameters
    gestPers = params["gestPers"]
    room = params["room"]
    
    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person in the room
    g.identifyByGestPose(gestPers)
    
    # [3] Answer the question/quiz?
    g.quiz()