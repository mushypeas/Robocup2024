# "answerToGestPrsInRoom": "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}",
def answerToGestPrsInRoom(g, params):
    # Answer the question of the person raising their right arm in the bedroom
    # Answer the quiz of the person raising their left arm in the kitchen
    # Answer the quiz of the person raising their left arm in the living room
    # Answer the question of the person raising their left arm in the bedroom

    # params = {'answerVerb': 'Answer', 'question': 'quiz', 'ofPrsPrep': 'of', 'gestPers': 'person raising their left arm', 'inLocPrep': 'in', 'room': 'kitchen'}
    # [0] Extract parameters
    answerVerb, question, ofPrsPrep, gestPers, inLocPrep, room = params['answerVerb'], params['question'], params['ofPrsPrep'], params['gestPers'], params['inLocPrep'], params['room']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person in the room
    # [TODO] Gesture Detection with OpenPose
    print(f"[FIND] {gestPers}")

    # [3] Answer the question/quiz?
    # 어떻게 할지 감이 안잡히네 어떤 quiz일지, 어떤 question일지