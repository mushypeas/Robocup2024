# "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
def talkInfoToGestPrsInRoom(g, params):
    # Tell the day of the week to the person pointing to the left in the office
    # Tell something about yourself to the person raising their right arm in the kitchen
    # Tell the time to the person raising their right arm in the living room
    # Tell your teams affiliation to the person pointing to the right in the bathroom
    
    # params = {'talkVerb': 'Tell', 'talk': 'the day of the week', 'talkPrep': 'to', 'gestPers': 'person pointing to the left', 'inLocPrep': 'in', 'room': 'office'}
    # [0] Extract parameters
    talkVerb, talk, talkPrep, gestPers, inLocPrep, room = params['talkVerb'], params['talk'], params['talkPrep'], params['gestPers'], params['inLocPrep'], params['room']

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] Find the person in the room
    # [TODO] Gesture Detection with OpenPose
    print(f"[FIND] {gestPers}")

    # [3] Talk to the person
    # [TODO] Speech Synthesis
    #     talk_list = ["something about yourself", "the time", "what day is today", "what day is tomorrow", "your teams name",
                #  "your teams country", "your teams affiliation", "the day of the week", "the day of the month"]
    print(f"[TALK] {talkVerb} {talk} {talkPrep} {gestPers} {inLocPrep} the {room}")