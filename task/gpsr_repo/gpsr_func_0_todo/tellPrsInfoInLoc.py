# "tellPrsInfoInLoc": "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}",
def tellPrsInfoInLoc(g, params):
    # Tell me the name of the person at the trashbin
    # Tell me the pose of the person at the storage rack
    # Tell me the gesture of the person in the office
    # Tell me the name of the person at the cabinet

    # params= {'tellVerb': 'Tell', 'persInfo': 'gesture', 'inRoom_atLoc': 'at the trashbin'}    
    # [0] Extract parameters
    tellVerb, persInfo, loc = params['tellVerb'], params['persInfo'], params['inRoom_atLoc']
    
    # [1] Move to the specifed room
    print(f"I'm moving to the {loc}")

    # [2] Find the person in the room
    print(f"Humans are detected in the {loc}")

    # [3] Get the information of the person
    if persInfo == 'name':
        # Face Detection
        pass
    elif persInfo == 'pose' or persInfo == 'gesture':
        # OpenPose
        print(f"[INFO] {tellVerb} me the gesture of the person {loc}")