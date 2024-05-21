# "tellPrsInfoAtLocToPrsAtLoc": "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}",
def tellPrsInfoAtLocToPrsAtLoc(g, params):
    # Tell the name of the person at the potted plant to the person at the lamp
    # Tell the gesture of the person at the entrance to the person at the desk
    # Tell the name of the person at the desk to the person at the sink
    # Tell the pose of the person at the entrance to the person at the trashbin

    # params = {'tellVerb': 'Tell', 'persInfo': 'name', 'loc': 'potted plant', 'loc2': 'lamp'}
    # [0] Extract parameters
    tellVerb, persInfo, loc, loc2 = params['tellVerb'], params['persInfo'], params['loc'], params['loc2']

    # [1] Move to the specified room
    g.move(loc)

    # [2] Find the person in the room
    # [2-1] name with FaceDetection
    # [2-2] pose, gesture with OpenPose
    pass 