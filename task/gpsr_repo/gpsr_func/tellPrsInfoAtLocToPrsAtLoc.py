# "tellPrsInfoAtLocToPrsAtLoc": "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}",
def tellPrsInfoAtLocToPrsAtLoc(g, params):
    # Tell the name of the person at the potted plant to the person at the lamp
    # Tell the gesture of the person at the entrance to the person at the desk
    # Tell the name of the person at the desk to the person at the sink
    # Tell the pose of the person at the entrance to the person at the trashbin
    print("Start tellPrsInfoAtLocToPrsAtLoc")

    # [0] Extract parameters
    loc = params['loc']
    loc2 = params['loc2']
    persInfo = params['persInfo']
        
    # [1] Move to the specified room
    g.move(loc)
    
    # [2] Get person's information
    if persInfo == 'name':
        gotInfo = g.getName()
        
    elif persInfo == 'pose':
        gotInfo = g.getPose()
        
    elif persInfo == 'gesture':
        gotInfo = g.getGest()
        
    # [3] Move to the second location
    g.move(loc2)
    
    # [4] Tell the person's information
    g.say(f'The {persInfo} of the person at the {loc} is {gotInfo}')

    g.task_finished_count += 1