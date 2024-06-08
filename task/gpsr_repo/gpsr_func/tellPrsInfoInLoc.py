# "tellPrsInfoInLoc": "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}",
def tellPrsInfoInLoc(g, params):
    # Tell me the name of the person at the trashbin
    # Tell me the pose of the person at the storage rack
    # Tell me the gesture of the person in the office
    # Tell me the name of the person at the cabinet
    print("Start tellPrsInfoInLoc")

    # [0] Extract parameters
    loc = g.extractLocFrominRoomatLoc(params['inRoom_atLoc'])
    persInfo = params['persInfo']
        
    if persInfo not in ['name', 'pose', 'gesture']:
        persInfo = 'name'
        
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
    g.move('gpsr_instruction_point')
    
    # [4] Tell the person's information
    g.say(f'The {persInfo} of the person at the {loc} is {gotInfo}')

    g.task_finished_count += 1