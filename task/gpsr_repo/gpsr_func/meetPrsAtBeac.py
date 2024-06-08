# "meetPrsAtBeac": "{meetVerb} {name} {inLocPrep} the {room} and {followup}"
def meetPrsAtBeac(g, params):
    # Meet Jules in the living room and follow them
    # Meet Angel in the kitchen and take them to the shelf
    # Meet Simone in the living room and follow them
    # Meet Axel in the office and follow them to the bedroom
    print("Start meetPrsAtBeac")
    
    # [0] Extract parameters
    name = params['name']
    room = params['room']
    followup = params['followup']
        
    # [1] Move to the specified room
    g.move(room)

    # [2] find the person
    g.identifyByName(name)

    # [3] Generate the followup comman
    g.exeFollowup(followup)

    g.task_finished_count += 1