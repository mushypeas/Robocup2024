# "meetPrsAtBeac": "{meetVerb} {name} {inLocPrep} the {room}",
def meetPrsAtBeac(g, params):
    # Meet Jules in the living room and follow them
    # Meet Angel in the kitchen and take them to the shelf
    # Meet Simone in the living room and follow them
    # Meet Axel in the office and follow them to the bedroom
    
    # [0] Extract parameters
    name, room, cmd = params['name'], params['room'], params['followup']  

    # [1] Move to the specified room
    move_gpsr(agent, room)

    # [2] find the person
    # face recognition?
    print(f"[FIND] {name}")

    # [3] Generate the followup comman
    followup(cmd)