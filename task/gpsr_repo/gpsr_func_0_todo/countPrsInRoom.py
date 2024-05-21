# "countPrsInRoom": "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}",
def countPrsInRoom(g, params):
    # Tell me how many waving persons are in the living room
    # Tell me how many persons pointing to the right are in the kitchen
    # Tell me how many persons pointing to the left are in the bedroom
    # Tell me how many lying persons are in the living room
    
    # [0] Extract parameters
    count, human, room = params['countVerb'], params['gestPers_posePers'], params['room']

    # [1] move to the specified room
    move_gpsr(agent, room)

    # [2] Count the number of persons in the room
    print(f"[COUNT] {count} {human} in the {room}")