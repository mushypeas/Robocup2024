# "countPrsInRoom": "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}",
def countPrsInRoom(g, params):
    # Tell me how many waving persons are in the living room
    # Tell me how many persons pointing to the right are in the kitchen
    # Tell me how many persons pointing to the left are in the bedroom
    # Tell me how many lying persons are in the living room
    print("Start countPrsInRoom")
    
    # [0] Extract parameters
    try:
        gestPosePersPlur = params["gestPersPlur_posePersPlur"]
        room = params["room"]

    except KeyError as e:
        g.cmdError()
        return
    
    gestPosePersPlur = g.cluster(gestPosePersPlur, g.gest_list + g.pose_list)
    room = g.cluster(room, )

    # [1] move to the specified room
    g.move(room)

    # [2] Count the number of persons in the room
    count = g.countGestPosePers(gestPosePersPlur)
    
    # [3] Output the count
    # TODO: Fix the grammar for singular and plural
    try:
        g.say(f"There are {count} {gestPosePersPlur} in the {room}")
    except:
        g.say(count)

    g.task_finished_count += 1