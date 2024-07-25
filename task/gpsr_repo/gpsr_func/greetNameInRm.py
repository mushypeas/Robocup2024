# "greetNameInRm": "{greetVerb} {name} {inLocPrep} the {room} and {followup}",
def greetNameInRm(g, params):
    # Salute Morgan in the living room and follow them to the storage rack
    # Say hello to Jules in the living room and tell the time
    # Say hello to Robin in the office and say your teams country
    # Greet Angel in the living room and say your teams name
    print("Start greetNameInRm")

    # [0] Extract parameters
    try:
        name = params['name']
        room = params['room']
        followup = params['followup']
    
    except Exception as e:
        print(e)
        g.cmdError()
        return
    
    name = g.cluster(name, g.names_list)
    room = g.cluster(room, g.rooms_list)

    # [1] Move to the specified room
    g.move(room)

    # [2] Find the person in the room
    g.identifyByName(name)
    g.say(f'Hello {name}')

    # [3] followup
    g.exeFollowup(followup)

    g.task_finished_count += 1