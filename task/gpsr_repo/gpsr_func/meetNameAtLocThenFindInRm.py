# "meetNameAtLocThenFindInRm": "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}",
def meetNameAtLocThenFindInRm(g, params):
    # Meet Charlie at the shelf then find them in the living room
    # Meet Jane at the desk lamp then locate them in the office
    # Meet Robin at the tv stand then find them in the office
    # Meet Adel at the side tables then look for them in the living room
    print("Start meetNameAtLocThenFindInRm")
    
    # [0] Extract parameters
    try:
        name = params['name']
        loc = params['loc']
        room = params['room']
    except Exception as e:
        print(e)
        g.cmdError()
        return
    
    name = g.cluster(name, g.names_list)
    loc = g.cluster(loc, g.loc_list)
    room = g.cluster(room, g.rooms_list)
        
    # [1] Move to the specified location
    g.move(loc)

    # [2] Find the person in the location
    g.identifyByName(name)
    
    # [3] Move to the specified room
    g.move(room)

    # [4] Find the person in the room
    g.identifyByName(name)

    g.task_finished_count += 1