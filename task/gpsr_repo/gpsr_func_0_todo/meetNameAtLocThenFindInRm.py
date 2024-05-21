# "meetNameAtLocThenFindInRm": "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}",
def meetNameAtLocThenFindInRm(g, params):
    # Meet Charlie at the shelf then find them in the living room
    # Meet Jane at the desk lamp then locate them in the office
    # Meet Robin at the tv stand then find them in the office
    # Meet Adel at the side tables then look for them in the living room
    
    # [0] Extract parameters
    name, loc, room = params['name'], params['loc'], params['room']

    # [1] Move to the specified location
    move_gpsr(agent, loc)

    # [2] Find the person in the location
    print(f"[FIND] {name} in the {loc}")

    # [3][TODO] Make human locate to the room
    # print('please follow me')

    # params = {'name': Robin, 'loc': 'tv stand', 'room': 'office'}