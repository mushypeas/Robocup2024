# "bringMeObjFromPlcmt": "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}",
# Give me a strawberry jello from the desk
# Bring me an apple from the refrigerator
# Give me an iced tea from the bedside table
# Give me a baseball from the bedside table
def bringMeObjFromPlcmt(g, params):
    params = {'bringVerb': 'Give', 'art': 'an', 'obj': 'apple', 'fromLocPrep': 'from', 'plcmtLoc': 'refrigerator'}

    # [0] Extract parameters
    bring, art, obj, loc = params['bringVerb'], params['art'], params['obj'], params['plcmtLoc']

    # [1] Move to the specified location
    g.move(loc)

    # [2] Find the object in the room
    g.pick(obj)

    # [3] Give the object to the person
    g.move('gpsr_instruction_point')
    g.place()