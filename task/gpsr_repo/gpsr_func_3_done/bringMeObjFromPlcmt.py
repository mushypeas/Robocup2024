import rospy

# "bringMeObjFromPlcmt": "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}",
def bringMeObjFromPlcmt(g, params):
    # Give me a strawberry jello from the desk
    # Bring me an apple from the refrigerator
    # Give me an iced tea from the bedside table
    # Give me a baseball from the bedside table
    print("Start BringMeObjFromPlcmt")
    
    # [0] Extract parameters
    try:
        obj = params['obj']
    except KeyError:
        print("[ERROR] obj is not defined")
        obj = 'any_obj'
        
    try:
        plcmtLoc = params['plcmtLoc']
    except KeyError:
        print("[ERROR] plcmtLoc is not defined")
        plcmtLoc = 'gpsr_instruction_point'
        
    # [1] Move to the specified location
    g.move(plcmtLoc)

    # [2] Find the object in the room
    g.pick(obj)

    # [3] Give the object to the person
    g.move('gpsr_instruction_point')
    print('moving to the instruction point')
    rospy.sleep(5)
    g.deliver()