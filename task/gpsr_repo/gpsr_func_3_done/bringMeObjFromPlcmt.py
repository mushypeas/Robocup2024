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
        plcmtLoc = params['plcmtLoc']
    except KeyError:
        g.cmdError()
        return
    
    obj = g.cluster(obj, g.object_names)
        
    # [1] Move to the specified location
    g.move(plcmtLoc)

    # [2] Find the object in the room
    g.pick(obj)

    # [3] Give the object to the person
    g.move('gpsr_instruction_point')
    g.deliver()

    g.task_finished_count += 1