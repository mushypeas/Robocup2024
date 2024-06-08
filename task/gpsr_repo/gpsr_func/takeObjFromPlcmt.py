import rospy

# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def takeObjFromPlcmt(g, params):
    # Fetch a dish from the refrigerator / and deliver it to the lying person in the bedroom
    # Get a drink from the storage rack / and deliver it to the person pointing to the right in the office
    # Get a cleaning supply from the tv stand / and bring it to me
    # Take a cola from the desk / and put it on the sofa
    print('Start takeObjFromPlcmt')
    
    # [0] Extract parameters
    obj = params['obj']
    plcmtLoc = params['plcmtLoc']
    followup = params['followup']

    # [1] Move to the specified space
    g.move(plcmtLoc)

    # [2] Take the object from the specified location
    g.pick(obj)

    # [3] Follow-up command
    g.exeFollowup(followup)

    g.task_finished_count += 1