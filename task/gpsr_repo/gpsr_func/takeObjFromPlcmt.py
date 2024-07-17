import rospy

# "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {followup}",
def takeObjFromPlcmt(g, params):
    # Fetch a dish from the refrigerator / and deliver it to the lying person in the bedroom
    # Get a drink from the storage rack / and deliver it to the person pointing to the right in the office
    # Get a cleaning supply from the tv stand / and bring it to me
    # Take a cola from the desk / and put it on the sofa
    print('Start takeObjFromPlcmt')
    
    # [0] Extract parameters
    try:
        obj_cat = params['obj_singCat']
        plcmtLoc = params['plcmtLoc']
        followup = params['followup']
    except:
        g.cmdError()
        return
    
    obj_cat = g.cluster(obj_cat, g.object_names + g.object_categories_singular)
    plcmtLoc = g.cluster(plcmtLoc, g.loc_list)

    # [1] Move to the specified space
    g.move(plcmtLoc)

    # [2] Take the object from the specified location
    if obj_cat in g.object_categories_singular:
        g.pickCat(obj_cat)
    
    else:
       g.pick(obj_cat)

    # [3] Follow-up command
    g.exeFollowup(followup)

    g.task_finished_count += 1