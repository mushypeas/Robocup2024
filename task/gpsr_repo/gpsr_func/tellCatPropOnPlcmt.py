import rospy

# "tellCatPropOnPlcmt": "{tellVerb} me what is the {objComp} {singCat} {onLocPrep} the {plcmtLoc}",
def tellCatPropOnPlcmt(g, params):
    # Tell me what is the biggest food on the sofa
    # Tell me what is the biggest snack on the sofa
    # Tell me what is the smallest food on the side tables
    # Tell me what is the biggest food on the kitchen table
    print("Start TellCatPropOnPlcmt")

    # [0] Extract parameters
    try:
        cat = params['singCat']
    except KeyError:
        print("No singCat in params")
        cat = 'object'

    try:    
        loc = params['plcmtLoc']
        comp = params['objComp']
    except KeyError:
        g.cmdError()
        return
    
    cat = g.cluster(cat, g.object_categories_singular)
    loc = g.cluster(loc, g.loc_list)
    comp = g.cluster(comp, g.object_comp_list)
    
    # [1] Move to the specified space
    g.move(loc)

    # [2] Find the objects in the room
    yolo_bbox = g.get_yolo_bbox(cat)

    iter_count = 0

    while yolo_bbox == []:
        yolo_bbox = g.get_yolo_bbox(cat)
        iter_count += 1

        if iter_count > 50:
            g.say(f"No {cat} found")
            return

    # [3] Find the object with the specified property
    if comp in ['biggest', 'largest']:
        targetObjId = g.findBiggestObjId(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)
        
    elif comp in ['smallest']:
        targetObjId = g.findSmallestObjId(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)

    elif comp in ['thinnest']:
        targetObjId = g.findThinnestObjId(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)
        
    elif comp in ['heaviest']:
        targetObjId = g.findHeaviestObjId(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)
        
    elif comp in ['lightest']:
        targetObjId = g.findLightestObjId(yolo_bbox)
        targetObjName = g.objIdToNameId(targetObjId)

    # [4] Tell the information
    try:
        robotOutput = f"The {comp} {cat} is {targetObjName}"
        g.say(robotOutput)
    
    except Exception as e:
        print(e)
        g.say(targetObjName)

    g.task_finished_count += 1