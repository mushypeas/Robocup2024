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
        comp = params['objComp']
    except KeyError:
        print("No objComp in params")
        comp = 'biggest'

    try:
        cat = params['singCat']
    except KeyError:
        print("No singCat in params")
        cat = 'object'

    try:    
        loc = params['plcmtLoc']
    except KeyError:
        print("No plcmtLoc in params")
        return
    
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
    robotOutput = f"The {comp} {cat} is {targetObjName}"
    g.say(robotOutput)