import rospy

# "tellObjPropOnPlcmt": "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}",
def tellObjPropOnPlcmt(g, params):
    # Tell me what is the heaviest object on the storage rack
    # Tell me what is the heaviest object on the bed
    # Tell me what is the smallest object on the dishwasher
    # Tell me what is the heaviest object on the cabinet
    print("Start TellObjPropOnPlcmt")
    
    # [0] Extract parameters
    comp = params['objComp']
    loc = params['plcmtLoc']

    # [1] Move to the specified space
    g.move(loc)

    # [2] Find the objects in the room
    yolo_bbox = g.get_yolo_bbox()

    while yolo_bbox == []:
        yolo_bbox = g.get_yolo_bbox()

    # [3] Find the object with the specified property
    if comp in ['biggest', 'largest']:
        targetObjId = g.findBiggestObj(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)
        
    elif comp in ['smallest']:
        targetObjId = g.findSmallestObj(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)

    elif comp in ['thinnest']:
        targetObjId = g.findThinnestObj(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)
        
    elif comp in ['heaviest']:
        targetObjId = g.findHeaviestObj(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)
        
    elif comp in ['lightest']:
        targetObjId = g.findLightestObj(yolo_bbox)
        targetObjName = g.objIdToName(targetObjId)

    # [4] Tell the information
    robotOutput = f"The {comp} object is {targetObjName}"
    g.say(robotOutput)