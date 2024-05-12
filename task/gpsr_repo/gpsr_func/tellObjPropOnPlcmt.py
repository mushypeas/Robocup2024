import rospy

# "tellObjPropOnPlcmt": "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}",
def tellObjPropOnPlcmt(g, params):    
    print("Start TellObjPropOnPlcmt")

    # [0] Extract parameters
    tell, comp, loc = params['tellVerb'], params['objComp'], params['plcmtLoc']

    # [1] Move to the specified space
    g.move(loc)

    # [2] Find the objects in the room
    print(f"[FIND] {tell} me what is the {comp} object {loc}")

    yolo_bbox = g.get_yolo_bbox()
    
    # [3] biggest, largest

    ObjIdArea = [(objInfo[4], objInfo[2] * objInfo[3]) for objInfo in yolo_bbox]
    objIdThinLen = [(objInfo[4], min(objInfo[2], objInfo[3])) for objInfo in yolo_bbox]

    if comp in ['biggest', 'largest']:
        targetObjId = max(ObjIdArea, key=lambda x: x[1])[0]
        targetObjName = g.agent.yolo_module.find_name_by_id(targetObjId)
        
    elif comp in ['smallest']:
        targetObjId = min(ObjIdArea, key=lambda x: x[1])[0]
        targetObjName = g.agent.yolo_module.find_name_by_id(targetObjId)

    elif comp in ['thinnest']:
        targetObjId = min(objIdThinLen, key=lambda x: x[1])[0]
        targetObjName = g.agent.yolo_module.find_name_by_id(targetObjId)

    ### TODO ###
    # elif comp in ['heaviest', 'lightest']:
    # define weight dictionary according to objId

    robotOutput = f"The {comp} object is {targetObjName}"
    g.say(robotOutput)