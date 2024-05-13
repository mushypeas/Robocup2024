import rospy

# "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
def countObjOnPlcmt(g, params):
    print("Start CountObjOnPlcmt")
    
    # [0] Extract parameters
    try:
        countVerb = params['countVerb'] 
    except KeyError:
        pass

    try:
        plurCat = params['plurCat']
    except KeyError:
        pass

    try:   
        onLocPrep = params['onLocPrep']
    except KeyError:
        pass

    try:    
       plcmtLoc = params['plcmtLoc']
    except KeyError:
        pass

    singCat = g.categoryPlur2Sing[plurCat]

    # [1] Find the object in the room
    g.move(plcmtLoc)
    

    g.say(f"Let me find {plurCat} in the {plcmtLoc}")
    yolo_bbox = g.get_yolo_bbox(singCat)
    rospy.sleep(5)

    numObj = len(yolo_bbox)

    # [2] Tell number of objects
    if numObj == 0:
        g.say(f"There's no {singCat} on the {plcmtLoc}")
    
    elif numObj == 1:
        g.say(f"There's a {singCat} on the {plcmtLoc}")
    
    else:
        g.say(f"There are {numObj} {plurCat} on the {plcmtLoc}")