import rospy

# "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
def countObjOnPlcmt(g, params):
    print("Start CountObjOnPlcmt")
    
    # [0] Extract parameters
    countVerb, plurCat, onLocPrep, plcmtLoc = params['countVerb'], params['plurCat'], params['onLocPrep'], params['plcmtLoc']
    singCat = g.categoryPlur2Sing[plurCat]

    # [1] Find the object in the room
    g.move(plcmtLoc)
    
    yolo_bbox = g.get_yolo_bbox(singCat)

    g.say(f"Let me find {plurCat} in the {plcmtLoc}")
    rospy.sleep(5)

    numObj = len(yolo_bbox)

    # [2] Tell number of objects
    if numObj == 0:
        g.say(f"There's no {singCat} on the {plcmtLoc}")
    
    elif numObj == 1:
        g.say(f"There's a {singCat} on the {plcmtLoc}")
    
    else:
        g.say(f"There are {numObj} {plurCat} on the {plcmtLoc}")