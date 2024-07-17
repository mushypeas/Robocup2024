import rospy

# "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
def countObjOnPlcmt(g, params):
    # Tell me how many drinks there are on the sofa
    # Tell me how many drinks there are on the sofa
    # Tell me how many cleaning supplies there are on the bedside table
    # Tell me how many cleaning supplies there are on the shelf
    # Tell me how many snacks there are on the tv stand
    # Tell me how many dishes there are on the kitchen table
    print("Start CountObjOnPlcmt")
    
    # [0] Extract parameters
    try:
        plurCat = params['plurCat']
        plcmtLoc = params['plcmtLoc']
    except KeyError:
        g.cmdError()
        return

    plurCat = g.cluster(plurCat, g.object_categories_plural)
    plcmtLoc = g.cluster(plcmtLoc, g.loc_list)

    singCat = g.categoryPlur2Sing[plurCat]

    # [1] Find the object in the room
    g.move(plcmtLoc)
    

    g.say(f"Let me find {plurCat} in the {plcmtLoc}")
    yolo_bbox = g.get_yolo_bbox(singCat)
    rospy.sleep(5)

    numObj = len(yolo_bbox)

    # [2] Tell number of objects
    try:
        if numObj == 0:
            g.say(f"There's no {singCat} on the {plcmtLoc}")
        
        elif numObj == 1:
            g.say(f"There's a {singCat} on the {plcmtLoc}")
        
        else:
            g.say(f"There are {numObj} {plurCat} on the {plcmtLoc}")
    
    except Exception as e:
        print(e)
        g.say(numObj)

    g.task_finished_count += 1