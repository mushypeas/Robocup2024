import rospy

# "followPrsAtLoc": "{followVerb} the {gestPers_posePers} {inRoom_atLoc}"
def followPrsAtLoc(g, params):
    # Follow the person raising their right arm at the armchair
    # Follow the standing person in the bedroom
    # Follow the sitting person at the dishwasher
    # Follow the person pointing to the left in the bedroom
    print("Start followPrsAtLoc")

    # [0] Extract parameters
    try:
        gestPosePers = params['gestPers_posePers']
        loc = params['inRoom_atLoc']
    except:
        g.cmdError()
        return

    gestPosePers = g.cluster(gestPosePers, g.gesture_person_list + g.pose_person_list)
    loc = g.cluster(loc, g.loc_list)

    # [1] Move to the specified room
    g.move(loc)

    # [2] Find the person with that pose in the room
    g.identifyByGestPose(gestPosePers)

    # [3] Follow the person
    g.follow()

    g.task_finished_count += 1