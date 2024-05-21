import rospy

# "followPrsAtLoc": "{followVerb} the {gestPers_posePers} {inRoom_atLoc}"
def followPrsAtLoc(g, params):
    # Follow the person raising their right arm at the armchair
    # Follow the standing person in the bedroom
    # Follow the sitting person at the dishwasher
    # Follow the person pointing to the left in the bedroom
    print("Start followPrsAtLoc")

    # [0] Extract parameters
    gestPosePers = params['gestPers_posePers']
    loc = g.extractLocFrominRoomatLoc(params['inRoom_atLoc'])

    # [1] Move to the specified room
    g.move(loc)

    # [2] Find the person with that pose in the room
    g.identifyByGestPose(gestPosePers)

    # [3] Follow the person
    g.follow()