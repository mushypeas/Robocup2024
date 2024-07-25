# "findPrs": "{findVerb} the {gestPers_posePers} and {followup}"
def findPrs(g, params):
    print("followup: findPrs")
    
    # [0] Extract the parameters
    gestPosePers = params['gestPers_posePers']
    followup = params['followup']

    gestPosePers = g.cluster(gestPosePers, g.gesture_person_list + g.pose_person_list)
    
    # [1] Find the person
    g.identifyByGestPose(g, gestPosePers)
    
    # [2] Execute the followup
    g.exeFollowup(followup)
