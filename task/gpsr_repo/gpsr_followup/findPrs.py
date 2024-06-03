# "findPrs": "{findVerb} the {gestPers_posePers} and {followup}"
def findPrs(g, params):
    print("followup: findPrs")
    
    # [0] Extract the parameters
    gestPosePers = params['gestPers_posePers']
    followup = params['followup']
    
    # [1] Find the person
    g.identifyByGestPose(g, gestPosePers)
    
    # [2] Execute the followup
    g.exeFollowup(followup)
