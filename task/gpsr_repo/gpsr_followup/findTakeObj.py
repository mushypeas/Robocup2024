# "findObj": "{findVerb} {art} {obj_singCat} and {takeVerb} it and {followup}"
def findTakeObj(g, params):
    print("followup: findTakeObj")
    
    # [0] Extract parameters
    obj = params["obj_singCat"]
    followup = params["followup"]
    
    # [1] Find the object and pick it
    g.pick(obj)
    
    # [2] Follow-up command
    g.exeFollowup(followup)
    