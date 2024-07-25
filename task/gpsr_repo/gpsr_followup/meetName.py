# "meetName": "{meetVerb} {name} and {followup}"
def meetName(g, params):
    print("followup: meetName")
    
    # [0] Extract parameters
    name = params['name']
    followup = params['followup']
    
    # [1] Meet the person
    g.identifyByName(name)
    
    # [2] followup
    g.exeFollowup(followup)