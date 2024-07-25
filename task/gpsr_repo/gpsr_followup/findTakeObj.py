# "findObj": "{findVerb} {art} {obj_singCat} and {takeVerb} it and {followup}"
def findTakeObj(g, params):
    print("followup: findTakeObj")
    
    # [0] Extract parameters
    obj_cat = params["obj_singCat"]
    followup = params["followup"]

    obj_cat = g.cluster(obj_cat, g.object_names + g.object_categories_singular)

    # [1] Find the object and pick it
    if obj_cat in g.object_categories_singular:
        g.pickCat(obj_cat)

    else:
        g.pick(obj_cat)
    
    # [2] Follow-up command
    g.exeFollowup(followup)
    