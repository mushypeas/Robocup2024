# "guidePrsToBeacon": "{guideVerb} them {toLocPrep} the {loc_room}"
def guidePrsToBeacon(g, params):
    print("Start guidePrsToBeacon")
    
    # [0] Extract the parameters
    loc = params["loc_room"]
    loc = g.cluster(loc, g.loc_list)
    
    # [1] Guide the person to the location
    g.guide(loc)