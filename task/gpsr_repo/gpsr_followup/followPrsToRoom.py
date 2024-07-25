# "followPrsToRoom": "{followVerb} them {toLocPrep} the {loc_room}"
def followPrsToRoom(g, params):
    print("followup: followPrsToRoom")
    
    # [0] Extract the parameters
    loc = params['loc_room']

    loc = g.cluster(loc, g.loc_list)
    
    # [1] Follow the person to the room
    g.followToLoc(loc)