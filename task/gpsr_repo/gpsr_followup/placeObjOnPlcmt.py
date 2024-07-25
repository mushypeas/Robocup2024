# "placeObjOnPlcmt": "{placeVerb} it {onLocPrep} the {plcmtLoc}"
def placeObjOnPlcmt(g, params):
    print("Start placeObjOnPlcmt")
    
    # [0] Extract the parameters
    loc = params['plcmtLoc']
    loc = g.cluster(loc, g.loc_list)
    
    # [1] move to the location
    g.move(loc)
    
    # [2] Place the object on the placement
    g.place(loc)