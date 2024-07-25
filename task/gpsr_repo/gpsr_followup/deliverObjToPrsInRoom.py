# "deliverObjToPrsInRoom": "{deliverVerb} it {deliverPrep} the {gestPers_posePers} {inLocPrep} the {room}"
def deliverObjToPrsInRoom(g, params):
    print("followup: deliverObjToPrsInRoom")
    
    # [0] Extract the parameters
    room = params['room']
    gestPosePers = params['gestPers_posePers']
    
    # [1] Move to the room
    g.move(room)
    
    # [2] Find the person
    g.identifyByGestPose(gestPosePers)
    
    # [3] Deliver the object to the person
    g.deliver()