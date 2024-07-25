# "deliverObjToPrsInRoom": "{deliverVerb} it {deliverPrep} the {gestPers_posePers} {inLocPrep} the {room}"
def deliverObjToPrsInRoom(g, params):
    print("followup: deliverObjToPrsInRoom")
    
    # [0] Extract the parameters
    room = params['room']
    gestPosePers = params['gestPers_posePers']

    room = g.cluster(room, g.rooms_list)
    gestPosePers = g.cluster(gestPosePers, g.gesture_person_list + g.pose_person_list)
    
    # [1] Move to the room
    g.move(room)
    
    # [2] Find the person
    g.identifyByGestPose(gestPosePers)
    
    # [3] Deliver the object to the person
    g.deliver()