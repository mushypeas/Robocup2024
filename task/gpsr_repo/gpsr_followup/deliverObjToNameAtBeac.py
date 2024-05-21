# "deliverObjToNameAtBeac": "{deliverVerb} it {deliverPrep} {name} {inLocPrep} the {room}"
def deliverObjToNameAtBeac(g, params):
    print("followup: deliverObjToNameAtBeac")
    
    # [0] Extract the parameters
    name = params['name']
    room = params['room']
    
    # [1] Move to the room
    g.move(room)
    
    # [2] Find the person
    g.identifyByName(name)
    
    # [3] Deliver the object
    g.deliver()