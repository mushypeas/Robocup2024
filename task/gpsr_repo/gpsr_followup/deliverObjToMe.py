import rospy

# "deliverObjToMe": "{deliverVerb} it to me"
def deliverObjToMe(g, params):
    print("followup: deliverObjToMe")
    
    # [1] Move to the instruction location
    g.move('gpsr_instruction_location')
    
    # [2] Deliver the object to the person
    g.deliver()