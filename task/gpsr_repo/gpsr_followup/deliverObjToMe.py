import rospy

# "deliverObjToMe": "{deliverVerb} it to me"
def deliverObjToMe(g, params):
    print("followup: deliverObjToMe")
    
    # [1] Move to the instruction location
    g.agent.move_abs_coordinate(g.return_point)
    
    # [2] Deliver the object to the person
    g.deliver()