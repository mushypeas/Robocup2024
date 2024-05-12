import rospy

def deliverObjToMe(g, params):
    g.move('gpsr_instruction_location')
    g.deliver()