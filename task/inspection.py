import rospy

def inspection(agent):
    agent.pose.move_pose()
    agent.initial_pose('zero')
    agent.say('start inspection')
    agent.door_open()
    agent.move_rel(1.0, 0, wait=True)
    agent.move_abs_safe('insp_target', thresh=0.45, timeout=5, giveup_timeout=10000, angle=90)
    agent.say('navigation succeeded.')