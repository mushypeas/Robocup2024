import rospy
from hsr_agent.agent_rest import Agent
from task.restaurant import restaurant

'''
# from task.set_the_table import set_the_table
from task.carry_my_luggage import carry_my_luggage
from task.receptionist import receptionist
from task.storing_groceries import storing_groceries
from task.serve_breakfast import serve_breakfast
from task.gpsr import gpsr
from task.clean_the_table import clean_the_table
from task.restaurant import restaurant
from task.stickler_for_the_rules import stickler_for_the_rules
from task.egpsr import egpsr
from task.inspection import inspection
'''

if __name__ == '__main__':

    rospy.init_node('main_client_hsr', disable_signals=True)
    agent = Agent()
    print('TASK restaurant without /pose_integrator & original /move_base')
    restaurant(agent)
