import rospy
from hsr_agent.agent import Agent
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
    print('TASK')
    print('0 : inspection')
    print('1 : carry_my_luggage')
    print('2 : receptionist')
    print('3 : storing_groceries')
    print('4 : serve_breakfast')
    print('5 : gpsr')
    print('6 : clean_the_table')
    print('7 : restaurant')
    print('8 : stickler_for_the_rules')
    print('9 : egpsr')
    print('10 : final')



    task_id = input('task num : ')

    if task_id == '0':
        from task.inspection import inspection
        inspection(agent)
    elif task_id == '1':
        from task.carry_my_luggage import carry_my_luggage
        carry_my_luggage(agent)
    elif task_id == '2':
        from task.receptionist import receptionist
        receptionist(agent)
    elif task_id == '3':
        from task.storing_groceries import StoringGroceries
        task = StoringGroceries(agent)
        task.run()
    elif task_id == '4':
        from task.serve_breakfast import serve_breakfast
        serve_breakfast(agent)
    elif task_id == '5':
        from task.gpsr import gpsr
        gpsr(agent)
    elif task_id == '6':
        from task.clean_the_table_bj import clean_the_table
        clean_the_table(agent)
    elif task_id == '7':
        from task.restaurant import restaurant
        restaurant(agent)
    elif task_id == '8':
        from task.stickler_for_the_rules import stickler_for_the_rules
        stickler_for_the_rules(agent)
    elif task_id == '9':
        from task.egpsr import egpsr
        egpsr(agent)
    elif task_id == '10':
        from task.final import final
        final(agent)
