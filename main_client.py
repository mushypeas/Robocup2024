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
    print()
    print('╭─────────────────────────────────────────────────────────────────────────╮')
    print('│                                                                         │')
    print('│              - Team TIDYBOY-DSPL ROBOCUP2024 Main Client -              │')
    print('│                                                                         │')
    print('╭───────────────────────── Select Task (0 - 10) ──────────────────────────╮')
    print('│                                                                         │')
    print('│   0  : Inspection                                                       │')
    print('│   1  : Carry My Luggage                                                 │')
    print('│   2  : Receptionist                                                     │')
    print('│   3  : Storing Groceries                                                │')
    print('│   4  : Serve Breakfast                                                  │')
    print('│   5  : GPSR (General Purpose Service Robot)                             │')
    print('│   6  : Clean the Table                                                  │')
    print('│   7  : Restaurant                                                       │')
    print('│   8  : Stickler for the Rules                                           │')
    print('│   9  : EGPSR (Enhanced General Purpose Service Robot)                   │')
    print('│   10 : Final                                                            │')
    print('│                                                                         │')
    print('╰─────────────────────────────────────────────────────────────────────────╯')


    task_id = input('task num : ')

    if task_id == '0':
        from task.inspection import inspection
        inspection(agent)
    elif task_id == '1':
        print("carry_my_luggage")
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
        from task.clean_the_table import clean_the_table
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
