import sys
sys.path.append("task/gpsr_repo/")
from gpsr_config import *
from gpsr_parser import ultimateParser

from _gpsr_main import GPSR
from itertools import cycle


class EGPSR(GPSR):
    def __init__(self, agent):
        super().__init__(agent)
        self.mov_fin = True
        self.room_cycle = cycle(self.rooms_list)

def egpsr(agent):
    eg = EGPSR(agent)
    
    g.move('gpsr_instruction_point')

    for cur_room in eg.room_cycle:
        if eg.task_finished_count >= task_iteration:
            break

        eg.move(cur_room)

        eg.identify()

        agent.say("Give a command after the ding sound.")
        rospy.sleep(2.5)

        inputText = g.hear(7.)
        agent.say(f"Given Command is {inputText}")

            
        # parse InputText 
        cmdName, params = ultimateParser(inputText)

        cmdName = g.cluster(cmdName, g.cmdNameTocmdFunc.keys()
        
        cmdFunc = g.cmdNameTocmdFunc[cmdName]
        cmdFunc(g, params)

        g.move('gpsr_instruction_point')