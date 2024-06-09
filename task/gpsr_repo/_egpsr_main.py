import sys
sys.path.append("task/gpsr_repo/")
from gpsr_config import *

from _gpsr_main import GPSR
from itertools import cycle


class EGPSR(GPSR):
    def __init__(self, agent):
        super().__init__(agent)
        self.mov_fin = True
        self.room_cycle = cycle(self.rooms_list)

def egpsr(agent):
    eg = EGPSR(agent)
    
    for cur_room in eg.room_cycle:
        if eg.task_finished_count >= task_iteration:
            break

        eg.move(cur_room)

        eg.identify()