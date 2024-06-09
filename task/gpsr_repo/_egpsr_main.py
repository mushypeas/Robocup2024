import sys
sys.path.append("task/gpsr_repo/")
from gpsr_config import *

from _gpsr_main import GPSR


class EGPSR(GPSR):
    def __init__(self, agent):
        super().__init__(agent)
        self.cur_room = 'living room'
        self.mov_fin = True

def egpsr(agent):
    eg = EGPSR(agent)
    
    while eg.task_finished_count < task_iteration:
        