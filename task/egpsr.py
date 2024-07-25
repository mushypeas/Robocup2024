import sys
sys.path.append('task/gpsr_repo/')

from task.gpsr_repo._egpsr_main import egpsr as eg

def egpsr(agent):
    eg(agent)