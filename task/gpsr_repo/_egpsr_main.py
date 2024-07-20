import sys
import rospy
sys.path.append("task/gpsr_repo/")
from gpsr_config import *
from gpsr_parser import ultimateParser
import subprocess

from _gpsr_main import GPSR


class EGPSR(GPSR):
    def __init__(self, agent):
        super().__init__(agent)

def egpsr(agent):
    openpose_path = "/home/tidy/Robocup2024/restaurant_openpose.sh"
    whisper_path = "/home/tidy/Robocup2024/whisper.sh"
    yolov7_pose_path = "/home/tidy/Robocup2024/yolov7_pose.sh"
    
    openpose_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {openpose_path}; exec bash']
    whisper_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {whisper_path}; exec bash']
    yolov7_pose_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {yolov7_pose_path}; exec bash']

    openpose_process = subprocess.Popen(openpose_command)
    whisper_process = subprocess.Popen(whisper_command)
    yolov7_pose_process = subprocess.Popen(yolov7_pose_command)
    
    g = EGPSR(agent)
    
    g.move('gpsr_instruction_point')

    while not rospy.is_shutdown():
        ## Find Waving Human
        for cur_room in g.rooms_list:
            g.move(cur_room)

            if not g.identifyWaving(): 
                continue

            while not rospy.is_shutdown():
                agent.say("Give a command after \n the ding sound.")
                rospy.sleep(2.5)

                inputText = g.hear(7.)
                agent.say(f"Given Command is \n {inputText}")
                rospy.sleep(3.5)
                agent.say("Is this right? answer yes or no after the ding sound")
                rospy.sleep(3)
                yes_no = g.cluster(g.hear(2.), ['yes', 'no'])
                
                if yes_no == 'yes':
                    break               
            
            # parse InputText 
            cmdName, params = ultimateParser(inputText)

            cmdName = g.cluster(cmdName, g.cmdNameTocmdFunc.keys())        
            cmdFunc = g.cmdNameTocmdFunc[cmdName]
            
            g.return_point = g.agent.get_pose()
            
            cmdFunc(g, params)
            
        ## TODO: unusual object in plcmtloc
        ## TODO: litter