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
        self.plcmt_locs = plcmt_locs
        self.plcmt_dict = plcmt_dict

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
    
    agent.pose.move_pose()
    agent.initial_pose('zero')
    agent.say('start gpsr')
    agent.door_open()
    agent.move_rel(1.0, 0, wait=True)
    
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
            try:
                cmdName, params = ultimateParser(inputText)

                cmdName = g.cluster(cmdName, g.cmdNameTocmdFunc.keys())        
                cmdFunc = g.cmdNameTocmdFunc[cmdName]
                
                g.return_point = g.agent.get_pose()
                
                cmdFunc(g, params)
            except:
                g.say("I think there's an error. see you again.")
                rospy.sleep(3.5)
            
        ## unusual object in plcmtloc
        for plcmt_loc in g.plcmt_locs:
            plcmt_loc_cat = plcmt_dict[plcmt_loc]
            plcmt_loc_available_items = g.category2objDict[plcmt_loc_cat]
            g.move(plcmt_loc)
            rospy.sleep(1)
            yolo_bbox = g.get_yolo_bbox()
            
            for item_info in yolo_bbox:
                objId = item_info[4]
                objName = g.objIdToName(objId)
                
                # Wrong item in plcmt_loc
                if objName not in plcmt_loc_available_items:
                    g.say(f"Oops! {objName} is in {plcmt_loc}")
                    rospy.sleep(3)
                    g.pick()    
                    
                    ## Find where to go
                    for cat in g.category2objDict:
                        if objName in cat:
                            optimal_plcmt_loc = cat_to_plcmt_loc(cat)
                            break
                        
                    g.move(optimal_plcmt_loc)
                    g.place(optimal_plcmt_loc)
                    
                    break
                        
        ## TODO: litter