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
    
    litter_room_cmd = input("litter: ")
    litter_room = None
    
    if litter_room_cmd == 'l' or litter_room_cmd == 'L':
        litter_room = 'living room'
    elif litter_room_cmd == 'h' or litter_room_cmd == 'H':
        litter_room = 'hallway'
    elif litter_room_cmd == 'k' or litter_room_cmd == 'K':
        litter_room = 'kitchen'
    elif litter_room_cmd == 'o' or litter_room_cmd == 'O':
        litter_room = 'office'        
    
    g = EGPSR(agent)
    
    agent.pose.move_pose()
    agent.initial_pose('zero')
    agent.say('start egpsr')
    agent.door_open()
    agent.move_rel(2.0, 0, wait=True)
    
    litter_left = True

    while not rospy.is_shutdown():
        # ## Find Waving Human
        for cur_room in g.rooms_list:
            g.move(cur_room)
            
            # litter_room이 있고 아직 가지 않았다면
            if litter_room and litter_left:
                # 그리고 현재 방이 litter_room이라면
                if litter_room == cur_room:
                    g.agent.pose.head_tilt(-10)
                    g.agent.pose.head_pan(30)
                    g.agent.pose.head_pan(-30)
                    g.agent.pose.head_pan(0)
                    g.say("oh my god. \n trash is over the floor.")
                    rospy.sleep(3.5)
                    
                    g.say(f"GIVE trash to me")
                    rospy.sleep(2)
                    g.agent.open_gripper()
                    g.say("3")
                    rospy.sleep(1)
                    g.say("2")
                    rospy.sleep(1)
                    g.say("1")
                    rospy.sleep(1)
                    g.agent.grasp()

                    g.move('trashcan')
                    g.say("take this and trash this.")
                    rospy.sleep(3)
                    g.say("3")
                    rospy.sleep(1)
                    g.say("2")
                    rospy.sleep(1)
                    g.say("1")
                    rospy.sleep(1)
                    g.agent.open_gripper()
                    rospy.sleep(3)
                    g.agent.grasp()
                    g.say("thank you!")
                    rospy.sleep(1.5)
                    
                    litter_left = False
                    g.move(litter_room)
            
            
            try:
                g.agent.pose.head_tilt(0)
                if not g.identifyWaving(): 
                    continue
            except:
                continue

            while not rospy.is_shutdown():
                g.say("Come close to mic, \n and Give a command after \n the ding sound.")
                rospy.sleep(4)

                inputText = g.hear(7.)
                g.say(f"Given Command is \n {inputText}")
                rospy.sleep(3.5)
                g.say("Is this right? answer yes or no after the ding sound")
                rospy.sleep(4)
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
            try:
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
                        g.pick(objName)    
                        
                        ## Find where to go
                        for cat in g.category2objDict:
                            if objName.replace('_', ' ') in g.category2objDict[cat] or objName.replace(' ', '_') in g.category2objDict[cat]:
                                print(objName, cat)
                                optimal_plcmt_loc = cat_to_plcmt_loc[cat]
                                break
                            
                        g.move(optimal_plcmt_loc)
                        g.place(optimal_plcmt_loc)
                        
                        break
            except Exception as e:
                g.say("sorry. i should go.")
                rospy.logwarn(e)
                rospy.sleep(1.5)
                pass
        
        