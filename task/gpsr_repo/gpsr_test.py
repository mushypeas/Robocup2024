import sys
import rospy
sys.path.append('task/gpsr_repo')
import _gpsr_main

def gpsr_test(agent):
    g = _gpsr_main.GPSR(agent)

    while True:
        cmd = input("Give me a task: ")

        if cmd == "exit":
            break

        if cmd == "img":
            g.img().show()

        if cmd == "hear":
            userSpoken = g.hear()
            print(f"user said: {userSpoken}")

        if cmd == "talk":
            talk_subject = input("What talk do you want?: ")
            g.talk(talk_subject)

        if cmd == "quiz":
            g.quiz()

        ### Perfectly working commands ###

        if cmd == "getName":
            name = g.getName()
            print(f"the name is: {name}")

        if cmd == "getPose":
            pose = g.getPose()
            print(f"the pose is: {pose}")

        if cmd == "getGest":
            gest = g.getGest()
            print(f"the gesture is: {gest}")

        if cmd == "getCloth":
            cloth = g.getCloth()
            print(f"the cloth is: {cloth}")

        if cmd == 'identify':
            g.identify()

        if cmd == "identifyStand":
            g.identifyByGestPose('standing person')
            
            
        if cmd == 'identifyWave':
            g.identifyByGestPose('waving person')

        if cmd == "pick":
            item_name = input("What item do you want to pick?: ")
            g.pick(item_name)
            
        if cmd == "pickCat":
            item_name = input("What item category do you want to pick?: ")
            g.pickCat(item_name)

        if cmd == "follow":
            g.follow()

        if cmd == 'clusterObj':
            name = input("What name do you want to cluster?: ")
            arr = g.object_names
            print(g.cluster(name, arr))


        if cmd == 'count':
            input_gestPose = input("What gesture or pose do you want to count?: ")
            cnt = g.countGestPosePers(input_gestPose)
            print(f"the count is: {cnt}")
            
        if cmd == 'identifyWaving':
            import subprocess
            openpose_path = "/home/tidy/Robocup2024/restaurant_openpose.sh"
            openpose_command = ['gnome-terminal', '--', 'bash', '-c', f'bash {openpose_path}; exec bash']
            openpose_process = subprocess.Popen(openpose_command)
            
            print("found?", g.identifyWaving)