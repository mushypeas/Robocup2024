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

        if cmd == "pick":
            item_name = input("What item do you want to pick?: ")
            g.pick(item_name)

        if cmd == "hear":
            userSpoken = g.hear()
            print(f"user said: {userSpoken}")

        if cmd == "talk":
            g.talk("the time")
            rospy.sleep(1)
            g.talk("what day is today")
            rospy.sleep(1)
            g.talk("what day is tomorrow")

        if cmd == "img":
            g.img().show()

        if cmd == "quiz":
            g.quiz()

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
            g.identifyByGestPose('standing')