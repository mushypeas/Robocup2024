import rospy

def set_the_table(agent):
    agent.say('start set the table')
    object_list = [('plate', True, 'side'),
                   ('bowl', True, 'side'),
                   ('spoon', False, 'top'),
                   ('knife', False, 'top'),
                   ('fork', False, 'top')]
    agent.open_gripper()
    agent.pose.move_pose()
    agent.move_abs('start_1')
    # plate
    for object, vertical, place_axis in object_list:
        agent.move_abs('shelf_front')

        agent.show_image(object)
        agent.pose.neutral_pose(vertical=vertical)
        agent.open_gripper()
        agent.say('please give me '+object)
        rospy.sleep(1)
        agent.say('five'); rospy.sleep(1)
        agent.say('four'); rospy.sleep(1)
        agent.say('three'); rospy.sleep(1)
        agent.say('two'); rospy.sleep(1)
        agent.say('one'); rospy.sleep(1)

        agent.grasp()
        agent.pose.move_pose(vertical=vertical)
        agent.move_abs('table_front')
        if place_axis == 'side':
            agent.pose.place_side_pose(vertical=vertical)
        else:
            agent.pose.pick_top_pose()
        try:
            if object == 'knife' or object == 'spoon':
                agent.move_rel(0, -0.2)
            elif object == 'fork':
                agent.move_rel(0, 0.2)
            agent.move_rel(0.2, 0)
        except:
            pass
        agent.open_gripper()

        agent.move_rel(-0.1, 0)
        agent.pose.move_pose()
