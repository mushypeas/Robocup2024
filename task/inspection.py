import rospy

def inspection(agent):
    agent.pose.move_pose()
    agent.initial_pose('zero')
    agent.say('start inspection')
    agent.door_open()
    agent.move_rel(1.0, 0, wait=True)
    agent.move_abs_safe_inspection('insp_target', thresh=0.95, timeout=10, giveup_timeout=10000, angle=90)
    agent.say('navigation succeeded.')


# import rospy
# from sensor_msgs.msg import Image
# from utils.depth_to_pc import Depth2PC


# import numpy as np
# class Inspection:
#     def __init__(self, agent):

#         rospy.Subscriber('/deeplab_ros_node/segmentation', Image, self._segment_cb)
#         self.agent = agent
#         self.d2pc = Depth2PC()

#         agent.pose.move_pose()
#         agent.initial_pose('zero')
#         agent.say('start inspection')
#         agent.door_open()
#         agent.move_rel(1.0, 0, wait=True)
#         agent.move_abs_safe('insp_target', thresh=0.85, timeout=10, giveup_timeout=10000, angle=90)
#         agent.say('navigation succeeded.')

#     def _segment_cb(self, data):
        
        
#         depth = np.asarray(self.agent.depth_image)

#         data_img = self.bridge.imgmsg_to_cv2(data, 'mono16')
#         self.seg_img = data_img




# def inspection(agent):
#     Inspection(agent)