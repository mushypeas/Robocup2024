import rospy
import cv2
import numpy as np
import tf
import ros_numpy
#from pyquaternion import Quaternion as Quaternion
from sensor_msgs.msg import PointCloud2, Image
from open3d import geometry
from hsr_agent.global_config import *

###################################
### Additional import if needed ###
###################################

class PseudoAgent:

    def __init__(self):

        self._depth_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image, self._depth_callback)
        self._pc_sub = rospy.Subscriber(PC_TOPIC, PointCloud2, self._pc_callback)
        self.pc = None
        self.listener = tf.TransformListener()

    def _depth_callback(self, data):
        self.stamp = data.header.stamp
        self.depth_for_pc = geometry.Image(ros_numpy.numpify(data).astype(np.uint16))
        self.depth_image = ros_numpy.numpify(data).astype(np.uint16)

    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)


    def distancing(self, height, dist=0.6):
        ############
        ### TODO ###
        ############
        try:
            trans, rot = self.listener.lookupTransform('/base_link', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))
            w = rot[3]
            x = rot[0]
            y = rot[1]
            z = rot[2]
            #myq = Quaternion(axis = [x,y,z], angle = w)
            mat = tf.transformations.quaternion_matrix([x, y, z, w])
            mat[0, 3] += trans[0]
            mat[1, 3] += trans[1]
            mat[2, 3] += trans[2]
            R = self.listener.fromTranslationRotation(trans, rot)
            print(R)
            print(mat)
            #myq.rotate()
            return R
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None





if __name__ == '__main__':
    #############################################
    ### This test is only for ROS bag looping ###
    #############################################
    import sys
    table = {'kitchen': 0.74,
             'breakfast': 0.61,
             'shelf_1': 0.77,
             'shelf_2': 1.14}
    try:
        pos_key = str(sys.argv[1])
        height = table[pos_key]
    except KeyError:
        print('Undefined key.')
        sys.exit(0)
    except IndexError:
        print('Need key of height.')
        sys.exit(0)
    rospy.init_node('distancing_test', disable_signals=True)
    agent = PseudoAgent()
    
    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.5)
            corr = 0
            myq = agent.distancing(height)
            basepc = agent.pc
            #basepc = np.ones(shape=(480, 640, 4))
            #for i in range(0,480):
            #   for j in range(0,640):
            #       basepc[i,j][0] = agent.pc[i,j][0]
            #       basepc[i,j][1] = agent.pc[i, j][1]
            #       basepc[i,j][2] = agent.pc[i, j][2]
            #basepc[i,j] = (myq*basepc[i,j].T).T
            X = basepc['x']
            Y = basepc['y']
            Z = basepc['z']
            H = np.ones((480, 640))
            PC = np.stack([X, Y, Z, H], axis=-1)
            cnt = 0
            """
            for i in range(0,480):
                meanx = 0
                meanz = 0
                for j in range(300,340):
                    #basepc[i,j][0] = PC[i,j][0]
                    #basepc[i,j][1] = agent.pc[i, j][1]
                    #basepc[i,j][2] = agent.pc[i, j][2]
                    #print(PC.shape)
                    #print(PC[240, 320])
                    d = np.matmul(myq, PC[i, j].T).T
                    meanz += d[2]
                    meanx += d[0]
                meanz = meanz/40
                meanx = meanx/40
                if((meanz-height)<0.01):
                    cnt += 1
                else: 
                    cnt = 0
                if(cnt == 10):
                    corr = meanx - 0.6
                    break

"""
            PC = np.reshape(PC, (-1, 4))
            PC = PC[np.logical_not(np.isnan(PC).sum(axis=-1))]
            basePC = np.matmul(PC, myq.T)
            filteredPC = basePC[np.abs(basePC[:, 2] - height) < 0.005]
            medianx = np.median(filteredPC[:, 0])
            filteredPC = filteredPC[np.abs(filteredPC[:, 0] - medianx) < 0.05]
            corr = np.mean(filteredPC[:, 0]) - 0.6
                    #convert the whole PC to base_link


            #print(agent.pc[240,320])
            #trans = agent.distancing(height)
            #print(trans)
            #print(myq)
            #print(agent.depth_image[240, 320])

            rospy.loginfo(f'Distancing Correction: {round(corr, 6)}m')
        except (TypeError, ValueError):
            rospy.loginfo(f'Unappropriare Returned Value: {corr}')
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            continue






