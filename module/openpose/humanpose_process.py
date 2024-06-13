import rospy
from std_msgs.msg import Int16MultiArray


def get_hand_points(detected_keypoints, personwise_keypoints, pose_hand_pub, ratio):

    ret = Int16MultiArray()
    # ret.data = [[1,2]]
    if(detected_keypoints == []):
        pose_hand_pub.publish(ret)
        return

    for personI in personwise_keypoints:
        # if personI[4] != -1 and personI[7] != -1 and personI[9] != -1 and personI[12] != -1:
        if personI[4] != -1 or personI[7] != -1:
            for i in detected_keypoints[4]:
                if(i[3]==personI[4]):
                    ret.data+=list(i[0:2])
                    break
            for i in detected_keypoints[7]:
                if(i[3]==personI[7]):
                    ret.data+=list(i[0:2])
                    break
    ret.data = list(map(lambda x:int(x//ratio),ret.data))
    # rospy.loginfo(f'hand {ret.data}')

    pose_hand_pub.publish(ret)
    # return ret.data
    return

def get_knee_points(detected_keypoints, knee_pose_pub, ratio):

    ret = Int16MultiArray()
    if len(detected_keypoints[9]) == 0 and len(detected_keypoints[12]) == 0:
        knee_pose_pub.publish(ret)
        return
    for _knee in detected_keypoints[9]:
        ret.data += [int(_k*(1.0/ratio)) for _k in _knee[:2]]
    for _knee in detected_keypoints[12]:
        ret.data += [int(_k*(1.0/ratio)) for _k in _knee[:2]]

    rospy.loginfo(f'knee {ret.data}')
    knee_pose_pub.publish(ret)


def get_ankle_points(detected_keypoints, ankle_pose_pub, ratio):

    ret = Int16MultiArray()
    if len(detected_keypoints[10]) == 0 and len(detected_keypoints[13]) == 0:
        ankle_pose_pub.publish(ret)
        return
    for _ankle in detected_keypoints[10]:
        ret.data += [int(_a*(1.0/ratio)) for _a in _ankle[:2]]
    for _ankle in detected_keypoints[13]:
        ret.data += [int(_a*(1.0/ratio)) for _a in _ankle[:2]]

    rospy.loginfo(f'ankle {ret.data}')
    ankle_pose_pub.publish(ret)
