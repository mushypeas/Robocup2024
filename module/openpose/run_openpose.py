'''
Original Code Source:
https://github.com/spmallick/learnopencv/blob/master/OpenPose-Multi-Person/multi-person-openpose.py
'''
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

from config import POSE_PAIRS, MAP_IDX, COLORS
from humanpose_process import get_hand_points, get_knee_points, get_ankle_points

import sys
sys.path.append('../../../robocup2024')
from hsr_agent.global_config import RGB_TOPIC

class OpenPoseWrapper:

    def __init__(self, BASE_DIR, detect_option, size_ratio=1.0, enable_viz=False) :
        # openpose
        proto  = BASE_DIR + 'pose_deploy_linevec.prototxt'
        weight = BASE_DIR + 'pose_iter_440000.caffemodel'
        self.detect_option = detect_option
        self.openpose = cv2.dnn.readNetFromCaffe(proto, weight)
        # GPU
        self.openpose.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.openpose.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        self.nPoints = 18
        # visualization option
        self.enable_viz = enable_viz
        # ROS subscriber
        self.rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, self.rgb_mp_cb)
        self.bridge = CvBridge()
        self.rgb_img = None
        # ROS publisher
        if enable_viz:
            self.openpose_pub = rospy.Publisher('/snu/openpose', Image, queue_size=10)
        if 'hands' in self.detect_option:
            self.pose_hand_pub = rospy.Publisher('/snu/openpose/hand', Int16MultiArray, queue_size=10)
        if 'knee' in self.detect_option:
            self.knee_pose_pub = rospy.Publisher('/snu/openpose/knee', Int16MultiArray, queue_size=10)
        if 'ankle' in self.detect_option:
            self.ankle_pose_pub = rospy.Publisher('/snu/openpose/ankle', Int16MultiArray, queue_size=10)
        self.bbox_pub = rospy.Publisher('/snu/openpose/bbox', Int16MultiArray, queue_size=10)
        self.human_bbox_pub = rospy.Publisher('/snu/openpose/human_bbox', Int16MultiArray, queue_size=10)
        # self.human_bbox_pub = rospy.Publisher('/snu/openpose/human_bbox', Int16MultiArray, queue_size=10)
        self.human_bbox_with_hand_pub = rospy.Publisher('/snu/openpose/human_bbox_with_pub', Int16MultiArray, queue_size=10)

        # Image Info
        self.width  = int(640 * size_ratio)
        self.height = int(480 * size_ratio)
        self.ratio = size_ratio
        self.input_scale = 1.0 / 255
        self.detected_keypoints = []
        self.personwise_keypoints = []


    def rgb_mp_cb(self, msg):
        bgr_img =  self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        return

    def run(self, thresh=0.1):
        if self.rgb_img is None: return
        img = self.rgb_img
        if self.ratio < 1.0:
            img = cv2.resize(img, (self.width, self.height), cv2.INTER_AREA)
        blob = cv2.dnn.blobFromImage(img,
                                     self.input_scale,
                                     (self.width, self.height),
                                     (0, 0, 0))
        self.openpose.setInput(blob)
        output = self.openpose.forward()
        # Detect pose keypoints & aggregate them by affinity maps
        detected_keypoints = []
        keypoints_list = np.zeros((0, 3))
        keypoint_id = 0
        for part in range(self.nPoints):
            probMap = output[0, part, :, :]
            probMap = cv2.resize(probMap, (self.width, self.height))
            keypoints = self.get_keypoints(probMap, thresh)
            keypoints_with_id = []
            for i in range(len(keypoints)):
                keypoints_with_id.append(keypoints[i] + (keypoint_id,))
                keypoints_list = np.vstack([keypoints_list, keypoints[i]])
                keypoint_id += 1
            detected_keypoints.append(keypoints_with_id)
        valid_pairs, invalid_pairs = self.get_valid_pairs(output, detected_keypoints)
        personwise_keypoints = self.get_personwise_keypoints(valid_pairs,
                                                            invalid_pairs,
                                                            keypoints_list)
        self.detected_keypoints = detected_keypoints
        self.personwise_keypoints = personwise_keypoints
        # print(f'Personwise Keypoints: {personwise_keypoints}')
        # print(f'Detected Keypoints: {detected_keypoints}')
        ####### postprocessing start ############
        if 'hands' in self.detect_option:
            get_hand_points(detected_keypoints, personwise_keypoints, self.pose_hand_pub, self.ratio)
            # hand_coord = np.int32(np.array(hand_coord).reshape(-1, 2) * self.ratio)
        if 'knee' in self.detect_option:
            get_knee_points(detected_keypoints, self.knee_pose_pub, self.ratio)
        if 'ankle' in self.detect_option:
            get_ankle_points(detected_keypoints, self.ankle_pose_pub, self.ratio)
        # todo -> move to humanpose_process.py
        handup_bbox_list = self.personwise_handup(keypoints_list, personwise_keypoints)

        # Send human_bbox info of all people
        human_bbox_list = self.personwise_human_bbox(keypoints_list, personwise_keypoints)
        human_bbox_msg = Int16MultiArray()
        human_bbox_data = []
        for human_bbox in human_bbox_list:
            human_bbox[0] = int(human_bbox[0] / self.ratio)
            human_bbox[1] = int(human_bbox[1] / self.ratio)
            human_bbox[2] = int(human_bbox[2] / self.ratio)
            human_bbox[3] = int(human_bbox[3] / self.ratio)
            rospy.loginfo(f'human_bbox: TOP LFET: {human_bbox[0:2]} BOTTOM RIGHT: {human_bbox[2:4]}')
            human_bbox_data += human_bbox
        human_bbox_msg.data = human_bbox_data
        self.human_bbox_pub.publish(human_bbox_msg)
        # handup_bbox_list = self.personwise_handup(keypoints_list, personwise_keypoints) # handup deactivated

        # human_bbox_list = self.personwise_human_bbox(keypoints_list, personwise_keypoints)
        human_bbox_with_hand_list = self.personwise_human_bbox_with_hand(detected_keypoints, keypoints_list, personwise_keypoints)

        # Visualize and publish Openpose result if enabled
        if self.enable_viz:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            for i in range(self.nPoints):
                for j in range(len(detected_keypoints[i])):
                    cv2.circle(img, detected_keypoints[i][j][0:2], 2, COLORS[i], -1, cv2.LINE_AA)
            for i in range(17):
                for n in range(len(personwise_keypoints)):
                    index = personwise_keypoints[n][np.array(POSE_PAIRS[i])]
                    if -1 in index:
                        continue
                    B = np.int32(keypoints_list[index.astype(int), 0])
                    A = np.int32(keypoints_list[index.astype(int), 1])
                    cv2.line(img, (B[0], A[0]), (B[1], A[1]), COLORS[i], 1, cv2.LINE_AA)
                    
            # Draw human bbox
            for bbox in human_bbox_with_hand_list:
                cv2.rectangle(img, bbox[0:2], bbox[2:4], (255, 0, 255), 2)

            # for coord in hand_coord:
            #     cv2.circle(img, tuple(coord), 6, (0, 0, 255), -1)
            for coord in human_bbox_with_hand_list:
                if coord[4] != -1:
                    cv2.circle(img, (coord[4], coord[5]), 6, (0, 0, 255), -1)
                if coord[6] != -1:
                    cv2.circle(img, (coord[6], coord[7]), 6, (0, 0, 255), -1)
                # cv2.circle(img, tuple(coord), 6, (0, 0, 255), -1)
            
            # # Draw handup bbox # handup deactivated
            # for bbox in handup_bbox_list:
            #     cv2.rectangle(img, bbox[0:2], bbox[2:4], (0, 255, 0), 2)
            img = cv2.resize(img,
                             (int(self.width / self.ratio), int(self.height / self.ratio)),
                             cv2.INTER_LINEAR)
            cv2.imshow('OpenPose', img)
            cv2.waitKey(1)
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.openpose_pub.publish(msg)
        # Send bbox info of hand-waving people
        msg = Int16MultiArray()
        data = []
        for bbox in handup_bbox_list:
            bbox[0] = int(bbox[0] / self.ratio)
            bbox[1] = int(bbox[1] / self.ratio)
            bbox[2] = int(bbox[2] / self.ratio)
            bbox[3] = int(bbox[3] / self.ratio)
            # rospy.loginfo(f'Found: TOP LFET: {bbox[0:2]} BOTTOM RIGHT: {bbox[2:4]}')
            data += bbox
        msg.data = data
        self.bbox_pub.publish(msg)

        # # Send human_bbox info of all people
        # human_bbox_msg = Int16MultiArray()
        # human_bbox_data = []
        # for human_bbox in human_bbox_list:
        #     human_bbox[0] = int(human_bbox[0] / self.ratio)
        #     human_bbox[1] = int(human_bbox[1] / self.ratio)
        #     human_bbox[2] = int(human_bbox[2] / self.ratio)
        #     human_bbox[3] = int(human_bbox[3] / self.ratio)
        #     # rospy.loginfo(f'human_bbox: TOP LFET: {human_bbox[0:2]} BOTTOM RIGHT: {human_bbox[2:4]}')
        #     human_bbox_data += human_bbox
        # human_bbox_msg.data = human_bbox_data
        # rospy.loginfo(f'human_bbox_data: {human_bbox_data}')
        # self.human_bbox_pub.publish(human_bbox_msg)
            
        # Send human_bbox_with_hand info of all people
        human_bbox_with_hand_msg = Int16MultiArray()
        human_bbox_with_hand_data = []
        for human_bbox in human_bbox_with_hand_list:
            human_bbox[0] = int(human_bbox[0] / self.ratio)
            human_bbox[1] = int(human_bbox[1] / self.ratio)
            human_bbox[2] = int(human_bbox[2] / self.ratio)
            human_bbox[3] = int(human_bbox[3] / self.ratio)
            if human_bbox[4] != -1:
                human_bbox[4] = int(human_bbox[4] / self.ratio)
                human_bbox[5] = int(human_bbox[5] / self.ratio)
            if human_bbox[6] != -1:
                human_bbox[6] = int(human_bbox[6] / self.ratio)
                human_bbox[7] = int(human_bbox[7] / self.ratio)
            # rospy.loginfo(f'human_bbox: TOP LFET: {human_bbox[0:2]} BOTTOM RIGHT: {human_bbox[2:4]}')
            human_bbox_with_hand_data += human_bbox
        human_bbox_with_hand_msg.data = human_bbox_with_hand_data
        rospy.loginfo(f'human_bbox_with_hand_list: {human_bbox_with_hand_list}')
        self.human_bbox_with_hand_pub.publish(human_bbox_with_hand_msg)

        return

    def get_keypoints(self, pmap, thresh=0.1):
        mapSmooth = cv2.GaussianBlur(pmap, (3, 3), 0, 0)
        mapMask = np.uint8(mapSmooth > thresh)
        keypoints = []
        #find the blobs
        contours, _ = cv2.findContours(mapMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #for each blob find the maxima
        for cnt in contours:
            blobMask = np.zeros(mapMask.shape)
            blobMask = cv2.fillConvexPoly(blobMask, cnt, 1)
            maskedProbMap = mapSmooth * blobMask
            _, maxVal, _, maxLoc = cv2.minMaxLoc(maskedProbMap)
            keypoints.append(maxLoc + (pmap[maxLoc[1], maxLoc[0]],))
        return keypoints

    def get_valid_pairs(self, output, detected_keypoints):

        valid_pairs = []
        invalid_pairs = []
        n_interp_samples = 10
        paf_score_th = 0.1
        conf_th = 0.7
        # loop for every POSE_PAIR
        for k in range(len(MAP_IDX)):
            # A->B constitute a limb
            pafA = output[0, MAP_IDX[k][0], :, :]
            pafB = output[0, MAP_IDX[k][1], :, :]
            pafA = cv2.resize(pafA, (self.width, self.height))
            pafB = cv2.resize(pafB, (self.width, self.height))

            # Find the keypoints for the first and second limb
            candA = detected_keypoints[POSE_PAIRS[k][0]]
            candB = detected_keypoints[POSE_PAIRS[k][1]]
            nA = len(candA)
            nB = len(candB)

            # If keypoints for the joint-pair is detected
            # check every joint in candA with every joint in candB
            # Calculate the distance vector between the two joints
            # Find the PAF values at a set of interpolated points between the joints
            # Use the above formula to compute a score to mark the connection valid
            if( nA != 0 and nB != 0):
                valid_pair = np.zeros((0,3))
                for i in range(nA):
                    max_j=-1
                    maxScore = -1
                    found = 0
                    for j in range(nB):
                        # Find d_ij
                        d_ij = np.subtract(candB[j][:2], candA[i][:2])
                        norm = np.linalg.norm(d_ij)
                        if norm:
                            d_ij = d_ij / norm
                        else:
                            continue
                        # Find p(u)
                        interp_coord = list(zip(np.linspace(candA[i][0],
                                                            candB[j][0], num=n_interp_samples),
                                                np.linspace(candA[i][1], candB[j][1],
                                                            num=n_interp_samples)))
                        # Find L(p(u))
                        paf_interp = []
                        for k in range(len(interp_coord)):
                            paf_interp.append([pafA[int(round(interp_coord[k][1])),
                                                    int(round(interp_coord[k][0]))],
                                               pafB[int(round(interp_coord[k][1])),
                                                    int(round(interp_coord[k][0]))] ])
                        # Find E
                        paf_scores = np.dot(paf_interp, d_ij)
                        avg_paf_score = sum(paf_scores)/len(paf_scores)

                        # Check if the connection is valid
                        # If the fraction of interpolated vectors aligned with PAF is higher
                        # then threshold -> Valid Pair
                        if ( len(np.where(paf_scores > paf_score_th)[0]) / n_interp_samples ) > conf_th :
                            if avg_paf_score > maxScore:
                                max_j = j
                                maxScore = avg_paf_score
                                found = 1
                    # Append the connection to the list
                    if found:
                        valid_pair = np.append(valid_pair, [[candA[i][3], candB[max_j][3], maxScore]], axis=0)

                # Append the detected connections to the global list
                valid_pairs.append(valid_pair)
            else: # If no keypoints are detected
                # print("No Connection : k = {}".format(k))
                invalid_pairs.append(k)
                valid_pairs.append([])
        return valid_pairs, invalid_pairs

    def get_personwise_keypoints(self, valid_pairs, invalid_pairs, keypoints_list):
        # the last number in each row is the overall score
        personwiseKeypoints = -1 * np.ones((0, 19))

        for k in range(len(MAP_IDX)):
            if k not in invalid_pairs:
                partAs = valid_pairs[k][:,0]
                partBs = valid_pairs[k][:,1]
                indexA, indexB = np.array(POSE_PAIRS[k])

                for i in range(len(valid_pairs[k])):
                    found = 0
                    person_idx = -1
                    for j in range(len(personwiseKeypoints)):
                        if personwiseKeypoints[j][indexA] == partAs[i]:
                            person_idx = j
                            found = 1
                            break

                    if found:
                        personwiseKeypoints[person_idx][indexB] = partBs[i]
                        personwiseKeypoints[person_idx][-1] += keypoints_list[partBs[i].astype(int), 2] + valid_pairs[k][i][2]

                    # if find no partA in the subset, create a new subset
                    elif not found and k < 17:
                        row = -1 * np.ones(19)
                        row[indexA] = partAs[i]
                        row[indexB] = partBs[i]
                        # add the keypoint_scores for the two keypoints and the paf_score
                        row[-1] = sum(keypoints_list[valid_pairs[k][i,:2].astype(int), 2]) + valid_pairs[k][i][2]
                        personwiseKeypoints = np.vstack([personwiseKeypoints, row])
        return personwiseKeypoints


    def personwise_handup(self, keypoints, personwise):

        bbox_list = []
        for person in personwise:
            # Why 17? These are the affinity map that actually construct the pose skeleton
            l_index = person[np.array(POSE_PAIRS[3])]
            r_index = person[np.array(POSE_PAIRS[5])]
            l_flag = False
            r_flag = False
            if not (-1 in l_index):
                B = np.int32(keypoints[l_index.astype(int), 0])
                A = np.int32(keypoints[l_index.astype(int), 1])
                if A[0] > A[1] + int(20 * self.ratio): l_flag = True
            if not (-1 in r_index):
                B = np.int32(keypoints[r_index.astype(int), 0])
                A = np.int32(keypoints[r_index.astype(int), 1])
                if A[0] > A[1] + int(20 * self.ratio): r_flag = True
            if l_flag or r_flag:
                x_max = 0
                x_min = self.width
                y_max = 0
                y_min = self.height
                for i in range(17):
                    index = person[np.array(POSE_PAIRS[i])]
                    if -1 in index: continue
                    B = np.int32(keypoints[index.astype(int), 0])
                    A = np.int32(keypoints[index.astype(int), 1])
                    if min(B[0], B[1]) < x_min: x_min = min(B[0], B[1])
                    if max(B[0], B[1]) > x_max: x_max = max(B[0], B[1])
                    if min(A[0], A[1]) < y_min: y_min = min(A[0], A[1])
                    if max(A[0], A[1]) > y_max: y_max = max(A[0], A[1])
                bbox_list.append([x_min, y_min, x_max, y_max])
        return bbox_list
    

    def personwise_human_bbox(self, keypoints, personwise):

        bbox_list = []
        for person in personwise:
            # l_index = person[np.array(POSE_PAIRS[3])]
            # r_index = person[np.array(POSE_PAIRS[5])]
            # l_flag = False
            # r_flag = False
            # if not (-1 in l_index):
            #     B = np.int32(keypoints[l_index.astype(int), 0])
            #     A = np.int32(keypoints[l_index.astype(int), 1])
            #     if A[0] > A[1] + int(20 * self.ratio): l_flag = True
            # if not (-1 in r_index):
            #     B = np.int32(keypoints[r_index.astype(int), 0])
            #     A = np.int32(keypoints[r_index.astype(int), 1])
            #     if A[0] > A[1] + int(20 * self.ratio): r_flag = True
            # if l_flag or r_flag:
            x_max = 0
            x_min = self.width
            y_max = 0
            y_min = self.height
            for i in range(17):
                index = person[np.array(POSE_PAIRS[i])]
                # if -1 in index: continue
                B = np.int32(keypoints[index.astype(int), 0])
                A = np.int32(keypoints[index.astype(int), 1])
                if min(B[0], B[1]) < x_min: x_min = min(B[0], B[1])
                if max(B[0], B[1]) > x_max: x_max = max(B[0], B[1])
                if min(A[0], A[1]) < y_min: y_min = min(A[0], A[1])
                if max(A[0], A[1]) > y_max: y_max = max(A[0], A[1])
            bbox_list.append([x_min, y_min, x_max, y_max])
        return bbox_list


    def personwise_human_bbox_with_hand(self, detected_keypoints, keypoints_list, personwise_keypoints):

        human_bbox_with_hand_list = []

        for person in personwise_keypoints:
            # bbox
            person_1 = person[person != -1][:-1]
            person_keypoints = np.int32(keypoints_list[person_1.astype(int)])
            x_max = max(person_keypoints[:, 0])
            x_min = min(person_keypoints[:, 0])
            y_max = max(person_keypoints[:, 1])
            y_min = min(person_keypoints[:, 1])

            # hand
            l_hand_x, l_hand_y, r_hand_x, r_hand_y = -1, -1, -1, -1
            if person[4] != -1 or person[7] != -1:
                for i in detected_keypoints[4]:
                    if(i[3]==person[4]):
                        # ret.data+=list(i[0:2])
                        l_hand_x, l_hand_y = i[0], i[1]
                        break
                for i in detected_keypoints[7]:
                    if(i[3]==person[7]):
                        # ret.data+=list(i[0:2])
                        r_hand_x, r_hand_y = i[0], i[1]
                        break

            human_bbox_with_hand_list.append([x_min, y_min, x_max, y_max, l_hand_x, l_hand_y, r_hand_x, r_hand_y])
        return human_bbox_with_hand_list
    # def person_grasp_drink(self, keypoints):


if __name__ == '__main__':
    import time
    rospy.init_node('run_snu_openpose', anonymous=False, disable_signals=True)
    BASE_DIR = './models/'
    # detect_option = ['handup', 'hands', 'knee']
    detect_option = [ 'knee', 'ankle']
    openpose = OpenPoseWrapper(BASE_DIR, detect_option, size_ratio=0.4, enable_viz=True)
    while not rospy.is_shutdown():
        # start_time = time.time()
        openpose.run()
        # getHandPoints(openpose)
        # print("time cost: ", (time.time() - start_time))