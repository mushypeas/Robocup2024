'''
Original Code Source:
https://github.com/spmallick/learnopencv/blob/master/OpenPose-Multi-Person/multi-person-openpose.py
'''
import rospy
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

from config import POSE_PAIRS, MAP_IDX, COLORS
#from humanpose_process import getHandPoints
class OpenPoseWrapper:

    def __init__(self, size_ratio=1.0, enable_viz=False) :

        # openpose
        BASE_DIR = '/home/tidy/Robocup2024/module/openpose/models/'
        proto  = BASE_DIR + 'pose_deploy_linevec.prototxt'
        weight = BASE_DIR + 'pose_iter_440000.caffemodel'
        self.openpose = cv2.dnn.readNetFromCaffe(proto, weight)
        #self.openpose.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        #self.openpose.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.nPoints = 18
        # visualization option
        self.enable_viz = enable_viz
        rgb_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        # ROS subscriber
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_mp_cb) 
        self.bridge = CvBridge()
        self.rgb_img = None
        # ROS publisher
        if enable_viz:
            self.openpose_pub = rospy.Publisher('/snu/openpose', Image, queue_size=10)
        self.bbox_pub = rospy.Publisher('/snu/openpose/bbox', Int16MultiArray, queue_size=10)
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

    def run(self, thresh=0.1): # 0.1

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

        # handup_bbox_list = self.personwise_handup(keypoints_list, personwise_keypoints)
        handup_bbox_list, handup_table_list = \
            self.personwise_handup(keypoints_list, personwise_keypoints)
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
            for bbox in handup_bbox_list:
                cv2.rectangle(img, bbox[0:2], bbox[2:4], (0, 255, 0), 3)
            img = cv2.resize(img,
                             (int(self.width / self.ratio), int(self.height / self.ratio)),
                             cv2.INTER_LINEAR)
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.openpose_pub.publish(msg)
        # Send bbox info of hand-waving people
        msg = Int16MultiArray()
        data = []
        '''
        for bbox in handup_bbox_list:
            bbox[0] = int(bbox[0] / self.ratio)
            bbox[1] = int(bbox[1] / self.ratio)
            bbox[2] = int(bbox[2] / self.ratio)
            bbox[3] = int(bbox[3] / self.ratio)
            rospy.loginfo(f'Found: TOP LEFT: {bbox[0:2]} BOTTOM RIGHT: {bbox[2:4]}')
            data += bbox
        '''
        for table, bbox in zip(handup_table_list, handup_bbox_list):
            table = np.where(table >= 0, (table / self.ratio).astype(np.int16), -1)
            bbox[0] = int(bbox[0] / self.ratio)
            bbox[1] = int(bbox[1] / self.ratio)
            bbox[2] = int(bbox[2] / self.ratio)
            bbox[3] = int(bbox[3] / self.ratio)
            rospy.loginfo(f'Found: TOP LEFT: {bbox[0:2]} BOTTOM RIGHT: {bbox[2:4]}')
            data += table.flatten().tolist()
        msg.data = data # N x 17 x 2
        self.bbox_pub.publish(msg)
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
        coord_table_list = []
        for person in personwise:
            # Why 17? These are the affinity map that actually construct the pose skeleton
            l_index = person[np.array(POSE_PAIRS[3])] #3,4
            l_subindex = person[np.array(POSE_PAIRS[2])] #2,3
            r_index = person[np.array(POSE_PAIRS[5])] #6,7
            r_subindex = person[np.array(POSE_PAIRS[4])] #5,6
            l_flag = False
            r_flag = False
            if not (-1 in l_index):
                B = np.int32(keypoints[l_index.astype(int), 0])
                A = np.int32(keypoints[l_index.astype(int), 1])
                AA = np.int32(keypoints[l_subindex.astype(int), 1])
                if AA[0] > A[1] : l_flag = True
            if not (-1 in r_index):
                B = np.int32(keypoints[r_index.astype(int), 0])
                A = np.int32(keypoints[r_index.astype(int), 1])
                AA = np.int32(keypoints[r_subindex.astype(int), 1])
                if AA[0] > A[1] : r_flag = True
            '''
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
            '''
            if l_flag or r_flag:
                coord_table = (-1 * np.ones((17, 2))).astype(np.int32)
                x_max = 0
                x_min = self.width
                y_max = 0
                y_min = self.height
                for i in range(17):
                    index = person[np.array(POSE_PAIRS[i])]
                    if -1 in index: continue
                    B = np.int32(keypoints[index.astype(int), 0])
                    A = np.int32(keypoints[index.astype(int), 1])
                    #coord_table[index.astype(int)[0]-1][0] = B[0]
                    #coord_table[index.astype(int)[0]-1][1] = A[0]
                    #coord_table[index.astype(int)[1]-1][0] = B[1]
                    #coord_table[index.astype(int)[1]-1][1] = A[1]
                    coord_table[POSE_PAIRS[i][0]-1][0] = B[0]
                    coord_table[POSE_PAIRS[i][0]-1][1] = A[0]
                    coord_table[POSE_PAIRS[i][1]-1][0] = B[1]
                    coord_table[POSE_PAIRS[i][1]-1][1] = A[1]
                    if min(B[0], B[1]) < x_min: x_min = min(B[0], B[1])
                    if max(B[0], B[1]) > x_max: x_max = max(B[0], B[1])
                    if min(A[0], A[1]) < y_min: y_min = min(A[0], A[1])
                    if max(A[0], A[1]) > y_max: y_max = max(A[0], A[1])
                bbox_list.append([x_min, y_min, x_max, y_max])
                coord_table_list.append(coord_table)
        return bbox_list, coord_table_list

    # def person_grasp_drink(self, keypoints):


if __name__ == '__main__':
    rospy.init_node('run_openpose', anonymous=False, disable_signals=True)
    openpose = OpenPoseWrapper(0.4, True)
    while not rospy.is_shutdown():
        # start_time = time.time()
        openpose.run()
        #getHandPoints(openpose)
        # print("time cost: ", (time.time() - start_time))








