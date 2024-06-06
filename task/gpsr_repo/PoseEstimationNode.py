import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import mediapipe as mp

class PoseEstimationNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose_counts = {"Standing": 0, "Sitting": 0, "Lying": 0}
        self.detected_people = []

    def classify_pose(self, landmarks):
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value].y
        right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP.value].y
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE.value].y
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE.value].y

        avg_hip = (left_hip + right_hip) / 2
        avg_shoulder = (left_shoulder + right_shoulder) / 2
        avg_knee = (left_knee + right_knee) / 2

        if avg_shoulder < avg_hip and avg_hip < avg_knee:
            return "Standing"
        elif avg_shoulder > avg_hip > avg_knee:
            return "Sitting"
        elif avg_shoulder > avg_hip and avg_shoulder > avg_knee:
            return "Lying"
        else:
            return "Unknown"

    def is_new_person(self, bbox, threshold=50):
        for person in self.detected_people:
            dist = np.linalg.norm(np.array(bbox) - np.array(person))
            if dist < threshold:
                return False
        return True

    def image_callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # BGR 이미지를 RGB로 변환
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # 포즈 추정
        results = self.pose.process(image)

        # RGB 이미지를 다시 BGR로 변환
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            for pose_landmarks in results.pose_landmarks:
                self.mp_drawing.draw_landmarks(image, pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
                pose_classification = self.classify_pose(pose_landmarks.landmark)

                # Bounding box 계산
                h, w, _ = image.shape
                xs = [landmark.x * w for landmark in pose_landmarks.landmark]
                ys = [landmark.y * h for landmark in pose_landmarks.landmark]
                bbox = [int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))]

                # 새로운 사람인지 확인
                if self.is_new_person(bbox):
                    self.detected_people.append(bbox)
                    self.pose_counts[pose_classification] += 1

            # 포즈 누적 정보를 기반으로 최종 포즈 분류 및 출력
            for idx, (pose, count) in enumerate(self.pose_counts.items()):
                cv2.putText(image, f'{pose}: {count}', (10, 30 + idx * 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # 결과 이미지 보여주기
        cv2.imshow('MediaPipe Pose', image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('pose_estimation_node', anonymous=True)
    pose_estimation_node = PoseEstimationNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting down pose estimation node.")

