#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from torchvision import transforms
from utils.datasets import letterbox
from utils.torch_utils import select_device
from models.experimental import attempt_load
from utils.general import non_max_suppression_kpt, xyxy2xywh
from utils.plots import output_to_keypoint, plot_skeleton_kpts, colors, plot_one_box_kpt

class PoseEstimator:
    def __init__(self):
        rospy.init_node('yolov7_pose_estimation', anonymous=True)
        
        # Load model
        self.device = select_device('')
        self.model = attempt_load('yolov7-w6-pose.pt', map_location=self.device)
        self.model.eval()

        # Initialize ROS publishers and subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('estimated_poses', PoseArray, queue_size=10)
        self.image_pub = rospy.Publisher('annotated_image', Image, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Process image
        try:
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image = letterbox(image, (640, 640), stride=64, auto=True)[0]
            image = transforms.ToTensor()(image)
            image = torch.tensor(np.array([image.numpy()]))

            image = image.to(self.device)
            image = image.float()

            with torch.no_grad():
                output_data, _ = self.model(image)

            output_data = non_max_suppression_kpt(output_data,
                                                  0.25,   # Conf. Threshold.
                                                  0.65,   # IoU Threshold.
                                                  nc=self.model.yaml['nc'],
                                                  nkpt=self.model.yaml['nkpt'],
                                                  kpt_label=True)

            output = output_to_keypoint(output_data)

            # Prepare PoseArray message
            pose_array_msg = PoseArray()
            pose_array_msg.header = msg.header

            # Draw keypoints and create PoseArray
            im0 = image[0].permute(1, 2, 0) * 255
            im0 = im0.cpu().numpy().astype(np.uint8)
            im0 = cv2.cvtColor(im0, cv2.COLOR_RGB2BGR)

            for i, pose in enumerate(output_data):
                if len(output_data):
                    for det_index, (*xyxy, conf, cls) in enumerate(reversed(pose[:,:6])):
                        kpts = pose[det_index, 6:]
                        plot_skeleton_kpts(im0, kpts, 3)
                        
                        # Add keypoints to PoseArray
                        for j in range(0, len(kpts), 3):
                            pose_msg = Pose()
                            pose_msg.position.x = float(kpts[j])
                            pose_msg.position.y = float(kpts[j+1])
                            pose_msg.position.z = float(kpts[j+2])  # confidence score
                            pose_array_msg.poses.append(pose_msg)

            # Publish results
            self.pose_pub.publish(pose_array_msg)
            
            # Publish annotated image
            annotated_img_msg = self.bridge.cv2_to_imgmsg(im0, "bgr8")
            annotated_img_msg.header = msg.header
            self.image_pub.publish(annotated_img_msg)

        except Exception as e:
            rospy.logerr("Error in image processing: %s", e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pose_estimator = PoseEstimator()
        pose_estimator.run()
    except rospy.ROSInterruptException:
        pass