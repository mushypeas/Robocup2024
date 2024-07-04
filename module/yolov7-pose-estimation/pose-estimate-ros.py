import cv2
import time
import torch
import argparse
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from torchvision import transforms
from utils.datasets import letterbox
from utils.torch_utils import select_device
from models.experimental import attempt_load
from utils.general import non_max_suppression_kpt, strip_optimizer
from utils.plots import output_to_keypoint, plot_one_box_kpt, colors

class PoseEstimator:
    def __init__(self, opt):
        self.opt = opt
        self.device = select_device(opt.device)
        self.model = attempt_load(opt.poseweights, map_location=self.device)
        self.model.eval()
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        
        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher('human_keypoints', Float32MultiArray, queue_size=10)
        self.image_sub = rospy.Subscriber(opt.img_topic, Image, self.image_callback)

        self.frame_count = 0
        self.total_fps = 0

        rospy.loginfo("PoseEstimator initialized")

    def image_callback(self, msg):
        rospy.loginfo("Received an image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def process_image(self, frame):
        rospy.loginfo(f"Processing frame {self.frame_count + 1}")

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = letterbox(image, (self.opt.img_size), stride=64, auto=True)[0]
        image = transforms.ToTensor()(image)
        image = torch.tensor(np.array([image.numpy()]))

        image = image.to(self.device)
        image = image.float()
        start_time = time.time()

        with torch.no_grad():
            output_data, _ = self.model(image)

        output_data = non_max_suppression_kpt(output_data,
                                              0.25,
                                              0.65,
                                              nc=self.model.yaml['nc'],
                                              nkpt=self.model.yaml['nkpt'],
                                              kpt_label=True)

        output = output_to_keypoint(output_data)

        im0 = image[0].permute(1, 2, 0) * 255
        im0 = im0.cpu().numpy().astype(np.uint8)
        im0 = cv2.cvtColor(im0, cv2.COLOR_RGB2BGR)

        pose_data = []

        for i, pose in enumerate(output_data):
            if len(output_data):
                for det_index, (*xyxy, conf, cls) in enumerate(reversed(pose[:,:6])):
                    kpts = pose[det_index, 6:]
                    person_data = kpts.tolist()  # numpy array를 list로 변환
                    pose_data.append(person_data)

        # Float32MultiArray 메시지 생성
        msg = Float32MultiArray()
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        msg.layout.dim[0].label = "person"
        msg.layout.dim[0].size = len(pose_data)
        msg.layout.dim[0].stride = len(pose_data) * len(pose_data[0]) if pose_data else 0
        msg.layout.dim[1].label = "keypoint"
        msg.layout.dim[1].size = len(pose_data[0]) if pose_data else 0
        msg.layout.dim[1].stride = len(pose_data[0]) if pose_data else 0
        msg.data = [item for sublist in pose_data for item in sublist]

        self.pose_pub.publish(msg)
        rospy.loginfo(f"Published pose data for {len(pose_data)} person(s)")

        end_time = time.time()
        fps = 1 / (end_time - start_time)
        self.total_fps += fps
        self.frame_count += 1

        if self.opt.view_img:
            cv2.putText(im0, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("YOLOv7 Pose Estimation Demo", im0)
            cv2.waitKey(1)

        rospy.loginfo(f"Finished processing frame {self.frame_count}")

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--poseweights', nargs='+', type=str, default='yolov7-w6-pose.pt', help='model path(s)')
    parser.add_argument('--img_topic', type=str, default='/hsrb/head_rgbd_sensor/rgb/image_rect_color', help='ROS image topic')
    parser.add_argument('--img_size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--device', type=str, default='cpu', help='cpu/0,1,2,3(gpu)')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    opt = parser.parse_args()
    return opt

def main(opt):
    rospy.init_node('yolov7_pose_estimation', anonymous=True)
    rospy.loginfo("Initializing YOLOv7 Pose Estimation node")
    pose_estimator = PoseEstimator(opt)
    rospy.spin()

if __name__ == "__main__":
    opt = parse_opt()
    strip_optimizer(opt.device, opt.poseweights)
    main(opt)