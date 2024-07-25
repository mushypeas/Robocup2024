import sys
sys.path.append('module/ByteTrack')
import argparse
import os
import os.path as osp
import time
import cv2
import torch

from loguru import logger

from yolox.data.data_augment import preproc
from yolox.exp import get_exp
from yolox.utils import fuse_model, get_model_info, postprocess
from yolox.utils.visualize import plot_tracking
from yolox.tracker.byte_tracker import BYTETracker
from yolox.tracking_utils.timer import Timer
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, String

# feature extraction
from get_feature import HumanId

IMAGE_EXT = [".jpg", ".jpeg", ".webp", ".bmp", ".png"]



def make_parser():
    parser = argparse.ArgumentParser("ByteTrack Demo!")
    # parser.add_argument(
    #     "demo", default="video", help="demo type, eg. image, video and webcam"
    # )
    parser.add_argument("-expn", "--experiment-name", type=str, default=None)
    parser.add_argument("-n", "--name", type=str, default=None, help="model name")

    parser.add_argument(
        #"--path", default="./datasets/mot/train/MOT17-05-FRCNN/img1", help="path to images or video"
        "--path", default="./videos/palace.mp4", help="path to images or video"
    )
    parser.add_argument("--camid", type=int, default=0, help="webcam demo camera id")
    parser.add_argument(
        "--save_result",
        action="store_true",
        help="whether to save the inference result of image/video",
    )

    # exp file
    parser.add_argument(
        "-f",
        "--exp_file",
        default='module/ByteTrack/exps/example/mot/yolox_s_mix_det.py',
        type=str,
        help="pls input your expriment description file",
    )
    parser.add_argument("-c", "--ckpt", default='module/ByteTrack/pretrained/bytetrack_s_mot17.pth.tar', \
                        type=str, help="ckpt for eval")
    parser.add_argument(
        "--device",
        default="gpu",
        type=str,
        help="device to run our model, can either be cpu or gpu",
    )
    parser.add_argument("--conf", default=None, type=float, help="test conf")
    parser.add_argument("--nms", default=None, type=float, help="test nms threshold")
    parser.add_argument("--tsize", default=None, type=int, help="test img size")
    parser.add_argument("--fps", default=30, type=int, help="frame rate (fps)")
    parser.add_argument(
        "--fp16",
        dest="fp16",
        default=True,
        action="store_true",
        help="Adopting mix precision evaluating.",
    )
    parser.add_argument(
        "--fuse",
        dest="fuse",
        default=True,
        action="store_true",
        help="Fuse conv and bn for testing.",
    )
    parser.add_argument(
        "--trt",
        dest="trt",
        default=False,
        action="store_true",
        help="Using TensorRT model for testing.",
    )
    # tracking args
    parser.add_argument("--track_thresh", type=float, default=0.5, help="tracking confidence threshold")
    parser.add_argument("--track_buffer", type=int, default=30, help="the frames for keep lost tracks")
    parser.add_argument("--match_thresh", type=float, default=0.8, help="matching threshold for tracking")
    parser.add_argument(
        "--aspect_ratio_thresh", type=float, default=1.6,
        help="threshold for filtering out boxes of which aspect ratio are above the given value."
    )
    parser.add_argument('--min_box_area', type=float, default=100, help='filter out tiny boxes')
    parser.add_argument("--mot20", dest="mot20", default=False, action="store_true", help="test mot20.")
    return parser


class Predictor(object):
    def __init__(
        self,
        model,
        exp,
        trt_file=None,
        decoder=None,
        device=torch.device("cpu"),
        fp16=False
    ):
        self.model = model
        self.decoder = decoder
        self.num_classes = exp.num_classes
        self.confthre = exp.test_conf
        self.nmsthre = exp.nmsthre
        self.test_size = exp.test_size
        self.device = device
        self.fp16 = fp16
        if trt_file is not None:
            from torch2trt import TRTModule

            model_trt = TRTModule()
            model_trt.load_state_dict(torch.load(trt_file))

            x = torch.ones((1, 3, exp.test_size[0], exp.test_size[1]), device=device)
            self.model(x)
            self.model = model_trt
        self.rgb_means = (0.485, 0.456, 0.406)
        self.std = (0.229, 0.224, 0.225)

    def inference(self, img, timer):
        img_info = {"id": 0}
        if isinstance(img, str):
            img_info["file_name"] = osp.basename(img)
            img = cv2.imread(img)
        else:
            img_info["file_name"] = None

        height, width = img.shape[:2]
        img_info["height"] = height
        img_info["width"] = width
        img_info["raw_img"] = img

        img, ratio = preproc(img, self.test_size, self.rgb_means, self.std)
        img_info["ratio"] = ratio
        img = torch.from_numpy(img).unsqueeze(0).float().to(self.device)
        if self.fp16:
            img = img.half()  # to FP16

        with torch.no_grad():
            timer.tic()
            outputs = self.model(img)
            if self.decoder is not None:
                outputs = self.decoder(outputs, dtype=outputs.type())
            outputs = postprocess(
                outputs, self.num_classes, self.confthre, self.nmsthre
            )
            #logger.info("Infer time: {:.4f}s".format(time.time() - t0))
        return outputs, img_info

def image_ros_demo(ros_img, predictor, exp, args, frame_id, tracker, human_id, human_feature, feature_on):
    print('human_id: ', human_id)
    # tracker = BYTETracker(args, frame_rate=args.fps)
    timer = Timer()
    if frame_id < 1000 : #TODO : byte 켠 초기에는 눈 앞의 human만 잡도록. frame 기준 확인 필요
        height, width, _ = ros_img.shape
        bar_width = width // 4 ## TODO : 지금은 왼쪽 25%, 오른쪽 25% 제거. 확인 필요
        ros_img[:, :bar_width] = 0
        ros_img[:, -bar_width:] = 0
        ros_img[:120, :] = 0

    outputs, img_info = predictor.inference(ros_img, timer)
    found_prev_human = False
    if outputs[0] is not None:
        online_targets = tracker.update(outputs[0], [img_info['height'], img_info['width']], exp.test_size)
        online_tlwhs = []
        online_ids = []
        online_scores = []
        target_tlwh = None
        target_score = None
        for t in online_targets:
            tlwh = t.tlwh
            tid = t.track_id
            print('Target index: ', tid)
            vertical = tlwh[2] / tlwh[3] > args.aspect_ratio_thresh
            if tlwh[2] * tlwh[3] > args.min_box_area and not vertical:
                if tid == human_id:
                    found_prev_human = True
                    print('Found prev human')
                    print('Target index: ', tid)
                    target_tlwh = tlwh
                    target_score = t.score
                    target_feature = human_feature
                online_tlwhs.append(tlwh)
                online_ids.append(tid)
                online_scores.append(t.score)


                if not found_prev_human:
                    print("not found prev human")
                    # if len(online_targets) == 0:
                    human_id = tid
                    target_feature = None
                    target_score = None
                    target_tlwh = None


        if human_id == -1: ##initialize
            random_target = online_targets[0]
            x, y, w, h = int(random_target.tlwh[0]), int(random_target.tlwh[1]), int(random_target.tlwh[2]), int(random_target.tlwh[3])
            human_reid = HumanId()
            target_feature = human_reid.get_vector(ros_img[y:y + h, x:x + w])
            print('Initial Target index: ', random_target.track_id)
            del human_reid

            human_id = random_target.track_id
            target_score = random_target.score
            target_tlwh = random_target.tlwh






            # else:
                # # if human_id == 1 : start bytetrack
                # if human_id == -1:
                #     random_target = online_targets[0]
                #     x, y, w, h = int(random_target.tlwh[0]), int(random_target.tlwh[1]), int(random_target.tlwh[2]), int(random_target.tlwh[3])
                #     human_reid = HumanId()
                #     target_feature = human_reid.get_vector(ros_img[y:y + h, x:x + w])
                #     print('Initial Target index: ', random_target.track_id)
                #     del human_reid
                # else:
                #     # feature extraction
                #     if feature_on:
                #         human_reid = HumanId()
                #         idx = human_reid.get_multi_vector(ros_img, online_targets, human_feature)
                #         target_feature = human_feature
                #         random_target = online_targets[idx]
                #         print('Target index Change: ', random_target.track_id)
                #         del human_reid
                #     # index 0 human
                #     else:
                #         random_target = online_targets[0]
                #         target_feature = human_feature

                # human_id = random_target.track_id
                # target_score = random_target.score
                # target_tlwh = random_target.tlwh

        timer.toc()
        online_im = plot_tracking(
            img_info['raw_img'], online_tlwhs, online_ids, frame_id=frame_id, fps=1. / timer.average_time
        )
        return [human_id, target_tlwh, target_score, target_feature], online_im, online_tlwhs
    else:
        timer.toc()
        online_im = img_info['raw_img']
        return [None, None, None, None], online_im, []


class Bytetrack_ros:
    def __init__(self, rgb_topic):
        rospy.Subscriber(rgb_topic, Image, self._rgb_cb)
        self.bridge = CvBridge()
        self.args = make_parser().parse_args()
        self.exp = get_exp(self.args.exp_file, self.args.name)
        self.frame_id = 1
        rospy.Subscriber('/snu/demotrack', String, self._bytetrack_cb)
        # publisher node for /snu/yolo
        self.yolo_pub = rospy.Publisher('/snu/carry_my_luggage_yolo', Int16MultiArray, queue_size=10)
        # self.human_bbox_pub = rospy.Publisher('/snu/human_bbox_yolo', Int16MultiArray, queue_size=10)
        self.byte_img_pub = rospy.Publisher('/snu/bytetrack_img', Image, queue_size=10)
        self.target_img_pub = rospy.Publisher('/snu/bytetrack_target', Image, queue_size=10)
        self.cur_human_idx = -1
        self.target_feature = None
        self.human_info_ary = [None, None, None]
        self.feature_on = True

        if not self.args.experiment_name:
            self.args.experiment_name = self.exp.exp_name

        output_dir = osp.join(self.exp.output_dir, self.args.experiment_name)
        os.makedirs(output_dir, exist_ok=True)

        if self.args.save_result:
            self.vis_folder = osp.join(output_dir, "track_vis")
            os.makedirs(self.vis_folder, exist_ok=True)

        if self.args.trt:
            self.args.device = "gpu"
        self.args.device = torch.device("cuda" if self.args.device == "gpu" else "cpu")

        logger.info("Args: {}".format(self.args))

        if self.args.conf is not None:
            self.exp.test_conf = self.args.conf
        if self.args.nms is not None:
            self.exp.nmsthre = self.args.nms
        if self.args.tsize is not None:
            self.exp.test_size = (self.args.tsize, self.args.tsize)

        model = self.exp.get_model().to(self.args.device)
        logger.info("Model Summary: {}".format(get_model_info(model, self.exp.test_size)))
        model.eval()

        if not self.args.trt:
            if self.args.ckpt is None:
                ckpt_file = osp.join(output_dir, "best_ckpt.pth.tar")
            else:
                ckpt_file = self.args.ckpt
            logger.info("loading checkpoint")
            ckpt = torch.load(ckpt_file, map_location="cpu")
            # load the model state dict
            model.load_state_dict(ckpt["model"])
            logger.info("loaded checkpoint done.")

        if self.args.fuse:
            logger.info("\tFusing model...")
            model = fuse_model(model)

        if self.args.fp16:
            model = model.half()  # to FP16

        if self.args.trt:
            assert not self.args.fuse, "TensorRT model is not support model fusing!"
            trt_file = osp.join(output_dir, "model_trt.pth")
            assert osp.exists(
                trt_file
            ), "TensorRT model is not found!\n Run python3 tools/trt.py first!"
            model.head.decode_in_inference = False
            decoder = model.head.decode_outputs
            logger.info("Using TensorRT to inference")
        else:
            trt_file = None
            decoder = None

        self.predictor = Predictor(model, self.exp, trt_file, decoder, self.args.device, self.args.fp16)
        self.current_time = time.localtime()
        self.tracker = BYTETracker(self.args, frame_rate=self.args.fps)
        # self.predictor, self.vis_folder, self.current_time = main(self.exp, self.args)
    def _bytetrack_cb(self, data):
        message = data.data
        if message == 'target':
            # capture target
            self.cur_human_idx = -1
        if message == 'off_feature':
            print('Stop Feature extration')
            # stop feature extraction
            self.feature_on = False

    def _rgb_cb(self, data):
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        try:
            human_info_ary, online_im, online_tlwhs = image_ros_demo(self.rgb_img, self.predictor, self.exp, self.args, self.frame_id, self.tracker, self.cur_human_idx, self.target_feature, self.feature_on)
            if human_info_ary[0] is not None:
                self.cur_human_idx = human_info_ary[0]
                print("self.cur_human_idx: ", self.cur_human_idx)

                self.target_feature = human_info_ary[3]
                x, y, w, h = int(human_info_ary[1][0]), int(human_info_ary[1][1]), int(human_info_ary[1][2]), int(human_info_ary[1][3])
                intary = Int16MultiArray()
                intary.data = [int(human_info_ary[0]), x, y, w, h, int(human_info_ary[2])]
                self.yolo_pub.publish(intary)

                if x < 0: x = 0
                if y < 0: y = 0
                bbox_msg = self.bridge.cv2_to_imgmsg(self.rgb_img[y:y + h, x:x + w], encoding='bgr8')
                self.target_img_pub.publish(bbox_msg)
            else:
                intary = Int16MultiArray()
                intary.data = [-1]
                self.yolo_pub.publish(intary)
            self.frame_id += 1

            head_msg = self.bridge.cv2_to_imgmsg(online_im, encoding='bgr8')
            self.byte_img_pub.publish(head_msg)
        except AttributeError as e:
            rospy.loginfo("init bytetrack...")
        # # for stickler for the rules
        # self.human_bbox_pub.publish(online_tlwhs)

        # cv2.imshow('dd', online_im)
        # cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node('dd')
    rgb_topic='/hsrb/head_rgbd_sensor/rgb/image_raw'
    Bytetrack_ros(rgb_topic)
    rospy.spin()
