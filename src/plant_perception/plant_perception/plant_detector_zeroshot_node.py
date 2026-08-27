#!/usr/bin/env python3
"""
Zero-shot plant/weed detector: Grounding DINO (open-vocabulary detection)
+ MobileSAM (promptable segmentation), running on the Jetson AGX Orin.

ALTERNATIVE to, not a replacement for, plant_detector_node.py (HSV) --
see the launch files' `detector` argument. HSV remains the default,
always-on, GPU-free continuous detector. This node adds
plant-versus-weed discrimination that HSV cannot do (see
docs/ZERO_SHOT_PERCEPTION_GUIDE.md), at the cost of being far slower per
inference -- which is why it is EVENT-GATED, not continuous.

EVENT-GATED, NOT CONTINUOUS -- this is not a design nicety, it is a hard
requirement. Grounding DINO + MobileSAM chained together take hundreds of
milliseconds per image; the 15 FPS camera stream has a 66.7 ms budget.
This node does NOT run on every frame. It sits idle until it receives a
message on `trigger_topic` (default /zeroshot_detect_trigger,
std_msgs/Empty), then runs exactly ONE detection pass against the most
recent synced RGB-D frame and publishes the result. mission_control is
expected to publish that trigger when the robot has arrived at and paused
near a plant -- the one moment the latency is affordable. There is no
mechanism here for interactive/manual re-prompting at runtime; the
detection prompts are fixed configuration (`plant_labels`,
`weed_labels`), set once, not re-entered by an operator.

MODEL LOADING -- uses HuggingFace `transformers`
(AutoModelForZeroShotObjectDetection / AutoProcessor,
IDEA-Research/grounding-dino-tiny) for Grounding DINO rather than the
IDEA-Research/GroundingDINO GitHub repository's own package. This is a
deliberate choice, not an arbitrary one: that repo's install compiles a
custom CUDA extension (its own README: "will be compiled under CPU-only
mode if no CUDA available", and even with CUDA available the compile is
slow and a documented source of Jetson-specific breakage). The
`transformers` integration is pure PyTorch -- no custom extension, no
compile step. MobileSAM is loaded via its own official package
(ChaoningZhang/MobileSAM), which has never needed a compiled extension
either way.

TWO PROMPT SETS, ONE OUTPUT TOPIC WITH A CRITICAL FILTER -- both
`plant_labels` and `weed_labels` are queried every trigger (this is how
plant-versus-weed discrimination actually happens). Only PLANT detections
are published to /plant_targets -- the mission pipeline
(semantic_mapper, target_manager) has always assumed everything on that
topic is something to visit, and geometry_msgs/PoseArray has no field to
attach a class label to an individual Pose even if it didn't. Weed
detections are published to a separate, diagnostic-only topic
(/detected_weeds) for visualization -- nothing downstream consumes it.

GEOMETRIC PLAUSIBILITY GATE, NOT the full PCL RANSAC filter -- an
earlier revision of this workspace had a full plant_geometry_filter C++
package (PCL plane removal + statistical outlier removal) for exactly
this purpose, removed along with YOLO. It was never compiled or verified
in this project's authoring environment. Rather than reintroduce an
unverified C++/PCL dependency the night before hardware bring-up, this
node does a lightweight, dependency-free plausibility check in pure numpy
(zeroshot_postprocess.cluster_centroid_and_plausibility) -- open-set
models are more prone to false positives than a closed-set detector, so
some geometric sanity-checking matters here more than it did for HSV, but
a simpler gate is the proportionate choice for what is genuinely a
same-day-before-deployment addition.

DEPTH/COLOUR ALIGNMENT ASSUMPTION -- back-projection here
(zeroshot_postprocess.backproject_mask_pixels) maps colour-pixel
coordinates into depth-image space by width/height RATIO alone, which is
only geometrically correct when `depth_topic` genuinely points at a
depth stream aligned to the colour camera's optical axis (see
plant_detector_node.py's ACCURACY NOTE for the same assumption on the
HSV path, and why alignment has to happen upstream in the camera driver
-- this node cannot correct for it after the fact).

NOT VERIFIED BY EXECUTION -- this sandbox has no CUDA, no aarch64, and no
downloaded model weights. Written against the documented, versioned
`transformers` and MobileSAM APIs, and everything that COULD be verified
without a GPU (prompt formatting, label matching, back-projection maths,
plausibility gating) has real tests -- see test_zeroshot_postprocess.py.
The model-loading and inference code itself needs the verification
sequence in docs/ZERO_SHOT_PERCEPTION_GUIDE.md run on the real Orin
before a live mission depends on it.
"""
import copy
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge
import message_filters

from plant_perception.zeroshot_postprocess import (
    format_grounding_dino_prompt, match_label_to_phrase,
    backproject_mask_pixels, cluster_centroid_and_plausibility,
)


class PlantDetectorZeroshotNode(Node):
    def __init__(self):
        super().__init__('plant_detector_zeroshot_node')
        d = self.declare_parameter
        d('color_topic', '/camera/color/image_raw_decompressed')
        d('depth_topic', '/camera/depth/image_raw_decompressed')
        d('info_topic', '/camera/color/camera_info')
        d('depth_optical_frame', 'camera_depth_optical_frame')
        d('trigger_topic', '/zeroshot_detect_trigger')

        d('grounding_dino_model_id', 'IDEA-Research/grounding-dino-tiny')
        d('mobile_sam_checkpoint', '')     # REQUIRED, see the guide
        d('plant_labels', ['ripe strawberry', 'strawberry plant canopy'])
        d('weed_labels', ['weed'])
        d('box_threshold', 0.35)
        d('text_threshold', 0.25)

        d('depth_min_m', 0.15)
        d('depth_max_m', 3.0)
        d('mask_stride', 2)
        d('min_cluster_points', 15)
        d('max_plant_extent_m', 1.0)
        d('min_plant_extent_m', 0.01)
        d('publish_debug', True)
        d('force_cuda', True)   # fail loudly if CUDA isn't active -- see
                                # _init_models, matching this project's
                                # existing fail-loud philosophy

        g = lambda n: self.get_parameter(n).value
        self.frame = g('depth_optical_frame')
        self.plant_labels = list(g('plant_labels'))
        self.weed_labels = list(g('weed_labels'))
        self.box_thresh = g('box_threshold')
        self.text_thresh = g('text_threshold')
        self.dmin, self.dmax = g('depth_min_m'), g('depth_max_m')
        self.mask_stride = int(g('mask_stride'))
        self.min_cluster_points = int(g('min_cluster_points'))
        self.max_extent = g('max_plant_extent_m')
        self.min_extent = g('min_plant_extent_m')
        self.publish_debug = g('publish_debug')

        mobile_sam_ckpt = os.path.expanduser(g('mobile_sam_checkpoint')) \
            if g('mobile_sam_checkpoint') else ''
        if not mobile_sam_ckpt:
            raise RuntimeError(
                "mobile_sam_checkpoint parameter is required and has no "
                "default -- point it at your downloaded mobile_sam.pt "
                "(see docs/ZERO_SHOT_PERCEPTION_GUIDE.md).")

        self.all_prompt = format_grounding_dino_prompt(
            self.plant_labels + self.weed_labels)
        self.get_logger().info(f'Grounding DINO prompt: "{self.all_prompt}"')

        (self.dino_model, self.dino_processor, self.sam_predictor,
         self.device) = self._init_models(
            g('grounding_dino_model_id'), mobile_sam_ckpt, g('force_cuda'))

        self.bridge = CvBridge()
        self.K = None
        self._latest_rgb = None
        self._latest_depth = None
        self._latest_header = None

        self.pose_pub = self.create_publisher(PoseArray, '/plant_targets', 10)
        self.weed_pub = self.create_publisher(PoseArray, '/detected_weeds', 10)
        self.status_pub = self.create_publisher(
            String, '/plant_detector_zeroshot/status', 10)
        self.debug_pub = self.create_publisher(
            Image, '/plant_detector_zeroshot/debug_image', 5)
        self.create_subscription(CameraInfo, g('info_topic'), self.info_cb, 10)

        rgb = message_filters.Subscriber(self, Image, g('color_topic'),
                                         qos_profile=qos_profile_sensor_data)
        dep = message_filters.Subscriber(self, Image, g('depth_topic'),
                                         qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, dep], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.rgbd_cache_cb)

        self.create_subscription(Empty, g('trigger_topic'), self.trigger_cb, 10)

        self.get_logger().info(
            'plant_detector_zeroshot_node ready (Grounding DINO + MobileSAM, '
            'event-gated -- publish Empty to '
            f"{g('trigger_topic')} to run one detection pass).")

    def _init_models(self, dino_model_id, mobile_sam_ckpt, force_cuda):
        """
        Explicit CUDA enforcement, not a silent CPU fallback -- the same
        reasoning as every other fail-loud provider check in this
        workspace (base_controller's serial connection, the removed
        YOLO node's ONNX Runtime provider check): a silent CPU fallback
        here wouldn't crash, it would turn a "few hundred ms" inference
        into several seconds, which then blocks the mission FSM far
        longer than intended and looks like "the robot froze" rather
        than a clear, immediate error.
        """
        import torch
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        from mobile_sam import sam_model_registry, SamPredictor

        if force_cuda and not torch.cuda.is_available():
            raise RuntimeError(
                "force_cuda is true but torch.cuda.is_available() is False. "
                "This almost always means the plain PyPI 'torch' wheel is "
                "installed instead of NVIDIA's Jetson build -- see "
                "docs/JETPACK_SETUP_GUIDE.md and "
                "docs/ZERO_SHOT_PERCEPTION_GUIDE.md. Set force_cuda:=false "
                "only to run on CPU deliberately, e.g. to isolate a model "
                "problem from a runtime problem (expect this to be far too "
                "slow for any real trigger-to-result latency).")
        device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.get_logger().info(f'Loading Grounding DINO ({dino_model_id})...')
        dino_processor = AutoProcessor.from_pretrained(dino_model_id)
        dino_model = AutoModelForZeroShotObjectDetection.from_pretrained(
            dino_model_id).to(device).eval()

        self.get_logger().info(f'Loading MobileSAM ({mobile_sam_ckpt})...')
        if not os.path.exists(mobile_sam_ckpt):
            raise RuntimeError(
                f"mobile_sam_checkpoint '{mobile_sam_ckpt}' does not exist "
                f"on disk -- check the path (see "
                f"docs/ZERO_SHOT_PERCEPTION_GUIDE.md Part 4.1).")
        mobile_sam = sam_model_registry['vit_t'](checkpoint=mobile_sam_ckpt)
        mobile_sam.to(device=device).eval()
        sam_predictor = SamPredictor(mobile_sam)

        self.get_logger().info(f'Zero-shot models loaded on device: {device}')
        return dino_model, dino_processor, sam_predictor, device

    def info_cb(self, msg):
        self.K = msg.k

    def rgbd_cache_cb(self, rgb_msg, dep_msg):
        """Continuously caches the most recent synced frame -- cheap
        (just references, no inference) -- so a trigger can act on a
        genuinely recent frame without waiting for the next sync."""
        self._latest_rgb = rgb_msg
        self._latest_depth = dep_msg
        self._latest_header = dep_msg.header

    def trigger_cb(self, _msg):
        if self._latest_rgb is None or self.K is None:
            self.get_logger().warn(
                'Zero-shot trigger received but no synced RGB-D frame or '
                'camera intrinsics available yet -- skipping this trigger.')
            return
        t0 = time.monotonic()
        try:
            self._run_detection(self._latest_rgb, self._latest_depth)
        except Exception as e:  # noqa: BLE001 -- one bad frame must not kill the node
            self.get_logger().error(f'Zero-shot detection pass failed: {e}')
            return
        elapsed_ms = (time.monotonic() - t0) * 1000.0
        self.get_logger().info(f'Zero-shot detection pass took {elapsed_ms:.0f} ms')

    def _run_detection(self, rgb_msg, dep_msg):
        import torch

        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        img_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        H, W = img_bgr.shape[:2]
        dH, dW = depth.shape[:2]
        is_mm = depth.dtype != np.float32
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        inputs = self.dino_processor(
            images=img_rgb, text=self.all_prompt, return_tensors='pt').to(self.device)
        with torch.no_grad():
            outputs = self.dino_model(**inputs)
        results = self.dino_processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids,
            box_threshold=self.box_thresh, text_threshold=self.text_thresh,
            target_sizes=[(H, W)])[0]

        boxes = results['boxes'].cpu().numpy()      # (N,4) xyxy, image pixel space
        phrases = results['labels']                  # list[str], length N
        scores = results['scores'].cpu().numpy()

        if boxes.shape[0] == 0:
            self.status_pub.publish(String(data='detections:0'))
            return

        self.sam_predictor.set_image(img_rgb)

        plant_points = []
        weed_points = []
        dbg = img_bgr.copy() if self.publish_debug else None

        for box, phrase, score in zip(boxes, phrases, scores):
            label = match_label_to_phrase(phrase, self.plant_labels + self.weed_labels)
            if label is None:
                continue   # open-set noise that matched neither prompt set closely
            is_weed = label in self.weed_labels

            masks, _, _ = self.sam_predictor.predict(
                box=box, multimask_output=False)
            mask = masks[0].astype(bool)   # (H, W), full colour-image resolution

            ys, xs = np.nonzero(mask)
            if self.mask_stride > 1:
                xs, ys = xs[::self.mask_stride], ys[::self.mask_stride]
            if xs.size == 0:
                continue

            points = backproject_mask_pixels(
                xs, ys, depth, color_wh=(W, H), depth_wh=(dW, dH),
                intrinsics=(fx, fy, cx, cy), depth_is_mm=is_mm,
                depth_min_m=self.dmin, depth_max_m=self.dmax)

            centroid, plausible = cluster_centroid_and_plausibility(
                points, min_points=self.min_cluster_points,
                max_extent_m=self.max_extent, min_extent_m=self.min_extent)
            if centroid is None or not plausible:
                continue

            (weed_points if is_weed else plant_points).append(centroid)

            if dbg is not None:
                x1, y1, x2, y2 = box.astype(int)
                colour = (0, 165, 255) if is_weed else (0, 255, 0)
                cv2.rectangle(dbg, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(dbg, f'{label} {score:.2f}', (x1, max(0, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)

        self._publish_poses(self.pose_pub, plant_points, dep_msg.header)
        self._publish_poses(self.weed_pub, weed_points, dep_msg.header)

        self.status_pub.publish(String(
            data=f'detections:{boxes.shape[0]} plants:{len(plant_points)} '
                f'weeds:{len(weed_points)}'))
        if dbg is not None:
            out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
            out.header = rgb_msg.header
            self.debug_pub.publish(out)

    def _publish_poses(self, publisher, centroids, header_src):
        if not centroids:
            return
        poses = PoseArray()
        poses.header = copy.deepcopy(header_src)
        poses.header.frame_id = self.frame
        for c in centroids:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), float(c[2])
            p.orientation.w = 1.0
            poses.poses.append(p)
        publisher.publish(poses)


def main():
    rclpy.init()
    node = PlantDetectorZeroshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
