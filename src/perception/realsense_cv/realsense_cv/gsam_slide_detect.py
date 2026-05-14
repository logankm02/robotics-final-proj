
import sys
# print("Current sys.path:", sys.path)
# sys.path.append('')

import argparse
import os
import gc
import cv2
import json
import torch
import numpy as np
import supervision as sv
import pycocotools.mask as mask_util
from pathlib import Path
from supervision.draw.color import ColorPalette
from realsense_cv.utils.supervision_utils import CUSTOM_COLOR_MAP
from PIL import Image as PILImage
from sam2.build_sam import build_sam2
from geometry_msgs.msg import TransformStamped

from sam2.sam2_image_predictor import SAM2ImagePredictor
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import math


DEFAULT_SAM2_CHECKPOINT = "/home/nano/CV/GSAM/checkpoints/sam2.1_hiera_small.pt"


def _candidate_checkpoint_paths(path_str):
    raw = os.path.expanduser(str(path_str))
    basename = Path(raw).name

    candidates = [
        raw,
        DEFAULT_SAM2_CHECKPOINT,
    ]

    if basename:
        candidates.extend(
            [
                f"/home/nano/CV/GSAM/checkpoints/{basename}",
                f"/home/nano/final_project_ws/src/perception/realsense_cv/models/{basename}",
            ]
        )

    seen = set()
    ordered = []
    for candidate in candidates:
        if candidate not in seen:
            seen.add(candidate)
            ordered.append(candidate)
    return ordered


def resolve_sam2_checkpoint(path_str):
    candidates = _candidate_checkpoint_paths(path_str)
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    raise FileNotFoundError(
        "SAM2 checkpoint not found. Checked: " + ", ".join(candidates)
    )


def rotation_matrix_to_quaternion(R):
    q = np.empty((4,), dtype=np.float64)
    trace = np.trace(R)

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

    return q[0], q[1], q[2], q[3]

class GSAMSlideDetectNode(Node):
    def __init__(self):
        super().__init__('gsam_slide_detect_node')

        # Declare parameters
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/camera/edges_overlay')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('service_name', '/detect_slides')
        self.declare_parameter('grounding_model', "IDEA-Research/grounding-dino-tiny")
        self.declare_parameter('text_prompt', "colored box.")
        self.declare_parameter('sam2_checkpoint', DEFAULT_SAM2_CHECKPOINT)
        self.declare_parameter('sam2_model_config', "configs/sam2.1/sam2.1_hiera_s.yaml")
        self.declare_parameter('force_cpu', False)
        self.declare_parameter('num_slots', 25)
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('depth_diff_threshold', 10)  # mm shallower than tray floor = wafer
        self.declare_parameter('min_tray_area_fraction', 0.02)
        # Default raised from 0.20 -> 0.45 because our trays sit close enough to
        # the camera that a single tray covers ~32% of the frame. Lower this if
        # you start getting whole-workbench detections through the filter.
        self.declare_parameter('max_tray_area_fraction', 0.45)
        self.declare_parameter('min_tray_short_side_px', 140)
        self.declare_parameter('max_trays', 2)
        # Color/hue occupancy detection (mirrors pick_place.py wafer_detect3 rules).
        # Operates on the perspective-warped tray ROI alongside depth.
        # Set enable_color_detection=False to fall back to depth-only behavior.
        # Use cv2.minAreaRect on the bbox-clipped SAM2 mask so the perspective
        # warp + slot grid follow the tray's actual orientation instead of
        # the axis-aligned Grounding DINO box. Set False to fall back to the
        # old axis-aligned behavior.
        self.declare_parameter('use_oriented_rect', True)
        self.declare_parameter('enable_color_detection', True)
        self.declare_parameter('color_interior_x_min', 0.12)  # band x-range as frac of tray_w
        self.declare_parameter('color_interior_x_max', 0.88)
        self.declare_parameter('color_hue_x', 0.45)           # hue patch x-center as frac of tray_w
        self.declare_parameter('dark_abs_thresh', 42.0)       # Rule 1: dark pixel absolute threshold
        self.declare_parameter('single_thresh', 90.0)         # Rule 2: brightness gate when hue is in range
        self.declare_parameter('hue_low_cv2', 70)             # Rule 2: lower hue bound (OpenCV 0-180)
        self.declare_parameter('hue_high_cv2', 100)           # Rule 2: upper hue bound (OpenCV 0-180)
        self.declare_parameter(
            'debug_output_dir',
            str(Path.home() / 'final_project_ws'),
        )

        # Get parameters
        self.grounding_model = self.get_parameter('grounding_model').value
        self.text_prompt = self.get_parameter('text_prompt').value
        self.sam2_checkpoint = resolve_sam2_checkpoint(
            self.get_parameter('sam2_checkpoint').value
        )
        self.sam2_model_config = self.get_parameter('sam2_model_config').value
        self.camera_frame = self.get_parameter('camera_frame').value
        force_cpu = self.get_parameter('force_cpu').value
        self.num_slots = self.get_parameter('num_slots').value
        self.depth_diff_threshold = self.get_parameter('depth_diff_threshold').value
        self.min_tray_area_fraction = float(
            self.get_parameter('min_tray_area_fraction').value
        )
        self.max_tray_area_fraction = float(
            self.get_parameter('max_tray_area_fraction').value
        )
        self.min_tray_short_side_px = int(
            self.get_parameter('min_tray_short_side_px').value
        )
        self.max_trays = max(1, int(self.get_parameter('max_trays').value))
        self.use_oriented_rect = bool(
            self.get_parameter('use_oriented_rect').value
        )
        self.enable_color_detection = bool(
            self.get_parameter('enable_color_detection').value
        )
        self.color_interior_x_min = float(
            self.get_parameter('color_interior_x_min').value
        )
        self.color_interior_x_max = float(
            self.get_parameter('color_interior_x_max').value
        )
        self.color_hue_x = float(self.get_parameter('color_hue_x').value)
        self.dark_abs_thresh = float(self.get_parameter('dark_abs_thresh').value)
        self.single_thresh = float(self.get_parameter('single_thresh').value)
        self.hue_low_cv2 = float(self.get_parameter('hue_low_cv2').value)
        self.hue_high_cv2 = float(self.get_parameter('hue_high_cv2').value)
        self.debug_output_dir = Path(
            self.get_parameter('debug_output_dir').value
        ).expanduser()
        self.debug_output_dir.mkdir(parents=True, exist_ok=True)
        input_topic = self.get_parameter('input_image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        output_topic = self.get_parameter('output_image_topic').value
        service_name = self.get_parameter('service_name').value
        info_topic = self.get_parameter('camera_info_topic').value


        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        self.last_image = None
        self.last_msg = None
        self.last_depth = None

        self.camera_matrix = None
        self.caminfo_sub = self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        self.processing_image = False
        self.device = "cuda" if torch.cuda.is_available() and not force_cpu else "cpu"
        self.get_logger().info(
            f'Using SAM2 checkpoint: {self.sam2_checkpoint}'
        )
        self.get_logger().info(
            f'Debug outputs will be written to: {self.debug_output_dir}'
        )

        if torch.cuda.is_available() and torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        self._load_models_with_fallback(force_cpu)

        self.image_sub = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)

        # Publisher for debug/intermediate images
        self.edge_pub = self.create_publisher(Image, output_topic, 10)

        # Create service
        self.service = self.create_service(
            Trigger,
            service_name,
            self.detect_slides_service_callback
        )

        self.get_logger().info(f'GSAM Slide Detect Service initialized at {service_name}')

    def _load_models_on_device(self, device):
        self.get_logger().info(f'Loading SAM2 on {device}...')
        sam2_model = build_sam2(
            self.sam2_model_config,
            self.sam2_checkpoint,
            device=device,
        )
        sam2_predictor = SAM2ImagePredictor(sam2_model)
        self.get_logger().info('Loading Grounding DINO processor...')
        processor = AutoProcessor.from_pretrained(
            self.grounding_model,
            local_files_only=True,
        )
        self.get_logger().info('Loading Grounding DINO weights...')
        grounding_model_net = AutoModelForZeroShotObjectDetection.from_pretrained(
            self.grounding_model,
            local_files_only=True,
        )
        self.get_logger().info(f'Moving Grounding DINO to {device}...')
        grounding_model_net = grounding_model_net.to(device)
        return sam2_predictor, processor, grounding_model_net

    def _clear_cuda_memory(self):
        gc.collect()
        if not torch.cuda.is_available():
            return
        try:
            torch.cuda.empty_cache()
        except Exception:
            pass
        try:
            torch.cuda.ipc_collect()
        except Exception:
            pass

    def _load_models_with_fallback(self, force_cpu):
        preferred_device = self.device
        try:
            (
                self.sam2_predictor,
                self.processor,
                self.grounding_model_net,
            ) = self._load_models_on_device(preferred_device)
            self.device = preferred_device
            self.get_logger().info(f'Using device: {self.device}')
            self.get_logger().info('Models loaded.')
            return
        except Exception as exc:
            if preferred_device != "cuda" or force_cpu:
                raise

            self.get_logger().warning(
                f'CUDA model load failed: {exc}. Retrying on CPU.'
            )
            self._clear_cuda_memory()

        (
            self.sam2_predictor,
            self.processor,
            self.grounding_model_net,
        ) = self._load_models_on_device("cpu")
        self.device = "cpu"
        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info('Models loaded.')

    def cv2_to_ros_image(self, cv_image, encoding='bgr8'):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = cv_image.shape[:2]
        msg.encoding = encoding
        msg.step = cv_image.shape[1] * cv_image.shape[2] if cv_image.ndim == 3 else cv_image.shape[1]
        msg.data = cv_image.tobytes()
        return msg

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics received")
    def ros_image_to_cv2(self, msg: Image):
        """
        Convert ROS Image message to OpenCV image without using CvBridge
        """
        # Get image dimensions
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        # Convert bytes to numpy array
        if encoding == 'bgr8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding == 'rgb8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        elif encoding == 'mono8':
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        elif encoding == '16UC1':
            cv_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
        else:
            self.get_logger().error(f'Unsupported encoding: {encoding}')
            return None

        return cv_image

    def image_callback(self, msg: Image):
        ### CV_image = (720, 1280, 3)
        # Convert ROS Image message to OpenCV image (BGR)
        cv_image = self.ros_image_to_cv2(msg)
        if cv_image is None:
            return
        if self.processing_image is False:
            # Store the last image for service processing
            self.last_image = cv_image
            self.last_msg = msg

    def depth_callback(self, msg: Image):
        if not self.processing_image:
            self.last_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    def detect_slides_service_callback(self, request, response):
        if self.last_image is None:
            response.success = False
            response.message = "No image available. Please wait for camera image."
            self.get_logger().warn('Service called but no image available yet')
            return response

        if self.last_depth is None:
            response.success = False
            response.message = "No depth image available. Check that aligned depth is enabled."
            self.get_logger().warn('Service called but no depth image available yet')
            return response

        try:
            # Save last_image temporarily for processing
            self.processing_image = True
            # Detect slides
            detected_slides = self.detect_slides(self.last_image, text_prompt=self.text_prompt)

            response.success = True
            response.message = f"Detected slides at slots: {detected_slides}"
            self.get_logger().info(f'Slide detection complete: {detected_slides}')

        except Exception as e:
            response.success = False
            response.message = f"Error during slide detection: {str(e)}"
            self.get_logger().error(f'Slide detection failed: {str(e)}')
            self.processing_image = False
        self.processing_image = False
        return response
    @staticmethod
    def sort_points(points):
        points = sorted(points, key = lambda x: x[0]**2 + x[1]**2)
        p1 = points[0]
        p4 = points[3]
        points = sorted(points[1:3], key = lambda x: abs(x[0]-p1[0])+abs(x[1]-p1[1]))
        p2 = points[0]
        p3 = points[1]
        return [p1, p2, p3, p4]
    @staticmethod
    def yukai_kernel(points, width=15, numSlots=25, img_shape=(720, 1280)):
        h, w = img_shape
        p1, p2, p3, p4 = points
        #points = sorted(points, key = lambda x: x[0]**2 + x[1]**2)
        #p1 = points[0]
        #p4 = points[3]
        #points = sorted(points[1:3], key = lambda x: abs(x[0]-p1[0])+abs(x[1]-p1[1]))
        #p2 = points[0]
        #p3 = points[1]

        """
        p1----p2
        \.     \
        \      \
        \.     \
        p3-----p4
        """

        kernels = []
        
        for i in range(1,numSlots+1):
            kernel = np.zeros((h, w), dtype=np.uint8)  # black image
            sx = int(p1[0]+(p3[0]-p1[0])*i/(numSlots+1))
            sy = int(p1[1]+(p3[1]-p1[1])*i/(numSlots+1))
            ex = int(p2[0]+(p4[0]-p2[0])*i/(numSlots+1))
            ey = int(p2[1]+(p4[1]-p2[1])*i/(numSlots+1))
            cv2.line(kernel, (sx, sy), (ex, ey), color=255, thickness=width)
            kernel = kernel.astype(np.float32)
            kernels.append(kernel)

        return kernels
    
    
    def deproject_pixel_to_3d(self, point, depth_m=None, sample_radius=3):
        u, v = point
        K = self.camera_matrix
        if K is None:
            raise ValueError("Camera intrinsic matrix K is not set.")
        fx = K[0, 0]; fy = K[1, 1]
        cx = K[0, 2]; cy = K[1, 2]

        if depth_m is None:
            if self.last_depth is None:
                raise ValueError("Depth image is not available for deprojection.")
            u_i = int(round(u))
            v_i = int(round(v))
            h, w = self.last_depth.shape
            search_radii = []
            for radius in (
                sample_radius,
                max(sample_radius * 2, 8),
                max(sample_radius * 4, 16),
                max(sample_radius * 8, 32),
            ):
                if radius not in search_radii:
                    search_radii.append(radius)

            valid = None
            for radius in search_radii:
                x1 = max(0, u_i - radius)
                x2 = min(w, u_i + radius + 1)
                y1 = max(0, v_i - radius)
                y2 = min(h, v_i + radius + 1)
                region = self.last_depth[y1:y2, x1:x2]
                valid = region[region > 0]
                if len(valid) > 0:
                    break

            if valid is None or len(valid) == 0:
                raise ValueError(f"No valid depth near pixel ({u}, {v}).")
            Z = float(np.median(valid)) / 1000.0
        else:
            Z = float(depth_m)
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy 

        return X, Y, Z
    
    def gsam_mask(self, image, text=None):
        image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        self.sam2_predictor.set_image(np.array(image.convert("RGB")))

        inputs = self.processor(images=image, text=text, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.grounding_model_net(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            # threshold=0.4,
            text_threshold=0.3,
            target_sizes=[image.size[::-1]]
        )

        """
        Results is a list of dict with the following structure:
        [
            {
                'scores': tensor([0.7969, 0.6469, 0.6002, 0.4220], device='cuda:0'), 
                'labels': ['car', 'tire', 'tire', 'tire'], 
                'boxes': tensor([[  89.3244,  278.6940, 1710.3505,  851.5143],
                                [1392.4701,  554.4064, 1628.6133,  777.5872],
                                [ 436.1182,  621.8940,  676.5255,  851.6897],
                                [1236.0990,  688.3547, 1400.2427,  753.1256]], device='cuda:0')
            }
        ]
        """

        input_boxes = results[0]["boxes"].cpu().numpy()
        if len(input_boxes) == 0:
            raise ValueError("Grounding DINO detected no objects matching the text prompt")

        per_box_masks = []
        for box in input_boxes:
            masks, scores, logits = self.sam2_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=box,
                multimask_output=False,
            )
            m = masks[0].squeeze().astype(np.uint8) * 255
            per_box_masks.append(m)

        print(f"detected {len(per_box_masks)} trays")
        # return masks AND the detection boxes — boxes are used for the perspective warp
        # since SAM2 masks can include background bleed that throws off mask_to_rect
        return per_box_masks, input_boxes

    def filter_tray_candidates(self, masks, det_boxes, image_shape):
        """Discard box detections that are clearly too large/small to be trays."""
        img_h, img_w = image_shape[:2]
        image_area = float(img_h * img_w)
        candidates = []

        for idx, (mask, det_box) in enumerate(zip(masks, det_boxes)):
            x1, y1, x2, y2 = det_box.astype(int)
            width = max(1, x2 - x1)
            height = max(1, y2 - y1)
            area = float(width * height)
            area_fraction = area / image_area
            short_side = min(width, height)
            mask_fill_fraction = float(np.count_nonzero(mask)) / area
            center_x = (x1 + x2) / 2.0

            reason = None
            if area_fraction < self.min_tray_area_fraction:
                reason = 'too_small'
            elif area_fraction > self.max_tray_area_fraction:
                reason = 'too_large'
            elif short_side < self.min_tray_short_side_px:
                reason = 'short_side_too_small'

            candidate = {
                'candidate_index': int(idx),
                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                'width': int(width),
                'height': int(height),
                'area_fraction': float(area_fraction),
                'short_side': int(short_side),
                'mask_fill_fraction': float(mask_fill_fraction),
                'center_x': float(center_x),
                'accepted': reason is None,
                'rejection_reason': reason,
            }
            candidates.append(candidate)

        with open(self.debug_output_dir / 'debug_tray_candidates.json', 'w', encoding='utf-8') as f:
            json.dump(candidates, f, indent=2)

        kept = []
        for candidate in candidates:
            self.get_logger().info(
                'Tray candidate '
                f"{candidate['candidate_index']}: bbox={candidate['bbox']} "
                f"area={candidate['area_fraction']:.3f} short={candidate['short_side']} "
                f"fill={candidate['mask_fill_fraction']:.3f} "
                f"{'KEEP' if candidate['accepted'] else 'DROP ' + candidate['rejection_reason']}"
            )
            if candidate['accepted']:
                kept.append(candidate)

        if len(kept) > self.max_trays:
            kept = sorted(kept, key=lambda item: item['area_fraction'], reverse=True)[:self.max_trays]

        # Stable tray numbering matters for source/destination indexing.
        # Number trays from left to right after filtering.
        kept = sorted(kept, key=lambda item: item['center_x'])

        selected_masks = [masks[item['candidate_index']] for item in kept]
        selected_boxes = [det_boxes[item['candidate_index']] for item in kept]

        self.get_logger().info(
            f'Selected {len(selected_boxes)} tray candidate(s) after filtering'
        )
        return selected_masks, selected_boxes
    @staticmethod
    def mask_to_rect(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = max(contours, key=cv2.contourArea)
        # minAreaRect always gives exactly 4 corners regardless of rounded edges
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        return [box.reshape(4, 1, 2)]

    @staticmethod
    def oriented_tray_quad(mask, det_box, image_shape):
        """
        Compute the oriented quadrilateral that bounds the tray.

        Pipeline: clip SAM2 mask to the Grounding DINO bbox (suppresses any
        background bleed outside the detection region), morphologically clean
        the result, take the largest contour, then `cv2.minAreaRect`.

        Returns:
            quad: np.ndarray shape (4, 2) float32, in [TL, TR, BL, BR] order
                  where TL-TR is the short edge of the tray that sits highest
                  in the image. Compatible with the existing warp src_pts
                  convention used elsewhere in this file.
            short_len, long_len: actual edge lengths in pixels.
            angle_deg: rotation angle of the long axis vs vertical, signed.
            None on failure.
        """
        img_h, img_w = image_shape[:2]
        x1, y1, x2, y2 = det_box.astype(int)
        x1 = max(0, x1); y1 = max(0, y1)
        x2 = min(img_w, x2); y2 = min(img_h, y2)
        if x2 <= x1 or y2 <= y1:
            return None

        clipped = np.zeros_like(mask)
        clipped[y1:y2, x1:x2] = mask[y1:y2, x1:x2]

        # Kernel size scales with image resolution so we don't over-erode on
        # small frames or under-clean on large ones.
        k = max(5, min(img_h, img_w) // 200)
        if k % 2 == 0:
            k += 1
        kernel = np.ones((k, k), np.uint8)
        clipped = cv2.morphologyEx(clipped, cv2.MORPH_CLOSE, kernel)
        clipped = cv2.morphologyEx(clipped, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(clipped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 100:
            return None

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)  # (4, 2) clockwise from bottom-most corner

        # Pair adjacent corners into edges; identify which two opposite edges
        # are the SHORT sides (across-tray, perpendicular to the slot stack).
        edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        edge_lens = [float(np.linalg.norm(box[a] - box[b])) for a, b in edges]
        if edge_lens[0] < edge_lens[1]:
            short_edges = [edges[0], edges[2]]
            short_len, long_len = edge_lens[0], edge_lens[1]
        else:
            short_edges = [edges[1], edges[3]]
            short_len, long_len = edge_lens[1], edge_lens[0]

        # Of the two short edges, the one with smaller mean y is "top".
        means_y = [(box[a][1] + box[b][1]) / 2.0 for a, b in short_edges]
        if means_y[0] <= means_y[1]:
            top_edge, bot_edge = short_edges[0], short_edges[1]
        else:
            top_edge, bot_edge = short_edges[1], short_edges[0]

        # Within each short edge, smaller x is "left".
        top = sorted([box[top_edge[0]], box[top_edge[1]]], key=lambda p: p[0])
        bot = sorted([box[bot_edge[0]], box[bot_edge[1]]], key=lambda p: p[0])
        p1, p2 = top  # TL, TR
        p3, p4 = bot  # BL, BR

        # Angle of long axis (BL -> TL) vs image-up (negative-y). Positive = CW.
        v_long = np.array(p1) - np.array(p3)
        angle_deg = float(np.degrees(np.arctan2(v_long[0], -v_long[1])))

        return (
            np.array([p1, p2, p3, p4], dtype=np.float32),
            float(short_len),
            float(long_len),
            angle_deg,
        )

    @staticmethod
    def filter_non_parallel(img, points, angle_thrd=10):
        """
        Filter edges to keep only lines parallel to the line (p1, p2)

        Args:
            img: Edge image
            points: List of 4 corner points [p1, p2, p3, p4]
            angle_thrd: Angle threshold in degrees
        """
        # 1) Ensure single-channel 8-bit
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = img.astype(np.uint8)
        lines = cv2.HoughLines(img, 1, np.pi/180, threshold=30)

        if lines is None:
            return np.zeros_like(img).astype(bool)

        # Calculate the reference angle from p1 to p2
        p1, p2, _, _ = points
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        # Calculate angle in radians (note: cv2 uses angle from x-axis)
        reference_angle = np.arctan2(dy, dx)

        # Convert to Hough line theta format (angle of normal to the line)
        # Hough theta is the angle of the normal, so we add pi/2
        target_theta = reference_angle + np.pi/2

        # Normalize to [0, pi] range as Hough uses
        while target_theta < 0:
            target_theta += np.pi
        while target_theta >= np.pi:
            target_theta -= np.pi

        print(f"Reference angle (p1->p2): {np.rad2deg(reference_angle):.2f}°")
        print(f"Target theta for Hough: {np.rad2deg(target_theta):.2f}°")

        filtered_lines = []
        for line in lines:
            rho, theta = line[0]

            # Check if angle is close to target_theta (considering wrapping at 0/pi)
            angle_diff = abs(theta - target_theta)
            # Also check the supplementary angle (theta + pi or theta - pi)
            angle_diff_supp = abs(abs(theta - target_theta) - np.pi)

            min_angle_diff = min(angle_diff, angle_diff_supp)

            if min_angle_diff < np.deg2rad(angle_thrd):
                filtered_lines.append((rho, theta))

        print(f"Filtered {len(filtered_lines)} lines out of {len(lines)} total lines")

        output = np.zeros_like(img)

        for rho, theta in filtered_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho

            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

            cv2.line(output, (x1, y1), (x2, y2), 255, 1)

        cv2.imwrite(str(self.debug_output_dir / "filtered_edges.png"), output)
        return output.astype(bool)

    def publish_slide_frames(self, points, slotInd, slot_xyz=None, msg=None):

        """
        Publish a TF for `slide_{slotInd:02d}` based on this tray's corner points.

        slotInd is the GLOBAL slot index. The local slot within the current
        tray is computed via `((slotInd - 1) % num_slots) + 1` so the X-offset
        formula stays consistent regardless of which tray we're in.

        Parent -> Child: (R_pc, t_pc)
        Child  -> New : (R_cn, t_cn)

        Returns Parent -> New: (R_pn, t_pn)
        """
        p1 = np.array(points[0], dtype=np.float64)
        p2 = np.array(points[1], dtype=np.float64)
        p3 = np.array(points[2], dtype=np.float64)  # not used here, but you can if you prefer
        if msg is None:
            msg = self.last_msg
        v_short = p2 - p1                     # [dx, dy]
        v_short3 = np.array([v_short[0], v_short[1], 0.0], dtype=np.float64)
        e_short = v_short3 / np.linalg.norm(v_short3)
        # long_side = np.linalg.norm(np.array(p3)-np.array(p1))
        z_cam = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        e_long = np.cross(e_short, z_cam)     # or np.cross(e_short, z_cam) depending on handedness
        e_long /= np.linalg.norm(e_long)

        R_box_cam = np.column_stack((e_long, e_short, z_cam))
        T_box_cam = (p1+p2)/2
        if slot_xyz is None:
            # Fallback to the previous tray-model estimate if a direct slot-center
            # 3D point is unavailable.
            local_slot = ((slotInd - 1) % self.num_slots) + 1
            T_slide_box = np.array([
                -0.00528 * local_slot - 0.01,
                0.0,
                0.0,
            ], dtype=np.float64)
            t_pn = R_box_cam @ T_slide_box + T_box_cam
        else:
            t_pn = np.array(slot_xyz, dtype=np.float64)
        R_pn = R_box_cam

        # Convert to quaternion
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_pn)

        # Build and send TF
        tfmsg = TransformStamped()
        # Stamp with the CURRENT time, not msg.header.stamp (the camera image
        # timestamp). On CPU, GSAM inference takes 60-90s per call, so the
        # image stamp would be far older than tf2's dynamic buffer window
        # (~10s) and the frame would be published yet un-lookupable. These
        # frames go out via the *static* broadcaster (see detect_slides),
        # which is timestamp-agnostic, so current time is the safe choice.
        tfmsg.header.stamp = self.get_clock().now().to_msg()
        tfmsg.header.frame_id = self.camera_frame
        tfmsg.child_frame_id = f"slide_{slotInd:02d}"

        tfmsg.transform.translation.x = float(t_pn[0])
        tfmsg.transform.translation.y = float(t_pn[1])
        tfmsg.transform.translation.z = float(t_pn[2])

        tfmsg.transform.rotation.x = float(qx)
        tfmsg.transform.rotation.y = float(qy)
        tfmsg.transform.rotation.z = float(qz)
        tfmsg.transform.rotation.w = float(qw)

        # Do NOT publish on the dynamic broadcaster here. detect_slides()
        # batches every slot into one self.static_br.sendTransform() call.
        # Publishing the same child frame on both /tf and /tf_static makes
        # tf2 flip-flop between dynamic and static storage for that frame.
        return R_pn, t_pn, tfmsg
    @staticmethod
    def slide_ind_post_process(detected_slides):
        # Remove slides that are too close to each other (within 2 slots), will filter out the latter one
        filtered_slides = []
        i=0
        while i < len(detected_slides):
            filtered_slides.append(detected_slides[i])
            j = i + 1
            while j < len(detected_slides) and detected_slides[j] <= detected_slides[i] + 2:
                j += 1
            i = j
        
        return filtered_slides

    def detect_slides(self, img, text_prompt=None):
        """
        Detect occupied wafer slots across all detected trays.
        Warps each tray to a flat top-down view to correct perspective, then uses
        adaptive thresholding (relative to each tray's background) to ignore shadows.
        Tray 0 → slots 1..num_slots, tray 1 → slots num_slots+1..2*num_slots, etc.

        Returns:
            List of occupied slot numbers (1-indexed, global across all trays)
        """
        if text_prompt is None:
            text_prompt = self.text_prompt

        num_slots = self.num_slots
        depth_img = self.last_depth.astype(np.float32)

        # Raw colorized depth debug image — no warp, just the sensor reading
        valid_px = depth_img[depth_img > 0]
        if len(valid_px) > 0:
            d_min, d_max = np.percentile(valid_px, 1), np.percentile(valid_px, 99)
            norm = np.clip((depth_img - d_min) / (d_max - d_min) * 255, 0, 255).astype(np.uint8)
            norm[depth_img == 0] = 0
            raw_colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
            raw_colored[depth_img == 0] = 0
            cv2.imwrite(str(self.debug_output_dir / "debug_depth_raw.jpg"), raw_colored)

        masks, det_boxes = self.gsam_mask(img, text=text_prompt)
        masks, det_boxes = self.filter_tray_candidates(masks, det_boxes, img.shape)
        overlay = img.copy()
        all_detected = []
        static_slot_transforms = []
        debug_trays = []

        for tray_idx, (mask, det_box) in enumerate(zip(masks, det_boxes)):
            cv2.imwrite(str(self.debug_output_dir / f"debug_mask_tray{tray_idx}.png"), mask)

            tray_x1, tray_y1, tray_x2, tray_y2 = det_box.astype(int)
            quad_angle_deg = 0.0
            quad_source = 'axis_aligned'

            oriented = None
            if self.use_oriented_rect:
                oriented = self.oriented_tray_quad(mask, det_box, img.shape)
                if oriented is None:
                    self.get_logger().warn(
                        f"Tray {tray_idx}: oriented_tray_quad returned None - "
                        "falling back to axis-aligned bbox"
                    )

            if oriented is not None:
                quad, short_len, long_len, quad_angle_deg = oriented
                p1, p2, p3, p4 = (quad[0], quad[1], quad[2], quad[3])
                tray_w = max(1, int(round(short_len)))
                tray_h = max(1, int(round(long_len)))
                quad_source = 'oriented_min_area_rect'
                self.get_logger().info(
                    f"Tray {tray_idx}: oriented quad - short={tray_w}px "
                    f"long={tray_h}px angle={quad_angle_deg:+.2f}°"
                )
            else:
                # Axis-aligned fallback (legacy behavior).
                p1 = np.array([tray_x1, tray_y1], dtype=np.float32)  # TL
                p2 = np.array([tray_x2, tray_y1], dtype=np.float32)  # TR
                p3 = np.array([tray_x1, tray_y2], dtype=np.float32)  # BL
                p4 = np.array([tray_x2, tray_y2], dtype=np.float32)  # BR
                tray_w = int(np.linalg.norm(p2 - p1))
                tray_h = int(np.linalg.norm(p3 - p1))

            points = [p1.tolist(), p2.tolist(), p3.tolist(), p4.tolist()]

            # Draw the actual tray quad (rotated if oriented_tray_quad fired)
            # plus the original axis-aligned bbox in a fainter color so you
            # can visually compare orientation alignment.
            quad_poly = np.array([p1, p2, p4, p3], dtype=np.int32)  # CW for poly draw
            cv2.polylines(overlay, [quad_poly.reshape(-1, 1, 2)],
                          isClosed=True, color=(255, 0, 0), thickness=3)
            cv2.rectangle(overlay, (tray_x1, tray_y1), (tray_x2, tray_y2),
                          (180, 180, 180), 1)

            # Inset 4 corners toward the centroid for depth-plane sampling.
            # Works regardless of orientation - just shrink the quad slightly.
            centroid = (p1 + p2 + p3 + p4) / 4.0
            inset_frac = 1.0 / 12.0  # ~8% toward center
            depth_points = []
            for corner in (p1, p2, p3, p4):
                inset_pt = corner + inset_frac * (centroid - corner)
                depth_points.append([int(inset_pt[0]), int(inset_pt[1])])
            depth_sample_radius = max(6, int(min(tray_w, tray_h) / 24))
            spatial_points = [
                self.deproject_pixel_to_3d(pt, sample_radius=depth_sample_radius)
                for pt in depth_points
            ]

            src_pts = np.array([p1, p2, p4, p3], dtype=np.float32)
            dst_pts = np.array([[0, 0], [tray_w, 0], [tray_w, tray_h], [0, tray_h]], dtype=np.float32)
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            M_inv = cv2.invert(M)[1]

            # Warp depth with nearest-neighbour to avoid interpolating depth values
            warped_depth = cv2.warpPerspective(depth_img, M, (tray_w, tray_h),
                                               flags=cv2.INTER_NEAREST)

            # Warp the color image too so the hue / dark-pixel rules from
            # pick_place.py (wafer_detect3) operate on a flat tray ROI that
            # lines up 1:1 with the slot grid we use for depth.
            if self.enable_color_detection:
                warped_color = cv2.warpPerspective(
                    img, M, (tray_w, tray_h), flags=cv2.INTER_LINEAR
                )
                warped_hsv = cv2.cvtColor(warped_color, cv2.COLOR_BGR2HSV)
                warped_gray = cv2.cvtColor(warped_color, cv2.COLOR_BGR2GRAY).astype(np.float32)
                cv2.imwrite(
                    str(self.debug_output_dir / f"debug_color_warp_tray{tray_idx}.jpg"),
                    warped_color,
                )
            else:
                warped_color = None
                warped_hsv = None
                warped_gray = None

            # Colorized depth debug image — normalize to valid pixel range for max contrast
            valid_px = warped_depth[warped_depth > 0]
            if len(valid_px) > 0:
                d_min, d_max = valid_px.min(), valid_px.max()
                if d_max > d_min:
                    norm = np.clip((warped_depth - d_min) / (d_max - d_min) * 255, 0, 255).astype(np.uint8)
                else:
                    norm = np.zeros_like(warped_depth, dtype=np.uint8)
                norm[warped_depth == 0] = 0  # keep invalid pixels black
                colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
                colored[warped_depth == 0] = 0
                cv2.imwrite(str(self.debug_output_dir / f"debug_depth_color_tray{tray_idx}.jpg"), colored)

            sample_radius = max(6, tray_h // (num_slots * 3))
            slot_depths = []
            sample_centers = []

            # Color/hue sampling geometry (warped tray frame)
            color_band_radius = max(4, tray_h // (num_slots * 2))
            color_xi_s = int(self.color_interior_x_min * tray_w)
            color_xi_e = int(self.color_interior_x_max * tray_w)
            if color_xi_e <= color_xi_s:
                color_xi_s, color_xi_e = 0, tray_w
            color_cx_hue = int(self.color_hue_x * tray_w)
            slot_color_min_b = []  # min column-mean brightness in interior band
            slot_color_hue = []    # mean hue (OpenCV 0-180) in center patch

            for i in range(1, num_slots + 1):
                t = i / (num_slots + 1)
                cx, cy = tray_w // 2, int(t * tray_h)
                sample_centers.append((cx, cy))
                sample_x1 = max(0, cx - sample_radius)
                sample_x2 = min(tray_w, cx + sample_radius)
                sample_y1 = max(0, cy - sample_radius)
                sample_y2 = min(tray_h, cy + sample_radius)
                region = warped_depth[sample_y1:sample_y2, sample_x1:sample_x2]
                valid = region[region > 0]
                slot_depths.append(float(np.median(valid)) if len(valid) > 0 else 0.0)

                # Color sampling (mirror pick_place.detect_wafers band+hue patch)
                if self.enable_color_detection and warped_gray is not None:
                    cy_b1 = max(0, cy - color_band_radius)
                    cy_b2 = min(tray_h, cy + color_band_radius)
                    band = warped_gray[cy_b1:cy_b2, color_xi_s:color_xi_e]
                    if band.size > 0:
                        col_means = np.mean(band, axis=0)
                        min_b = float(np.min(col_means))
                    else:
                        min_b = 255.0

                    hx1 = max(0, color_cx_hue - color_band_radius)
                    hx2 = min(tray_w, color_cx_hue + color_band_radius)
                    hy1 = max(0, cy - color_band_radius)
                    hy2 = min(tray_h, cy + color_band_radius)
                    hue_patch = warped_hsv[hy1:hy2, hx1:hx2, 0]
                    hue_val = float(np.mean(hue_patch)) if hue_patch.size > 0 else 0.0
                else:
                    min_b = float('nan')
                    hue_val = float('nan')

                slot_color_min_b.append(min_b)
                slot_color_hue.append(hue_val)

            valid_depths = [d for d in slot_depths if d > 0]
            if not valid_depths:
                self.get_logger().warn(f"Tray {tray_idx}: no valid depth readings, skipping")
                continue
            tray_floor = np.percentile(valid_depths, 75)
            self.get_logger().info(
                f"Tray {tray_idx}: floor depth {tray_floor:.1f} mm, diff threshold {self.depth_diff_threshold} mm"
            )

            slot_offset = tray_idx * num_slots

            tray_debug = {
                'tray_index': int(tray_idx),
                'bbox': {
                    'x1': int(tray_x1),
                    'y1': int(tray_y1),
                    'x2': int(tray_x2),
                    'y2': int(tray_y2),
                    'width': int(tray_x2 - tray_x1),
                    'height': int(tray_y2 - tray_y1),
                },
                'quad_source': quad_source,
                'quad_angle_deg': float(quad_angle_deg),
                'quad_corners': {
                    'p1_tl': [float(p1[0]), float(p1[1])],
                    'p2_tr': [float(p2[0]), float(p2[1])],
                    'p3_bl': [float(p3[0]), float(p3[1])],
                    'p4_br': [float(p4[0]), float(p4[1])],
                },
                'warped_size_px': {'w': int(tray_w), 'h': int(tray_h)},
                'depth_sample_radius_px': int(depth_sample_radius),
                'depth_sample_points': [
                    {'x': int(point[0]), 'y': int(point[1])}
                    for point in depth_points
                ],
                'tray_floor_depth_mm': float(tray_floor),
                'slot_sample_centers': [],
            }

            for i, (depth_val, (cx_w, cy_w)) in enumerate(zip(slot_depths, sample_centers), 1):
                global_slot = slot_offset + i
                depth_diff = tray_floor - depth_val
                depth_occupied = depth_val > 0 and depth_diff > self.depth_diff_threshold

                # Color rules (mirror pick_place.detect_wafers): Rule 1 = absolute
                # dark pixel; Rule 2 = hue in wafer band + moderately dark.
                min_b = slot_color_min_b[i - 1]
                hue_val = slot_color_hue[i - 1]
                if self.enable_color_detection and not (np.isnan(min_b) or np.isnan(hue_val)):
                    color_rule1 = min_b < self.dark_abs_thresh
                    color_rule2 = (
                        (self.hue_low_cv2 < hue_val < self.hue_high_cv2)
                        and (min_b < self.single_thresh)
                    )
                    color_occupied = bool(color_rule1 or color_rule2)
                else:
                    color_rule1 = False
                    color_rule2 = False
                    color_occupied = False

                occupied = bool(depth_occupied or color_occupied)
                self.get_logger().info(
                    f"Tray {tray_idx} slot {i} (global {global_slot}): "
                    f"depth {depth_val:.1f} mm  diff {depth_diff:.1f} mm  "
                    f"min_b {min_b:.1f}  hue {hue_val:.1f}  "
                    f"depth_occ={depth_occupied} color_occ={color_occupied} "
                    f"(r1={color_rule1} r2={color_rule2})  "
                    f"final={'OCCUPIED' if occupied else 'empty'}"
                )

                pt = cv2.perspectiveTransform(np.array([[[cx_w, cy_w]]], dtype=np.float32), M_inv)[0][0]
                cx_o, cy_o = int(pt[0]), int(pt[1])
                color = (0, 0, 255) if occupied else (0, 255, 0)
                cv2.circle(overlay, (cx_o, cy_o), 8, color, 2)
                cv2.putText(overlay, str(global_slot), (cx_o - 8, cy_o - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                # Publish a frame for every slot (occupied or empty) so that the
                # planner can look up destination slots in another tray, not just
                # the source slots that have wafers.
                slot_depth_m = (depth_val if depth_val > 0 else tray_floor) / 1000.0
                slot_xyz = self.deproject_pixel_to_3d((cx_o, cy_o), depth_m=slot_depth_m)
                _, t_pn, tfmsg = self.publish_slide_frames(
                    spatial_points,
                    global_slot,
                    slot_xyz=slot_xyz,
                )
                static_slot_transforms.append(tfmsg)
                tray_debug['slot_sample_centers'].append({
                    'slot': int(i),
                    'global_slot': int(global_slot),
                    'pixel_x': int(cx_o),
                    'pixel_y': int(cy_o),
                    'warped_pixel_x': int(cx_w),
                    'warped_pixel_y': int(cy_w),
                    'depth_mm': float(depth_val),
                    'depth_diff_mm': float(depth_diff),
                    'depth_cv_occupied': bool(depth_occupied),
                    'color_min_b': None if np.isnan(min_b) else float(min_b),
                    'color_hue_cv2': None if np.isnan(hue_val) else float(hue_val),
                    'color_rule1_dark': bool(color_rule1),
                    'color_rule2_hue': bool(color_rule2),
                    'color_occupied': bool(color_occupied),
                    'occupied': bool(occupied),
                    'camera_frame_xyz': {
                        'x': float(t_pn[0]),
                        'y': float(t_pn[1]),
                        'z': float(t_pn[2]),
                    },
                })
                if occupied:
                    all_detected.append(global_slot)

            debug_trays.append(tray_debug)

        if static_slot_transforms:
            self.static_br.sendTransform(static_slot_transforms)

        cv2.imwrite(str(self.debug_output_dir / "debug_slots.jpg"), overlay)
        with open(self.debug_output_dir / 'debug_slot_geometry.json', 'w', encoding='utf-8') as f:
            json.dump(
                {
                    'camera_frame': self.camera_frame,
                    'detected_slots': [int(slot) for slot in all_detected],
                    'color_detection': {
                        'enabled': bool(self.enable_color_detection),
                        'interior_x_min': float(self.color_interior_x_min),
                        'interior_x_max': float(self.color_interior_x_max),
                        'hue_x': float(self.color_hue_x),
                        'dark_abs_thresh': float(self.dark_abs_thresh),
                        'single_thresh': float(self.single_thresh),
                        'hue_low_cv2': float(self.hue_low_cv2),
                        'hue_high_cv2': float(self.hue_high_cv2),
                    },
                    'trays': debug_trays,
                },
                f,
                indent=2,
            )
        self.edge_pub.publish(self.cv2_to_ros_image(overlay))

        return all_detected


def main(args=None):
    rclpy.init(args=args)
    node = GSAMSlideDetectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
