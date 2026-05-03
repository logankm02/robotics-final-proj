
import sys
# print("Current sys.path:", sys.path)
# sys.path.append('')

import argparse
import os
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

# RGB wafer detection constants (ported from pick_place.py, F1=1.0 on test image)
_INTERIOR_X      = (0.12, 0.88)   # exclude tray teeth on left/right
_BAND_RADIUS     = 12             # vertical half-height of each slot sample band
_X_HUE           = 0.45          # x fraction for hue sample (center of tray interior)
_DARK_ABS_THRESH = 42             # Rule 1: min brightness → group wafer
_SINGLE_THRESH   = 90             # Rule 2: brightness ceiling for single wafer
_HUE_LOW_CV2     = 70             # Rule 2: hue window low  (cv2 scale 0-180)
_HUE_HIGH_CV2    = 100            # Rule 2: hue window high


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
        self.declare_parameter('sam2_checkpoint', "/home/bryan/final_project_ws/src/perception/realsense_cv/models/sam2.1_hiera_small.pt")
        self.declare_parameter('sam2_model_config', "configs/sam2.1/sam2.1_hiera_s.yaml")
        self.declare_parameter('force_cpu', False)
        self.declare_parameter('num_slots', 25)
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('depth_diff_threshold', 10)  # mm shallower than tray floor = wafer

        # Get parameters
        self.grounding_model = self.get_parameter('grounding_model').value
        self.text_prompt = self.get_parameter('text_prompt').value
        self.sam2_checkpoint = self.get_parameter('sam2_checkpoint').value
        self.sam2_model_config = self.get_parameter('sam2_model_config').value
        self.camera_frame = self.get_parameter('camera_frame').value
        force_cpu = self.get_parameter('force_cpu').value
        self.num_slots = self.get_parameter('num_slots').value
        self.depth_diff_threshold = self.get_parameter('depth_diff_threshold').value
        input_topic = self.get_parameter('input_image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        output_topic = self.get_parameter('output_image_topic').value
        service_name = self.get_parameter('service_name').value
        info_topic = self.get_parameter('camera_info_topic').value


        self.br = StaticTransformBroadcaster(self)

        self.last_image = None
        self.last_msg = None
        self.last_depth = None

        self.camera_matrix = None
        self.caminfo_sub = self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        self.processing_image = False
        self.device = "cuda" if torch.cuda.is_available() and not force_cpu else "cpu"
        self.get_logger().info(f'Using device: {self.device}')

        torch.autocast(device_type=self.device, dtype=torch.bfloat16).__enter__()

        if torch.cuda.is_available() and torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        # Load models once at startup
        self.get_logger().info('Loading SAM2 and Grounding DINO models...')
        sam2_model = build_sam2(self.sam2_model_config, self.sam2_checkpoint, device=self.device)
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)
        self.processor = AutoProcessor.from_pretrained(self.grounding_model)
        self.grounding_model_net = AutoModelForZeroShotObjectDetection.from_pretrained(self.grounding_model).to(self.device)
        self.get_logger().info('Models loaded.')

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
    
    
    def deproject_pixel_to_3d(self,point):
        u, v = point
        K = self.camera_matrix
        if K is None:
            raise ValueError("Camera intrinsic matrix K is not set.")
        fx = K[0, 0]; fy = K[1, 1]
        cx = K[0, 2]; cy = K[1, 2]

        Z = 0.288
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

        cv2.imwrite("filtered_edges.png", output)
        return output.astype(bool)

    def publish_slide_frames(self, points, slotInd, msg=None):
        
        """
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
        T_slide_box = np.array([
            -0.00528*slotInd-0.01,
            0.0,
            0.0,
        ], dtype=np.float64)
        t_pn = R_box_cam @ T_slide_box + T_box_cam   # shape (3,)
        R_pn = R_box_cam

        # Convert to quaternion
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_pn)

        # Build and send TF
        tfmsg = TransformStamped()
        tfmsg.header.stamp = msg.header.stamp
        tfmsg.header.frame_id = self.camera_frame
        tfmsg.child_frame_id = f"slide_{slotInd:02d}"

        tfmsg.transform.translation.x = float(t_pn[0])
        tfmsg.transform.translation.y = float(t_pn[1])
        tfmsg.transform.translation.z = float(t_pn[2])

        tfmsg.transform.rotation.x = float(qx)
        tfmsg.transform.rotation.y = float(qy)
        tfmsg.transform.rotation.z = float(qz)
        tfmsg.transform.rotation.w = float(qw)

        self.br.sendTransform(tfmsg)
        return R_pn, t_pn
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

    # ------------------------------------------------------------------
    # RGB wafer detection helpers (ported from pick_place.py)
    # ------------------------------------------------------------------

    @staticmethod
    def _hue_at_patch(roi_rgb, cy, cx, r, h, w):
        patch = roi_rgb[max(0, cy-r):min(h, cy+r), max(0, cx-r):min(w, cx+r)]
        if patch.size == 0:
            return 0.0
        rp = patch[:,:,0].astype(np.float32)/255
        gp = patch[:,:,1].astype(np.float32)/255
        bp = patch[:,:,2].astype(np.float32)/255
        maxc = np.maximum(rp, np.maximum(gp, bp))
        diff = maxc - np.minimum(rp, np.minimum(gp, bp))
        hue = np.zeros_like(rp)
        mr = (maxc == rp) & (diff > 0)
        mg = (maxc == gp) & (diff > 0)
        mb = (maxc == bp) & (diff > 0)
        hue[mr] = (60.0 * ((gp[mr]-bp[mr])/diff[mr])) % 360
        hue[mg] = 60.0 * ((bp[mg]-rp[mg])/diff[mg]) + 120
        hue[mb] = 60.0 * ((rp[mb]-gp[mb])/diff[mb]) + 240
        return float(np.mean(hue)) / 2.0

    @staticmethod
    def _detect_wafers_rgb(img_rgb: np.ndarray, bbox: tuple, num_slots: int) -> list:
        """Return per-slot occupancy using RGB brightness + hue rules."""
        x1, y1, x2, y2 = bbox
        roi = img_rgb[y1:y2, x1:x2]
        h, w = roi.shape[:2]
        gray = np.mean(roi, axis=2).astype(np.float32)
        xi_s = int(_INTERIOR_X[0] * w)
        xi_e = int(_INTERIOR_X[1] * w)
        occupied = []
        for slot in range(1, num_slots + 1):
            t = slot / (num_slots + 1)
            cy = int(t * h)
            band = gray[max(0, cy-_BAND_RADIUS):min(h, cy+_BAND_RADIUS), xi_s:xi_e]
            if band.size == 0:
                occupied.append(False)
                continue
            min_b = float(np.min(np.mean(band, axis=0)))
            cx_hue = int(_X_HUE * w)
            hue = GSAMSlideDetectNode._hue_at_patch(roi, cy, cx_hue, _BAND_RADIUS, h, w)
            rule1 = min_b < _DARK_ABS_THRESH
            rule2 = (_HUE_LOW_CV2 < hue < _HUE_HIGH_CV2) and (min_b < _SINGLE_THRESH)
            occupied.append(rule1 or rule2)
        return occupied

    @staticmethod
    def _slot_pixel(bbox: tuple, slot: int, num_slots: int) -> tuple:
        """Pixel coords of slot center in the original image frame."""
        x1, y1, x2, y2 = bbox
        t = slot / (num_slots + 1)
        cy = int(y1 + t * (y2 - y1))
        cx = (x1 + x2) // 2
        return (cx, cy)

    @staticmethod
    def _mask_angle(mask: np.ndarray) -> float:
        """
        Return tray tilt in degrees from vertical using cv2.minAreaRect on the SAM2 mask.
        Positive = clockwise. 0 = perfectly upright.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0
        cnt = max(contours, key=cv2.contourArea)
        _, (w, h), angle = cv2.minAreaRect(cnt)
        # minAreaRect angle is in [-90, 0). For a tall box (h > w after swap),
        # add 90 so the angle describes tilt from vertical rather than from horizontal.
        if w > h:
            angle += 90.0
        return float(angle)

    def publish_pick_place(self, pick_xyz: tuple, place_xyz: tuple,
                           pick_angle_deg: float = 0.0, place_angle_deg: float = 0.0,
                           msg=None):
        """Publish pick_pose and place_pose as static TF frames in the camera frame.
        Rotation is a Z-axis yaw equal to the tray tilt angle from vertical."""
        if msg is None:
            msg = self.last_msg
        stamp = msg.header.stamp if msg else self.get_clock().now().to_msg()
        for name, xyz, angle_deg in [
            ("pick_pose",  pick_xyz,  pick_angle_deg),
            ("place_pose", place_xyz, place_angle_deg),
        ]:
            a = np.radians(angle_deg) / 2.0
            tfmsg = TransformStamped()
            tfmsg.header.stamp = stamp
            tfmsg.header.frame_id = self.camera_frame
            tfmsg.child_frame_id = name
            tfmsg.transform.translation.x = float(xyz[0])
            tfmsg.transform.translation.y = float(xyz[1])
            tfmsg.transform.translation.z = float(xyz[2])
            tfmsg.transform.rotation.x = 0.0
            tfmsg.transform.rotation.y = 0.0
            tfmsg.transform.rotation.z = float(np.sin(a))
            tfmsg.transform.rotation.w = float(np.cos(a))
            self.br.sendTransform(tfmsg)

    def detect_slides(self, img, text_prompt=None):
        """
        Detect occupied wafer slots using RGB brightness + hue rules (ported from pick_place.py).
        Identifies source tray (has wafers) and dest tray (empty), then computes pick/place
        pixel coords, depjects to 3D, and publishes pick_pose / place_pose TF frames.

        Returns:
            List of occupied slot numbers (1-indexed, global across all trays)
        """
        if text_prompt is None:
            text_prompt = self.text_prompt

        num_slots = self.num_slots
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        masks, det_boxes = self.gsam_mask(img, text=text_prompt)
        overlay = img.copy()
        all_detected = []

        tray_bboxes = []
        tray_occupancies = []
        tray_spatial_points = []
        tray_angles = []

        for tray_idx, (mask, det_box) in enumerate(zip(masks, det_boxes)):
            cv2.imwrite(f"debug_mask_tray{tray_idx}.png", mask)

            x1, y1, x2, y2 = det_box.astype(int)
            bbox = (x1, y1, x2, y2)
            tray_bboxes.append(bbox)

            p1 = [x1, y1]; p2 = [x2, y1]; p3 = [x1, y2]; p4 = [x2, y2]
            spatial_points = [self.deproject_pixel_to_3d(pt) for pt in [p1, p2, p3, p4]]
            tray_spatial_points.append(spatial_points)

            cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 0, 0), 3)

            tray_angles.append(self._mask_angle(mask))
            occupied = self._detect_wafers_rgb(img_rgb, bbox, num_slots)
            tray_occupancies.append(occupied)

            slot_offset = tray_idx * num_slots
            for i, occ in enumerate(occupied, 1):
                global_slot = slot_offset + i
                cx_o, cy_o = self._slot_pixel(bbox, i, num_slots)
                color = (0, 0, 255) if occ else (0, 255, 0)
                cv2.circle(overlay, (cx_o, cy_o), 8, color, 2)
                cv2.putText(overlay, str(global_slot), (cx_o - 8, cy_o - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                if occ:
                    all_detected.append(global_slot)
                    self.publish_slide_frames(spatial_points, global_slot)

            self.get_logger().info(
                f"Tray {tray_idx}: {sum(occupied)}/{num_slots} slots occupied"
            )

        # --- Pick / Place ---
        if len(tray_bboxes) >= 2:
            occ_counts = [sum(o) for o in tray_occupancies]
            src_idx  = int(np.argmax(occ_counts))
            dest_idx = 1 - src_idx

            src_filled = [s + 1 for s, o in enumerate(tray_occupancies[src_idx])  if o]
            dest_empty  = [s + 1 for s, o in enumerate(tray_occupancies[dest_idx]) if not o]

            if src_filled and dest_empty:
                pick_slot  = src_filled[0]
                place_slot = dest_empty[len(dest_empty) // 2]

                pick_px  = self._slot_pixel(tray_bboxes[src_idx],  pick_slot,  num_slots)
                place_px = self._slot_pixel(tray_bboxes[dest_idx], place_slot, num_slots)

                pick_xyz  = self.deproject_pixel_to_3d(pick_px)
                place_xyz = self.deproject_pixel_to_3d(place_px)

                pick_angle  = tray_angles[src_idx]
                place_angle = tray_angles[dest_idx]

                self.get_logger().info(
                    f"PICK  slot {pick_slot:>2} tray {src_idx}  "
                    f"px={pick_px}  xyz=({pick_xyz[0]:.4f}, {pick_xyz[1]:.4f}, {pick_xyz[2]:.4f})  "
                    f"angle={pick_angle:+.2f}°"
                )
                self.get_logger().info(
                    f"PLACE slot {place_slot:>2} tray {dest_idx}  "
                    f"px={place_px}  xyz=({place_xyz[0]:.4f}, {place_xyz[1]:.4f}, {place_xyz[2]:.4f})  "
                    f"angle={place_angle:+.2f}°"
                )

                self.publish_pick_place(pick_xyz, place_xyz, pick_angle, place_angle)

                cv2.drawMarker(overlay, pick_px,  (0, 255, 80),  cv2.MARKER_CROSS, 30, 3)
                cv2.drawMarker(overlay, place_px, (0, 180, 255), cv2.MARKER_CROSS, 30, 3)

        cv2.imwrite("debug_slots.jpg", overlay)
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
