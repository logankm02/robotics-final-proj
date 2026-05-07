#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive
from planning_interfaces.srv import (
    PickPlaceService,
    MoveToTarget,
    ContinuousPickPlace,
    WaferPickPlace,
)
from std_srvs.srv import SetBool, Trigger
import tf2_ros
from tf2_ros import TransformException

from planning.ik import IKPlanner
from realsense_cv.slide_detector import SlideDetector

import numpy as np
from scipy.spatial.transform import Rotation as R
import asyncio
import time
import re

# Gripper geometry (in meters). Total tool length below the flange,
# and the length of the soft pads at the end of the fingers.
# When the pad CENTER must coincide with the wafer center, the flange
# sits FLANGE_TO_GRIP_CENTER above the wafer pose Z.
GRIPPER_LENGTH = 0.14002          # 140.02 mm — flange to gripper tip
GRIPPER_PAD_LENGTH = 0.066        # 66 mm — length of grip pads
FLANGE_TO_GRIP_CENTER = GRIPPER_LENGTH - GRIPPER_PAD_LENGTH / 2.0  # ~0.107 m


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # Parameters
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('end_effector_link', 'link_6')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('approach_distance', 0.1)
        self.declare_parameter('z_velocity_scale', 0.2) # Slow, safe
        self.declare_parameter('xy_velocity_scale', 0.4) # Moderate
        self.declare_parameter('z_acceleration_scale', 0.2)
        self.declare_parameter('xy_acceleration_scale', 0.4)
        self.declare_parameter('gsam_service_wait_timeout', 120.0)
        self.declare_parameter('gsam_service_poll_period', 1.0)
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.z_vel_scale = self.get_parameter('z_velocity_scale').value
        self.xy_vel_scale = self.get_parameter('xy_velocity_scale').value
        self.z_accel_scale = self.get_parameter('z_acceleration_scale').value
        self.xy_accel_scale = self.get_parameter('xy_acceleration_scale').value
        self.gsam_service_wait_timeout = float(
            self.get_parameter('gsam_service_wait_timeout').value
        )
        self.gsam_service_poll_period = max(
            0.1,
            float(self.get_parameter('gsam_service_poll_period').value)
        )

        # Alignment method selection
        self.declare_parameter('alignment_method', 'perpendicular')
        self.alignment_method = self.get_parameter('alignment_method').value
        self.get_logger().info(f'Alignment method: {self.alignment_method}')

        # Detection mode selection
        self.declare_parameter('detection_mode', 'marker')  # 'marker' or 'gsam'
        self.detection_mode = self.get_parameter('detection_mode').value
        self.get_logger().info(f'Detection mode: {self.detection_mode}')
        
        # Validate detection mode (Only marker or gsam currently supported)
        if self.detection_mode not in ['marker', 'gsam']:
            self.get_logger().error(f'Invalid detection_mode: {self.detection_mode}')
            raise ValueError(f'Invalid detection_mode: {self.detection_mode}')

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # IK Planner
        self.ik_planner = IKPlanner()
        self.get_logger().info('IK Planner initialized')

        # Slide Detector
        self.slide_detector = SlideDetector()
        self.get_logger().info('Slide Detector initialized')

        # TF listener for robot state queries
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('TF listener initialized')

        # Joint state tracking
        self.current_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1
        )

        # Use MoveIt's action interface for plan+execute. Direct execution
        # of trajectories generated via /plan_kinematic_path was returning
        # CONTROL_FAILED on this TM driver even for trivial, reachable moves,
        # while MoveGroup execution succeeds on the same targets.
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup executor')

        self.joint_traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/tmr_arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.joint_traj_client.wait_for_server()
        self.get_logger().info('Connected to joint trajectory controller')

        # Gripper service client
        self.gripper_client = self.create_client(
            SetBool,
            '/gripper/control',
            callback_group=self.callback_group
        )
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Gripper service not available, will continue anyway')
        else:
            self.get_logger().info('Connected to gripper controller')

        # GSAM detection service client
        if self.detection_mode == 'gsam':
            self.gsam_client = self.create_client(
                Trigger,
                '/detect_slides',
                callback_group=self.callback_group
            )
            self.get_logger().info('GSAM slide detection client initialized')
            if self.gsam_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().info('Connected to GSAM slide detector')
            else:
                self.get_logger().warn(
                    'GSAM service not ready yet. '
                    'The node will wait for it when detection is requested.'
                )

        # Service for pick and place
        self.pick_place_srv = self.create_service(
            PickPlaceService,
            'pick_and_place',
            self.pick_place_callback,
            callback_group=self.callback_group
        )

        # Service for move to target
        self.move_to_target_srv = self.create_service(
            MoveToTarget,
            'move_to_target',
            self.move_to_target_callback,
            callback_group=self.callback_group
        )

        # Service for continuous pick and place
        self.continuous_srv = self.create_service(
            ContinuousPickPlace,
            'continuous_pick_place',
            self.continuous_pick_place_callback,
            callback_group=self.callback_group
        )

        # Service for container-to-container wafer pick and place
        self.wafer_pick_place_srv = self.create_service(
            WaferPickPlace,
            'wafer_pick_place',
            self.wafer_pick_place_callback,
            callback_group=self.callback_group
        )

        self.processing = False

        self.get_logger().info('Ready! Pick-and-Place node initialized')

    async def detect_slides_gsam(self):
        """
        Call GSAM service to detect slides in storage box.
        
        Returns:
            List of slot numbers (e.g., [3, 7, 12, 18]), or empty list if failed
        """
        try:
            if not self.wait_for_gsam_service():
                return []

            request = Trigger.Request()
            
            self.get_logger().info('Calling GSAM slide detection service...')
            future = self.gsam_client.call_async(request)
            response = await future
            
            if not response.success:
                self.get_logger().error(f'GSAM detection failed: {response.message}')
                return []
            
            # Parse response message
            # Format: "Detected slides at slots: [3, 7, 12, 18]"
            self.get_logger().info(f'GSAM result: {response.message}')
            
            match = re.search(r'\[([\d, ]+)\]', response.message)
            if match:
                numbers_str = match.group(1).strip()
                if numbers_str:
                    slot_numbers = [int(x.strip()) for x in numbers_str.split(',')]
                    self.get_logger().info(f'Detected slides in {len(slot_numbers)} slots: {slot_numbers}')
                    return slot_numbers
                else:
                    # Empty brackets []
                    self.get_logger().info('No slides detected (empty box)')
                    return []
            else:
                self.get_logger().warn('Could not parse slot numbers from GSAM response')
                return []
                
        except Exception as e:
            self.get_logger().error(f'Exception calling GSAM service: {e}')
            import traceback
            traceback.print_exc()
            return []

    def wait_for_gsam_service(self):
        """Wait for the GSAM service to become ready."""
        if self.detection_mode != 'gsam':
            return True

        if self.gsam_client.service_is_ready():
            return True

        timeout = self.gsam_service_wait_timeout
        start_time = time.time()
        self.get_logger().info('Waiting for GSAM slide detection service...')

        while rclpy.ok():
            if self.gsam_client.wait_for_service(timeout_sec=self.gsam_service_poll_period):
                waited = time.time() - start_time
                self.get_logger().info(
                    f'Connected to GSAM slide detector after {waited:.1f}s'
                )
                return True

            waited = time.time() - start_time
            if timeout > 0.0 and waited >= timeout:
                self.get_logger().error(
                    'GSAM service /detect_slides not available after '
                    f'{timeout:.1f}s'
                )
                return False

            self.get_logger().warn(
                f'GSAM service still loading after {waited:.1f}s; waiting...'
            )

        return False

    def normalize_quaternion(self, quat):
        """
        Normalize a quaternion to ensure it has unit length.

        Args:
            quat: Array-like [x, y, z, w] or geometry_msgs Quaternion

        Returns:
            Normalized quaternion as numpy array [x, y, z, w]
        """
        if hasattr(quat, 'x'):
            # geometry_msgs.msg.Quaternion
            q = np.array([quat.x, quat.y, quat.z, quat.w])
        else:
            q = np.array(quat)

        norm = np.linalg.norm(q)
        if norm < 1e-10:
            self.get_logger().warn('Quaternion norm too small, using identity quaternion')
            return np.array([0.0, 0.0, 0.0, 1.0])

        return q / norm

    def print_joint_state(self, prev_js,js):
        self.get_logger().info('prev Joint State:')
        for n,p in zip(prev_js.name,prev_js.position):
            self.get_logger().info(f'  {n}: {p:.4f}')
        self.get_logger().info('new Joint State:')
        for n,p in zip(js.name,js.position):
            self.get_logger().info(f'  {n}: {p:.4f}')

    def joint_state_callback(self, msg):
        """Store latest joint state"""
        self.current_joint_state = msg

    async def move_to_target_callback(self, request, response):
        """Service callback to move to target position"""
        if self.processing:
            response.success = False
            response.message = "Already processing"
            return response

        self.processing = True

        try:
            # Normalize target pose quaternion
            target_quat_norm = self.normalize_quaternion(request.target_pose.pose.orientation)
            request.target_pose.pose.orientation.x = target_quat_norm[0]
            request.target_pose.pose.orientation.y = target_quat_norm[1]
            request.target_pose.pose.orientation.z = target_quat_norm[2]
            request.target_pose.pose.orientation.w = target_quat_norm[3]

            success = await self.move_to_target(request.target_pose, velocity_scale=self.xy_vel_scale, acceleration_scale=self.xy_accel_scale)
            response.success = success
            response.message = "Success" if success else "Failed to reach target"
        except Exception as e:
            self.get_logger().error(f'Exception in move_to_target: {e}')
            response.success = False
            response.message = f"Exception: {str(e)}"
        finally:
            self.processing = False

        return response

    async def pick_place_callback(self, request, response):
        """Execute pick and place using IK + motion planning"""
        if self.processing:
            response.success = False
            response.message = "Already processing"
            return response

        if self.current_joint_state is None:
            response.success = False
            response.message = "No joint state available"
            return response

        self.processing = True
        
        try:
            pick_pose = request.pick_pose
            place_pose = request.place_pose

            # Normalize quaternions for pick and place poses
            pick_quat_norm = self.normalize_quaternion(pick_pose.pose.orientation)
            pick_pose.pose.orientation.x = pick_quat_norm[0]
            pick_pose.pose.orientation.y = pick_quat_norm[1]
            pick_pose.pose.orientation.z = pick_quat_norm[2]
            pick_pose.pose.orientation.w = pick_quat_norm[3]

            place_quat_norm = self.normalize_quaternion(place_pose.pose.orientation)
            place_pose.pose.orientation.x = place_quat_norm[0]
            place_pose.pose.orientation.y = place_quat_norm[1]
            place_pose.pose.orientation.z = place_quat_norm[2]
            place_pose.pose.orientation.w = place_quat_norm[3]

            self.get_logger().info('='*60)
            self.get_logger().info('Starting Pick and Place (IK-based)')
            self.get_logger().info(f'Pick:  x={pick_pose.pose.position.x:.3f}, '
                                 f'y={pick_pose.pose.position.y:.3f}, '
                                 f'z={pick_pose.pose.position.z:.3f}')
            self.get_logger().info(f'Place: x={place_pose.pose.position.x:.3f}, '
                                 f'y={place_pose.pose.position.y:.3f}, '
                                 f'z={place_pose.pose.position.z:.3f}')
            self.get_logger().info('='*60)

            # Build job queue
            job_queue = []

            # 1. Compute IK for pick approach
            self.get_logger().info('Step 1/8: Computing IK for pick approach...')
            
            pick_approach = self.get_approach_pose(pick_pose)
            ik_result = self.ik_planner.compute_ik(
                self.current_joint_state,
                pick_approach.pose.position.x,
                pick_approach.pose.position.y,
                pick_approach.pose.position.z,
                pick_approach.pose.orientation.x,
                pick_approach.pose.orientation.y,
                pick_approach.pose.orientation.z,
                pick_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick approach"
                return response
            job_queue.append((pick_approach, self.xy_vel_scale, self.xy_accel_scale))

            # 2. Compute IK for pick position
            self.get_logger().info('Step 2/8: Computing IK for pick...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                pick_pose.pose.position.x,
                pick_pose.pose.position.y,
                pick_pose.pose.position.z,
                pick_pose.pose.orientation.x,
                pick_pose.pose.orientation.y,
                pick_pose.pose.orientation.z,
                pick_pose.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick"
                return response
            job_queue.append((pick_pose, self.z_vel_scale, self.z_accel_scale))

            # 3. Close gripper
            job_queue.append('close_gripper')

            # 4. Retreat
            self.get_logger().info('Step 4/8: Computing IK for pick retreat...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                pick_approach.pose.position.x,
                pick_approach.pose.position.y,
                pick_approach.pose.position.z,
                pick_approach.pose.orientation.x,
                pick_approach.pose.orientation.y,
                pick_approach.pose.orientation.z,
                pick_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for pick retreat"
                return response
            job_queue.append((pick_approach, self.z_vel_scale, self.z_accel_scale))

            # 5. Place approach
            self.get_logger().info('Step 5/8: Computing IK for place approach...')
            place_approach = self.get_approach_pose(place_pose)
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_approach.pose.position.x,
                place_approach.pose.position.y,
                place_approach.pose.position.z,
                place_approach.pose.orientation.x,
                place_approach.pose.orientation.y,
                place_approach.pose.orientation.z,
                place_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place approach"
                return response
            job_queue.append((place_approach, self.xy_vel_scale, self.xy_accel_scale))

            # 6. Place position
            self.get_logger().info('Step 6/8: Computing IK for place...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_pose.pose.position.x,
                place_pose.pose.position.y,
                place_pose.pose.position.z,
                place_pose.pose.orientation.x,
                place_pose.pose.orientation.y,
                place_pose.pose.orientation.z,
                place_pose.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place"
                return response
            job_queue.append((place_pose, self.z_vel_scale, self.z_accel_scale))

            # 7. Open gripper
            job_queue.append('open_gripper')

            # 8. Retreat
            self.get_logger().info('Step 8/8: Computing IK for place retreat...')
            ik_result = self.ik_planner.compute_ik(
                ik_result,
                place_approach.pose.position.x,
                place_approach.pose.position.y,
                place_approach.pose.position.z,
                place_approach.pose.orientation.x,
                place_approach.pose.orientation.y,
                place_approach.pose.orientation.z,
                place_approach.pose.orientation.w
            )
            if ik_result is None:
                response.success = False
                response.message = "IK failed for place retreat"
                return response
            job_queue.append((place_approach, self.z_vel_scale, self.z_accel_scale))

            # Execute all jobs
            success = await self.execute_job_queue(job_queue)
            
            if success:
                self.get_logger().info('='*60)
                self.get_logger().info('Pick and Place Complete!')
                self.get_logger().info('='*60)
                response.success = True
                response.message = "Success"
            else:
                response.success = False
                response.message = "Execution failed"

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            response.success = False
            response.message = f"Exception: {str(e)}"
        finally:
            self.processing = False

        return response

    async def continuous_pick_place_callback(self, request, response):
        """
        Executes pick-and-place cycles until no more slides are detected.
        """
        
        if self.processing:
            response.success = False
            response.message = "Already processing"
            response.slides_picked = 0
            return response
        
        if self.current_joint_state is None:
            response.success = False
            response.message = "No joint state available"
            response.slides_picked = 0
            return response
        
        self.processing = True
        slides_picked = 0

        # Extract parameters
        pick_scan = request.pick_scan_pose
        place_scan = request.place_scan_pose
        pick_dist = request.pick_distance
        retreat_dist = request.retreat_distance
        place_dist = request.place_distance
        place_rotation_y = request.place_rotation_y_deg

        # Normalize quaternions for scan poses
        pick_scan_quat_norm = self.normalize_quaternion(pick_scan.pose.orientation)
        pick_scan.pose.orientation.x = pick_scan_quat_norm[0]
        pick_scan.pose.orientation.y = pick_scan_quat_norm[1]
        pick_scan.pose.orientation.z = pick_scan_quat_norm[2]
        pick_scan.pose.orientation.w = pick_scan_quat_norm[3]

        place_scan_quat_norm = self.normalize_quaternion(place_scan.pose.orientation)
        place_scan.pose.orientation.x = place_scan_quat_norm[0]
        place_scan.pose.orientation.y = place_scan_quat_norm[1]
        place_scan.pose.orientation.z = place_scan_quat_norm[2]
        place_scan.pose.orientation.w = place_scan_quat_norm[3]

        self.get_logger().info('='*60)
        self.get_logger().info('CONTINUOUS PICK-AND-PLACE STARTED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Detection mode: {self.detection_mode.upper()}')
        self.get_logger().info(f'Alignment method: {self.alignment_method}')
        self.get_logger().info(f'Pick distance: {pick_dist}m (TODO: TUNE)')
        self.get_logger().info(f'Retreat distance: {retreat_dist}m (TODO: TUNE)')
        self.get_logger().info(f'Place distance: {place_dist}m (TODO: TUNE)')
        self.get_logger().info(f'Place rotation (Y-axis): {place_rotation_y}° (TODO: TUNE)')
        self.get_logger().info('='*60)

        try:
            # ========================================
            # GSAM MODE: Call detection service once
            # ========================================
            if self.detection_mode == 'gsam':
                self.get_logger().info('')
                self.get_logger().info('='*60)
                self.get_logger().info('STEP 1: Move to pick scan pose for GSAM detection')
                self.get_logger().info('='*60)
                
                if not await self.move_to_target(pick_scan, velocity_scale=self.xy_vel_scale, acceleration_scale=self.xy_accel_scale):
                    response.success = False
                    response.message = "Failed to reach pick scan pose"
                    response.slides_picked = 0
                    return response
                
                # Brief pause for camera to stabilize
                time.sleep(0.5)
                
                # Call GSAM detection
                self.get_logger().info('='*60)
                self.get_logger().info('STEP 2: Calling GSAM detection')
                self.get_logger().info('='*60)
                
                detected_slots = await self.detect_slides_gsam()
                
                if not detected_slots:
                    self.get_logger().info('No slides detected by GSAM - operation complete')
                    response.success = True
                    response.message = "No slides detected"
                    response.slides_picked = 0
                    self.processing = False
                    return response
                
                # Update slide detector with detected slot numbers
                self.slide_detector.set_slide_indices(detected_slots)
                self.get_logger().info(f'GSAM detected slides in slots: {detected_slots}')
                self.get_logger().info(f'Will process {len(detected_slots)} slides')
                
                # Wait a moment for TF frames to be published
                time.sleep(0.2)
            
            # ========================================
            # MAIN LOOP: Pick each detected slide
            # ========================================
            while True:
                self.get_logger().info('')
                self.get_logger().info('='*60)
                self.get_logger().info(f'CYCLE {slides_picked + 1}')
                self.get_logger().info('='*60)
                
                # ========================================
                # MARKER MODE: Move to scan pose each cycle
                # ========================================
                if self.detection_mode == 'marker':
                    self.get_logger().info('Step 1: Moving to pick scan pose...')
                    if not await self.move_to_target(pick_scan, velocity_scale=self.xy_vel_scale, acceleration_scale=self.xy_accel_scale):
                        response.success = False
                        response.message = "Failed to reach pick scan pose"
                        response.slides_picked = slides_picked
                        return response
                    
                    # Brief pause for camera/marker detection to stabilize
                    time.sleep(0.5)
                
                # ========================================
                # DETECT SLIDE (both modes use slide_detector)
                # ========================================
                self.get_logger().info(f'Step 2: Detecting slide (mode: {self.detection_mode})...')
                slide_pose = self.slide_detector.wait_for_slide(timeout=10.0)
                
                if slide_pose is None:
                    self.get_logger().info('No more slides detected - operation complete')
                    break
                
                self.get_logger().info(f'Slide detected at ({slide_pose.pose.position.x:.3f}, '
                                     f'{slide_pose.pose.position.y:.3f}, '
                                     f'{slide_pose.pose.position.z:.3f})')
                
                # ========================================
                # BUILD JOB QUEUE FOR THIS SLIDE
                # ========================================
                self.get_logger().info('Building job queue for this slide...')
                
                job_queue = []
                current_state = self.current_joint_state
                
                # ========================================
                # Job 1: Align with slide
                # Use direct method for GSAM, perpendicular for marker (or as configured)
                # ========================================
                alignment_method = 'direct' if self.detection_mode == 'gsam' else self.alignment_method
                self.get_logger().info(f'  Job 1/9: Computing align pose (method: {alignment_method})...')
                
                align_pose = self.compute_alignment_pose(slide_pose, method=alignment_method)
                                
                ik_align = self.ik_planner.compute_ik(
                    current_state,
                    align_pose.pose.position.x,
                    align_pose.pose.position.y,
                    align_pose.pose.position.z,
                    align_pose.pose.orientation.x,
                    align_pose.pose.orientation.y,
                    align_pose.pose.orientation.z,
                    align_pose.pose.orientation.w
                )
                if ik_align is None:
                    self.get_logger().error('IK failed for align pose - skipping this slide')
                    continue
                job_queue.append((align_pose, self.xy_vel_scale, self.xy_accel_scale))
                current_state = ik_align
                
                # Job 2: Lower for grasp
                self.get_logger().info(f'  Job 2/9: Computing lower ({pick_dist}m) - TODO: TUNE')
                lower_pose = self.offset_pose_z(align_pose, -pick_dist)
                
                ik_lower = self.ik_planner.compute_ik(
                    current_state,
                    lower_pose.pose.position.x,
                    lower_pose.pose.position.y,
                    lower_pose.pose.position.z,
                    lower_pose.pose.orientation.x,
                    lower_pose.pose.orientation.y,
                    lower_pose.pose.orientation.z,
                    lower_pose.pose.orientation.w
                )
                if ik_lower is None:
                    self.get_logger().error('IK failed for lower pose - skipping this slide')
                    continue
                job_queue.append((lower_pose, self.z_vel_scale, self.z_accel_scale))
                current_state = ik_lower
                
                # Job 3: Close gripper
                self.get_logger().info('  Job 3/9: Close gripper')
                job_queue.append('close_gripper')
                
                # Job 4: Lift after grasp
                self.get_logger().info(f'  Job 4/9: Computing lift ({retreat_dist}m) - TODO: TUNE')
                lift_pose = self.offset_pose_z(lower_pose, retreat_dist)
                
                ik_lift = self.ik_planner.compute_ik(
                    current_state,
                    lift_pose.pose.position.x,
                    lift_pose.pose.position.y,
                    lift_pose.pose.position.z,
                    lift_pose.pose.orientation.x,
                    lift_pose.pose.orientation.y,
                    lift_pose.pose.orientation.z,
                    lift_pose.pose.orientation.w
                )
                if ik_lift is None:
                    self.get_logger().error('IK failed for lift pose - skipping this slide')
                    continue
                job_queue.append((lift_pose, self.xy_vel_scale, self.xy_accel_scale))
                current_state = ik_lift
                
                # Job 5: Move to place scan pose
                self.get_logger().info('  Job 5/9: Computing place scan pose...')
                ik_place_scan = self.ik_planner.compute_ik(
                    current_state,
                    place_scan.pose.position.x,
                    place_scan.pose.position.y,
                    place_scan.pose.position.z,
                    place_scan.pose.orientation.x,
                    place_scan.pose.orientation.y,
                    place_scan.pose.orientation.z,
                    place_scan.pose.orientation.w
                )
                if ik_place_scan is None:
                    self.get_logger().error('IK failed for place scan pose - skipping this slide')
                    continue
                job_queue.append((place_scan, self.xy_vel_scale, self.xy_accel_scale))
                current_state = ik_place_scan
                
                # Job 6: Align with target (for now, same as place_scan)
                self.get_logger().info('  Job 6/9: Target align (same as scan for now)')
                # Already there from job 5
                
                # Job 7: Lower for place
                self.get_logger().info(f'  Job 7/9: Computing place lower ({place_dist}m, rot={place_rotation_y}°) - TODO: TUNE')
                place_lower_pose = self.offset_pose_z(place_scan, -place_dist, rotation_y_deg=place_rotation_y)
                
                ik_place_lower = self.ik_planner.compute_ik(
                    current_state,
                    place_lower_pose.pose.position.x,
                    place_lower_pose.pose.position.y,
                    place_lower_pose.pose.position.z,
                    place_lower_pose.pose.orientation.x,
                    place_lower_pose.pose.orientation.y,
                    place_lower_pose.pose.orientation.z,
                    place_lower_pose.pose.orientation.w
                )
                if ik_place_lower is None:
                    self.get_logger().error('IK failed for place lower - skipping this slide')
                    continue
                job_queue.append((place_lower_pose, self.z_vel_scale, self.z_accel_scale))
                current_state = ik_place_lower
                
                # Job 8: Open gripper
                self.get_logger().info('  Job 8/9: Open gripper')
                job_queue.append('open_gripper')
                
                # Job 9: Lift after place
                self.get_logger().info(f'  Job 9/9: Computing final lift ({retreat_dist}m)...')
                final_lift_pose = self.offset_pose_z(place_lower_pose, retreat_dist)
                
                ik_final_lift = self.ik_planner.compute_ik(
                    current_state,
                    final_lift_pose.pose.position.x,
                    final_lift_pose.pose.position.y,
                    final_lift_pose.pose.position.z,
                    final_lift_pose.pose.orientation.x,
                    final_lift_pose.pose.orientation.y,
                    final_lift_pose.pose.orientation.z,
                    final_lift_pose.pose.orientation.w
                )
                if ik_final_lift is None:
                    self.get_logger().error('IK failed for final lift - skipping this slide')
                    continue
                job_queue.append((final_lift_pose, self.xy_vel_scale, self.xy_accel_scale))
                
                # ========================================
                # EXECUTE QUEUE
                # ========================================
                self.get_logger().info(f'All IK solutions found! Queue has {len(job_queue)} jobs')
                self.get_logger().info('Executing queue...')
                
                success = await self.execute_job_queue(job_queue)
                
                if success:
                    slides_picked += 1
                    self.get_logger().info(f'Slide {slides_picked} complete!')
                else:
                    self.get_logger().error('Execution failed for this slide')
                
                # Brief pause before next slide
                time.sleep(1.0)
            
            # All done
            self.get_logger().info('='*60)
            self.get_logger().info(f'OPERATION COMPLETE - Successfully picked {slides_picked} slides')
            self.get_logger().info('='*60)
            response.success = True
            response.message = f"Successfully picked {slides_picked} slides"
            response.slides_picked = slides_picked
            
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            import traceback
            traceback.print_exc()
            response.success = False
            response.message = f"Exception after {slides_picked} slides: {str(e)}"
            response.slides_picked = slides_picked
        
        finally:
            self.processing = False

        return response

    async def wafer_pick_place_callback(self, request, response):
        """
        Container-to-container wafer pick-and-place.

        Steps:
          1. Move to scan_pose (also serves as starting/home position).
          2. Trigger GSAM detection — both trays are seen in the same frame,
             which makes every slot's TF available afterwards.
          3. For each occupied slot S in the SOURCE tray, compute the
             matching slot in the DESTINATION tray and move:
                 src_approach -> src_grasp -> close
                 -> src_retreat -> dst_approach -> dst_place -> open
                 -> dst_retreat
          4. Return to scan_pose.

        Z values are computed ABSOLUTELY using the wafer's TF Z and the
        gripper geometry (FLANGE_TO_GRIP_CENTER), so we don't need a tuned
        `pick_distance` parameter.
        """
        if self.processing:
            response.success = False
            response.message = "Already processing"
            response.wafers_picked = 0
            return response

        if self.current_joint_state is None:
            response.success = False
            response.message = "No joint state available"
            response.wafers_picked = 0
            return response

        if self.detection_mode != 'gsam':
            response.success = False
            response.message = (
                "wafer_pick_place requires detection_mode='gsam' "
                f"(currently '{self.detection_mode}'). Restart with that param."
            )
            response.wafers_picked = 0
            return response

        self.processing = True
        wafers_picked = 0

        scan_pose = request.scan_pose
        src_tray = int(request.source_tray_index)
        dst_tray = int(request.dest_tray_index)
        num_slots = int(request.num_slots) if request.num_slots > 0 else 25
        approach_clearance = float(request.approach_clearance) if request.approach_clearance > 0 else 0.05
        lift_clearance = float(request.lift_clearance) if request.lift_clearance > 0 else 0.10

        if src_tray == dst_tray:
            response.success = False
            response.message = "source_tray_index must differ from dest_tray_index"
            response.wafers_picked = 0
            self.processing = False
            return response

        # Normalize scan-pose quaternion
        q = self.normalize_quaternion(scan_pose.pose.orientation)
        scan_pose.pose.orientation.x = q[0]
        scan_pose.pose.orientation.y = q[1]
        scan_pose.pose.orientation.z = q[2]
        scan_pose.pose.orientation.w = q[3]

        self.get_logger().info('='*60)
        self.get_logger().info('WAFER CONTAINER-TO-CONTAINER PICK-AND-PLACE')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Source tray: {src_tray}   Destination tray: {dst_tray}')
        self.get_logger().info(f'Slots per tray: {num_slots}')
        self.get_logger().info(f'Approach clearance: {approach_clearance:.3f} m')
        self.get_logger().info(f'Lift clearance:     {lift_clearance:.3f} m')
        self.get_logger().info(f'Gripper length: {GRIPPER_LENGTH*1000:.2f} mm, '
                               f'flange→grip center: {FLANGE_TO_GRIP_CENTER*1000:.1f} mm')
        self.get_logger().info('='*60)

        try:
            # 1. Move to scan/home pose
            self.get_logger().info('STEP 1: Move to scan/home pose')
            if not await self.move_to_target(scan_pose,
                                             velocity_scale=self.xy_vel_scale,
                                             acceleration_scale=self.xy_accel_scale):
                response.success = False
                response.message = "Failed to reach scan pose"
                response.wafers_picked = 0
                return response

            time.sleep(0.5)  # let camera settle

            # 2. GSAM detection — populates slide_XX TFs for every slot in every tray
            self.get_logger().info('STEP 2: Trigger GSAM detection')
            detected_slots = await self.detect_slides_gsam()
            if not detected_slots:
                self.get_logger().info('No wafers detected — operation complete')
                response.success = True
                response.message = "No wafers detected"
                response.wafers_picked = 0
                self.processing = False
                return response

            # Filter to source-tray slots only.
            src_slots = sorted([s for s in detected_slots
                                if (s - 1) // num_slots == src_tray])
            if not src_slots:
                self.get_logger().info(
                    f'GSAM saw {detected_slots} but none are in source tray {src_tray}'
                )
                response.success = True
                response.message = "No wafers in source tray"
                response.wafers_picked = 0
                self.processing = False
                return response

            self.get_logger().info(
                f'STEP 3: {len(src_slots)} wafer(s) in source tray: {src_slots}'
            )

            time.sleep(0.3)  # let TF frames propagate

            # 3. Pick each wafer and place it in the matching slot of dst tray.
            for src_slot in src_slots:
                local_slot = ((src_slot - 1) % num_slots) + 1
                dst_slot = dst_tray * num_slots + local_slot

                self.get_logger().info('')
                self.get_logger().info('-' * 60)
                self.get_logger().info(
                    f'WAFER {wafers_picked+1}/{len(src_slots)}: '
                    f'src slide_{src_slot:02d} -> dst slide_{dst_slot:02d}'
                )
                self.get_logger().info('-' * 60)

                src_pose = self.slide_detector.get_slide_pose(src_slot, timeout=2.0)
                if src_pose is None:
                    self.get_logger().error(
                        f'Could not lookup source TF slide_{src_slot:02d}, skipping'
                    )
                    continue

                dst_pose = self.slide_detector.get_slide_pose(dst_slot, timeout=2.0)
                if dst_pose is None:
                    self.get_logger().error(
                        f'Could not lookup destination TF slide_{dst_slot:02d}, skipping'
                    )
                    continue

                # Build job queue for this wafer
                job_queue = self._build_wafer_job_queue(
                    src_pose, dst_pose, approach_clearance, lift_clearance
                )
                if job_queue is None:
                    self.get_logger().error('IK failed building queue, skipping wafer')
                    continue

                self.get_logger().info(f'Executing {len(job_queue)} jobs')
                if await self.execute_job_queue(job_queue):
                    wafers_picked += 1
                    self.get_logger().info(f'Wafer {wafers_picked} placed!')
                else:
                    self.get_logger().error('Execution failed for this wafer')

                time.sleep(0.5)

            # 4. Return to scan/home pose
            self.get_logger().info('STEP 4: Return to scan/home pose')
            await self.move_to_target(scan_pose,
                                      velocity_scale=self.xy_vel_scale,
                                      acceleration_scale=self.xy_accel_scale)

            self.get_logger().info('='*60)
            self.get_logger().info(f'DONE. Picked & placed {wafers_picked} wafer(s).')
            self.get_logger().info('='*60)
            response.success = True
            response.message = f"Picked {wafers_picked} wafer(s)"
            response.wafers_picked = wafers_picked

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            import traceback
            traceback.print_exc()
            response.success = False
            response.message = f"Exception after {wafers_picked} wafer(s): {e}"
            response.wafers_picked = wafers_picked

        finally:
            self.processing = False

        return response

    def _build_wafer_job_queue(self, src_pose, dst_pose,
                               approach_clearance, lift_clearance):
        """
        Build the IK job queue for one src->dst wafer transfer.
        Returns a list of (joint_state, vel, accel) tuples and gripper commands,
        or None if any IK step fails.
        """
        queue = []
        current = self.current_joint_state

        # Use 'direct' alignment for both source and destination — keeps the
        # gripper jaws aligned with the slot's long axis. (This was the
        # recommended method for GSAM in the original pipeline.)
        # Pass current_z=0.0 so compute_alignment_pose doesn't bother
        # looking up the live link_6 TF; we overwrite Z anyway.
        align_method = 'direct'

        src_align = self.compute_alignment_pose(src_pose, current_z=0.0, method=align_method)
        dst_align = self.compute_alignment_pose(dst_pose, current_z=0.0, method=align_method)

        # ---- SOURCE SIDE ----
        # Approach above source slot
        src_approach = self._wafer_pose_at(src_align, src_pose,
                                           offset_z=FLANGE_TO_GRIP_CENTER + approach_clearance)
        ik = self._ik(current, src_approach)
        if ik is None: return None
        queue.append((src_approach, self.xy_vel_scale, self.xy_accel_scale))
        current = ik

        # Lower to grasp height
        src_grasp = self._wafer_pose_at(src_align, src_pose,
                                        offset_z=FLANGE_TO_GRIP_CENTER)
        ik = self._ik(current, src_grasp)
        if ik is None: return None
        queue.append((src_grasp, self.z_vel_scale, self.z_accel_scale))
        current = ik

        queue.append('close_gripper')

        # Lift up after grasp (transport height)
        src_lift = self._wafer_pose_at(src_align, src_pose,
                                       offset_z=FLANGE_TO_GRIP_CENTER + lift_clearance)
        ik = self._ik(current, src_lift)
        if ik is None: return None
        queue.append((src_lift, self.z_vel_scale, self.z_accel_scale))
        current = ik

        # ---- TRANSPORT to destination approach ----
        dst_approach = self._wafer_pose_at(dst_align, dst_pose,
                                           offset_z=FLANGE_TO_GRIP_CENTER + approach_clearance)
        ik = self._ik(current, dst_approach)
        if ik is None: return None
        queue.append((dst_approach, self.xy_vel_scale, self.xy_accel_scale))
        current = ik

        # Lower to place height
        dst_place = self._wafer_pose_at(dst_align, dst_pose,
                                        offset_z=FLANGE_TO_GRIP_CENTER)
        ik = self._ik(current, dst_place)
        if ik is None: return None
        queue.append((dst_place, self.z_vel_scale, self.z_accel_scale))
        current = ik

        queue.append('open_gripper')

        # Lift up after place
        dst_lift = self._wafer_pose_at(dst_align, dst_pose,
                                       offset_z=FLANGE_TO_GRIP_CENTER + lift_clearance)
        ik = self._ik(current, dst_lift)
        if ik is None: return None
        queue.append((dst_lift, self.z_vel_scale, self.z_accel_scale))

        return queue

    def _wafer_pose_at(self, align_pose, slide_pose, offset_z):
        """
        Build a target flange PoseStamped at (slide.x, slide.y, slide.z + offset_z)
        with the orientation from `align_pose` (already aligned with the slot).
        """
        out = PoseStamped()
        out.header = align_pose.header
        out.pose.position.x = slide_pose.pose.position.x
        out.pose.position.y = slide_pose.pose.position.y
        out.pose.position.z = slide_pose.pose.position.z + offset_z
        out.pose.orientation = align_pose.pose.orientation
        return out

    def _ik(self, seed_state, pose_stamped):
        """Thin wrapper around ik_planner.compute_ik using a PoseStamped."""
        return self.ik_planner.compute_ik(
            seed_state,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        )

    def compute_alignment_pose(self, slide_pose, current_z=None, method=None):
        """
        Compute flange pose aligned with slide.

        Position: slide XY, keep current Z height (from pick_scan_pose)

        Args:
            slide_pose: PoseStamped of detected slide
            current_z: Current Z height (if None, gets from robot TF)
            method: Alignment method ('perpendicular' or 'direct')
                    If None, uses self.alignment_method parameter

        Methods:
            'perpendicular' -> Flange Y aligned with Slide X
            'direct' - Flange X -> Slide X, Flange Y -> Slide Y

        Returns:
            PoseStamped for flange alignment pose
        """
        # Use parameter if method not specified
        if method is None:
            method = self.alignment_method

        # Get current Z from robot TF if not provided
        if current_z is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base',
                    'link_6',
                    rclpy.time.Time()
                )
                current_z = transform.transform.translation.z
                self.get_logger().info(f'Using current Z height: {current_z:.3f}m')
            except (TransformException, AttributeError) as e:
                self.get_logger().error(f'FATAL: Cannot get current Z position from TF: {e}')
                raise RuntimeError(f'Failed to lookup transform base->link_6: {e}')

        # Extract slide orientation
        slide_quat = np.array([
            slide_pose.pose.orientation.x,
            slide_pose.pose.orientation.y,
            slide_pose.pose.orientation.z,
            slide_pose.pose.orientation.w
        ])

        slide_rot = R.from_quat(slide_quat)
        slide_matrix = slide_rot.as_matrix()
        slide_x_axis = slide_matrix[:, 0]  # First column = X-axis
        slide_y_axis = slide_matrix[:, 1]  # Second column = Y-axis

        # Z-axis: always pointing down (both methods)
        flange_z = np.array([0, 0, -1])

        # Compute flange axes based on method
        if method == 'direct':
            # DIRECT: Flange X->Slide X, Flange Y->Slide Y
            self.get_logger().debug('Using DIRECT alignment')

            # X-axis: aligned with slide X-axis, projected to horizontal
            flange_x = slide_x_axis.copy()
            flange_x[2] = 0.0
            norm_x = np.linalg.norm(flange_x)
            if norm_x < 1e-6:
                self.get_logger().warn('Slide X-axis vertical, using world X fallback')
                flange_x = np.array([1, 0, 0])
            else:
                flange_x /= norm_x

            # Y-axis: aligned with slide Y-axis, projected to horizontal
            flange_y = slide_y_axis.copy()
            flange_y[2] = 0.0
            norm_y = np.linalg.norm(flange_y)
            if norm_y < 1e-6:
                self.get_logger().warn('Slide Y-axis vertical, computing from X and Z')
                flange_y = np.cross(flange_z, flange_x)
            else:
                flange_y /= norm_y

            # Re-orthogonalize
            flange_x = np.cross(flange_y, flange_z)
            flange_x /= np.linalg.norm(flange_x)
            flange_y = np.cross(flange_z, flange_x)

        else:  # 'perpendicular' (default)
            # PERPENDICULAR: Flange Y->Slide X
            self.get_logger().debug('Using PERPENDICULAR alignment')

            # Y-axis: aligned with slide X-axis, projected to horizontal
            flange_y = slide_x_axis.copy()
            flange_y[2] = 0.0
            norm_y = np.linalg.norm(flange_y)
            if norm_y < 1e-6:
                self.get_logger().warn('Slide X-axis vertical, using world Y fallback')
                flange_y = np.array([0, 1, 0])
            else:
                flange_y /= norm_y

            # X-axis: Y cross Z (right-hand rule)
            flange_x = np.cross(flange_y, flange_z)
            flange_x /= np.linalg.norm(flange_x)

            # Recompute Y to ensure orthogonality
            flange_y = np.cross(flange_z, flange_x)

        # Build rotation matrix
        flange_rot_mat = np.column_stack([flange_x, flange_y, flange_z])
        flange_rot = R.from_matrix(flange_rot_mat)
        flange_quat = flange_rot.as_quat()

        # Normalize quaternion
        flange_quat_norm = self.normalize_quaternion(flange_quat)

        # Build pose
        align_pose = PoseStamped()
        align_pose.header.frame_id = 'base'
        align_pose.header.stamp = self.get_clock().now().to_msg()
        align_pose.pose.position.x = slide_pose.pose.position.x
        align_pose.pose.position.y = slide_pose.pose.position.y
        align_pose.pose.position.z = current_z
        align_pose.pose.orientation.x = flange_quat_norm[0]
        align_pose.pose.orientation.y = flange_quat_norm[1]
        align_pose.pose.orientation.z = flange_quat_norm[2]
        align_pose.pose.orientation.w = flange_quat_norm[3]

        return align_pose
    
    def offset_pose_z(self, pose, delta_z, rotation_y_deg=0.0):
        """
        Create new pose with Z offset and optional rotation around flange Y-axis.

        Args:
            pose: Original PoseStamped
            delta_z: Z offset in meters (positive = up, negative = down)
            rotation_y_deg: Rotation around flange Y-axis in degrees (default: 0.0)

        Returns:
            New PoseStamped with Z offset and rotation applied
        """
        new_pose = PoseStamped()
        new_pose.header = pose.header
        new_pose.pose.position.x = pose.pose.position.x
        new_pose.pose.position.y = pose.pose.position.y
        new_pose.pose.position.z = pose.pose.position.z + delta_z

        # If no rotation, just copy orientation
        if abs(rotation_y_deg) < 1e-6:
            new_pose.pose.orientation = pose.pose.orientation
        else:
            # Get original orientation
            orig_quat = np.array([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ])
            orig_rot = R.from_quat(orig_quat)

            # Create rotation around Y-axis (in flange frame)
            delta_rot = R.from_euler('y', rotation_y_deg, degrees=True)

            # Apply rotation: new = original * delta (right multiply for local frame rotation)
            new_rot = orig_rot * delta_rot
            new_quat = new_rot.as_quat()

            # Normalize and set
            new_quat_norm = self.normalize_quaternion(new_quat)
            new_pose.pose.orientation.x = new_quat_norm[0]
            new_pose.pose.orientation.y = new_quat_norm[1]
            new_pose.pose.orientation.z = new_quat_norm[2]
            new_pose.pose.orientation.w = new_quat_norm[3]

        return new_pose

    async def execute_job_queue(self, job_queue):
        """Execute all jobs in the queue with configurable speeds"""
        for i, job in enumerate(job_queue):
            self.get_logger().info(f'Executing job {i+1}/{len(job_queue)}...')

            # Handle tuple (PoseStamped, velocity, acceleration)
            if isinstance(job, tuple) and len(job) == 3:
                target, vel_scale, accel_scale = job

                if isinstance(target, PoseStamped):
                    if not await self.move_to_target(
                        target,
                        velocity_scale=vel_scale,
                        acceleration_scale=accel_scale
                    ):
                        return False

                elif isinstance(target, JointState):
                    trajectory = self.ik_planner.plan_to_joints(
                        target,
                        start_joint_state=self.current_joint_state,
                        velocity_scale=vel_scale,
                        acceleration_scale=accel_scale
                    )
                    if trajectory is None:
                        self.get_logger().error('Planning failed for queued joint target')
                        return False

                    if not await self.execute_joint_trajectory(trajectory):
                        return False

                    self.print_joint_state(self.current_joint_state, target)

                else:
                    self.get_logger().error(f'Unsupported queued target type: {type(target)}')
                    return False

            # Handle old-style JointState (backwards compatibility)
            elif isinstance(job, JointState):
                trajectory = self.ik_planner.plan_to_joints(
                    job,
                    start_joint_state=self.current_joint_state,
                    velocity_scale=self.xy_vel_scale,
                    acceleration_scale=self.xy_accel_scale
                )
                if trajectory is None:
                    self.get_logger().error('Planning failed for queued joint target')
                    return False

                if not await self.execute_joint_trajectory(trajectory):
                    return False

                self.print_joint_state(self.current_joint_state, job)
                   
            elif job == 'close_gripper':
                self.get_logger().info('Closing gripper...')
                if not await self.control_gripper(True):
                    self.get_logger().error('Failed to close gripper')
                    return False

            elif job == 'open_gripper':
                self.get_logger().info('Waiting 500ms before opening gripper...')
                time.sleep(0.5)  # 500ms delay to let slide settle
                self.get_logger().info('Opening gripper...')
                if not await self.control_gripper(False):
                    self.get_logger().error('Failed to open gripper')
                    return False
                
        return True

    async def execute_pose_goal(self, target_pose,
                                velocity_scale=0.2,
                                acceleration_scale=0.2):
        """Plan and execute a pose goal through MoveGroup."""
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity_scale
        goal.request.max_acceleration_scaling_factor = acceleration_scale

        goal_constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = target_pose.header.frame_id
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        pos_constraint.constraint_region.primitives[0].dimensions = [0.01]
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = target_pose.header.frame_id
        ori_constraint.link_name = self.end_effector_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(goal_constraints)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_handle = await self.move_group_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result = await goal_handle.get_result_async()
        if result.result.error_code.val != 1:
            self.get_logger().error(
                f'MoveGroup execution failed with code {result.result.error_code.val}'
            )
            return False

        return True

    async def execute_joint_trajectory(self, trajectory):
        """Execute a planned joint trajectory on the TM arm controller."""
        joint_traj = trajectory.joint_trajectory
        if not joint_traj.points:
            self.get_logger().error('Planned joint trajectory is empty')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        goal_handle = await self.joint_traj_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error('Joint trajectory goal rejected')
            return False

        result = await goal_handle.get_result_async()
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                'Joint trajectory execution failed with code '
                f'{result.result.error_code}: {result.result.error_string}'
            )
            return False

        return True

    async def control_gripper(self, close: bool):
        """Control gripper: True to close, False to open"""
        if not self.gripper_client.service_is_ready():
            self.get_logger().warn('Gripper service not available')
            return True  # Continue anyway

        request = SetBool.Request()
        request.data = close

        future = self.gripper_client.call_async(request)
        response = await future

        if response.success:
            action = "closed" if close else "opened"
            self.get_logger().info(f'Gripper {action}: {response.message}')
            return True
        else:
            self.get_logger().error(f'Gripper control failed: {response.message}')
            return False

    def get_approach_pose(self, target_pose):
        """Get approach pose above target"""
        approach = PoseStamped()
        approach.header = target_pose.header
        approach.pose.position.x = target_pose.pose.position.x
        approach.pose.position.y = target_pose.pose.position.y
        approach.pose.position.z = target_pose.pose.position.z + self.approach_distance
        approach.pose.orientation = target_pose.pose.orientation
        return approach

    async def move_to_target(self, target_pose, velocity_scale=0.2, acceleration_scale=0.2):
        """Move end effector to target position using IK"""
        if self.current_joint_state is None:
            self.get_logger().error('No joint state available')
            return False

        try:
            self.get_logger().info('='*60)
            self.get_logger().info('Moving to target position (IK-based)')
            self.get_logger().info(f'Target: x={target_pose.pose.position.x:.3f}, '
                                 f'y={target_pose.pose.position.y:.3f}, '
                                 f'z={target_pose.pose.position.z:.3f}')
            self.get_logger().info(f'Speed: velocity={velocity_scale*100:.0f}%, accel={acceleration_scale*100:.0f}%')
            self.get_logger().info('='*60)

            # Compute IK for target position
            self.get_logger().info('Computing IK for target...')
            ik_result = self.ik_planner.compute_ik(
                self.current_joint_state,
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w
            )

            if ik_result is None:
                self.get_logger().error('IK failed for target position')
                return False

            trajectory = self.ik_planner.plan_to_joints(
                ik_result,
                start_joint_state=self.current_joint_state,
                velocity_scale=velocity_scale,
                acceleration_scale=acceleration_scale
            )
            if trajectory is None:
                self.get_logger().error('Joint-space planning failed for target position')
                return False

            self.get_logger().info('Executing planned joint trajectory...')
            success = await self.execute_joint_trajectory(trajectory)

            if success:
                self.get_logger().info('='*60)
                self.get_logger().info('Move to target complete!')
                self.get_logger().info('='*60)

            return success

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    
    node.executor = executor
    executor.add_node(node)
    executor.add_node(node.slide_detector)  # Add SlideDetector to executor for TF callbacks

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
