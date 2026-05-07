#!/usr/bin/env python3
"""
Wafer Container-to-Container Pick-and-Place launch file.

Brings up:
  - RealSense D435i camera (RGB + aligned depth)
  - Camera-to-flange static TF
  - GSAM slide detection service (publishes a slide_XX TF for every slot
    in every detected tray, occupied or empty)
  - Slide detector helper (TF lookup)
  - Gripper controller (Arduino over USB)
  - Pick-and-place node with `wafer_pick_place` service

Call the service with a scan pose where the camera sees BOTH trays:

    ros2 service call /wafer_pick_place \
        planning_interfaces/srv/WaferPickPlace \
        "{scan_pose: { ... }, source_tray_index: 0, dest_tray_index: 1, \
          num_slots: 25, approach_clearance: 0.05, lift_clearance: 0.10}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gripper_port_arg = DeclareLaunchArgument(
        'gripper_port',
        default_value='/dev/ttyCH341USB0',
        description='Serial port for Arduino gripper controller'
    )
    gripper_port = LaunchConfiguration('gripper_port')

    # If True, the gripper node will fail hard when the Arduino is not
    # reachable. Default False so the rest of the pipeline can be exercised
    # without the gripper plugged in. Set to True for a real run.
    require_gripper_arg = DeclareLaunchArgument(
        'require_gripper',
        default_value='false',
        description='Fail launch if the gripper hardware is not available'
    )
    require_gripper = LaunchConfiguration('require_gripper')

    # If GPU memory is tight (Jetson with 8 GB unified RAM running VS Code,
    # browser, etc.), pass force_gsam_cpu:=true to load SAM2 + Grounding DINO
    # on CPU instead. Slower but won't OOM.
    force_gsam_cpu_arg = DeclareLaunchArgument(
        'force_gsam_cpu',
        default_value='false',
        description='Force the GSAM detector to run on CPU instead of CUDA'
    )
    force_gsam_cpu = LaunchConfiguration('force_gsam_cpu')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            # 1280x720x30 instead of 1920x1080 — the higher RGB profile
            # combined with depth + IR1 + IR2 + alignment exceeds what
            # this Jetson's USB chain delivers reliably; the D435i
            # browns out and re-enumerates mid-stream. GSAM/SAM2
            # resize internally so we lose nothing useful.
            'rgb_camera.color_profile': '1280x720x30',
            'align_depth.enable': 'true',
        }.items(),
    )

    camera_tf_node = Node(
        package='planning',
        executable='camera_tf',
        name='camera_tf',
        output='screen',
    )

    gsam_detect_node = Node(
        package='realsense_cv',
        executable='gsam_slide_detect',
        name='gsam_slide_detect',
        output='screen',
        parameters=[{
            'text_prompt': 'colored box.',
            'grounding_model': 'IDEA-Research/grounding-dino-tiny',
            # Checkpoint lives in the GSAM workspace, not under realsense_cv.
            'sam2_checkpoint': '/home/nano/CV/GSAM/checkpoints/sam2.1_hiera_small.pt',
            'sam2_model_config': 'configs/sam2.1/sam2.1_hiera_s.yaml',
            'num_slots': 25,
            'force_cpu': force_gsam_cpu,
        }]
    )

    gripper_node = Node(
        package='actuation',
        executable='gripper',
        name='gripper',
        output='screen',
        parameters=[{
            'serial_port': gripper_port,
            'require_hardware': require_gripper,
        }]
    )

    pick_and_place_node = Node(
        package='planning',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
        parameters=[{
            'detection_mode': 'gsam',
            # 'direct' aligns flange axes with the slot axes, which is the
            # right mode for the gsam_slide_detect frame convention.
            'alignment_method': 'direct',
        }]
    )

    # NOTE: deliberately not using a "shutdown if any node exits" handler.
    # During testing without the Arduino, the gripper node may exit; we
    # don't want that to tear down the camera / detector / planner.

    return LaunchDescription([
        gripper_port_arg,
        require_gripper_arg,
        force_gsam_cpu_arg,
        realsense_launch,
        camera_tf_node,
        gsam_detect_node,
        gripper_node,
        pick_and_place_node,
    ])
