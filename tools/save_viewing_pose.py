#!/usr/bin/env python3
"""
save_viewing_pose.py - Capture the robot's current joint configuration and
write it to src/planning/config/viewing_pose.yaml as the named viewing/home
pose used by pick_and_place.

The TM driver must be running and streaming /joint_states.

Usage:
  1. Jog the TM12 to the desired viewing pose on the pendant.
  2. python3 tools/save_viewing_pose.py
  3. Relaunch pick_and_place with the captured pose:
       ros2 run planning pick_and_place --ros-args \
         --params-file src/planning/config/viewing_pose.yaml \
         -p detection_mode:=gsam -p alignment_method:=direct
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
                   'joint_4', 'joint_5', 'joint_6']

# <repo-root>/src/planning/config/viewing_pose.yaml
DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'src', 'planning', 'config', 'viewing_pose.yaml',
)


class JointGrabber(Node):
    def __init__(self):
        super().__init__('save_viewing_pose')
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg):
        self.latest = msg

    def grab(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None:
                return self.latest
        return None


def main():
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_YAML

    rclpy.init()
    node = JointGrabber()
    try:
        msg = node.grab()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if msg is None:
        print('ERROR: no /joint_states received in 5s - is the TM driver running?',
              file=sys.stderr)
        return 1

    pos_by_name = dict(zip(msg.name, msg.position))
    try:
        ordered = [float(pos_by_name[n]) for n in ARM_JOINT_NAMES]
    except KeyError as exc:
        print(f'ERROR: joint {exc} missing from /joint_states', file=sys.stderr)
        return 1

    header = [
        '# Named "home / viewing" joint configuration for the TM12 (radians).',
        '#',
        '# This is the pose the arm returns to between picks in the',
        '# /wafer_pick_place loop and the viewpoint GSAM detection runs from.',
        '# It is also reachable on demand via the /go_to_viewing_pose service.',
        '#',
        '# Re-capture: jog the arm, then `python3 tools/save_viewing_pose.py`.',
        '#',
        f'# Captured: {time.strftime("%Y-%m-%d %H:%M:%S")}',
        '# Joint order: joint_1 .. joint_6',
        '/pick_and_place:',
        '  ros__parameters:',
        '    use_viewing_joint_pose: true',
        '    viewing_joint_positions:',
    ]
    body = [f'      - {v!r}' for v in ordered]
    content = '\n'.join(header + body) + '\n'

    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
    with open(yaml_path, 'w') as f:
        f.write(content)

    print(f'Saved viewing pose to {yaml_path}')
    for n, v in zip(ARM_JOINT_NAMES, ordered):
        print(f'  {n}: {v}')
    print()
    print('Relaunch pick_and_place with:')
    print('  ros2 run planning pick_and_place --ros-args \\')
    print(f'    --params-file {yaml_path} \\')
    print('    -p detection_mode:=gsam -p alignment_method:=direct')
    print()
    print('NOTE: also paste these into DEFAULT_VIEWING_JOINT_POSITIONS in')
    print('src/planning/planning/pick_and_place.py if you want them as the')
    print('built-in default (no --params-file needed).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
