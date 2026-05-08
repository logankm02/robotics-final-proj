#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from planning_interfaces.srv import WaferPickPlace
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException


class WaferPickPlaceRunner(Node):
    def __init__(self):
        super().__init__('wafer_pick_place_runner')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = self.create_client(WaferPickPlace, '/wafer_pick_place')
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/wafer_pick_place service is not available')

    def current_flange_pose(self, timeout_sec: float = 3.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base',
                    'link_6',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                pose = PoseStamped()
                pose.header.frame_id = 'base'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                return pose
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)
        return None

    def run(self, source_tray, dest_tray, num_slots, approach_clearance, lift_clearance):
        scan_pose = self.current_flange_pose()
        if scan_pose is None:
            raise RuntimeError('Could not determine current flange pose from TF')

        request = WaferPickPlace.Request()
        request.scan_pose = scan_pose
        request.source_tray_index = int(source_tray)
        request.dest_tray_index = int(dest_tray)
        request.num_slots = int(num_slots)
        request.approach_clearance = float(approach_clearance)
        request.lift_clearance = float(lift_clearance)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=300.0)
        response = future.result()
        if response is None:
            raise RuntimeError('/wafer_pick_place returned no response')

        print(
            f"success={response.success} wafers_picked={response.wafers_picked} "
            f"message={response.message}"
        )
        return 0 if response.success else 1


def main():
    parser = argparse.ArgumentParser(
        description='Call /wafer_pick_place using the robot’s current flange pose as scan_pose.'
    )
    parser.add_argument('--source-tray', type=int, default=0)
    parser.add_argument('--dest-tray', type=int, default=1)
    parser.add_argument('--num-slots', type=int, default=25)
    parser.add_argument('--approach-clearance', type=float, default=0.05)
    parser.add_argument('--lift-clearance', type=float, default=0.10)
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = WaferPickPlaceRunner()
        return node.run(
            source_tray=args.source_tray,
            dest_tray=args.dest_tray,
            num_slots=args.num_slots,
            approach_clearance=args.approach_clearance,
            lift_clearance=args.lift_clearance,
        )
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
