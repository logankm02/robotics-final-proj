#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('grip_delay', 0.5)
        self.declare_parameter('ack_timeout', 1.0)
        # When True (default), missing/unreachable hardware is fatal. When
        # False, the node logs a warning and exposes /gripper/control as a
        # no-op so the rest of the pipeline can be exercised without the
        # Arduino connected.
        self.declare_parameter('require_hardware', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.grip_delay = self.get_parameter('grip_delay').value
        self.ack_timeout = float(self.get_parameter('ack_timeout').value)
        self.require_hardware = self.get_parameter('require_hardware').value

        self.serial = None  # populated only on successful open
        try:
            self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino reset

            if self.serial.in_waiting:
                ready_msg = self.serial.readline().decode().strip()
                self.get_logger().info(f'Arduino: {ready_msg}')

        except Exception as e:
            if self.require_hardware:
                self.get_logger().error(f'Failed to connect to Arduino: {e}')
                raise
            self.get_logger().warn(
                f'Gripper hardware not available at {self.serial_port}: {e}. '
                f'Running in NO-OP mode (require_hardware=False).'
            )
            self.serial = None

        # Initialize to open position (no-op if no hardware)
        self.open_gripper()
        
        # Create service
        self.gripper_service = self.create_service(
            SetBool,
            '/gripper/control',
            self.gripper_service_callback
        )
        
        self.get_logger().info('=' * 40)
        self.get_logger().info('Gripper Controller Initialized')
        self.get_logger().info(f'Serial Port: {self.serial_port}')
        self.get_logger().info('=' * 40)
    
    def send_command(self, cmd):
        """Send command to Arduino and wait briefly for an acknowledgement."""
        if self.serial is None:
            self.get_logger().info(f'[NO-OP] would send {cmd!r} to gripper')
            return 'NO-OP'
        try:
            self.serial.reset_input_buffer()
            self.serial.write(cmd.encode())
            self.serial.flush()
            deadline = time.time() + self.ack_timeout
            while time.time() < deadline:
                if self.serial.in_waiting:
                    response = self.serial.readline().decode(errors='replace').strip()
                    if response:
                        self.get_logger().info(f'Arduino: {response}')
                        return response
                time.sleep(0.05)
            self.get_logger().warn(
                f'No acknowledgement from Arduino after {self.ack_timeout:.1f}s '
                f'for command {cmd!r}'
            )
            return None

        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return None

    @staticmethod
    def _ack_matches(response, expected):
        if response is None:
            return False
        normalized = response.strip().upper()
        return normalized == expected or expected in normalized

    def open_gripper(self):
        """Open gripper."""
        self.get_logger().info('Opening gripper...')
        response = self.send_command('O')
        time.sleep(self.grip_delay)
        if response == 'NO-OP':
            self.get_logger().info('Gripper open skipped (NO-OP mode).')
            return True
        if self._ack_matches(response, 'OPEN'):
            self.get_logger().info('Gripper open confirmed.')
            return True
        self.get_logger().warn('Gripper open command was not confirmed by hardware.')
        return False

    def close_gripper(self):
        """Close gripper."""
        self.get_logger().info('Closing gripper...')
        response = self.send_command('C')
        time.sleep(self.grip_delay)
        if response == 'NO-OP':
            self.get_logger().info('Gripper close skipped (NO-OP mode).')
            return True
        if self._ack_matches(response, 'CLOSED'):
            self.get_logger().info('Gripper close confirmed.')
            return True
        self.get_logger().warn('Gripper close command was not confirmed by hardware.')
        return False
    
    def gripper_service_callback(self, request, response):
        """Main gripper control service."""
        try:
            if request.data:
                success = self.close_gripper()
                response.message = 'Gripper closed' if success else 'Gripper close not confirmed'
            else:
                success = self.open_gripper()
                response.message = 'Gripper opened' if success else 'Gripper open not confirmed'
            
            response.success = success
            
        except Exception as e:
            self.get_logger().error(f'Gripper control failed: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down...')

        # Close serial connection if open
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        # rclpy's default SIGINT handler may have already shut the context
        # down (especially under launch), so guard the second shutdown call.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
