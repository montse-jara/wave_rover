#!/usr/bin/env python3
import json
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_timeout', 0.5)

        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('max_linear_mps', 0.25)
        self.declare_parameter('max_wheel_cmd', 40)
        self.declare_parameter('min_wheel_cmd', 18)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_wheel_cmd = int(self.get_parameter('max_wheel_cmd').value)
        self.min_wheel_cmd = int(self.get_parameter('min_wheel_cmd').value)

        self.ser = serial.Serial(port, baud, timeout=1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.last_cmd_time = self.get_clock().now()
        self.stopped = True

        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        self.get_logger().info(f'cmd_timeout={self.cmd_timeout}s')
        self.get_logger().info(
            f'wheel_base={self.wheel_base}, max_linear_mps={self.max_linear_mps}, '
            f'max_wheel_cmd={self.max_wheel_cmd}, min_wheel_cmd={self.min_wheel_cmd}'
        )

    def send_json(self, payload):
        data = (json.dumps(payload) + '\n').encode('utf-8')
        self.ser.write(data)
        self.get_logger().info(f"Sent: {payload}")

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def apply_deadband(self, val):
        if val == 0:
            return 0
        sign = 1 if val > 0 else -1
        mag = abs(val)
        if mag < self.min_wheel_cmd:
            mag = self.min_wheel_cmd
        return sign * mag

    def twist_to_wheels(self, linear_x, angular_z):
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        left_cmd = int(round((left_mps / self.max_linear_mps) * self.max_wheel_cmd))
        right_cmd = int(round((right_mps / self.max_linear_mps) * self.max_wheel_cmd))

        left_cmd = self.clamp(left_cmd, -self.max_wheel_cmd, self.max_wheel_cmd)
        right_cmd = self.clamp(right_cmd, -self.max_wheel_cmd, self.max_wheel_cmd)

        left_cmd = self.apply_deadband(left_cmd)
        right_cmd = self.apply_deadband(right_cmd)

        if abs(linear_x) < 1e-4 and abs(angular_z) < 1e-4:
            left_cmd = 0
            right_cmd = 0

        return left_cmd, right_cmd

    def send_drive(self, linear_x, angular_z):
        left_cmd, right_cmd = self.twist_to_wheels(linear_x, angular_z)
        payload = {
            "T": 1,
            "L": int(left_cmd),
            "R": int(right_cmd)
        }
        self.send_json(payload)

    def send_stop(self):
        self.send_json({"T": 1, "L": 0, "R": 0})

    def cmd_vel_callback(self, msg: Twist):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        self.last_cmd_time = self.get_clock().now()
        self.send_drive(linear_x, angular_z)

        is_zero = abs(linear_x) < 1e-4 and abs(angular_z) < 1e-4
        self.stopped = is_zero

    def watchdog_callback(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout and not self.stopped:
            self.get_logger().warn('cmd_vel timeout reached, sending stop')
            self.send_stop()
            self.stopped = True

    def destroy_node(self):
        try:
            self.send_stop()
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()