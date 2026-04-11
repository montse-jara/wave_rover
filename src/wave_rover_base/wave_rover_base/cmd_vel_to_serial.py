#!/usr/bin/env python3

import json
import serial
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # Serial parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        # Robot tuning parameters
        self.declare_parameter('wheel_base', 0.23)          # meters
        self.declare_parameter('max_linear_mps', 0.18)      # slow top speed for Nav2
        self.declare_parameter('max_angular_radps', 0.80)   # clamp turn rate
        self.declare_parameter('angular_scale', 0.60)       # reduce turning aggressiveness

        # WAVE ROVER T:1 command range is typically around [-0.5, 0.5]
        # Keep this intentionally lower so the robot stays gentle.
        self.declare_parameter('output_limit', 0.22)        # max |L|, |R| sent over serial
        self.declare_parameter('min_output_cmd', 0.10)      # enough to make wheels move

        # Timing / smoothing
        self.declare_parameter('cmd_timeout', 0.50)         # stop if Nav2 goes silent
        self.declare_parameter('control_rate_hz', 20.0)     # smoother command updates
        self.declare_parameter('smoothing_alpha', 0.25)     # 0..1, lower = smoother/slower changes

        # Topic
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        self.output_limit = float(self.get_parameter('output_limit').value)
        self.min_output_cmd = float(self.get_parameter('min_output_cmd').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        # State
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_sent_left = None
        self.last_sent_right = None
        self.stopped = True

        # Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Opened serial port {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        # Subscriber
        self.sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Control loop timer
        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            'cmd_vel_to_serial started '
            f'(topic={cmd_vel_topic}, max_linear_mps={self.max_linear_mps}, '
            f'output_limit={self.output_limit}, min_output_cmd={self.min_output_cmd}, '
            f'smoothing_alpha={self.smoothing_alpha})'
        )

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def apply_deadband_float(self, value):
        """
        Keeps very small commands at zero, but bumps small nonzero values
        up to min_output_cmd so the wheels can actually start moving.
        """
        if abs(value) < 1e-4:
            return 0.0
        if abs(value) < self.min_output_cmd:
            return self.min_output_cmd if value > 0.0 else -self.min_output_cmd
        return value

    def smooth_value(self, current, target):
        """
        First-order smoothing:
        new = current + alpha * (target - current)
        Lower alpha = smoother, less jerky, but slower response.
        """
        return current + self.smoothing_alpha * (target - current)

    def cmd_vel_callback(self, msg: Twist):
        # Read Nav2/body command
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        # Clamp requested motion so Nav2 cannot demand too much
        linear_x = self.clamp(linear_x, -self.max_linear_mps, self.max_linear_mps)
        angular_z = self.clamp(angular_z, -self.max_angular_radps, self.max_angular_radps)

        self.target_linear_x = linear_x
        self.target_angular_z = angular_z
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False

    def twist_to_wheels(self, linear_x, angular_z):
        """
        Differential-drive conversion:
          left  = v - w * wheel_base / 2
          right = v + w * wheel_base / 2

        Then map wheel speeds into WAVE ROVER T:1 command space.
        """
        # Reduce turn aggressiveness slightly
        angular_z *= self.angular_scale

        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        # Clamp wheel speeds
        left_mps = self.clamp(left_mps, -self.max_linear_mps, self.max_linear_mps)
        right_mps = self.clamp(right_mps, -self.max_linear_mps, self.max_linear_mps)

        # Map wheel m/s into limited output range for WAVE ROVER
        # +/- max_linear_mps  -> +/- output_limit
        left_cmd = (left_mps / self.max_linear_mps) * self.output_limit
        right_cmd = (right_mps / self.max_linear_mps) * self.output_limit

        # Clamp one more time for safety
        left_cmd = self.clamp(left_cmd, -self.output_limit, self.output_limit)
        right_cmd = self.clamp(right_cmd, -self.output_limit, self.output_limit)

        # Apply a small minimum command so the wheels actually move
        left_cmd = self.apply_deadband_float(left_cmd)
        right_cmd = self.apply_deadband_float(right_cmd)

        # Round to reduce serial spam/noise
        left_cmd = round(left_cmd, 3)
        right_cmd = round(right_cmd, 3)

        return left_cmd, right_cmd

    def send_json(self, payload):
        line = json.dumps(payload, separators=(',', ':')) + '\n'
        self.ser.write(line.encode('utf-8'))

    def send_drive(self, linear_x, angular_z):
        left_cmd, right_cmd = self.twist_to_wheels(linear_x, angular_z)

        # Avoid resending identical commands over and over
        if left_cmd == self.last_sent_left and right_cmd == self.last_sent_right:
            return

        payload = {
            "T": 1,
            "L": left_cmd,
            "R": right_cmd
        }

        self.send_json(payload)
        self.last_sent_left = left_cmd
        self.last_sent_right = right_cmd

        self.get_logger().debug(f'Sent: {payload}')

    def send_stop(self):
        payload = {"T": 1, "L": 0.0, "R": 0.0}
        self.send_json(payload)
        self.last_sent_left = 0.0
        self.last_sent_right = 0.0
        self.get_logger().info('Sent STOP')

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        # Watchdog stop if Nav2/cmd_vel goes silent
        if elapsed > self.cmd_timeout:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0

        # Smooth toward target command
        self.current_linear_x = self.smooth_value(self.current_linear_x, self.target_linear_x)
        self.current_angular_z = self.smooth_value(self.current_angular_z, self.target_angular_z)

        # Snap tiny values to zero so robot settles cleanly
        if abs(self.current_linear_x) < 1e-3:
            self.current_linear_x = 0.0
        if abs(self.current_angular_z) < 1e-3:
            self.current_angular_z = 0.0

        # If timed out and already basically stopped, send one stop and stay quiet
        if elapsed > self.cmd_timeout:
            if not self.stopped and self.current_linear_x == 0.0 and self.current_angular_z == 0.0:
                self.send_stop()
                self.stopped = True
            return

        # Otherwise send smoothed drive command
        self.send_drive(self.current_linear_x, self.current_angular_z)
        self.stopped = False

    def destroy_node(self):
        try:
            self.send_stop()
        except Exception:
            pass

        try:
            if hasattr(self, 'ser') and self.ser.is_open:
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