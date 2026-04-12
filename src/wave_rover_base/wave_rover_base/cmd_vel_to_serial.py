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
        self.declare_parameter('port', '/dev/rover_base')
        self.declare_parameter('baud', 115200)

        # Command mode:
        #   T1  -> {"T":1,"L":float,"R":float}   range about [-0.5, 0.5]
        #   T11 -> {"T":11,"L":int,"R":int}      range [-255, 255]
        self.declare_parameter('command_mode', 'T1')

        # Robot tuning parameters
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('max_linear_mps', 0.20)
        self.declare_parameter('max_angular_radps', 1.00)
        self.declare_parameter('angular_scale', 0.85)

        # T1 output range
        self.declare_parameter('output_limit', 0.35)
        self.declare_parameter('min_output_cmd', 0.18)

        # T11 output range
        self.declare_parameter('pwm_limit', 220)
        self.declare_parameter('min_pwm_cmd', 120)

        # Timing / smoothing
        self.declare_parameter('cmd_timeout', 0.50)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('smoothing_alpha', 0.50)

        # Topic
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.command_mode = str(self.get_parameter('command_mode').value).upper()

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        self.output_limit = float(self.get_parameter('output_limit').value)
        self.min_output_cmd = float(self.get_parameter('min_output_cmd').value)

        self.pwm_limit = int(self.get_parameter('pwm_limit').value)
        self.min_pwm_cmd = int(self.get_parameter('min_pwm_cmd').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        if self.command_mode not in ('T1', 'T11'):
            raise ValueError(f"Unsupported command_mode '{self.command_mode}', use 'T1' or 'T11'")

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
            self.ser = serial.Serial(
                self.port,
                self.baud,
                dsrdtr=None,
                timeout=0.1
            )
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            self.get_logger().info(f'Opened serial port {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        # Optional: enable command echo on the rover
        try:
            self.send_json({"T": 143, "cmd": 1})
            self.get_logger().info('Enabled rover serial echo')
        except Exception as e:
            self.get_logger().warn(f'Could not enable rover echo: {e}')

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
            f'cmd_vel_to_serial started (mode={self.command_mode}, topic={cmd_vel_topic}, '
            f'max_linear_mps={self.max_linear_mps}, smoothing_alpha={self.smoothing_alpha})'
        )

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def apply_deadband_float(self, value):
        if abs(value) < 1e-4:
            return 0.0
        if abs(value) < self.min_output_cmd:
            return self.min_output_cmd if value > 0.0 else -self.min_output_cmd
        return value

    def apply_deadband_pwm(self, value):
        if value == 0:
            return 0
        if abs(value) < self.min_pwm_cmd:
            return self.min_pwm_cmd if value > 0 else -self.min_pwm_cmd
        return value

    def smooth_value(self, current, target):
        return current + self.smoothing_alpha * (target - current)

    def cmd_vel_callback(self, msg: Twist):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        linear_x = self.clamp(linear_x, -self.max_linear_mps, self.max_linear_mps)
        angular_z = self.clamp(angular_z, -self.max_angular_radps, self.max_angular_radps)

        self.target_linear_x = linear_x
        self.target_angular_z = angular_z
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False

    def twist_to_wheels(self, linear_x, angular_z):
        angular_z *= self.angular_scale

        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        left_mps = self.clamp(left_mps, -self.max_linear_mps, self.max_linear_mps)
        right_mps = self.clamp(right_mps, -self.max_linear_mps, self.max_linear_mps)

        return left_mps, right_mps

    def wheels_to_t1(self, left_mps, right_mps):
        left_cmd = (left_mps / self.max_linear_mps) * self.output_limit
        right_cmd = (right_mps / self.max_linear_mps) * self.output_limit

        left_cmd = self.clamp(left_cmd, -self.output_limit, self.output_limit)
        right_cmd = self.clamp(right_cmd, -self.output_limit, self.output_limit)

        left_cmd = self.apply_deadband_float(left_cmd)
        right_cmd = self.apply_deadband_float(right_cmd)

        return round(left_cmd, 3), round(right_cmd, 3)

    def wheels_to_t11(self, left_mps, right_mps):
        left_pwm = int(round((left_mps / self.max_linear_mps) * self.pwm_limit))
        right_pwm = int(round((right_mps / self.max_linear_mps) * self.pwm_limit))

        left_pwm = int(self.clamp(left_pwm, -self.pwm_limit, self.pwm_limit))
        right_pwm = int(self.clamp(right_pwm, -self.pwm_limit, self.pwm_limit))

        left_pwm = self.apply_deadband_pwm(left_pwm)
        right_pwm = self.apply_deadband_pwm(right_pwm)

        return left_pwm, right_pwm

    def send_json(self, payload):
        line = json.dumps(payload, separators=(',', ':')) + '\n'
        self.ser.write(line.encode('utf-8'))

    def send_drive(self, linear_x, angular_z):
        left_mps, right_mps = self.twist_to_wheels(linear_x, angular_z)

        if self.command_mode == 'T1':
            left_cmd, right_cmd = self.wheels_to_t1(left_mps, right_mps)
            payload = {"T": 1, "L": left_cmd, "R": right_cmd}
        else:
            left_cmd, right_cmd = self.wheels_to_t11(left_mps, right_mps)
            payload = {"T": 11, "L": left_cmd, "R": right_cmd}

        if left_cmd == self.last_sent_left and right_cmd == self.last_sent_right:
            return

        self.send_json(payload)
        self.last_sent_left = left_cmd
        self.last_sent_right = right_cmd

        self.get_logger().info(f'Sent: {payload}')

    def send_stop(self):
        if self.command_mode == 'T1':
            payload = {"T": 1, "L": 0.0, "R": 0.0}
            self.last_sent_left = 0.0
            self.last_sent_right = 0.0
        else:
            payload = {"T": 11, "L": 0, "R": 0}
            self.last_sent_left = 0
            self.last_sent_right = 0

        self.send_json(payload)
        self.get_logger().info('Sent STOP')

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0

        self.current_linear_x = self.smooth_value(self.current_linear_x, self.target_linear_x)
        self.current_angular_z = self.smooth_value(self.current_angular_z, self.target_angular_z)

        if abs(self.current_linear_x) < 1e-3:
            self.current_linear_x = 0.0
        if abs(self.current_angular_z) < 1e-3:
            self.current_angular_z = 0.0

        if elapsed > self.cmd_timeout:
            if not self.stopped and self.current_linear_x == 0.0 and self.current_angular_z == 0.0:
                self.send_stop()
                self.stopped = True
            return

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