#!/usr/bin/env python3
# Standard libraries
import json
import serial
# ROS2 Libraries
import rclpy
from rclpy.node import Node
# Velocity Commands
from geometry_msgs.msg import Twist


class CmdVelToSerial(Node):
    def __init__(self):
        #Initialize class as a ROS2 node named "cmd_vel_to_serial"
        super().__init__('cmd_vel_to_serial')

        # Serial parameters
        # How Rubik Pi connects to WaveRover over USB
        # Device path
        self.declare_parameter('port', '/dev/rover_base')
        # Serial baud rate
        self.declare_parameter('baud', 115200)
        # Rover protocol mode (T1 is used for the WaveRover (Speed from -.5 - .5 in commands))
        self.declare_parameter('command_mode', 'T1')

        # Optional sign / wiring corrections
        # Useful for bug testing inverts directions or angles
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('swap_left_right', False)
        self.declare_parameter('invert_angular', False)

        # Robot geometry / limits
        # Defines max speeds and robot geometry
        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('max_linear_mps', 0.16)
        self.declare_parameter('max_angular_radps', 1.20)
        self.declare_parameter('angular_scale', 1.6)

        # T1 command scaling
        # T1 uses outputs from -0.5 to 0.5
        self.declare_parameter('output_limit', 0.50)
        # Prevents motors from stalling at low speeds
        # Fixes issue with robot not moving at lower speeds
        self.declare_parameter('min_output_cmd', 0.18)

        # T11 command scaling (kept for completeness)
        # Uses PWM values does not work as well for wave rover
        self.declare_parameter('pwm_limit', 140)
        self.declare_parameter('min_pwm_cmd', 60)

        # Timing / smoothing
        # Stop robot if no command is recieved
        self.declare_parameter('cmd_timeout', 1.0)
        # Control loop frequency
        self.declare_parameter('control_rate_hz', 20.0)
        # Low-pass filter for smoothing
        self.declare_parameter('smoothing_alpha', 0.65)

        # Forward-drive boost in wheel-speed domain (m/s)
        # Helps not have robot stall when moving forward slowly
        self.declare_parameter('forward_boost_cmd', 0.08)
        self.declare_parameter('forward_boost_linear_threshold', 0.03)
        self.declare_parameter('forward_boost_angular_threshold', 0.12)

        # Turn-in-place boost in wheel-speed domain (m/s)
        # Allows robot to turn in place without stalling
        self.declare_parameter('turn_in_place_boost', 0.12)
        self.declare_parameter('turn_in_place_linear_threshold', 0.03)
        self.declare_parameter('turn_in_place_angular_threshold', 0.12)

        # Topic
        # ROS topic where velocity commands are recieved
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Read parameters
        # Gets all parameters and loads them into local variables
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.command_mode = str(self.get_parameter('command_mode').value).upper()

        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.swap_left_right = bool(self.get_parameter('swap_left_right').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)

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

        self.forward_boost_cmd = float(self.get_parameter('forward_boost_cmd').value)
        self.forward_boost_linear_threshold = float(
            self.get_parameter('forward_boost_linear_threshold').value
        )
        self.forward_boost_angular_threshold = float(
            self.get_parameter('forward_boost_angular_threshold').value
        )

        self.turn_in_place_boost = float(self.get_parameter('turn_in_place_boost').value)
        self.turn_in_place_linear_threshold = float(
            self.get_parameter('turn_in_place_linear_threshold').value
        )
        self.turn_in_place_angular_threshold = float(
            self.get_parameter('turn_in_place_angular_threshold').value
        )

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        # Insure command mode is valid
        if self.command_mode not in ('T1', 'T11'):
            raise ValueError(
                f"Unsupported command_mode '{self.command_mode}', use 'T1' or 'T11'"
            )

        # Ensure boost values don't exceed max speed
        # Note: When giving commands higher than max speed robot will stop moving
        if self.forward_boost_cmd > self.max_linear_mps:
            self.get_logger().warn(
                'forward_boost_cmd is greater than max_linear_mps; clamping to max_linear_mps'
            )
            self.forward_boost_cmd = self.max_linear_mps

        if self.turn_in_place_boost > self.max_linear_mps:
            self.get_logger().warn(
                'turn_in_place_boost is greater than max_linear_mps; clamping to max_linear_mps'
            )
            self.turn_in_place_boost = self.max_linear_mps

        # State
        # Current and intial velocities for smoothing
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        # Commands recieved from last command time
        self.last_cmd_time = self.get_clock().now()
        # Last sent values for left and right
        self.last_sent_left = None
        self.last_sent_right = None
        #Check if robot is currently stopped
        self.stopped = True

        # Serial
        # Opens serial connection to waverover through USBC
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                dsrdtr=None,
                timeout=0.1
            )
            # Disable hardware flow control lines
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            
            self.get_logger().info(f'Opened serial port {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        # Optional rover serial echo
        # Used for debugging and very helpful leave on while testing
        try:
            self.send_json({"T": 143, "cmd": 1})
            self.get_logger().info('Enabled rover serial echo')
        except Exception as e:
            self.get_logger().warn(f'Could not enable rover echo: {e}')

        # Subscriber
        # Subscribes to velocity commands from nav stack
        self.sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Control loop timer
        # Runs at a fixed rate and sends commands continuously
        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        #Start logs (useful for debugging)
        self.get_logger().info(
            'cmd_vel_to_serial started '
            f'(mode={self.command_mode}, topic={cmd_vel_topic}, '
            f'max_linear_mps={self.max_linear_mps}, '
            f'max_angular_radps={self.max_angular_radps}, '
            f'angular_scale={self.angular_scale}, '
            f'output_limit={self.output_limit}, '
            f'min_output_cmd={self.min_output_cmd}, '
            f'forward_boost_cmd={self.forward_boost_cmd}, '
            f'turn_in_place_boost={self.turn_in_place_boost}, '
            f'smoothing_alpha={self.smoothing_alpha})'
        )

    # Utility Functions
    # Clamps a value between low and high limits useful since waverover has max and minimum for commands
    def clamp(self, value, low, high):
        return max(low, min(high, value))

    # Prevent small values that won't move the motors. Ensures a stable minimum value
    def apply_deadband_float(self, value):
        if abs(value) < 1e-4:
            return 0.0
        if abs(value) < self.min_output_cmd:
            return self.min_output_cmd if value > 0.0 else -self.min_output_cmd
        return value

    # Same as above but for pwm values
    def apply_deadband_pwm(self, value):
        if value == 0:
            return 0
        if abs(value) < self.min_pwm_cmd:
            return self.min_pwm_cmd if value > 0 else -self.min_pwm_cmd
        return value

    # Smooths motions and prevents accelleration that is to quick and will throw off lidar
    def smooth_value(self, current, target):
        return current + self.smoothing_alpha * (target - current)

    # ROS Callback
    # Called when a new velocity command is recieved
    def cmd_vel_callback(self, msg: Twist):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        #Clamps speed limits
        linear_x = self.clamp(linear_x, -self.max_linear_mps, self.max_linear_mps)
        angular_z = self.clamp(angular_z, -self.max_angular_radps, self.max_angular_radps)

        # Store speeds as targers for smoothing loop
        self.target_linear_x = linear_x
        self.target_angular_z = angular_z
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False

        # Log of commands useful for debugging
        self.get_logger().info(
            f'Received cmd_vel: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}'
        )

    # Kinematics
    # Converts robot velocity twist command to left or right wheel speeds
    def twist_to_wheels(self, linear_x, angular_z):
        # Tune scale to insure robot actually turns
        angular_z *= self.angular_scale

        # Standard drive equations
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        # Clamp values to max wheel speed
        left_mps = self.clamp(left_mps, -self.max_linear_mps, self.max_linear_mps)
        right_mps = self.clamp(right_mps, -self.max_linear_mps, self.max_linear_mps)

        return left_mps, right_mps

    # Boost functions prevent commands that do not move the robot
    def apply_forward_boost(self, left_mps, right_mps, linear_x, angular_z):
        # If robot is moving forward (not turning)
        is_forward_drive = (
            #Moving forward enough
            abs(linear_x) > self.forward_boost_linear_threshold
            and abs(angular_z) < self.forward_boost_angular_threshold # Not turning much
        )

        # If not moving forward do nothing
        if not is_forward_drive:
            return left_mps, right_mps

        # If moving forward (Linear is positive)
        if linear_x > 0.0:
            # If left wheel speed is small but not zero boost it to be usable by waverover
            if 0.0 < abs(left_mps) < self.forward_boost_cmd:
                left_mps = self.forward_boost_cmd if left_mps > 0.0 else -self.forward_boost_cmd
            # Do the same for right wheel
            if 0.0 < abs(right_mps) < self.forward_boost_cmd:
                right_mps = self.forward_boost_cmd if right_mps > 0.0 else -self.forward_boost_cmd
        else:
            # If moving backward boost backward same as for forward
            if 0.0 < abs(left_mps) < self.forward_boost_cmd:
                left_mps = -self.forward_boost_cmd if left_mps < 0.0 else self.forward_boost_cmd
            if 0.0 < abs(right_mps) < self.forward_boost_cmd:
                right_mps = -self.forward_boost_cmd if right_mps < 0.0 else self.forward_boost_cmd

        # Return boosted wheel speeds
        return left_mps, right_mps

    def apply_turn_in_place_boost(self, left_mps, right_mps, linear_x, angular_z):
        # Check if robot is turning in place
        is_turn_in_place = (
            # If no forward movement
            abs(linear_x) < self.turn_in_place_linear_threshold
            and abs(angular_z) > self.turn_in_place_angular_threshold # Check for turning command
        )

        # If not change nothing
        if not is_turn_in_place:
            return left_mps, right_mps

        # Ensure left wheel speed has enough power to move
        if abs(left_mps) < self.turn_in_place_boost:
            left_mps = self.turn_in_place_boost if left_mps >= 0.0 else -self.turn_in_place_boost

        # Same for right wheel speed
        if abs(right_mps) < self.turn_in_place_boost:
            right_mps = self.turn_in_place_boost if right_mps >= 0.0 else -self.turn_in_place_boost

        return left_mps, right_mps

    def wheels_to_t1(self, left_mps, right_mps):
        # Convert wheel speeds to t1 mode
        left_cmd = (left_mps / self.max_linear_mps) * self.output_limit
        right_cmd = (right_mps / self.max_linear_mps) * self.output_limit

        # Clamp commands to maximum and minimum ranges
        left_cmd = self.clamp(left_cmd, -self.output_limit, self.output_limit)
        right_cmd = self.clamp(right_cmd, -self.output_limit, self.output_limit)

        # Apply a minimum threshold to avoid stalling
        left_cmd = self.apply_deadband_float(left_cmd)
        right_cmd = self.apply_deadband_float(right_cmd)

        # Round to 3 decimal places
        return round(left_cmd, 3), round(right_cmd, 3)

    # Same for PWM
    def wheels_to_t11(self, left_mps, right_mps):
        left_pwm = int(round((left_mps / self.max_linear_mps) * self.pwm_limit))
        right_pwm = int(round((right_mps / self.max_linear_mps) * self.pwm_limit))

        left_pwm = int(self.clamp(left_pwm, -self.pwm_limit, self.pwm_limit))
        right_pwm = int(self.clamp(right_pwm, -self.pwm_limit, self.pwm_limit))

        left_pwm = self.apply_deadband_pwm(left_pwm)
        right_pwm = self.apply_deadband_pwm(right_pwm)

        return left_pwm, right_pwm

    def send_json(self, payload):
        # Convert commands to JSON in format required by waverover
        line = json.dumps(payload, separators=(',', ':')) + '\n'
        # Send encoded string over serial to waverover
        self.ser.write(line.encode('utf-8'))

    def send_drive(self, linear_x, angular_z):
        # Check for inverted turning useful for bugtesting
        if self.invert_angular:
            angular_z = -angular_z

        # Convert linear/angular velocity into left/right wheel speeds
        left_mps, right_mps = self.twist_to_wheels(linear_x, angular_z)

        # Check for swapping wheel speeds useful for bug testing
        if self.swap_left_right:
            left_mps, right_mps = right_mps, left_mps

        # Check for invert directions useful for bugtesting
        if self.invert_left:
            left_mps = -left_mps
        if self.invert_right:
            right_mps = -right_mps

        # Store raw values for debugging
        raw_left_mps, raw_right_mps = left_mps, right_mps

        # Apply forward motion boost
        left_mps, right_mps = self.apply_forward_boost(
            left_mps, right_mps, linear_x, angular_z
        )

        # Apply turn in place boost
        left_mps, right_mps = self.apply_turn_in_place_boost(
            left_mps, right_mps, linear_x, angular_z
        )

        # Convert to commands based on mode
        if self.command_mode == 'T1':
            left_cmd, right_cmd = self.wheels_to_t1(left_mps, right_mps)
            payload = {"T": 1, "L": left_cmd, "R": right_cmd}
        else:
            left_cmd, right_cmd = self.wheels_to_t11(left_mps, right_mps)
            payload = {"T": 11, "L": left_cmd, "R": right_cmd}

        #Check if command is same as last (Keeps logs cleaner can be disabled for additional bug testing)
        same_as_last = (
            left_cmd == self.last_sent_left and right_cmd == self.last_sent_right
        )

        # Send command to waverover
        self.send_json(payload)
        # Save the last sent command
        self.last_sent_left = left_cmd
        self.last_sent_right = right_cmd

        # Load old debug info when command doesnt change
        if same_as_last:
            self.get_logger().debug(
            f"Re-sent drive: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}, payload={payload}"
        )
        # Log full info when command changes
        else:
            self.get_logger().info(
            "Sent drive: "
            f"linear_x={linear_x:.3f}, angular_z={angular_z:.3f}, "
            f"wheel_mps_raw=({raw_left_mps:.3f}, {raw_right_mps:.3f}), "
            f"wheel_mps_final=({left_mps:.3f}, {right_mps:.3f}), "
            f"payload={payload}"
         )


    def send_stop(self):
        # Send a zero speed command different for each mode
        if self.command_mode == 'T1':
            payload = {"T": 1, "L": 0.0, "R": 0.0}
            self.last_sent_left = 0.0
            self.last_sent_right = 0.0
        else:
            payload = {"T": 11, "L": 0, "R": 0}
            self.last_sent_left = 0
            self.last_sent_right = 0

        # Send a stop command to the rover
        self.send_json(payload)
        self.get_logger().info('Sent STOP')

    def control_loop(self):
        # Get the current time and comute time since last command
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        # If command is too old force speeds to zero
        # Needed so robot doesn't continue running with no new commands
        if elapsed > self.cmd_timeout:
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0

        # Smoothly move speeds to the target speed (Prevents lidar shake)
        self.current_linear_x = self.smooth_value(self.current_linear_x, self.target_linear_x)
        self.current_angular_z = self.smooth_value(self.current_angular_z, self.target_angular_z)

        # Snap small values to 0 to save battery on rover
        if abs(self.current_linear_x) < 1e-3:
            self.current_linear_x = 0.0
        if abs(self.current_angular_z) < 1e-3:
            self.current_angular_z = 0.0

        # If timed out stop robot once it has fully slowed down
        if elapsed > self.cmd_timeout:
            if not self.stopped and self.current_linear_x == 0.0 and self.current_angular_z == 0.0:
                self.send_stop()
                self.stopped = True
            return

        # Else send drive command
        self.send_drive(self.current_linear_x, self.current_angular_z)
        self.stopped = False

    def destroy_node(self):
        # Try to stop robot before shutting down
        try:
            self.send_stop()
        except Exception:
            pass

        # Close serial connection
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        # Call parent class destructor
        super().destroy_node()

# MAIN
def main(args=None):
    # Initialize ROS2 system
    rclpy.init(args=args)
    # Create node instance
    node = CmdVelToSerial()

    # Try to keep node running and processing callbacks
    try:
        rclpy.spin(node)
    # Clean stop on ctrl-c
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup on exit
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    # Enter into main
    main()
