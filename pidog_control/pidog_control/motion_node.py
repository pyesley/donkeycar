#!/usr/bin/env python3
"""
PiDog Motion Node - Receives motion commands and controls the robot dog.

Run this on the PiDog (Raspberry Pi 5).

Subscribes:
    /pidog/cmd_vel (geometry_msgs/Twist) - Velocity commands
    /pidog/cmd (std_msgs/String) - Discrete commands (stand, sit, walk, etc.)

The node translates high-level commands into leg movements using pidog_fusion.
"""

import sys
import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Add pidog path for imports
sys.path.insert(0, os.path.expanduser('~/pidog/claude'))

try:
    from pidog_fusion import PidogFusion
    PIDOG_AVAILABLE = True
except ImportError as e:
    PIDOG_AVAILABLE = False
    print(f"Warning: pidog_fusion not available ({e}). Running in simulation mode.")


class MotionState:
    """Robot motion state machine"""
    IDLE = 'idle'
    STANDING = 'standing'
    WALKING = 'walking'
    SITTING = 'sitting'
    LYING = 'lying'


class PiDogMotionNode(Node):
    """ROS2 node for PiDog motion control"""

    def __init__(self):
        super().__init__('pidog_motion_node')

        # Parameters
        self.declare_parameter('walk_speed', 50)  # Speed 1-100
        self.declare_parameter('step_count', 2)   # Steps per walk command

        self.walk_speed = self.get_parameter('walk_speed').value
        self.step_count = self.get_parameter('step_count').value

        # State
        self.state = MotionState.IDLE
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.cmd_lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # Stop if no commands for 0.5s
        self.is_moving = False
        self.motion_lock = threading.Lock()

        # QoS profiles
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/pidog/cmd_vel', self.cmd_vel_callback, qos_cmd)
        self.cmd_sub = self.create_subscription(
            String, '/pidog/cmd', self.cmd_callback, qos_cmd)

        # Status publisher
        self.status_pub = self.create_publisher(
            String, '/pidog/motion/status', 10)

        # Initialize PiDog hardware
        self.pidog = None
        if PIDOG_AVAILABLE:
            try:
                self.pidog = PidogFusion()
                self.get_logger().info("PiDog hardware initialized")
                # Start in standing position
                self._do_stand()
            except Exception as e:
                self.get_logger().error(f"Failed to initialize PiDog: {e}")
                self.pidog = None
        else:
            self.get_logger().warn("Running in simulation mode (no hardware)")

        # Motion control timer (20 Hz)
        self.motion_timer = self.create_timer(0.05, self.motion_loop)

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("PiDog Motion Node started")
        self.get_logger().info("  Subscribing to: /pidog/cmd_vel, /pidog/cmd")
        self.get_logger().info("  Publishing to: /pidog/motion/status")

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        with self.cmd_lock:
            self.target_linear_x = msg.linear.x
            self.target_angular_z = msg.angular.z
            self.last_cmd_time = time.time()

    def cmd_callback(self, msg: String):
        """Handle discrete commands"""
        cmd = msg.data.lower().strip()
        self.get_logger().info(f"Received command: {cmd}")

        # Execute command in a separate thread to avoid blocking
        threading.Thread(target=self._execute_command, args=(cmd,), daemon=True).start()

    def _execute_command(self, cmd: str):
        """Execute a discrete command"""
        with self.motion_lock:
            if cmd == 'stand':
                self._do_stand()
            elif cmd == 'sit':
                self._do_sit()
            elif cmd == 'rest' or cmd == 'lie':
                self._do_lie()
            elif cmd == 'stretch':
                self._do_stretch()
            elif cmd == 'wag' or cmd == 'wag_tail':
                self._do_wag()
            elif cmd == 'bark':
                self._do_bark()
            elif cmd == 'howl':
                self._do_howl()
            elif cmd == 'wave':
                self._do_wave()
            elif cmd == 'shake':
                self._do_shake()
            elif cmd == 'pushup':
                self._do_pushup()
            elif cmd == 'stop':
                self._do_stop()
            else:
                self.get_logger().warn(f"Unknown command: {cmd}")

    def motion_loop(self):
        """Main motion control loop - runs at 20 Hz"""
        with self.cmd_lock:
            linear_x = self.target_linear_x
            angular_z = self.target_angular_z
            time_since_cmd = time.time() - self.last_cmd_time

        # Timeout - stop if no recent commands
        if time_since_cmd > self.cmd_timeout:
            if self.state == MotionState.WALKING:
                with self.motion_lock:
                    self._do_stop()
            return

        # Don't interrupt ongoing motion
        if self.is_moving:
            return

        # Process motion commands
        if abs(linear_x) < 0.1 and abs(angular_z) < 0.1:
            # No significant motion command
            return
        else:
            # Motion requested - execute in separate thread
            threading.Thread(
                target=self._do_move, args=(linear_x, angular_z), daemon=True
            ).start()

    def _do_move(self, linear_x: float, angular_z: float):
        """Execute movement based on velocity commands"""
        if not self.pidog or self.is_moving:
            return

        with self.motion_lock:
            self.is_moving = True
            try:
                # Determine motion type
                if abs(linear_x) > abs(angular_z) * 0.5:
                    # Primarily forward/backward
                    if linear_x > 0:
                        self.state = MotionState.WALKING
                        self.get_logger().debug("Walking forward")
                        self.pidog.do_action('forward', speed=self.walk_speed, step_count=self.step_count)
                    else:
                        self.state = MotionState.WALKING
                        self.get_logger().debug("Walking backward")
                        self.pidog.do_action('backward', speed=self.walk_speed, step_count=self.step_count)
                else:
                    # Primarily turning
                    if angular_z > 0:
                        self.state = MotionState.WALKING
                        self.get_logger().debug("Turning left")
                        self.pidog.do_action('turn_left', speed=self.walk_speed, step_count=self.step_count)
                    else:
                        self.state = MotionState.WALKING
                        self.get_logger().debug("Turning right")
                        self.pidog.do_action('turn_right', speed=self.walk_speed, step_count=self.step_count)

            except Exception as e:
                self.get_logger().error(f"Motion error: {e}")
            finally:
                self.is_moving = False

    def _do_stand(self):
        """Stand up in neutral position"""
        self.state = MotionState.STANDING
        self.get_logger().info("Standing")

        if self.pidog:
            try:
                self.pidog.do_action('stand', speed=self.walk_speed)
                self.pidog.head_move([0, 0, 0], immediately=True)
            except Exception as e:
                self.get_logger().error(f"Stand error: {e}")

    def _do_sit(self):
        """Sit down"""
        self.state = MotionState.SITTING
        self.get_logger().info("Sitting")

        if self.pidog:
            try:
                self.pidog.do_action('sit', speed=self.walk_speed)
            except Exception as e:
                self.get_logger().error(f"Sit error: {e}")

    def _do_lie(self):
        """Lie down / rest position"""
        self.state = MotionState.LYING
        self.get_logger().info("Lying down")

        if self.pidog:
            try:
                self.pidog.do_action('lie', speed=self.walk_speed)
            except Exception as e:
                self.get_logger().error(f"Lie error: {e}")

    def _do_stretch(self):
        """Do a stretch"""
        self.get_logger().info("Stretching")

        if self.pidog:
            try:
                self.pidog.do_action('stretch', speed=self.walk_speed)
            except Exception as e:
                self.get_logger().error(f"Stretch error: {e}")

    def _do_wag(self):
        """Wag tail"""
        self.get_logger().info("Wagging tail")

        if self.pidog:
            try:
                self.pidog.tail_wag(count=3)
            except Exception as e:
                self.get_logger().error(f"Wag error: {e}")

    def _do_bark(self):
        """Make bark sound"""
        self.get_logger().info("Barking")

        if self.pidog:
            try:
                self.pidog.bark(count=2)
            except Exception as e:
                self.get_logger().error(f"Bark error: {e}")

    def _do_howl(self):
        """Make howl sound"""
        self.get_logger().info("Howling")

        if self.pidog:
            try:
                self.pidog.howl()
            except Exception as e:
                self.get_logger().error(f"Howl error: {e}")

    def _do_wave(self):
        """Wave with front right paw"""
        self.get_logger().info("Waving")

        if self.pidog:
            try:
                # Lift front right leg and wave
                # Leg indices: 0=FL_upper, 1=FR_upper, 2=BL_upper, 3=BR_upper
                #              4=FL_lower, 5=FR_lower, 6=BL_lower, 7=BR_lower
                # First stand on 3 legs
                self.pidog.legs_move([0, -45, 0, 0, 0, 45, 0, 0], speed=50)
                time.sleep(0.3)
                # Wave
                for _ in range(3):
                    self.pidog.legs_move([0, -30, 0, 0, 0, 60, 0, 0], speed=80)
                    time.sleep(0.2)
                    self.pidog.legs_move([0, -60, 0, 0, 0, 30, 0, 0], speed=80)
                    time.sleep(0.2)
                # Return to stand
                self.pidog.do_action('stand', speed=50)
            except Exception as e:
                self.get_logger().error(f"Wave error: {e}")

    def _do_shake(self):
        """Shake body"""
        self.get_logger().info("Shaking")

        if self.pidog:
            try:
                # Shake head and wag tail
                for _ in range(3):
                    self.pidog.head_move([20, 0, 0], immediately=True)
                    time.sleep(0.15)
                    self.pidog.head_move([-20, 0, 0], immediately=True)
                    time.sleep(0.15)
                self.pidog.head_move([0, 0, 0], immediately=True)
                self.pidog.tail_wag(count=2)
            except Exception as e:
                self.get_logger().error(f"Shake error: {e}")

    def _do_pushup(self):
        """Do pushups"""
        self.get_logger().info("Pushups")

        if self.pidog:
            try:
                for _ in range(3):
                    # Down
                    self.pidog.legs_move([30, 30, -30, -30, 30, 30, -30, -30], speed=50)
                    time.sleep(0.3)
                    # Up
                    self.pidog.legs_move([0, 0, 0, 0, 0, 0, 0, 0], speed=50)
                    time.sleep(0.3)
            except Exception as e:
                self.get_logger().error(f"Pushup error: {e}")

    def _do_stop(self):
        """Stop all motion and stand"""
        if self.state != MotionState.STANDING:
            self.get_logger().debug("Stopping motion")
            self._do_stand()

    def publish_status(self):
        """Publish current motion status"""
        msg = String()
        with self.cmd_lock:
            msg.data = f"state={self.state}, linear={self.target_linear_x:.2f}, angular={self.target_angular_z:.2f}"
        self.status_pub.publish(msg)

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down motion node")
        self._do_stand()


def main(args=None):
    rclpy.init(args=args)

    node = PiDogMotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
