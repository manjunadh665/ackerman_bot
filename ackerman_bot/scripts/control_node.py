#!/usr/bin/env python3
"""
Control Node - Ackermann steering robot navigates to detected box.
States: WAIT → CURVE → ALIGN → STOP
        CURVE → SEARCH (if box lost too long) → CURVE (when re-found)

Key Ackermann rule: linear.x must always be meaningful when steering.
Low linear.x + high angular.z = pivot/spin instead of arc.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math


class ControlNode(Node):


    STATE_WAIT   = 'INITIALIZING'
    STATE_SEARCH = 'SCANNING'
    STATE_CURVE  = 'NAVIGATING'
    STATE_ALIGN  = 'CENTERING'
    STATE_STOP   = 'DOCKED'

    # ── Ackermann driving params ───────────────────────────────────
    SEARCH_TURN_SPEED  = 0.18    # angular.z while searching (slow in-place spin)
    SEARCH_TIMEOUT     = 6.0     # seconds before reversing spin direction

    CURVE_LINEAR       = 0.30    # forward speed while curving — must be > 0.25 for Ackermann
    CURVE_KP           = 0.9     # locked angle → steer gain
    MAX_STEER          = 0.40    # max angular.z (~26 deg equivalent)
    CURVE_LOST_LIMIT   = 10       # frames before CURVE gives up (~0.4s at 20Hz)

    ALIGN_LINEAR       = 0.25    # forward speed during fine alignment
    ALIGN_KP           = 0.0025  # pixel error → steer gain
    ALIGN_LOST_LIMIT   = 10      # frames before ALIGN gives up

    STOP_AREA          = 42000   # px² — stop when box fills this much of frame
    # ─────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('control_node')

        # Vision data (updated by callbacks)
        self.box_detected = False
        self.box_error    = 0.0    # pixels, +right -left from image center
        self.box_area     = 0.0    # px²
        self.box_angle    = 0.0    # radians

        # Navigation state
        self.state            = self.STATE_WAIT
        self.locked_angle     = 0.0
        self.search_dir       = 1.0    # +1 or -1 for spin direction
        self.search_timer     = 0.0    # accumulates time in SEARCH state
        self.curve_lost_count = 0      # consecutive frames box not seen in CURVE
        self.align_lost_count = 0      # consecutive frames box not seen in ALIGN

        # Subscriptions
        self.create_subscription(Bool,    '/box_detected', self.detected_cb, 10)
        self.create_subscription(Float32, '/box_error',    self.error_cb,    10)
        self.create_subscription(Float32, '/box_area',     self.area_cb,     10)
        self.create_subscription(Float32, '/box_angle',    self.angle_cb,    10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20 Hz control loop
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            'Navigation node ready — Scanning for the box... (initial state: WAIT)')

    # ── Callbacks ─────────────────────────────────────────────────
    def detected_cb(self, msg): self.box_detected = msg.data
    def error_cb(self, msg):    self.box_error    = msg.data
    def area_cb(self, msg):     self.box_area     = msg.data
    def angle_cb(self, msg):    self.box_angle    = msg.data

    # ── Helpers ───────────────────────────────────────────────────
    def publish(self, lin, ang):
        cmd = Twist()
        cmd.linear.x  = float(lin)
        cmd.angular.z = float(ang)
        self.pub_cmd.publish(cmd)

    def stop(self):
        self.publish(0.0, 0.0)

    def clamp(self, val, limit):
        return max(-limit, min(limit, val))

    # ── Main loop ─────────────────────────────────────────────────
    def control_loop(self):

        # ── STOP — publish zero and stay here forever ─────────────
        if self.state == self.STATE_STOP:
            self.stop()
            self.get_logger().info(
                'STOP — reached box.', throttle_duration_sec=5.0)
            return

        # ── Global stop condition (fires from any moving state) ───
        if self.state not in (self.STATE_WAIT, self.STATE_SEARCH):
            if self.box_detected and self.box_area >= self.STOP_AREA:
                self.stop()
                self.state = self.STATE_STOP
                self.get_logger().info(
                    f'STOP triggered — area={int(self.box_area)} >= {self.STOP_AREA}')
                return

        # ── WAIT — robot completely still ─────────────────────────
        # Vision node handles all stability filtering.
        # We just wait for box_detected = True.
        if self.state == self.STATE_WAIT:
            self.stop()
            if self.box_detected:
                self.locked_angle     = self.box_angle
                self.curve_lost_count = 0
                self.state            = self.STATE_CURVE
                self.get_logger().info(
                    f'WAIT → CURVE | locked_angle='
                    f'{math.degrees(self.locked_angle):.1f}° '
                    f'err={self.box_error:.0f}px '
                    f'area={int(self.box_area)}px²')
            else:
                self.get_logger().info(
                    'WAIT — no box yet...', throttle_duration_sec=2.0)
            return

        # ── SEARCH — rotate slowly to find box ────────────────────
        if self.state == self.STATE_SEARCH:
            self.search_timer += self.dt

            # Reverse spin direction after timeout
            if self.search_timer >= self.SEARCH_TIMEOUT:
                self.search_dir  *= -1.0
                self.search_timer = 0.0
                self.get_logger().info(
                    f'SEARCH — reversing spin direction to {self.search_dir:+.0f}')

            # Slow in-place rotation
            # For Ackermann at standstill, linear=0 + angular = pivot
            self.publish(0.0, self.SEARCH_TURN_SPEED * self.search_dir)

            if self.box_detected:
                # Re-lock the angle freshly when box reappears
                self.locked_angle     = self.box_angle
                self.curve_lost_count = 0
                self.search_timer     = 0.0
                self.state            = self.STATE_CURVE
                self.get_logger().info(
                    f'SEARCH → CURVE | locked_angle='
                    f'{math.degrees(self.locked_angle):.1f}°')
            else:
                self.get_logger().info(
                    f'SEARCH | spinning dir={self.search_dir:+.0f} '
                    f't={self.search_timer:.1f}s',
                    throttle_duration_sec=0.5)
            return

        # ── CURVE — drive Ackermann arc using locked angle ─────────
        # CRITICAL for Ackermann: linear.x must be meaningful.
        # If speed is too low, the robot pivots in place like a diff-drive.
        # The locked_angle is captured once and held — prevents flicker
        # from bad detections changing the direction mid-arc.
        if self.state == self.STATE_CURVE:

            steer = self.clamp(
                -(self.CURVE_KP * self.locked_angle),
                self.MAX_STEER)

            lin = self.CURVE_LINEAR
            # Ensure enough forward speed when making a sharp arc
            if abs(steer) > 0.3:
                lin = max(lin, 0.40)

            self.publish(lin, steer)

            if not self.box_detected:
                self.curve_lost_count += 1
                if self.curve_lost_count >= self.CURVE_LOST_LIMIT:
                    self.state            = self.STATE_SEARCH
                    self.curve_lost_count = 0
                    self.get_logger().info(
                        f'CURVE → SEARCH — box lost for '
                        f'{self.CURVE_LOST_LIMIT} frames')
            else:
                self.curve_lost_count = 0  # reset on any good detection

                # Transition to fine alignment when error is small
                if abs(self.box_error) < 75:
                    self.align_lost_count = 0
                    self.state            = self.STATE_ALIGN
                    self.get_logger().info(
                        f'CURVE → ALIGN | err={self.box_error:.0f}px')

            self.get_logger().info(
                f'CURVE | locked={math.degrees(self.locked_angle):.1f}° '
                f'steer={math.degrees(steer):.1f}° '
                f'lin={lin:.2f} '
                f'err={self.box_error:.0f}px '
                f'area={int(self.box_area)}px² '
                f'lost={self.curve_lost_count}/{self.CURVE_LOST_LIMIT}',
                throttle_duration_sec=0.3)
            return

        # ── ALIGN — pixel-level correction while moving forward ────
        if self.state == self.STATE_ALIGN:

            if not self.box_detected:
                # Box temporarily lost — coast forward briefly
                self.align_lost_count += 1
                self.publish(self.ALIGN_LINEAR * 0.5, 0.0)

                if self.align_lost_count >= self.ALIGN_LOST_LIMIT:
                    # Lost too long — re-lock and go back to search
                    self.align_lost_count = 0
                    self.curve_lost_count = 0
                    self.state            = self.STATE_SEARCH
                    self.get_logger().info(
                        'ALIGN → SEARCH — box lost for too long')
                else:
                    self.get_logger().info(
                        f'ALIGN — box lost, coasting '
                        f'({self.align_lost_count}/{self.ALIGN_LOST_LIMIT})',
                        throttle_duration_sec=0.5)
                return

            # Box visible — reset lost counter
            self.align_lost_count = 0

            steer = self.clamp(
                -(self.ALIGN_KP * self.box_error),
                self.MAX_STEER)

            self.publish(self.ALIGN_LINEAR, steer)

            self.get_logger().info(
                f'ALIGN | err={self.box_error:.0f}px '
                f'steer={math.degrees(steer):.1f}° '
                f'area={int(self.box_area)}px²',
                throttle_duration_sec=0.3)

            # If error drifts large again — re-lock angle and curve
            if abs(self.box_error) > 100:
                self.locked_angle     = self.box_angle
                self.curve_lost_count = 0
                self.state            = self.STATE_CURVE
                self.get_logger().info(
                    f'ALIGN → CURVE | large drift '
                    f'err={self.box_error:.0f}px, re-locked '
                    f'{math.degrees(self.locked_angle):.1f}°')
            return


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()