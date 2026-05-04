#!/usr/bin/env python3
"""
Vision Node - Two-mode detection (SEARCHING strict / TRACKING relaxed).

Root cause of losing box when close:
  - Box area grows large → was filtered by MAX_AREA
  - Box shifts below ROI line → was cut off
  - Box+floor merge into one wide contour → aspect filter kills it

Fixes:
  - NO MAX_AREA limit (only aspect > 3.0 kills floor)
  - Full image ROI while tracking (100%)
  - Wider area range in tracking mode
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class VisionNode(Node):

    # ── Area ──────────────────────────────────────────────────────
    MIN_AREA_SEARCH = 500      # minimum while searching
    MIN_AREA_TRACK  = 200      # smaller minimum while tracking (box may be partially visible)
    # NO MAX_AREA — aspect ratio filter handles the floor contour

    APPROX_EPSILON = 0.035
    FOCAL_LENGTH   = 385

    # ── Stability ─────────────────────────────────────────────────
    STABLE_FRAMES = 5    # frames needed to confirm
    LOST_FRAMES   = 20   # frames needed to lose (at ~15fps ≈ 1.7s)

    # ── Strict filters (SEARCHING) ────────────────────────────────
    STRICT_VERTICES    = 4
    STRICT_CORNER      = 0.85
    STRICT_CIRCULARITY = 0.85
    STRICT_SOLIDITY    = 0.82
    STRICT_ASPECT_MIN  = 0.65
    STRICT_ASPECT_MAX  = 2.0
    STRICT_EXTENT      = 0.75

    # ── Relaxed filters (TRACKING) ────────────────────────────────
    TRACK_VERTICES_MIN = 4
    TRACK_VERTICES_MAX = 6
    TRACK_CORNER       = 0.50   # very relaxed — perspective warps corners
    TRACK_CIRCULARITY  = 0.95   # very relaxed
    TRACK_SOLIDITY     = 0.65
    TRACK_ASPECT_MIN   = 0.40   # wide range — perspective distortion
    TRACK_ASPECT_MAX   = 3.0    # NOTE: floor asp is ~4.9, still filtered
    TRACK_EXTENT       = 0.55

    # ── ROI ───────────────────────────────────────────────────────
    ROI_SEARCH = 0.60   # top 60% while searching — avoids floor
    ROI_TRACK  = 1.00   # FULL image while tracking — box may be anywhere

    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        self.box_seen_count = 0
        self.box_lost_count = 0
        self.box_confirmed  = False

        self.last_error = 0.0
        self.last_angle = 0.0
        self.last_area  = 0.0
        self.last_cx    = -1
        self.last_cy    = -1

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pub_detected = self.create_publisher(Bool,    '/box_detected', 10)
        self.pub_error    = self.create_publisher(Float32, '/box_error',    10)
        self.pub_angle    = self.create_publisher(Float32, '/box_angle',    10)
        self.pub_area     = self.create_publisher(Float32, '/box_area',     10)
        self.pub_debug    = self.create_publisher(Image,   '/camera/debug', 10)
        self.pub_thresh   = self.create_publisher(Image,   '/camera/thresh', 10)

        self.get_logger().info('Vision started Detection — SEARCHING mode.')

    def corner_angle_score(self, approx):
        pts = approx.reshape(-1, 2)
        n   = len(pts)
        if n < 3:
            return 0.0
        scores = []
        for i in range(n):
            p1 = pts[(i - 1) % n].astype(float)
            p2 = pts[i].astype(float)
            p3 = pts[(i + 1) % n].astype(float)
            v1 = p1 - p2
            v2 = p3 - p2
            l1 = np.linalg.norm(v1)
            l2 = np.linalg.norm(v2)
            if l1 < 1 or l2 < 1:
                continue
            cos_a = np.clip(np.dot(v1, v2) / (l1 * l2), -1.0, 1.0)
            angle = math.degrees(math.acos(cos_a))
            scores.append(1.0 - abs(angle - 90.0) / 90.0)
        return float(np.mean(scores)) if scores else 0.0

    def check_strict(self, v, circ, corner, sol, asp, ext, convex):
        return (
            v == self.STRICT_VERTICES
            and convex
            and circ   < self.STRICT_CIRCULARITY
            and corner > self.STRICT_CORNER
            and sol    > self.STRICT_SOLIDITY
            and self.STRICT_ASPECT_MIN < asp < self.STRICT_ASPECT_MAX
            and ext    > self.STRICT_EXTENT
        )

    def check_relaxed(self, v, circ, corner, sol, asp, ext, convex):
        return (
            self.TRACK_VERTICES_MIN <= v <= self.TRACK_VERTICES_MAX
            and circ   < self.TRACK_CIRCULARITY
            and corner > self.TRACK_CORNER
            and sol    > self.TRACK_SOLIDITY
            and self.TRACK_ASPECT_MIN < asp < self.TRACK_ASPECT_MAX
            and ext    > self.TRACK_EXTENT
        )

    def image_callback(self, msg):
        frame    = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w     = frame.shape[:2]
        image_cx = w // 2
        image_cy = h // 2
        debug    = frame.copy()

        # Full image when tracking — box may have moved to bottom of frame
        roi_frac    = self.ROI_TRACK if self.box_confirmed else self.ROI_SEARCH
        roi_h       = int(h * roi_frac)
        roi         = frame[0:roi_h, 0:w]
        min_area    = self.MIN_AREA_TRACK if self.box_confirmed else self.MIN_AREA_SEARCH
        mode_str    = 'TRACKING' if self.box_confirmed else 'SEARCHING'

        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresh = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = np.ones((7, 7), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.get_logger().info(
            f'[{mode_str}] Contours: {len(contours)} ROI={int(roi_frac*100)}%',
            throttle_duration_sec=0.5)

        # Draw overlays
        cv2.line(debug, (0, roi_h), (w, roi_h), (255, 255, 0), 1)
        cv2.line(debug, (image_cx, 0), (image_cx, h), (0, 255, 0), 1)
        cv2.putText(debug, f'MODE:{mode_str} ROI:{int(roi_frac*100)}%',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 255) if self.box_confirmed else (0, 255, 0), 2)

        raw_box_found = False
        raw_box_cx    = 0
        raw_box_cy    = 0
        raw_box_area  = 0.0
        best_score    = -1.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter < 1:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)
            epsilon     = self.APPROX_EPSILON * perimeter
            approx      = cv2.approxPolyDP(cnt, epsilon, True)
            vertices    = len(approx)
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect      = float(bw) / bh if bh > 0 else 0

            # ── Kill floor/horizon contour ──────────────────────────
            # Floor: asp ~4.9 (extremely wide flat band)
            # Also kill very low circularity — horizon line is circ ~0.25
            if aspect > 3.5 or circularity < 0.30:
                cv2.drawContours(debug, [approx], -1, (0, 60, 60), 1)
                self.get_logger().info(
                    f'[FLOOR KILLED] area={int(area)} asp={aspect:.2f} circ={circularity:.2f}',
                    throttle_duration_sec=1.0)
                continue

            is_convex    = cv2.isContourConvex(approx)
            hull         = cv2.convexHull(cnt)
            hull_area    = cv2.contourArea(hull)
            solidity     = float(area) / hull_area if hull_area > 0 else 0
            corner_score = self.corner_angle_score(approx)
            extent       = float(area) / (bw * bh) if bw * bh > 0 else 0

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            self.get_logger().info(
                f'[{mode_str}] area={int(area)} v={vertices} '
                f'circ={circularity:.2f} sol={solidity:.2f} '
                f'asp={aspect:.2f} corner={corner_score:.2f} '
                f'ext={extent:.2f} convex={is_convex}',
                throttle_duration_sec=0.5)

            # Draw candidate
            cv2.drawContours(debug, [approx], -1, (128, 128, 128), 1)
            cv2.putText(debug, f'v={vertices} asp={aspect:.1f} c={corner_score:.2f}',
                        (x, max(y - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (128, 128, 128), 1)

            # Apply appropriate filter
            if self.box_confirmed:
                hit = self.check_relaxed(
                    vertices, circularity, corner_score,
                    solidity, aspect, extent, is_convex)
            else:
                hit = self.check_strict(
                    vertices, circularity, corner_score,
                    solidity, aspect, extent, is_convex)

            if hit:
                # Tracking: prefer candidate closest to last known position
                if self.box_confirmed and self.last_cx >= 0:
                    dist  = math.hypot(cx - self.last_cx, cy - self.last_cy)
                    score = area / (dist + 1.0)
                else:
                    score = float(area)

                if score > best_score:
                    best_score    = score
                    raw_box_found = True
                    raw_box_cx    = cx
                    raw_box_cy    = cy
                    raw_box_area  = area

                cv2.drawContours(debug, [approx], -1, (0, 0, 255), 2)
                cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(debug,
                            f'BOX area={int(area)} asp={aspect:.2f}',
                            (x, max(y - 5, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)

        # ── Temporal stability ─────────────────────────────────────
        if raw_box_found:
            self.box_seen_count += 1
            self.box_lost_count  = 0
            self.last_cx         = raw_box_cx
            self.last_cy         = raw_box_cy

            pixel_error     = float(raw_box_cx - image_cx)
            self.last_error = pixel_error
            self.last_angle = float(np.arctan2(pixel_error, self.FOCAL_LENGTH))
            self.last_area  = raw_box_area

            if not self.box_confirmed and self.box_seen_count >= self.STABLE_FRAMES:
                self.box_confirmed = True
                self.get_logger().info(
                    f'=== BOX DETECTED → TRACKING === '
                    f'err={pixel_error:.0f}px '
                    f'angle={math.degrees(self.last_angle):.1f}deg '
                    f'area={int(raw_box_area)}px²')
        else:
            self.box_lost_count += 1
            self.box_seen_count  = 0

            if self.box_confirmed and self.box_lost_count >= self.LOST_FRAMES:
                self.box_confirmed = False
                self.last_cx       = -1
                self.last_cy       = -1
                self.get_logger().info(
                    f'=== Box not detected → SEARCHING ===')

        # ── Publish ────────────────────────────────────────────────
        detected_msg = Bool()
        error_msg    = Float32()
        angle_msg    = Float32()
        area_msg     = Float32()

        if self.box_confirmed:
            detected_msg.data = True
            error_msg.data    = self.last_error
            angle_msg.data    = self.last_angle
            area_msg.data     = self.last_area

            target_cx = int(image_cx + self.last_error)
            cv2.drawMarker(debug, (target_cx, image_cy),
                           (255, 255, 0), cv2.MARKER_CROSS, 30, 3)
            cv2.line(debug, (image_cx, image_cy), (target_cx, image_cy),
                     (255, 255, 0), 2)
            cv2.putText(debug,
                        f'TRACKING err={self.last_error:.0f}px '
                        f'area={int(self.last_area)}px² '
                        f'lost={self.box_lost_count}/{self.LOST_FRAMES}',
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            self.get_logger().info(
                f'TRACKING | err={self.last_error:.0f}px '
                f'angle={math.degrees(self.last_angle):.1f}deg '
                f'area={int(self.last_area)}px² '
                f'lost={self.box_lost_count}/{self.LOST_FRAMES}',
                throttle_duration_sec=0.5)
        else:
            detected_msg.data = False
            error_msg.data    = 0.0
            angle_msg.data    = 0.0
            area_msg.data     = 0.0

            cv2.putText(debug,
                        f'SEARCHING seen={self.box_seen_count}/{self.STABLE_FRAMES} '
                        f'lost={self.box_lost_count}/{self.LOST_FRAMES}',
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            self.get_logger().info(
                f'Searching | seen={self.box_seen_count}/{self.STABLE_FRAMES} '
                f'lost={self.box_lost_count}/{self.LOST_FRAMES}',
                throttle_duration_sec=1.0)

        self.pub_detected.publish(detected_msg)
        self.pub_error.publish(error_msg)
        self.pub_angle.publish(angle_msg)
        self.pub_area.publish(area_msg)

        debug_msg        = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)

        thresh_full             = np.zeros((h, w), dtype=np.uint8)
        thresh_full[0:roi_h, :] = thresh
        thresh_msg              = self.bridge.cv2_to_imgmsg(thresh_full, encoding='mono8')
        thresh_msg.header       = msg.header
        self.pub_thresh.publish(thresh_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()