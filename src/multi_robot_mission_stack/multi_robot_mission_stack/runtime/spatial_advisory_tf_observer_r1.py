#!/usr/bin/env python3
"""
R1 runtime seam: publish advisory-only degraded_passage from one TF/spatial condition.

Single runtime source family:
- TF relative pose between two robot base frames

Single trigger:
- Euclidean distance(robot_a, robot_b) <= distance_threshold_m

Single output:
- std_msgs/String JSON degraded_passage advisory on one configured topic
"""

from __future__ import annotations

import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from multi_robot_mission_stack.semantic.degraded_passage_candidate_v85 import (
    DegradedPassageAssemblyCandidate,
    assemble_degraded_passage_record_from_candidate,
)
from multi_robot_mission_stack.transport.degraded_passage_json_v87 import (
    encode_degraded_passage_record_json,
)


def euclidean_distance_m(x: float, y: float, z: float) -> float:
    return math.sqrt(float(x) ** 2 + float(y) ** 2 + float(z) ** 2)


def should_emit_advisory(distance_m: float, threshold_m: float) -> bool:
    if not math.isfinite(distance_m) or not math.isfinite(threshold_m):
        return False
    return distance_m <= threshold_m


def seam_location_ref(robot_a_tf_topic: str, robot_b_tf_topic: str, threshold_m: float) -> str:
    return "tf:%s<->%s:dist<=%.2f" % (robot_a_tf_topic, robot_b_tf_topic, threshold_m)


class SpatialAdvisoryTfObserverR1(Node):
    def __init__(self) -> None:
        super().__init__("spatial_advisory_tf_observer_r1")
        self.declare_parameter("robot_a_tf_topic", "/robot1_ns/tf")
        self.declare_parameter("robot_b_tf_topic", "/robot2_ns/tf")
        self.declare_parameter("robot_base_child_frame", "base_footprint")
        self.declare_parameter("distance_threshold_m", 1.5)
        self.declare_parameter("publish_topic", "/semantic/degraded_passage_p1_1")
        self.declare_parameter("poll_period_sec", 0.5)
        self.declare_parameter("min_publish_interval_sec", 2.0)
        self.declare_parameter("source_robot_id", "robot1")
        self.declare_parameter("sensor_class", "tf_relative_pose")
        self.declare_parameter("degradation_class", "narrow_clearance")
        self.declare_parameter("recommended_speed_factor", 0.6)
        self.declare_parameter("ttl_sec", 5.0)
        self.declare_parameter("confidence", 0.75)

        self._robot_a_tf_topic = str(self.get_parameter("robot_a_tf_topic").value)
        self._robot_b_tf_topic = str(self.get_parameter("robot_b_tf_topic").value)
        self._base_child_frame = str(self.get_parameter("robot_base_child_frame").value)
        self._threshold = float(self.get_parameter("distance_threshold_m").value)
        self._poll_period_sec = float(self.get_parameter("poll_period_sec").value)
        self._min_publish_interval_sec = float(self.get_parameter("min_publish_interval_sec").value)
        self._source_robot_id = str(self.get_parameter("source_robot_id").value)
        self._sensor_class = str(self.get_parameter("sensor_class").value)
        self._degradation_class = str(self.get_parameter("degradation_class").value)
        self._recommended_speed_factor = float(self.get_parameter("recommended_speed_factor").value)
        self._ttl_sec = float(self.get_parameter("ttl_sec").value)
        self._confidence = float(self.get_parameter("confidence").value)
        self._publish_topic = str(self.get_parameter("publish_topic").value)

        self._last_publish_mono_sec = 0.0
        self._last_lookup_warn_mono_sec = 0.0
        self._pos_a: tuple[float, float, float] | None = None
        self._pos_b: tuple[float, float, float] | None = None

        self.create_subscription(TFMessage, self._robot_a_tf_topic, self._on_robot_a_tf, 10)
        self.create_subscription(TFMessage, self._robot_b_tf_topic, self._on_robot_b_tf, 10)
        self._pub = self.create_publisher(String, self._publish_topic, 10)
        self.create_timer(self._poll_period_sec, self._tick)
        self.get_logger().info(
            "R1 TF seam active: %s + %s child=%s threshold=%.2fm topic=%s"
            % (
                self._robot_a_tf_topic,
                self._robot_b_tf_topic,
                self._base_child_frame,
                self._threshold,
                self._publish_topic,
            )
        )

    def _extract_base_pos_from_tf(self, msg: TFMessage) -> tuple[float, float, float] | None:
        for tf in msg.transforms:
            child = str(tf.child_frame_id).strip()
            if child != self._base_child_frame and not child.endswith("/" + self._base_child_frame):
                continue
            parent = str(tf.header.frame_id).strip()
            if parent != "odom" and not parent.endswith("/odom"):
                continue
            tr = tf.transform.translation
            return (float(tr.x), float(tr.y), float(tr.z))
        return None

    def _on_robot_a_tf(self, msg: TFMessage) -> None:
        pos = self._extract_base_pos_from_tf(msg)
        if pos is not None:
            self._pos_a = pos

    def _on_robot_b_tf(self, msg: TFMessage) -> None:
        pos = self._extract_base_pos_from_tf(msg)
        if pos is not None:
            self._pos_b = pos

    def _tick(self) -> None:
        if self._pos_a is None or self._pos_b is None:
            now_mono = self.get_clock().now().nanoseconds / 1e9
            if now_mono - self._last_lookup_warn_mono_sec >= 5.0:
                self.get_logger().warn("R1 TF seam waiting for base pose transforms on both TF topics")
                self._last_lookup_warn_mono_sec = now_mono
            return

        dx = self._pos_a[0] - self._pos_b[0]
        dy = self._pos_a[1] - self._pos_b[1]
        dz = self._pos_a[2] - self._pos_b[2]
        distance = euclidean_distance_m(dx, dy, dz)
        if not should_emit_advisory(distance, self._threshold):
            return

        now_mono = self.get_clock().now().nanoseconds / 1e9
        if now_mono - self._last_publish_mono_sec < self._min_publish_interval_sec:
            return

        now_utc = datetime.now(timezone.utc)
        location_ref = seam_location_ref(self._robot_a_tf_topic, self._robot_b_tf_topic, self._threshold)
        asm = assemble_degraded_passage_record_from_candidate(
            DegradedPassageAssemblyCandidate(
                location_ref=location_ref,
                source_robot_id=self._source_robot_id,
                confidence=self._confidence,
                ttl_sec=self._ttl_sec,
                degradation_class=self._degradation_class,
                recommended_speed_factor=self._recommended_speed_factor,
                sensor_class=self._sensor_class,
            ),
            timestamp_utc=now_utc,
        )
        msg = String()
        msg.data = encode_degraded_passage_record_json(asm.record)
        self._pub.publish(msg)
        self._last_publish_mono_sec = now_mono
        self.get_logger().info(
            "R1 TF seam emitted degraded advisory (distance=%.2fm <= %.2fm location_ref=%s belief_id=%s)"
            % (distance, self._threshold, location_ref, asm.record["belief_id"])
        )


def main() -> None:
    rclpy.init()
    try:
        node = SpatialAdvisoryTfObserverR1()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
