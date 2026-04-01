"""
tunnel_targeting/targeting_node.py
===================================
Targeting Node
--------------
Responsibility:
  • Subscribe to a raw camera feed (sensor_msgs/Image).
  • Detect the docking target (stub — replace with your real library call).
  • Publish a TargetPose when a target is found.
  • Publish a std_msgs/Bool indicating whether the target is reachable
    (calls the reachability checker stub below).

Topics
------
  Sub : /camera/image_raw          (sensor_msgs/Image)
  Pub : /tunnel/target_pose        (tunnel_msgs/TargetPose)
  Pub : /tunnel/target_reachable   (std_msgs/Bool)

Parameters
----------
  confidence_threshold (float, default 0.6)
      Minimum detection confidence to publish a target.
  detection_rate_hz (float, default 10.0)
      Rate at which to re-check / re-publish.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from tunnel_msgs.msg import TargetPose


# FIXME: Integrate with visualization code
# ---------------------------------------------------------------------------
# STUB — replace with real detection library
# ---------------------------------------------------------------------------
def detect_target(image_msg) -> dict | None:
    """
    Analyse an image and return a detection dict, or None if nothing found.

    Expected return format:
        {
            "pose":       geometry_msgs.msg.Pose,
            "confidence": float,          # 0.0 – 1.0
            "target_id":  str,
        }

    Replace this function body with your actual vision pipeline.
    """
    # Placeholder: pretend we always see a target straight ahead at 1 m
    pose = Pose(
        position=Point(x=1.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return {"pose": pose, "confidence": 0.95, "target_id": "stub_target"}


# FIXME: Replace with actual reachability & collision logic
def is_target_reachable(target_pose: Pose) -> bool:
    """
    Stub reachability check — replace with workspace-boundary logic.

    Typically you will:
      1. Convert the target pose to the platform frame.
      2. Verify all cable lengths are within [min_length, max_length].
      3. Check that no mechanical limits are violated.
    """
    # Placeholder: always reachable
    return True


# ---------------------------------------------------------------------------


class TargetingNode(Node):
    def __init__(self):
        super().__init__("targeting_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("confidence_threshold", 0.6)
        self.declare_parameter("detection_rate_hz", 10.0)

        self.conf_thresh = (
            self.get_parameter("confidence_threshold")
            .get_parameter_value()
            .double_value
        )
        rate_hz = (
            self.get_parameter("detection_rate_hz").get_parameter_value().double_value
        )

        # ── State ────────────────────────────────────────────────────────────
        self._latest_image: Image | None = None

        # ── Subscriber ──────────────────────────────────────────────────────
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/camera/image_raw", self._image_cb, cam_qos)

        # ── Publishers ───────────────────────────────────────────────────────
        self._target_pub = self.create_publisher(TargetPose, "/tunnel/target_pose", 10)
        self._reachable_pub = self.create_publisher(
            Bool, "/tunnel/target_reachable", 10
        )

        # ── Detection timer ──────────────────────────────────────────────────
        self.create_timer(1.0 / rate_hz, self._detection_tick)

        self.get_logger().info(
            f"TargetingNode ready (conf_threshold={self.conf_thresh}, "
            f"rate={rate_hz} Hz)"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        self._latest_image = msg

    def _detection_tick(self):
        if self._latest_image is None:
            return

        detection = detect_target(self._latest_image)

        reachable_msg = Bool()

        if detection is None or detection["confidence"] < self.conf_thresh:
            # No valid target — publish invalid TargetPose + not reachable
            tp = TargetPose()
            tp.header.stamp = self.get_clock().now().to_msg()
            tp.header.frame_id = "world"
            tp.is_valid = False
            tp.confidence = detection["confidence"] if detection else 0.0
            self._target_pub.publish(tp)

            reachable_msg.data = False
            self._reachable_pub.publish(reachable_msg)
            return

        # Valid detection
        tp = TargetPose()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = "world"
        tp.pose = detection["pose"]
        tp.confidence = detection["confidence"]
        tp.target_id = detection["target_id"]
        tp.is_valid = True
        self._target_pub.publish(tp)

        reachable = is_target_reachable(detection["pose"])
        reachable_msg.data = reachable
        self._reachable_pub.publish(reachable_msg)

        self.get_logger().debug(
            f"Target '{tp.target_id}' conf={tp.confidence:.2f} reachable={reachable}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
