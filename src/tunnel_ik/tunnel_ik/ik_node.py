"""
tunnel_ik/ik_node.py
=====================
Simulation / IK Node
--------------------
Responsibility:
  • Subscribe to /tunnel/target_pose and /tunnel/target_reachable.
  • When a reachable target arrives, compute a trajectory and solve IK.
  • Publish an IKSolution (cable lengths + velocities) for the controller.
  • Expose FK as a helper (used internally for error feedback).

Topics
------
  Sub : /tunnel/target_pose      (tunnel_msgs/TargetPose)
  Sub : /tunnel/target_reachable (std_msgs/Bool)
  Sub : /tunnel/platform_state   (tunnel_msgs/PlatformState)  — for FK feedback
  Pub : /tunnel/ik_solution      (tunnel_msgs/IKSolution)

Parameters
----------
  num_cables        (int,   default 6)  — number of cable legs
  anchor_radius     (float, default 1.0) — anchor ring radius [m]
  platform_radius   (float, default 0.3) — end-effector ring radius [m]
  min_cable_length  (float, default 0.1) — hard lower limit [m]
  max_cable_length  (float, default 3.0) — hard upper limit [m]
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from tunnel_msgs.msg import TargetPose, IKSolution, PlatformState


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def quaternion_to_matrix(q: Quaternion) -> np.ndarray:
    """Convert a ROS Quaternion to a 3×3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def build_anchor_points(num_cables: int, radius: float) -> np.ndarray:
    """
    Evenly-spaced anchor attachment points on a circle in the XY plane.
    Returns shape (num_cables, 3).
    """
    angles = np.linspace(0, 2 * math.pi, num_cables, endpoint=False)
    return np.column_stack(
        [
            radius * np.cos(angles),
            radius * np.sin(angles),
            np.zeros(num_cables),
        ]
    )


def build_platform_points(num_cables: int, radius: float) -> np.ndarray:
    """
    Evenly-spaced platform attachment points (in the platform's local frame).
    Returns shape (num_cables, 3).
    """
    angles = np.linspace(0, 2 * math.pi, num_cables, endpoint=False)
    return np.column_stack(
        [
            radius * np.cos(angles),
            radius * np.sin(angles),
            np.zeros(num_cables),
        ]
    )


# FIXME: replace with Frank's IK/FK calculations
# ---------------------------------------------------------------------------
# IK / FK stubs — replace with your teammate's implementation
# ---------------------------------------------------------------------------


def compute_ik(
    target_pose: Pose,
    anchor_pts: np.ndarray,
    platform_pts: np.ndarray,
    min_len: float,
    max_len: float,
) -> tuple[bool, np.ndarray, str]:
    """
    Stub cable-driven IK.

    For a cable-driven Stewart-like platform the IK is *analytic*:
      For each cable i:
        p_world_i = R @ platform_pts[i] + t        (platform attachment in world)
        L_i = ||anchor_pts[i] - p_world_i||

    Returns
    -------
    (success, leg_lengths, failure_reason)
    """
    t = np.array(
        [target_pose.position.x, target_pose.position.y, target_pose.position.z]
    )
    R = quaternion_to_matrix(target_pose.orientation)

    lengths = []
    for a_pt, p_pt in zip(anchor_pts, platform_pts):
        p_world = R @ p_pt + t
        length = float(np.linalg.norm(a_pt - p_world))
        lengths.append(length)

    lengths = np.array(lengths)

    # Feasibility check
    if np.any(lengths < min_len):
        return False, lengths, f"Cable(s) below min length {min_len} m"
    if np.any(lengths > max_len):
        return False, lengths, f"Cable(s) exceed max length {max_len} m"

    return True, lengths, ""


def compute_fk(
    leg_lengths: np.ndarray,
    anchor_pts: np.ndarray,
    platform_pts: np.ndarray,
) -> Pose:
    """
    Stub forward kinematics.

    Real implementation: solve via numerical optimisation or lookup table.
    This stub returns a zeroed pose — replace with your teammate's solver.
    """
    pose = Pose()
    pose.orientation.w = 1.0  # identity rotation
    # FIXME: replace with actual FK computation
    return pose


# FIXME: implement proper trajectory planning
def plan_trajectory(
    current_pose: Pose,
    target_pose: Pose,
    num_steps: int = 10,
) -> list[Pose]:
    """
    Stub trajectory planner.
    Returns a linearly-interpolated list of poses from current to target.
    Replace with a proper motion planner (trapezoidal profile, CHOMP, etc.).
    """
    steps = []
    for i in range(1, num_steps + 1):
        alpha = i / num_steps
        p = Pose()
        p.position.x = (
            current_pose.position.x * (1 - alpha) + target_pose.position.x * alpha
        )
        p.position.y = (
            current_pose.position.y * (1 - alpha) + target_pose.position.y * alpha
        )
        p.position.z = (
            current_pose.position.z * (1 - alpha) + target_pose.position.z * alpha
        )
        # Orientation: SLERP stub — keep target orientation throughout
        p.orientation = target_pose.orientation
        steps.append(p)
    return steps


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("num_cables", 6)
        self.declare_parameter("anchor_radius", 1.0)
        self.declare_parameter("platform_radius", 0.3)
        self.declare_parameter("min_cable_length", 0.1)
        self.declare_parameter("max_cable_length", 3.0)

        n = self.get_parameter("num_cables").get_parameter_value().integer_value
        a_r = self.get_parameter("anchor_radius").get_parameter_value().double_value
        p_r = self.get_parameter("platform_radius").get_parameter_value().double_value
        self._min_len = (
            self.get_parameter("min_cable_length").get_parameter_value().double_value
        )
        self._max_len = (
            self.get_parameter("max_cable_length").get_parameter_value().double_value
        )

        self._anchor_pts = build_anchor_points(n, a_r)
        self._platform_pts = build_platform_points(n, p_r)

        # ── State ────────────────────────────────────────────────────────────
        self._target_reachable = False
        self._current_pose = Pose()
        self._current_pose.orientation.w = 1.0
        self._trajectory: list[Pose] = []
        self._traj_index = 0

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(TargetPose, "/tunnel/target_pose", self._target_cb, 10)
        self.create_subscription(
            Bool, "/tunnel/target_reachable", self._reachable_cb, 10
        )
        self.create_subscription(
            PlatformState, "/tunnel/platform_state", self._state_cb, 10
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self._ik_pub = self.create_publisher(IKSolution, "/tunnel/ik_solution", 10)

        self.get_logger().info(
            f"IKNode ready | {n} cables | " f"anchor_r={a_r} m | platform_r={p_r} m"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _reachable_cb(self, msg: Bool):
        self._target_reachable = msg.data

    def _state_cb(self, msg: PlatformState):
        self._current_pose = msg.current_pose

    def _target_cb(self, msg: TargetPose):
        if not msg.is_valid:
            return
        if not self._target_reachable:
            self.get_logger().info(
                "Target detected but marked unreachable — skipping IK."
            )
            return

        # Plan trajectory from current pose to target
        self._trajectory = plan_trajectory(self._current_pose, msg.pose)
        self._traj_index = 0
        self.get_logger().info(
            f"New trajectory planned: {len(self._trajectory)} waypoints "
            f"→ target '{msg.target_id}'"
        )
        self._publish_next_waypoint()

    # ── Internal ─────────────────────────────────────────────────────────────

    def _publish_next_waypoint(self):
        """Solve IK for the next trajectory waypoint and publish."""
        if self._traj_index >= len(self._trajectory):
            self.get_logger().info("Trajectory complete.")
            return

        waypoint = self._trajectory[self._traj_index]
        self._traj_index += 1

        success, lengths, reason = compute_ik(
            waypoint,
            self._anchor_pts,
            self._platform_pts,
            self._min_len,
            self._max_len,
        )

        sol = IKSolution()
        sol.header.stamp = self.get_clock().now().to_msg()
        sol.header.frame_id = "world"
        sol.success = success
        sol.leg_lengths = lengths.tolist()
        sol.leg_velocities = [0.0] * len(lengths)  # TODO: velocity profile
        sol.end_effector_pose = compute_fk(
            lengths, self._anchor_pts, self._platform_pts
        )
        sol.failure_reason = reason

        self._ik_pub.publish(sol)

        if not success:
            self.get_logger().warn(
                f"IK failed at waypoint {self._traj_index}: {reason}"
            )
        else:
            self.get_logger().debug(
                f"IK waypoint {self._traj_index}/{len(self._trajectory)} → "
                f"lengths={[f'{l:.3f}' for l in lengths]}"
            )

    def advance_trajectory(self):
        """Called by the controller (via state feedback) when a waypoint is reached."""
        self._publish_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
