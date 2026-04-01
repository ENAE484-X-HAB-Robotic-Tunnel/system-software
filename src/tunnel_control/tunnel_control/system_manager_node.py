"""
tunnel_control/system_manager_node.py
======================================
System Manager (Mission Executive)
------------------------------------
This is the top-level orchestration node.  It owns the high-level state
machine that drives the full pipeline:

  IDLE → TARGETING → REACHABILITY_CHECK → TRAJECTORY → IK
       → MOVING → DOCKING → DONE  (or → FAULT at any stage)

It does NOT do signal processing or control maths — it watches the outputs
of every other node and decides what phase the mission is in, logging
transitions and emitting a /tunnel/system_status string for the operator.

Topics
------
  Sub : /tunnel/target_pose        (tunnel_msgs/TargetPose)
  Sub : /tunnel/target_reachable   (std_msgs/Bool)
  Sub : /tunnel/ik_solution        (tunnel_msgs/IKSolution)
  Sub : /tunnel/platform_state     (tunnel_msgs/PlatformState)
  Pub : /tunnel/system_status      (std_msgs/String)   — human-readable
  Pub : /tunnel/mission_phase      (std_msgs/String)   — machine-readable enum

Parameters
----------
  watchdog_timeout_sec (float, default 2.0)
      If no target is seen for this many seconds while in TARGETING, emit a
      FAULT.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from tunnel_msgs.msg import TargetPose, IKSolution, PlatformState


# Mission phases (string constants so they read cleanly in logs / tools)
class Phase:
    IDLE = "IDLE"
    TARGETING = "TARGETING"
    REACHABILITY = "REACHABILITY_CHECK"
    IK_PENDING = "IK_PENDING"
    MOVING = "MOVING"
    DOCKING = "DOCKING"
    DONE = "DONE"
    FAULT = "FAULT"


class SystemManagerNode(Node):
    def __init__(self):
        super().__init__("system_manager_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("watchdog_timeout_sec", 2.0)
        self._watchdog_timeout = (
            self.get_parameter("watchdog_timeout_sec")
            .get_parameter_value()
            .double_value
        )

        # ── State ────────────────────────────────────────────────────────────
        self._phase = Phase.IDLE
        self._last_target_time: float | None = None
        self._target_reachable = False
        self._ik_ready = False

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(TargetPose, "/tunnel/target_pose", self._target_cb, 10)
        self.create_subscription(
            Bool, "/tunnel/target_reachable", self._reachable_cb, 10
        )
        self.create_subscription(IKSolution, "/tunnel/ik_solution", self._ik_cb, 10)
        self.create_subscription(
            PlatformState, "/tunnel/platform_state", self._state_cb, 10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "/tunnel/system_status", 10)
        self._phase_pub = self.create_publisher(String, "/tunnel/mission_phase", 10)

        # ── Heartbeat / watchdog timer ───────────────────────────────────────
        self.create_timer(0.5, self._heartbeat)

        # Kick off the mission
        self._transition(Phase.TARGETING)
        self.get_logger().info("SystemManager ready — mission started.")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _target_cb(self, msg: TargetPose):
        if msg.is_valid:
            self._last_target_time = self.get_clock().now().nanoseconds * 1e-9
            if self._phase == Phase.TARGETING:
                self._transition(Phase.REACHABILITY)
        else:
            if self._phase == Phase.REACHABILITY:
                self._transition(Phase.TARGETING)

    def _reachable_cb(self, msg: Bool):
        self._target_reachable = msg.data
        if self._phase == Phase.REACHABILITY:
            if msg.data:
                self._transition(Phase.IK_PENDING)
            else:
                self.get_logger().warn("Target not reachable — aborting.")
                self._transition(Phase.DONE)  # graceful stop (not a fault)

    def _ik_cb(self, msg: IKSolution):
        if self._phase == Phase.IK_PENDING:
            if msg.success:
                self._ik_ready = True
                self._transition(Phase.MOVING)
            else:
                self.get_logger().error(f"IK failed: {msg.failure_reason}")
                self._transition(Phase.FAULT)

    def _state_cb(self, msg: PlatformState):
        # Mirror the controller's fine-grained phase transitions
        if msg.phase == "DOCKING" and self._phase == Phase.MOVING:
            self._transition(Phase.DOCKING)
        elif msg.phase == "DONE" and self._phase == Phase.DOCKING:
            self._transition(Phase.DONE)
        elif msg.phase == "FAULT":
            self._transition(Phase.FAULT)

    # ── Watchdog ─────────────────────────────────────────────────────────────

    def _heartbeat(self):
        # Publish current phase for monitoring tools
        self._publish_phase()

        # Watchdog: fault if we lose target while actively targeting
        if self._phase == Phase.TARGETING and self._last_target_time is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_target_time > self._watchdog_timeout:
                self.get_logger().warn(
                    f"No valid target for {self._watchdog_timeout}s — FAULT."
                )
                self._transition(Phase.FAULT)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _transition(self, new_phase: str):
        if new_phase == self._phase:
            return
        old = self._phase
        self._phase = new_phase
        msg = f"Mission phase: {old} → {new_phase}"
        self.get_logger().info(msg)
        status = String()
        status.data = msg
        self._status_pub.publish(status)
        self._publish_phase()

    def _publish_phase(self):
        p = String()
        p.data = self._phase
        self._phase_pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = SystemManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
