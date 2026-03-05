"""
tunnel_controller/controller_node.py
=====================================
Controller Node
---------------
Implements the closed-loop control loop described as:

  input → compute_error → compensator → plant
       ↑                                     |
       └── feedback_elements ← at_target? ←──┘
              (determine EE pose, compute leg lengths)

Responsibility:
  • Subscribe to /tunnel/ik_solution  — desired cable lengths from IK.
  • Subscribe to /tunnel/platform_state — current measured state (feedback).
  • Run a PD compensator per cable.
  • Publish /tunnel/motor_commands (Float64MultiArray) to the plant.
  • Publish /tunnel/platform_state with the updated pose estimate.
  • Detect docking completion and publish system phase transitions.

Topics
------
  Sub : /tunnel/ik_solution      (tunnel_msgs/IKSolution)
  Sub : /tunnel/platform_state   (tunnel_msgs/PlatformState)  [own output, loopback]
  Pub : /tunnel/motor_commands   (std_msgs/Float64MultiArray)
  Pub : /tunnel/platform_state   (tunnel_msgs/PlatformState)

Parameters
----------
  num_cables          (int,   default 6)
  control_rate_hz     (float, default 50.0)
  kp                  (float, default 1.0)   — proportional gain
  kd                  (float, default 0.05)  — derivative gain
  at_target_threshold (float, default 0.005) — [m] per-cable error for "at target"
  docking_threshold   (float, default 0.002) — tighter threshold for docking complete
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose
from tunnel_msgs.msg import IKSolution, PlatformState


# ---------------------------------------------------------------------------
# Compensator — one PD controller per cable
# ---------------------------------------------------------------------------


class PDController:
    """Simple per-axis PD controller."""

    def __init__(self, kp: float, kd: float, dt: float):
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self._prev_error = 0.0

    def compute(self, error: float) -> float:
        derivative = (error - self._prev_error) / self.dt
        self._prev_error = error
        return self.kp * error + self.kd * derivative

    def reset(self):
        self._prev_error = 0.0


# ---------------------------------------------------------------------------
# Feedback elements — determine end-effector pose from cable lengths
# ---------------------------------------------------------------------------


def estimate_ee_pose_from_lengths(leg_lengths: np.ndarray) -> Pose:
    """
    Stub: derive end-effector pose from measured cable lengths (FK).
    Replace with your real FK solver (same one used in ik_node).
    """

    pose = Pose()
    pose.orientation.w = 1.0
    # FIXME: call real FK
    return pose


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("num_cables", 6)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("at_target_threshold", 0.005)
        self.declare_parameter("docking_threshold", 0.002)

        n = self.get_parameter("num_cables").get_parameter_value().integer_value
        rate_hz = (
            self.get_parameter("control_rate_hz").get_parameter_value().double_value
        )
        kp = self.get_parameter("kp").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value
        self._at_target_thresh = (
            self.get_parameter("at_target_threshold").get_parameter_value().double_value
        )
        self._docking_thresh = (
            self.get_parameter("docking_threshold").get_parameter_value().double_value
        )

        dt = 1.0 / rate_hz
        self._controllers = [PDController(kp, kd, dt) for _ in range(n)]
        self._num_cables = n

        # ── State ────────────────────────────────────────────────────────────
        self._desired_lengths: np.ndarray | None = None
        self._measured_lengths: np.ndarray = np.zeros(n)
        self._phase = "IDLE"
        self._ik_success = True

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(IKSolution, "/tunnel/ik_solution", self._ik_cb, 10)
        self.create_subscription(
            PlatformState, "/tunnel/platform_state", self._state_cb, 10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._motor_pub = self.create_publisher(
            Float64MultiArray, "/tunnel/motor_commands", 10
        )
        self._state_pub = self.create_publisher(
            PlatformState, "/tunnel/platform_state", 10
        )

        # ── Control timer ────────────────────────────────────────────────────
        self.create_timer(dt, self._control_tick)

        self.get_logger().info(
            f"ControllerNode ready | {n} cables | "
            f"kp={kp} kd={kd} | rate={rate_hz} Hz"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _ik_cb(self, msg: IKSolution):
        if not msg.success:
            self.get_logger().warn(f"Received failed IK solution: {msg.failure_reason}")
            self._ik_success = False
            self._transition("FAULT")
            return

        self._ik_success = True
        self._desired_lengths = np.array(msg.leg_lengths)

        # Reset integrators / derivative state when a new goal arrives
        for ctrl in self._controllers:
            ctrl.reset()

        self._transition("MOVING")

    def _state_cb(self, msg: PlatformState):
        """
        Receive feedback from plant / sensors.
        In practice this comes from encoder reads or cable-length sensors.
        """
        if msg.current_leg_lengths:
            self._measured_lengths = np.array(msg.current_leg_lengths)

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_tick(self):
        if self._phase in ("IDLE", "FAULT", "DONE"):
            return
        if self._desired_lengths is None:
            return

        # 1. Compute current error
        errors = self._desired_lengths - self._measured_lengths

        # 2. Compensator → motor commands
        commands = np.array(
            [ctrl.compute(err) for ctrl, err in zip(self._controllers, errors)]
        )

        # 3. Send to plant
        self._publish_motor_commands(commands)

        # 4. Feedback: estimate end-effector pose from current leg lengths
        ee_pose = estimate_ee_pose_from_lengths(self._measured_lengths)

        # 5. Publish state
        state = PlatformState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "world"
        state.current_pose = ee_pose
        state.current_leg_lengths = self._measured_lengths.tolist()
        state.leg_errors = errors.tolist()
        state.pose_error = float(np.linalg.norm(errors))
        state.phase = self._phase

        # 6. At-target check
        max_err = float(np.max(np.abs(errors)))

        if self._phase == "MOVING" and max_err < self._at_target_thresh:
            self._transition("DOCKING")

        if self._phase == "DOCKING" and max_err < self._docking_thresh:
            self._transition("DONE")
            state.at_target = True

        state.phase = self._phase
        self._state_pub.publish(state)

        self.get_logger().debug(
            f"[{self._phase}] max_err={max_err:.4f} m | "
            f"cmds={[f'{c:.3f}' for c in commands]}"
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _publish_motor_commands(self, commands: np.ndarray):
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = "cables"
        dim.size = self._num_cables
        dim.stride = self._num_cables
        msg.layout.dim = [dim]
        msg.data = commands.tolist()
        self._motor_pub.publish(msg)

    def _transition(self, new_phase: str):
        if new_phase != self._phase:
            self.get_logger().info(f"Phase: {self._phase} → {new_phase}")
            self._phase = new_phase


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
