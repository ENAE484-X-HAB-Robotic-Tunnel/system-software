"""
launch/tunnel_system.launch.py
================================
Launches the complete tunnel robot pipeline:
  1. system_manager_node   — mission executive
  2. targeting_node        — visual target detection
  3. ik_node               — trajectory + IK solver
  4. controller_node       — closed-loop cable controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Configurable arguments ───────────────────────────────────────────────
    num_cables_arg = DeclareLaunchArgument(
        "num_cables", default_value="6", description="Number of cable legs"
    )
    anchor_radius_arg = DeclareLaunchArgument(
        "anchor_radius", default_value="1.0", description="Anchor ring radius [m]"
    )
    platform_radius_arg = DeclareLaunchArgument(
        "platform_radius", default_value="0.3", description="Platform ring radius [m]"
    )
    kp_arg = DeclareLaunchArgument("kp", default_value="1.0", description="PD kp gain")
    kd_arg = DeclareLaunchArgument("kd", default_value="0.05", description="PD kd gain")

    # ── Nodes ────────────────────────────────────────────────────────────────
    system_manager = Node(
        package="tunnel_control",
        executable="system_manager_node",
        name="system_manager_node",
        output="screen",
        parameters=[{"watchdog_timeout_sec": 2.0}],
    )

    targeting = Node(
        package="tunnel_targeting",
        executable="targeting_node",
        name="targeting_node",
        output="screen",
        parameters=[
            {"confidence_threshold": 0.6},
            {"detection_rate_hz": 10.0},
        ],
    )

    ik = Node(
        package="tunnel_ik",
        executable="ik_node",
        name="ik_node",
        output="screen",
        parameters=[
            {"num_cables": LaunchConfiguration("num_cables")},
            {"anchor_radius": LaunchConfiguration("anchor_radius")},
            {"platform_radius": LaunchConfiguration("platform_radius")},
            {"min_cable_length": 0.1},
            {"max_cable_length": 3.0},
        ],
    )

    controller = Node(
        package="tunnel_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {"num_cables": LaunchConfiguration("num_cables")},
            {"control_rate_hz": 50.0},
            {"kp": LaunchConfiguration("kp")},
            {"kd": LaunchConfiguration("kd")},
            {"at_target_threshold": 0.005},
            {"docking_threshold": 0.002},
        ],
    )

    return LaunchDescription(
        [
            num_cables_arg,
            anchor_radius_arg,
            platform_radius_arg,
            kp_arg,
            kd_arg,
            system_manager,
            targeting,
            ik,
            controller,
        ]
    )
