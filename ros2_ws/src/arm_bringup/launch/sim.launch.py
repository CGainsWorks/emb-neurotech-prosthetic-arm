"""The whole arm, over ROS 2 topics, against the simulated board.

    ros2 launch arm_bringup sim.launch.py

Starts the description, the bridge (simulated backend), the gesture mapper, the
joint controller and a scripted gesture source. Nothing here needs hardware and
nothing here is a measurement: the board and the wearer are models and the
sEMG is synthetic.

For the version that produces numbers rather than pictures, run

    ros2 run arm_bringup sim_demo

which drives the same libraries directly, deterministically and writes JSON.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("arm_description")
    control_share = get_package_share_directory("arm_control")

    urdf_path = os.path.join(description_share, "urdf", "a1_arm.urdf")
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    parameters = os.path.join(control_share, "config", "control.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="sim",
            description="sim for the simulated board, serial for a real one."),
        DeclareLaunchArgument(
            "port", default_value="/dev/ttyUSB0",
            description="Serial device, when backend is serial."),
        DeclareLaunchArgument(
            "frame_loss", default_value="0.0",
            description="Fraction of frames the simulated link drops."),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="arm_interface",
            executable="firmware_bridge",
            output="screen",
            parameters=[
                parameters,
                {
                    "backend": LaunchConfiguration("backend"),
                    "port": LaunchConfiguration("port"),
                    "sim.frame_loss": LaunchConfiguration("frame_loss"),
                },
            ],
        ),
        Node(
            package="arm_control",
            executable="gesture_mapper",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="arm_control",
            executable="joint_controller",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="arm_bringup",
            executable="gesture_replay",
            output="screen",
        ),
    ])
