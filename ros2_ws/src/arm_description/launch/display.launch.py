"""Publish the arm's transforms from the joint states on /joint_states.

Nothing here simulates anything. It is robot_state_publisher and the
description, which is what RViz needs to draw the arm; the joint states come
from arm_interface, whether that is talking to a board or to the simulated
device. arm_bringup/launch/sim.launch.py starts the whole thing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("arm_description")
    urdf_path = os.path.join(share, "urdf", "a1_arm.urdf")

    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Follow /clock rather than the wall clock."),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
