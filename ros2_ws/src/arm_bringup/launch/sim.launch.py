"""The whole arm, over ROS 2 topics, against the simulated board.

    ros2 launch arm_bringup sim.launch.py                      # scripted gestures
    ros2 launch arm_bringup sim.launch.py semg_source:=replay  # sEMG in, joints out

Starts the description, the bridge (simulated backend), the gesture mapper and
the joint controller, plus a source of gesture decisions:

  script   gesture_replay publishes decisions directly, from a sequence in
           config/demo.yaml. The classifier and the sEMG play no part.
  replay   semg_replay plays a synthetic sEMG file onto semg/frame and
           classifier_stub turns it into decisions behind the classifier's
           interface. The whole chain runs: sEMG, decision, trajectory, joint
           command, simulated joints. test/test_end_to_end.launch.py drives
           this mode.

Nothing here needs hardware and nothing here is a measurement: the board and
the wearer are models and the sEMG is synthetic.

For the version that produces numbers rather than pictures, run

    ros2 run arm_bringup sim_demo

which drives the same libraries directly, deterministically and writes JSON.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("arm_description")
    control_share = get_package_share_directory("arm_control")

    urdf_path = os.path.join(description_share, "urdf", "a1_arm.urdf")
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    parameters = os.path.join(control_share, "config", "control.yaml")
    demo = os.path.join(get_package_share_directory("arm_bringup"), "config", "demo.yaml")

    source = LaunchConfiguration("semg_source")
    replaying = PythonExpression(["'", source, "' == 'replay'"])
    scripted = PythonExpression(["'", source, "' == 'script'"])

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
        DeclareLaunchArgument(
            "semg_source", default_value="script", choices=["script", "replay"],
            description="script: scripted gesture decisions. replay: a synthetic "
                        "sEMG replay through the classifier interface stub."),

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
                    # The replay is the sEMG source in replay mode.
                    "publish_semg": PythonExpression(["'", source, "' != 'replay'"]),
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
            parameters=[demo],
            condition=IfCondition(scripted),
        ),
        Node(
            package="arm_bringup",
            executable="semg_replay",
            output="screen",
            parameters=[demo],
            condition=IfCondition(replaying),
        ),
        Node(
            package="arm_bringup",
            executable="classifier_stub",
            output="screen",
            parameters=[demo],
            condition=IfCondition(replaying),
        ),
    ])
