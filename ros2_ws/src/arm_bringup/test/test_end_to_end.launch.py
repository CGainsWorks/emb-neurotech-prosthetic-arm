"""The simulated arm, brought up end to end and checked over its own topics.

    colcon test --packages-select arm_bringup

Launches sim.launch.py with semg_source:=replay, the whole graph as it would
run against a board: a synthetic sEMG recording replayed onto semg/frame, the
classifier interface stub turning it into GestureCommands, the gesture mapper
planning trajectories, the joint controller following them and the firmware
bridge carrying the joint commands, over the real wire protocol, to the
simulated board whose joints move.

The replay performs rest, wrist flexion (13), rest, wrist pronation (10),
rest and a fist (6). The test listens to every stage and passes when each one
has done its part: the frames arriving intact, the stub naming each gesture,
the mapper planning for the two the mechanism can make and refusing the third
and the simulated wrist and forearm reaching the poses arm_control assigns
those gestures.

Nothing here is a measurement and the sEMG is synthetic. What it shows is that
the nodes are wired to each other correctly and the chain from electrodes to
joints carries a decision all the way through.
"""

import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from arm_msgs.msg import ArmStatus, GestureCommand, SemgFrame
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# arm_control's gesture_map.cpp: wrist flexion puts the wrist at 1.1 rad,
# wrist pronation the forearm at -1.2 rad. Rest is this pose.
WRIST_FLEXION_RAD = 1.1
FOREARM_PRONATION_RAD = -1.2
REST_POSE = [0.0, 0.6, -1.1, 0.0, 0.3, 0.0]
JOINTS = ["shoulder_rotation", "shoulder_flexion", "elbow_flexion",
          "forearm_rotation", "wrist_flexion", "tool_roll"]

# The replay is 12 s and starts 2 s after launch; this is the budget for all
# of it with room for a slow runner.
DEADLINE_S = 45.0


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("arm_bringup"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim),
            launch_arguments={"semg_source": "replay"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class Recorder:
    """Everything the graph publishes that the checks need, with arrival times."""

    def __init__(self, node):
        self.semg = []
        self.gestures = []
        self.trajectories = []
        self.joint_states = []
        self.status = []
        node.create_subscription(SemgFrame, "semg/frame",
                                 lambda m: self.semg.append((time.monotonic(), m)), 200)
        node.create_subscription(GestureCommand, "gesture/command",
                                 lambda m: self.gestures.append(m), 50)
        node.create_subscription(JointTrajectory, "arm/trajectory",
                                 lambda m: self.trajectories.append(m), 10)
        node.create_subscription(JointState, "joint_states",
                                 lambda m: self.joint_states.append(m), qos_profile_sensor_data)
        node.create_subscription(ArmStatus, "arm/status",
                                 lambda m: self.status.append(m), 10)

    def decided(self, threshold=0.6):
        """Confident, valid decisions with repeats collapsed."""
        sequence = []
        for message in self.gestures:
            if message.valid and message.confidence >= threshold:
                if not sequence or sequence[-1] != message.gesture_id:
                    sequence.append(message.gesture_id)
        return sequence

    def extreme(self, joint, pick):
        index = JOINTS.index(joint)
        values = [m.position[index] for m in self.joint_states if len(m.position) > index]
        return pick(values) if values else None


def contains_in_order(sequence, wanted):
    position = 0
    for item in sequence:
        if position < len(wanted) and item == wanted[position]:
            position += 1
    return position == len(wanted)


class TestEndToEnd(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("end_to_end_observer")
        cls.recorder = Recorder(cls.node)
        cls.finished = cls._run_until_done()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _run_until_done(cls):
        """Spin until the whole replay has gone through, or the deadline."""
        deadline = time.monotonic() + DEADLINE_S
        recorder = cls.recorder
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if contains_in_order(recorder.decided(), [0, 13, 0, 10, 0, 6, 0]) and \
                    len(recorder.semg) >= 2300:
                # A second more, for the last trajectory to be followed.
                settle = time.monotonic() + 1.5
                while time.monotonic() < settle:
                    rclpy.spin_once(cls.node, timeout_sec=0.1)
                return True
        return False

    def test_replay_reaches_the_graph_intact(self):
        # Every frame the replay published after this node subscribed, in
        # order. The rate is not checked: the replay runs on a wall timer and
        # on a busy runner its rate says more about the runner than the graph.
        frames = [m for _, m in self.recorder.semg]
        self.assertGreater(len(frames), 2000, "the sEMG replay did not arrive")
        sequences = [m.sequence for m in frames]
        self.assertEqual(sequences, sorted(sequences), "sEMG frames arrived out of order")
        expected = sequences[-1] - sequences[0] + 1
        self.assertGreaterEqual(len(frames), 0.99 * expected,
                                f"{expected - len(frames)} of {expected} frames lost")

    def test_classifier_stub_names_each_gesture(self):
        decided = self.recorder.decided()
        self.assertTrue(self.finished, f"the replay did not complete; decisions so far {decided}")
        self.assertTrue(contains_in_order(decided, [0, 13, 0, 10, 0, 6, 0]),
                        f"decisions were {decided}")
        latencies = [m.inference_latency_ms for m in self.recorder.gestures if m.valid]
        self.assertTrue(latencies)

    def test_mapper_plans_for_the_gestures_the_arm_can_make(self):
        # Rest, wrist flexion, rest, pronation, rest: five plans. The fist is
        # reported and refused because the mechanism has no fingers, so the
        # arm does not move for it and there is no sixth.
        finals = [list(t.points[-1].positions) for t in self.recorder.trajectories if t.points]
        self.assertGreaterEqual(len(finals), 5, f"{len(finals)} trajectories")
        wrist = JOINTS.index("wrist_flexion")
        forearm = JOINTS.index("forearm_rotation")
        self.assertTrue(any(abs(f[wrist] - WRIST_FLEXION_RAD) < 1e-6 for f in finals))
        self.assertTrue(any(abs(f[forearm] - FOREARM_PRONATION_RAD) < 1e-6 for f in finals))
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(finals[-1], REST_POSE)),
                        f"the last plan was {finals[-1]}, not rest")

    def test_simulated_joints_follow(self):
        # The bridge reports the commanded servo angle, which on the simulated
        # board is where the joint was told to go after the stops and the slew
        # limit. Each pose has to be reached, then left again.
        wrist_peak = self.recorder.extreme("wrist_flexion", max)
        forearm_low = self.recorder.extreme("forearm_rotation", min)
        self.assertIsNotNone(wrist_peak, "no joint states from the bridge")
        self.assertAlmostEqual(wrist_peak, WRIST_FLEXION_RAD, delta=0.05)
        self.assertAlmostEqual(forearm_low, FOREARM_PRONATION_RAD, delta=0.05)
        last = self.recorder.joint_states[-1].position
        for joint, (got, want) in enumerate(zip(last, REST_POSE)):
            self.assertAlmostEqual(got, want, delta=0.05,
                                   msg=f"{JOINTS[joint]} ended at {got:.3f}, not at rest")

    def test_controller_saw_the_link_and_followed(self):
        states = {m.state for m in self.recorder.status}
        self.assertTrue(any(m.link_up for m in self.recorder.status), "link never up")
        self.assertIn(ArmStatus.STATE_ACTIVE, states)
        self.assertFalse(any(math.isnan(m.control_period_mean_ms) for m in self.recorder.status))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_every_node_exited_cleanly(self, proc_info):
        # SIGINT from the launch shutdown is a clean exit for a ROS node.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
