"""Robot Framework keywords that check what the emulated board produced.

The capture is decoded by pa_link, which is built from the firmware's own
protocol code, so these checks read the board's output exactly as the ROS 2
bridge would. Everything here is about an emulated board: a pass says the
firmware image behaves as designed on Renode's model of an STM32H743, not that
it does on silicon.
"""

import json
import math
import os
import subprocess

# firmware/src/main.cpp, calibration_for(): gear ratio and direction per joint.
# Design values, not measured ones. The test checks the image against them.
GEAR = [3.0, 3.0, 2.5, 2.0, 1.5, 1.0]
REVERSED = [False, True, False, False, True, False]
CENTRE_US = 1500.0
US_PER_RAD = 1000.0 / math.pi
PERIOD_US = 3003  # 1e6 / 333 Hz, truncated, as pwm_servo.cpp computes it

STATE_IDLE, STATE_ACTIVE, STATE_HOLDING = 1, 2, 3
FAULT_COMMAND_TIMEOUT = 1 << 0

# Offsets into an STM32 general-purpose or advanced timer.
TIM_CCER, TIM_PSC, TIM_ARR = 0x20, 0x28, 0x2C
TIM_CCR = [0x34, 0x38, 0x3C, 0x40]


def _work(name):
    return os.path.join(os.environ["A1_WORK"], name)


def expected_pulse_us(joint, target_rad):
    """Pulse width the firmware should put on a joint's pin for a target."""
    servo = target_rad / GEAR[joint]
    if REVERSED[joint]:
        servo = -servo
    return CENTRE_US + servo * US_PER_RAD


def _field(register, index, width):
    return (register >> (index * width)) & ((1 << width) - 1)


class a1_checks:  # noqa: N801  Robot imports the class named after the file
    ROBOT_LIBRARY_SCOPE = "SUITE"

    def __init__(self):
        self._frames = None

    # --- loading -----------------------------------------------------------

    def _decoded(self):
        if self._frames is None:
            out = subprocess.run(
                [os.environ["A1_PA_LINK"], "decode", _work("uart1.bin")],
                check=True, capture_output=True, text=True).stdout
            self._frames = [json.loads(line) for line in out.splitlines() if line]
        return self._frames

    def _of(self, kind):
        return [f for f in self._decoded() if f["type"] == kind]

    def _pwm_writes(self):
        writes = []
        with open(_work("pwm.csv"), encoding="ascii") as handle:
            for line in handle:
                time_us, timer, offset, value = (int(x) for x in line.split(","))
                writes.append((time_us, timer, offset, value))
        return writes

    # --- keywords ----------------------------------------------------------

    def pins_should_be_multiplexed(self, gpioa_moder, gpioa_afrh, gpiob_moder, gpiob_afrl,
                                   gpioe_moder, gpioe_afrh):
        """The GPIO registers as the firmware left them, against PINOUT.md.

        Servo PWM: PE9, PE11, PE13, PE14 on AF1 (TIM1) and PB4, PB5 on AF2
        (TIM3). Host link: PA9, PA10 on AF7 (USART1). IMU: PB6, PB7 on AF4
        (I2C1). The open-drain setting on the I2C pins is not checked: Renode's
        GPIO model does not implement OTYPER and reads it back as zero.
        """
        regs = {name: int(str(value).strip(), 16) for name, value in (
            ("gpioa_moder", gpioa_moder), ("gpioa_afrh", gpioa_afrh),
            ("gpiob_moder", gpiob_moder), ("gpiob_afrl", gpiob_afrl), ("gpioe_moder", gpioe_moder),
            ("gpioe_afrh", gpioe_afrh))}
        expected = [
            # (port, pin, alternate function, MODER, AFR register, AFR index)
            ("E", 9, 1, "gpioe_moder", "gpioe_afrh", 1),
            ("E", 11, 1, "gpioe_moder", "gpioe_afrh", 3),
            ("E", 13, 1, "gpioe_moder", "gpioe_afrh", 5),
            ("E", 14, 1, "gpioe_moder", "gpioe_afrh", 6),
            ("B", 4, 2, "gpiob_moder", "gpiob_afrl", 4),
            ("B", 5, 2, "gpiob_moder", "gpiob_afrl", 5),
            ("B", 6, 4, "gpiob_moder", "gpiob_afrl", 6),
            ("B", 7, 4, "gpiob_moder", "gpiob_afrl", 7),
            ("A", 9, 7, "gpioa_moder", "gpioa_afrh", 1),
            ("A", 10, 7, "gpioa_moder", "gpioa_afrh", 2),
        ]
        for port, pin, af, moder, afr, index in expected:
            mode = _field(regs[moder], pin, 2)
            if mode != 2:
                raise AssertionError(f"P{port}{pin} is in mode {mode}, not alternate function")
            got = _field(regs[afr], index, 4)
            if got != af:
                raise AssertionError(f"P{port}{pin} is on AF{got}, expected AF{af}")

    def capture_should_decode_cleanly(self, minimum_frames=100):
        summary = self._of("summary")[-1]
        if summary["crc_errors"] or summary["malformed"] or summary["overflows"]:
            raise AssertionError(f"link errors in the capture: {summary}")
        if summary["frames"] < int(minimum_frames):
            raise AssertionError(f"only {summary['frames']} frames decoded")
        unexpected = self._of("unexpected")
        if unexpected:
            raise AssertionError(f"frames of unknown type: {unexpected[:3]}")
        return summary["frames"]

    def frame_rates_should_match_the_board_configuration(self, seconds):
        """sEMG at 200 Hz, telemetry and IMU at 100 Hz, status at 10 Hz."""
        seconds = float(seconds)
        expected = {"semg": 200.0, "telemetry": 100.0, "imu": 100.0, "status": 10.0}
        report = {}
        for kind, rate in expected.items():
            count = len(self._of(kind))
            report[kind] = count
            if abs(count - rate * seconds) > max(2.0, 0.02 * rate * seconds):
                raise AssertionError(f"{count} {kind} frames in {seconds} s, expected about "
                                     f"{rate * seconds:.0f}")
        sequences = [f["sequence"] for f in self._of("semg")]
        gaps = [b - a for a, b in zip(sequences, sequences[1:]) if b - a != 1]
        if gaps:
            raise AssertionError(f"gaps in the sEMG frame counter: {gaps[:5]}")
        return report

    def board_timestamps_should_track_virtual_time(self):
        """micros() comes from the DWT cycle counter: it should advance at 1 MHz."""
        semg = self._of("semg")
        first, last = semg[0], semg[-1]
        elapsed_us = last["timestamp_us"] - first["timestamp_us"]
        frames = last["sequence"] - first["sequence"]
        per_frame = elapsed_us / frames
        if abs(per_frame - 5000.0) > 5.0:
            raise AssertionError(f"{per_frame:.1f} us between sEMG frames, expected 5000")
        return per_frame

    def supervisor_should_pass_through(self, *states):
        """The status stream's state sequence, with repeats collapsed."""
        seen = []
        for status in self._of("status"):
            if not seen or seen[-1] != status["state"]:
                seen.append(status["state"])
        wanted = [int(s) for s in states]
        if seen != wanted:
            raise AssertionError(f"supervisor went through {seen}, expected {wanted}")
        return seen

    def commands_received_should_be(self, count):
        received = self._of("status")[-1]["commands_received"]
        if received != int(count):
            raise AssertionError(f"board counted {received} commands, host sent {count}")

    def command_timeout_should_be_reported_after(self, last_command_ms):
        """The first status after the commands stop carries the timeout fault."""
        for status in self._of("status"):
            if status["uptime_ms"] > float(last_command_ms) + 30:
                if not status["faults"] & FAULT_COMMAND_TIMEOUT:
                    raise AssertionError(f"no command timeout fault at {status['uptime_ms']} ms")
                return status["uptime_ms"]
        raise AssertionError("no status frame after the commands stopped")

    def telemetry_should_reach_targets(self, *targets):
        targets = [float(t) for t in targets]
        reached = None
        for frame in self._of("telemetry"):
            if all(abs(a - t) < 1e-4 for a, t in zip(frame["angle_rad"], targets)):
                reached = frame
                break
        if reached is None:
            raise AssertionError(f"telemetry never reported the targets {targets}")
        for joint, (pulse, target) in enumerate(zip(reached["pulse_us"], targets)):
            want = expected_pulse_us(joint, target)
            if abs(pulse - want) > 1.0:
                raise AssertionError(f"joint {joint}: {pulse} us reported, {want:.1f} us expected")
        return reached["timestamp_us"]

    def timers_should_be_programmed_for_333_hz(self):
        """PSC and ARR as the firmware wrote them, against Renode's 200 MHz timer clock."""
        writes = self._pwm_writes()
        for timer in (1, 3):
            psc = [v for _, t, o, v in writes if t == timer and o == TIM_PSC]
            arr = [v for _, t, o, v in writes if t == timer and o == TIM_ARR]
            if not psc or not arr:
                raise AssertionError(f"TIM{timer} prescaler or period never written")
            period_us = (psc[-1] + 1) * (arr[-1] + 1) / 200.0
            if abs(period_us - PERIOD_US) > 0.5:
                raise AssertionError(f"TIM{timer} period {period_us} us, expected {PERIOD_US}")
        return PERIOD_US

    def compare_registers_should_match_telemetry(self):
        """Every pulse width the telemetry reports was written to a compare register.

        Joints 0 to 3 are TIM1 channels 1 to 4 and joints 4 and 5 are TIM3
        channels 1 and 2.
        """
        written = {joint: set() for joint in range(6)}
        for _, timer, offset, value in self._pwm_writes():
            if offset in TIM_CCR:
                channel = TIM_CCR.index(offset)
                joint = channel if timer == 1 else 4 + channel
                if joint < 6:
                    written[joint].add(value)
        for frame in self._of("telemetry"):
            for joint, pulse in enumerate(frame["pulse_us"]):
                if pulse and pulse not in written[joint] and pulse + 1 not in written[joint] \
                        and pulse - 1 not in written[joint]:
                    raise AssertionError(f"joint {joint}: telemetry reported {pulse} us at "
                                         f"{frame['timestamp_us']} us and no compare register "
                                         f"held it")
        return {joint: len(values) for joint, values in written.items()}

    def outputs_should_be_enabled_only_while_commanded(self, first_command_ms, relax_ms):
        """CCER on both timers: off at start, on with the first command, off at relax."""
        transitions = {1: [], 3: []}
        for time_us, timer, offset, value in self._pwm_writes():
            if offset != TIM_CCER:
                continue
            enabled = value != 0
            history = transitions[timer]
            if not history or history[-1][1] != enabled:
                history.append((time_us, enabled))
        for timer, history in transitions.items():
            states = [enabled for _, enabled in history]
            # Written as disabled at start-up, enabled by the supervisor, then
            # disabled again when it relaxes. Exactly that and nothing else.
            if states != [False, True, False]:
                raise AssertionError(f"TIM{timer} output enable went {history}")
            if history[1][0] < float(first_command_ms) * 1000:
                raise AssertionError(f"TIM{timer} enabled at {history[1][0]} us, before any "
                                     f"command")
            if history[2][0] < float(relax_ms) * 1000:
                raise AssertionError(f"TIM{timer} disabled at {history[2][0]} us, before the "
                                     f"supervisor's relax time")
        return transitions

    def semg_envelopes_should_follow_the_stimulus(self):
        """Each contraction in the stimulus lifts its own four channels and only those.

        Windows are taken well inside each burst and well clear of both, so
        the filters' settling does not enter into it.
        """
        def mean_envelopes(start_s, end_s):
            frames = [f for f in self._of("semg")
                      if start_s * 1e6 <= f["timestamp_us"] <= end_s * 1e6]
            if not frames:
                raise AssertionError(f"no sEMG frames between {start_s} and {end_s} s")
            return [sum(f["envelopes"][c] for f in frames) / len(frames) for c in range(8)]

        rest = mean_envelopes(0.5, 0.75)
        first = mean_envelopes(1.0, 1.5)
        second = mean_envelopes(2.1, 2.5)
        for name, burst, active in (("first", first, range(0, 4)), ("second", second, range(4, 8))):
            for channel in range(8):
                ratio = burst[channel] / max(rest[channel], 1.0)
                if channel in active and ratio < 10.0:
                    raise AssertionError(f"{name} contraction: channel {channel} envelope only "
                                         f"{ratio:.1f} times rest")
                if channel not in active and ratio > 2.0:
                    raise AssertionError(f"{name} contraction: channel {channel} rose "
                                         f"{ratio:.1f} times rest and should not have")
        saturated = [f for f in self._of("semg") if f["saturated_mask"]]
        if saturated:
            raise AssertionError(f"{len(saturated)} sEMG frames flagged saturated")
        return {"rest": rest, "first": first, "second": second}

    def imu_should_report(self, roll_deg, temperature_c):
        """Roll from the simulated accelerometer, die temperature over telemetry."""
        imu = [f for f in self._of("imu") if f["flags"] & 1]
        if not imu:
            raise AssertionError("the orientation estimate never became trusted")
        roll = math.degrees(imu[-1]["roll_rad"])
        if abs(roll - float(roll_deg)) > 0.5:
            raise AssertionError(f"roll {roll:.2f} degrees, expected {roll_deg}")
        temperature = self._of("telemetry")[-1]["temperature_c"]
        if abs(temperature - float(temperature_c)) > 0.11:
            raise AssertionError(f"board temperature {temperature} C, expected {temperature_c}")
        return roll
