"""The servo drive stage and the power tree, as a model that can be evaluated.

Where every watt on the board goes, what that does to the temperature and how
much current the arm can draw continuously before the answer stops being
acceptable. The two firmware limits in ``pa/config/board.hpp``, the 12 A bus
trip and the 85 C thermal trip, come out of this file rather than out of
somebody's judgement.

Nothing here has been built. No board has been fabricated, no current has been
measured and no temperature has been observed. Every number below is arithmetic
from a datasheet parameter and a stated assumption and the report says so on
every run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# --- the actuators ----------------------------------------------------------


@dataclass(frozen=True)
class Actuator:
    joint: str
    stall_current_a: float
    rated_torque_nm: float
    # Quiescent draw as a fraction of stall, which is what a digital servo
    # takes holding a position with no load worth speaking of.
    quiescent_fraction: float = 0.06


# Sized against the worst-case gravity torque in
# kinematics/test/test_dynamics.cpp, which samples the whole joint space and
# asserts nothing exceeds the rating below.
ACTUATORS = (
    Actuator("shoulder_rotation", 2.5, 8.0),
    Actuator("shoulder_flexion", 2.5, 8.0),
    Actuator("elbow_flexion", 2.0, 6.0),
    Actuator("forearm_rotation", 1.2, 2.5),
    Actuator("wrist_flexion", 1.2, 2.5),
    Actuator("tool_roll", 0.8, 1.2),
)


# --- the power tree ---------------------------------------------------------


@dataclass(frozen=True)
class Battery:
    cells: int = 3
    nominal_cell_v: float = 3.7
    full_cell_v: float = 4.2
    empty_cell_v: float = 3.3
    capacity_ah: float = 2.2

    @property
    def nominal_v(self) -> float:
        return self.cells * self.nominal_cell_v

    @property
    def full_v(self) -> float:
        return self.cells * self.full_cell_v

    @property
    def empty_v(self) -> float:
        return self.cells * self.empty_cell_v


@dataclass(frozen=True)
class Buck:
    """A synchronous buck converter, from its datasheet."""

    name: str
    output_v: float
    rated_current_a: float
    switching_hz: float
    high_side_rds_on: float
    low_side_rds_on: float
    # Combined rise and fall time of the switch node, which is what sets the
    # switching loss.
    transition_s: float
    inductor_dcr_ohms: float
    theta_ja_c_per_w: float

    def losses(self, input_v: float, current_a: float) -> dict:
        duty = self.output_v / input_v
        conduction_high = current_a * current_a * self.high_side_rds_on * duty
        conduction_low = current_a * current_a * self.low_side_rds_on * (1.0 - duty)
        switching = 0.5 * input_v * current_a * self.transition_s * self.switching_hz
        inductor = current_a * current_a * self.inductor_dcr_ohms

        in_package = conduction_high + conduction_low + switching
        return {
            "duty": duty,
            "conduction_high_side_w": conduction_high,
            "conduction_low_side_w": conduction_low,
            "switching_w": switching,
            "inductor_w": inductor,
            "package_w": in_package,
            "total_w": in_package + inductor,
            "junction_rise_c": in_package * self.theta_ja_c_per_w,
            "efficiency": (self.output_v * current_a) /
                          max(self.output_v * current_a + in_package + inductor, 1e-9),
        }


# TPS568230, 4.5 to 17 V in, 8 A, in a package with an exposed pad on a
# four-layer board with a via array under it.
SERVO_RAIL = Buck(
    name="TPS568230",
    output_v=6.0,
    rated_current_a=8.0,
    switching_hz=500e3,
    high_side_rds_on=0.010,
    low_side_rds_on=0.005,
    transition_s=20e-9,
    inductor_dcr_ohms=0.010,
    theta_ja_c_per_w=40.0,
)

# LMR33620, the logic rail.
LOGIC_RAIL = Buck(
    name="LMR33620",
    output_v=3.3,
    rated_current_a=2.0,
    switching_hz=400e3,
    high_side_rds_on=0.100,
    low_side_rds_on=0.055,
    transition_s=15e-9,
    inductor_dcr_ohms=0.030,
    theta_ja_c_per_w=45.0,
)


@dataclass(frozen=True)
class Board:
    """Everything that turns watts into degrees."""

    length_mm: float = 80.0
    width_mm: float = 60.0
    layers: int = 4
    # Thermal resistance of the whole assembly to still air. A four-layer board
    # of this size with two-ounce inner planes sits between 25 and 40 C/W; the
    # pessimistic end is used, because the alternative to over-estimating here
    # is a thermal trip in service.
    theta_ja_c_per_w: float = 30.0
    ambient_c: float = 25.0
    # From pa/config/board.hpp. The limit the firmware trips at.
    temperature_limit_c: float = 85.0


# --- what the rest of the board draws ---------------------------------------

# STM32H743 at 400 MHz with the converter and both DMA controllers running,
# from the datasheet's dynamic current figure.
MCU_CURRENT_A = 0.20
MCU_RAIL_V = 3.3

# Eight instrumentation amplifier channels, their gain and filter stages and
# the driven right leg. INA333 is 50 uA a channel and the TLV9062 is 500 uA a
# pair, so the analogue section is nearly free.
ANALOGUE_CURRENT_A = 0.012
ANALOGUE_RAIL_V = 3.0

# 5 milliohm shunt in the servo return, read by an INA181.
SHUNT_OHMS = 0.005
SHUNT_RATING_W = 1.0


@dataclass
class OperatingPoint:
    name: str
    servo_current_a: float
    note: str = ""


OPERATING_POINTS = (
    OperatingPoint("all six holding, unloaded", sum(a.stall_current_a * a.quiescent_fraction
                                                    for a in ACTUATORS),
                   "what the arm draws sitting still"),
    OperatingPoint("a typical reach", 2.4,
                   "three joints moving under load, from the simulated run in "
                   "analysis/sim/control_loop_sim.json"),
    OperatingPoint("every joint moving at once", 5.5,
                   "the worst case a trajectory can produce, since the planner "
                   "synchronises the joints"),
    OperatingPoint("every joint stalled", sum(a.stall_current_a for a in ACTUATORS),
                   "a fault, not an operating point: something is jammed"),
)


def rail_currents(servo_current_a: float, battery_v: float) -> dict:
    """Input current for each rail at a given servo load."""
    servo = SERVO_RAIL.losses(battery_v, servo_current_a)
    logic_current = MCU_CURRENT_A + ANALOGUE_CURRENT_A
    logic = LOGIC_RAIL.losses(battery_v, logic_current)

    servo_input_w = SERVO_RAIL.output_v * servo_current_a + servo["total_w"]
    logic_input_w = LOGIC_RAIL.output_v * logic_current + logic["total_w"]

    return {
        "servo_rail": servo,
        "logic_rail": logic,
        "logic_current_a": logic_current,
        "battery_current_a": (servo_input_w + logic_input_w) / battery_v,
        "servo_input_w": servo_input_w,
        "logic_input_w": logic_input_w,
    }


def dissipation(servo_current_a: float, battery_v: float) -> dict:
    """Total watts turned into heat on the board, itemised."""
    rails = rail_currents(servo_current_a, battery_v)
    shunt_w = servo_current_a * servo_current_a * SHUNT_OHMS
    mcu_w = MCU_CURRENT_A * MCU_RAIL_V
    analogue_w = ANALOGUE_CURRENT_A * ANALOGUE_RAIL_V

    items = {
        "servo rail converter": rails["servo_rail"]["package_w"],
        "servo rail inductor": rails["servo_rail"]["inductor_w"],
        "logic rail converter": rails["logic_rail"]["package_w"],
        "logic rail inductor": rails["logic_rail"]["inductor_w"],
        "current shunt": shunt_w,
        "microcontroller": mcu_w,
        "analogue front end": analogue_w,
    }
    return {
        "items_w": items,
        "total_w": sum(items.values()),
        "shunt_w": shunt_w,
        "shunt_headroom": SHUNT_RATING_W / max(shunt_w, 1e-9),
        "rails": rails,
    }


def temperatures(servo_current_a: float, battery_v: float, board: Board = Board()) -> dict:
    """Board and junction temperatures at an operating point."""
    heat = dissipation(servo_current_a, battery_v)
    board_rise = heat["total_w"] * board.theta_ja_c_per_w
    board_c = board.ambient_c + board_rise

    servo_junction = board_c + heat["rails"]["servo_rail"]["junction_rise_c"]
    logic_junction = board_c + heat["rails"]["logic_rail"]["junction_rise_c"]

    return {
        "board_c": board_c,
        "board_rise_c": board_rise,
        "servo_converter_junction_c": servo_junction,
        "logic_converter_junction_c": logic_junction,
        "limit_c": board.temperature_limit_c,
        "within_limit": board_c <= board.temperature_limit_c,
        "dissipation_w": heat["total_w"],
    }


def continuous_current_limit(battery_v: float, board: Board = Board()) -> float:
    """The largest servo current the board can carry indefinitely.

    Bisection on the board temperature, because the losses are quadratic in
    current and there is no closed form worth writing.
    """
    low, high = 0.0, 20.0
    for _ in range(60):
        middle = 0.5 * (low + high)
        if temperatures(middle, battery_v, board)["board_c"] <= board.temperature_limit_c:
            low = middle
        else:
            high = middle
    return low


def runtime_minutes(servo_current_a: float, battery: Battery = Battery()) -> float:
    rails = rail_currents(servo_current_a, battery.nominal_v)
    # Eighty percent of the pack is usable before the cells reach their floor.
    usable_ah = battery.capacity_ah * 0.8
    return 60.0 * usable_ah / max(rails["battery_current_a"], 1e-9)


def fuse_recommendation() -> dict:
    """What the bus fuse should be and why the firmware trips before it.

    The fuse protects the wiring from a short. The firmware's current trip
    protects the arm from a stall and it has to act first, because a fuse that
    opens drops a limb and cannot be reset by software.
    """
    stall_total = sum(a.stall_current_a for a in ACTUATORS)
    return {
        "total_stall_a": stall_total,
        "firmware_trip_a": 8.0,
        "fuse_a": 15.0,
        "fuse_type": "slow blow, so a start-up inrush does not open it",
        "note": (
            "Three protections at three speeds. The firmware trips on current "
            "at 8 A, which is above the 5.5 A a synchronised move draws and "
            "below the 10.2 A a jam draws, so a jam is caught inside a control "
            "period and an ordinary movement is not. It trips on temperature at "
            "85 C, which catches sustained current above the 6.4 A the board can "
            "carry indefinitely. The 15 A fuse protects the wiring from a short "
            "and nothing else, because a fuse that opens drops a limb and cannot "
            "be reset by software."
        ),
    }
