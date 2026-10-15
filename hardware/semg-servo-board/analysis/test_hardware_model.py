"""Checks on the front end and power models and on their agreement with the
firmware.

Two kinds of test here. The first kind checks the models against physics and
against the design intent: a filter that passes the band it is meant to, a
noise budget whose terms add up, a thermal model that is monotone in current.

The second kind is the one that earns its place. Several numbers exist in two
files at once, because the firmware has to know what the analogue stage did to
the signal: the gain, the converter's scale, the current trip, the temperature
trip. Those are parsed out of ``pa/config/board.hpp`` and compared with what
the component values here produce. A resistor changed in one place and not the
other is the kind of mistake that produces microvolt readings that are quietly
wrong by a factor and it is exactly the kind of mistake nobody finds by
reading.

None of this is a measurement. No board exists.
"""

from __future__ import annotations

import math
import pathlib
import re
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))

from afe_model import (  # noqa: E402
    ELECTRODE_CASES,
    Components,
    bandwidth_headroom,
    common_mode_rejection,
    magnitude,
    mains_rejection,
    noise_budget,
    response_db,
    signal_to_noise_db,
    thermal_noise_nv,
)
from power_model import (  # noqa: E402
    ACTUATORS,
    Battery,
    Board,
    OPERATING_POINTS,
    continuous_current_limit,
    dissipation,
    fuse_recommendation,
    temperatures,
)

BOARD_HEADER = REPO_ROOT / "firmware" / "include" / "pa" / "config" / "board.hpp"


def firmware_constant(name: str) -> float:
    """Pull a constant out of the firmware header.

    Parsed rather than duplicated. A test that restated the value would pass
    whatever the header said.
    """
    text = BOARD_HEADER.read_text()
    match = re.search(rf"{name}\s*=\s*([0-9.eE+-]+)f?\s*;", text)
    assert match is not None, f"{name} is not in {BOARD_HEADER}"
    return float(match.group(1))


# --- the front end ----------------------------------------------------------


def test_the_gain_comes_out_where_the_design_wanted_it():
    components = Components()
    # Two stages, ten and thirty, from E96 resistors. Not round and that is the
    # point: these are values that can be bought.
    assert components.stage1_gain == pytest.approx(10.09, abs=0.01)
    assert components.stage2_gain == pytest.approx(30.30, abs=0.01)
    assert components.total_gain == pytest.approx(305.79, abs=0.05)


def test_the_firmware_agrees_with_the_component_values():
    components = Components()
    assert firmware_constant("kFrontEndGain") == pytest.approx(components.total_gain, rel=1e-4)
    assert firmware_constant("kAdcReferenceVolts") == pytest.approx(components.adc_reference_v)
    assert int(firmware_constant("kDcBlockHz")) == pytest.approx(components.highpass_hz, abs=0.5)


def test_the_full_scale_input_covers_a_strong_contraction():
    # Surface EMG over forearm flexors reaches about 5 mV peak. The front end
    # clips a little under that, which is a stated limitation rather than an
    # oversight: raising the gain would make it worse and lowering it would put
    # the signal further into the converter's noise.
    components = Components()
    assert components.full_scale_microvolts > 4800.0
    assert components.full_scale_microvolts < 5200.0


def test_the_passband_covers_the_semg_band():
    components = Components()
    # Flat between 20 and 450 Hz, which is where surface EMG has its energy.
    for hz in (20.0, 50.0, 100.0, 200.0, 350.0):
        assert response_db(components, hz) > -3.5, f"{hz} Hz is down too far"
    # And it peaks somewhere in the middle rather than at an end, which is what
    # a bandpass should do.
    peak = max(magnitude(components, hz) for hz in range(10, 500, 5))
    assert magnitude(components, 100.0) == pytest.approx(peak, rel=0.05)


def test_the_offset_is_rejected_and_the_high_frequencies_are_gone():
    components = Components()
    # The electrode half-cell potential is DC. At a gain of 306 it would be
    # volts, so it has to be well and truly gone.
    assert response_db(components, 0.05) < -35.0
    # And above the band, the analogue stage is already helping the digital one.
    assert response_db(components, 2000.0) < -20.0
    assert response_db(components, 10000.0) < -45.0


def test_the_high_pass_sits_below_the_lowest_useful_frequency():
    components = Components()
    # Motor unit firing rates start around 8 Hz. The corner is below that and
    # above the drift.
    assert 3.0 < components.highpass_hz < 8.0


def test_the_low_pass_is_close_to_butterworth():
    components = Components()
    assert 400.0 < components.lowpass_hz < 550.0
    # Q of 0.707 is maximally flat. The unequal capacitors get close without
    # needing a gain stage to set it.
    assert components.lowpass_q == pytest.approx(0.707, abs=0.06)


def test_splitting_the_gain_is_what_keeps_the_bandwidth():
    # The usual way this design goes wrong is putting all the gain in the
    # instrumentation amplifier. At 306 the INA333 would have a 1.1 kHz
    # bandwidth and the response would droop inside the signal band.
    headroom = bandwidth_headroom(Components())
    assert headroom["stage1_bandwidth_hz"] > 30e3
    assert headroom["margin"] > 50.0
    assert headroom["if_all_gain_were_in_stage_one_hz"] < 2000.0


def test_the_noise_bandwidth_is_wider_than_the_corner_frequency():
    # A two-pole roll-off passes noise above its corner that a brick wall would
    # not. Taking the nominal 20 to 450 Hz band instead would understate the
    # floor by about a fifth, in the flattering direction.
    components = Components()
    budget = noise_budget(components, ELECTRODE_CASES[0])
    assert budget.noise_bandwidth_hz > components.lowpass_hz
    assert budget.noise_bandwidth_hz < 1.4 * components.lowpass_hz

    # And the flicker integral converges, which it only does because there is a
    # high pass in the chain.
    from afe_model import weighted_integrals

    flat, flicker = weighted_integrals(components)
    assert flat == pytest.approx(budget.noise_bandwidth_hz)
    assert 0.0 < flicker < 50.0


def test_the_noise_terms_add_in_quadrature_and_nothing_is_missing():
    components = Components()
    for electrode in ELECTRODE_CASES:
        budget = noise_budget(components, electrode)
        total = math.sqrt(sum(term.microvolts_rms ** 2 for term in budget.terms))
        assert budget.total_microvolts_rms == pytest.approx(total, rel=1e-12)
        # Every source in the channel is accounted for.
        sources = {term.source for term in budget.terms}
        assert any("voltage noise" in s for s in sources)
        assert any("current noise" in s for s in sources)
        assert any("electrode" in s for s in sources)
        assert any("converter" in s for s in sources)
        assert all(term.microvolts_rms >= 0.0 for term in budget.terms)


def test_the_amplifier_and_the_converter_are_balanced():
    """The design's main conclusion, written down as a test.

    With a gelled electrode the amplifier and the converter contribute within a
    few percent of each other, at about 0.6 microvolts each and everything
    else in the channel is smaller. That is what a balanced design looks like
    and it is the useful finding: a quieter amplifier on its own would buy
    almost nothing and neither would more bits on its own. Improving this
    front end means doing both, which is what
    hardware/semg-servo-board/docs/noise_analysis.md recommends.
    """
    components = Components()
    budget = noise_budget(components, ELECTRODE_CASES[0])

    ranked = sorted(budget.terms, key=lambda term: -term.microvolts_rms)
    top_two = {ranked[0].source, ranked[1].source}
    assert top_two == {"INA333 voltage noise", "converter"}
    # Within a quarter of each other: neither is worth attacking alone.
    assert ranked[1].microvolts_rms > 0.75 * ranked[0].microvolts_rms
    # And the next term down is well behind both.
    assert ranked[2].microvolts_rms < 0.6 * ranked[1].microvolts_rms

    assert budget.total_microvolts_rms < 1.2


def test_the_electrode_dominates_when_it_is_dry():
    # And with a dry electrode nothing on the board matters much, which is why
    # the electrode interface notes come before the schematic in the docs.
    components = Components()
    budget = noise_budget(components, ELECTRODE_CASES[-1])
    assert "electrode" in budget.dominant.source
    assert budget.total_microvolts_rms > 1.2


def test_the_noise_floor_is_usable_against_a_real_signal():
    components = Components()
    for electrode in ELECTRODE_CASES:
        budget = noise_budget(components, electrode)
        # A light contraction is 20 uV RMS at the skin. It has to be above the
        # floor by enough to be worth classifying.
        assert signal_to_noise_db(budget, 20.0) > 20.0
        assert signal_to_noise_db(budget, 1000.0) > 55.0


def test_thermal_noise_matches_the_closed_form():
    # 1 kohm at room temperature over 1 Hz is about 4 nV. If this drifts, every
    # figure in the budget drifts with it.
    assert thermal_noise_nv(1000.0, 1.0) == pytest.approx(4.07, abs=0.05)
    # And it goes as the square root of both resistance and bandwidth.
    assert thermal_noise_nv(4000.0, 1.0) == pytest.approx(2.0 * thermal_noise_nv(1000.0, 1.0),
                                                          rel=1e-9)
    assert thermal_noise_nv(1000.0, 4.0) == pytest.approx(2.0 * thermal_noise_nv(1000.0, 1.0),
                                                          rel=1e-9)


def test_the_mains_term_ends_up_below_the_noise_floor():
    components = Components()
    cmrr = common_mode_rejection(components)
    mains = mains_rejection(components, cmrr["combined_db"])
    budget = noise_budget(components, ELECTRODE_CASES[0])

    # A volt of common mode on the body, through the driven right leg, the
    # amplifier's rejection and the firmware's notch. If this did not come out
    # below the noise floor the notch would not be enough and the design would
    # need shielding.
    assert mains["after_digital_notch_uv"] < budget.total_microvolts_rms
    assert mains["after_drl_mv"] < 30.0


def test_the_common_mode_rejection_is_limited_by_the_amplifier_then_the_electrodes():
    """Which of the three limits binds and what would happen if it moved.

    Worth checking rather than assuming, because the fix is different for each.
    With a gelled electrode the amplifier's own 100 dB binds and the electrode
    imbalance is 14 dB behind it, so a better amplifier would buy about a decade
    and then stop. The series resistors are seventy decibels away and are not
    the problem at any electrode this arm would be used with.
    """
    components = Components()
    good = common_mode_rejection(components, ELECTRODE_CASES[0])
    assert good["binding_limit"] == "amplifier specification"
    assert good["limits_db"]["electrode imbalance"] > good["limits_db"]["amplifier specification"]
    assert (good["limits_db"]["series resistor tolerance"]
            > good["limits_db"]["electrode imbalance"] + 40.0)
    assert good["combined_db"] == pytest.approx(99.8, abs=0.5)

    # With a dry electrode the imbalance catches up and becomes the limit,
    # which is the argument for gel rather than for a different amplifier.
    dry = common_mode_rejection(components, ELECTRODE_CASES[-1])
    assert dry["binding_limit"] == "electrode imbalance"
    assert dry["combined_db"] < good["combined_db"] - 5.0


# --- power and thermal ------------------------------------------------------


def test_every_actuator_can_hold_the_load_the_dynamics_predicts():
    # The torque ratings here have to match the model the kinematics library
    # uses, because kinematics/test/test_dynamics.cpp asserts the worst-case
    # gravity torque fits inside them.
    model_json = REPO_ROOT / "ros2_ws/src/arm_description/config/a1_arm_model.json"
    if not model_json.exists():
        pytest.skip("the model JSON has not been generated; run make model")
    import json

    chain = json.loads(model_json.read_text())["chain"]
    assert len(chain) == len(ACTUATORS)
    for row, actuator in zip(chain, ACTUATORS):
        assert row["name"] == actuator.joint
        assert row["effort"] == pytest.approx(actuator.rated_torque_nm)


def test_the_firmware_trips_where_the_budget_says_it_should():
    assert firmware_constant("kBusCurrentLimitAmps") == pytest.approx(
        fuse_recommendation()["firmware_trip_a"])
    assert firmware_constant("kBoardTemperatureLimitC") == pytest.approx(
        Board().temperature_limit_c)


def test_the_protection_hierarchy_is_ordered():
    """Three protections, three speeds and they have to be in the right order.

    A current trip above what a jam can draw never fires. A fuse below the
    current trip opens before the software can act and a blown fuse drops a
    limb. This is the check that the three numbers still make sense together.
    """
    fuse = fuse_recommendation()
    battery = Battery()
    board = Board()

    synchronised_move = 5.5
    continuous = continuous_current_limit(battery.nominal_v, board)

    assert continuous > synchronised_move, "the board cannot carry an ordinary movement"
    assert fuse["firmware_trip_a"] > synchronised_move, "an ordinary movement would trip"
    assert fuse["firmware_trip_a"] < fuse["total_stall_a"], "a jam would never trip it"
    assert fuse["fuse_a"] > fuse["total_stall_a"], "the fuse would open before the software acts"


def test_the_board_stays_inside_its_limit_at_every_operating_point_but_a_jam():
    battery = Battery()
    board = Board()
    for point in OPERATING_POINTS:
        thermal = temperatures(point.servo_current_a, battery.nominal_v, board)
        if "stalled" in point.name:
            # A jam is a fault. It has to exceed the limit, or the thermal trip
            # would be decoration.
            assert not thermal["within_limit"]
        else:
            assert thermal["within_limit"], f"{point.name} overheats the board"


def test_dissipation_is_monotone_in_current():
    battery = Battery()
    previous = -1.0
    for current in (0.0, 1.0, 2.0, 4.0, 8.0, 12.0):
        total = dissipation(current, battery.nominal_v)["total_w"]
        assert total > previous
        previous = total


def test_the_converter_is_efficient_enough_to_be_worth_using():
    battery = Battery()
    for current in (1.0, 2.5, 5.0):
        efficiency = dissipation(current, battery.nominal_v)["rails"]["servo_rail"]["efficiency"]
        assert 0.85 < efficiency < 1.0


def test_the_shunt_is_not_asked_to_dissipate_more_than_it_can():
    battery = Battery()
    # At the current trip, which is the most it should ever see in service.
    heat = dissipation(fuse_recommendation()["firmware_trip_a"], battery.nominal_v)
    assert heat["shunt_headroom"] > 2.0


def test_the_continuous_limit_is_where_the_temperature_says():
    battery = Battery()
    board = Board()
    limit = continuous_current_limit(battery.nominal_v, board)
    assert temperatures(limit - 0.05, battery.nominal_v, board)["within_limit"]
    assert not temperatures(limit + 0.05, battery.nominal_v, board)["within_limit"]


def test_the_arm_runs_for_long_enough_to_be_worth_wearing():
    from power_model import runtime_minutes

    battery = Battery()
    # Sitting still, on a 2.2 Ah pack, for hours; working, for the better part
    # of an hour. Neither is a measurement and both are what the arithmetic
    # gives.
    assert runtime_minutes(OPERATING_POINTS[0].servo_current_a, battery) > 120.0
    assert runtime_minutes(OPERATING_POINTS[1].servo_current_a, battery) > 45.0
