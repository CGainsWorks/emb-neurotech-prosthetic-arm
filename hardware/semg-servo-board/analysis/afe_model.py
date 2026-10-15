"""The sEMG analogue front end, as a model that can be evaluated.

One channel, three stages and the component values that set them. Everything
in hardware/semg-servo-board/docs/ that quotes a number gets it from here and
so does the gain constant the firmware uses to turn ADC counts back into
microvolts at the electrode.

Nothing here has been built. No board has been fabricated and no measurement
exists. What this file contains is a design and the arithmetic that follows
from it and every output it produces says so.

The stages and why each one is there:

  1. Instrumentation amplifier, gain 10. Low, on purpose. The electrode
     half-cell potential is tens of millivolts of DC and the input stage has to
     survive it; at a gain of 300 in one stage it would sit against a rail.
  2. Single-pole high pass at 4.8 Hz, then gain 30. The offset is gone by the
     time the signal is large, which is what makes the second stage possible.
  3. Two-pole low pass at 477 Hz. Surface EMG has essentially no energy above
     it and it is the analogue half of the anti-alias filter for the 2 kHz
     converter.

The total gain is 306, not the 500 a textbook would suggest and the reason is
the input range: at 500 the front end clips at 3 mV at the electrode and a
strong contraction over forearm flexors reaches 5 mV.
"""

from __future__ import annotations

import cmath
import math
from dataclasses import dataclass, field

# --- physical constants -----------------------------------------------------

BOLTZMANN = 1.380649e-23
ROOM_TEMPERATURE_K = 300.0


@dataclass(frozen=True)
class Components:
    """Every value on the schematic that affects the response or the noise.

    E96 series throughout, so these are values that can be bought rather than
    values that make the arithmetic tidy. That is why the gain comes out at 306
    and not at 300.
    """

    # Input protection and radio-frequency filter.
    r_series: float = 1.0e3          # ohms, per input
    c_differential: float = 1.0e-9   # farads, across the pair
    c_common_mode: float = 100e-12   # farads, each input to ground
    r_bias: float = 10.0e6           # ohms, each input to mid-rail

    # Stage one: INA333 instrumentation amplifier, G = 1 + 100k / Rg.
    r_gain: float = 11.0e3
    ina_gain_resistor_numerator: float = 100.0e3

    # Stage two: high pass into an inverting gain stage.
    c_highpass: float = 1.0e-6
    r_highpass: float = 33.0e3
    r_feedback: float = 1.0e6

    # Stage three: Sallen-Key low pass, unequal capacitors for a Q near 0.707.
    r_lowpass_1: float = 15.0e3
    r_lowpass_2: float = 15.0e3
    c_lowpass_1: float = 33.0e-9
    c_lowpass_2: float = 15.0e-9

    # Converter.
    adc_reference_v: float = 3.0
    adc_bits: int = 16
    # Effective noise of the STM32H743 converter at 16 bits, in least
    # significant bits RMS. A design assumption from the datasheet's effective
    # number of bits, not a measurement and the noise budget is dominated by
    # it, which is the interesting conclusion rather than an inconvenience.
    adc_noise_lsb_rms: float = 4.0

    @property
    def stage1_gain(self) -> float:
        return 1.0 + self.ina_gain_resistor_numerator / self.r_gain

    @property
    def stage2_gain(self) -> float:
        return self.r_feedback / self.r_highpass

    @property
    def total_gain(self) -> float:
        return self.stage1_gain * self.stage2_gain

    @property
    def highpass_hz(self) -> float:
        return 1.0 / (2.0 * math.pi * self.r_highpass * self.c_highpass)

    @property
    def lowpass_hz(self) -> float:
        product = math.sqrt(
            self.r_lowpass_1 * self.r_lowpass_2 * self.c_lowpass_1 * self.c_lowpass_2
        )
        return 1.0 / (2.0 * math.pi * product)

    @property
    def lowpass_q(self) -> float:
        numerator = math.sqrt(
            self.r_lowpass_1 * self.r_lowpass_2 * self.c_lowpass_1 * self.c_lowpass_2
        )
        denominator = self.c_lowpass_2 * (self.r_lowpass_1 + self.r_lowpass_2)
        return numerator / denominator

    @property
    def input_filter_hz(self) -> float:
        """Differential corner of the input RC, which sets the RF rejection."""
        return 1.0 / (2.0 * math.pi * 2.0 * self.r_series * self.c_differential)

    @property
    def full_scale_microvolts(self) -> float:
        """Largest differential input the chain can carry without clipping."""
        return (self.adc_reference_v / 2.0) / self.total_gain * 1e6

    @property
    def microvolts_per_count(self) -> float:
        full_scale_counts = (1 << (self.adc_bits - 1)) - 1
        return self.full_scale_microvolts / full_scale_counts


@dataclass(frozen=True)
class Amplifier:
    """The noise a device contributes, from its datasheet."""

    name: str
    voltage_noise_nv_rthz: float      # at 1 kHz, above the corner
    voltage_noise_corner_hz: float    # where the 1/f term equals the flat one
    current_noise_fa_rthz: float
    current_noise_corner_hz: float = 100.0


# Texas Instruments INA333: 25 nV/rtHz at 1 kHz, 1/f corner around 10 Hz,
# 100 fA/rtHz. The part is chosen for exactly this: a micropower
# instrumentation amplifier with a low enough corner to be useful at 20 Hz.
INA333 = Amplifier("INA333", 25.0, 10.0, 100.0)

# Texas Instruments TLV9062, the gain and filter stages.
TLV9062 = Amplifier("TLV9062", 10.0, 100.0, 200.0)


@dataclass(frozen=True)
class ElectrodeInterface:
    """The skin, which is part of the circuit whether or not it is drawn.

    Impedance depends on preparation more than on anything the board does:
    a gelled Ag/AgCl electrode on abraded skin is tens of kilohms, a dry
    electrode on unprepared skin is hundreds. Both are modelled, because the
    difference between them is larger than every other term in the budget put
    together.
    """

    name: str
    impedance_ohms: float


ELECTRODE_CASES = (
    ElectrodeInterface("gelled Ag/AgCl, prepared skin", 10.0e3),
    ElectrodeInterface("gelled Ag/AgCl, unprepared skin", 50.0e3),
    ElectrodeInterface("dry electrode", 200.0e3),
)


# --- transfer function ------------------------------------------------------


def stage_response(components: Components, hz: float) -> complex:
    """Complex response of the whole chain at one frequency, gain included."""
    s = 2j * math.pi * hz

    # Input RC, differential.
    input_pole = 1.0 / (1.0 + s * 2.0 * components.r_series * components.c_differential)

    # Instrumentation amplifier: flat over this band, so its own bandwidth is
    # not modelled here. Checked separately in bandwidth_headroom().
    stage1 = components.stage1_gain

    # High pass into the inverting stage.
    z_in = components.r_highpass + 1.0 / (s * components.c_highpass) if hz > 0 else float("inf")
    stage2 = -components.r_feedback / z_in if hz > 0 else 0.0

    # Sallen-Key, unity gain.
    w0 = 2.0 * math.pi * components.lowpass_hz
    q = components.lowpass_q
    stage3 = w0 * w0 / (s * s + (w0 / q) * s + w0 * w0)

    return input_pole * stage1 * stage2 * stage3


def magnitude(components: Components, hz: float) -> float:
    return abs(stage_response(components, hz))


def response_db(components: Components, hz: float) -> float:
    value = magnitude(components, hz)
    return 20.0 * math.log10(value / components.total_gain) if value > 0 else -300.0


def bandwidth_headroom(components: Components, amplifier: Amplifier = INA333) -> dict:
    """Is the instrumentation amplifier fast enough at this gain?

    The INA333 has a 350 kHz gain-bandwidth product. At a gain of 10 that is a
    35 kHz closed-loop bandwidth, which is seventy times the top of the band.
    Worth stating rather than assuming: the usual way this design goes wrong is
    putting all the gain in the first stage, at which point the bandwidth is
    700 Hz and the response is drooping inside the signal.
    """
    gain_bandwidth_hz = 350e3
    closed_loop_hz = gain_bandwidth_hz / components.stage1_gain
    return {
        "amplifier": amplifier.name,
        "gain_bandwidth_hz": gain_bandwidth_hz,
        "stage1_gain": components.stage1_gain,
        "stage1_bandwidth_hz": closed_loop_hz,
        "signal_band_top_hz": 450.0,
        "margin": closed_loop_hz / 450.0,
        "if_all_gain_were_in_stage_one_hz": gain_bandwidth_hz / components.total_gain,
    }


# --- noise ------------------------------------------------------------------


def weighted_integrals(components: Components, low_hz: float = 0.05,
                       high_hz: float = 50e3, steps: int = 6000) -> tuple:
    """Two integrals of the channel's own response, which is what noise sees.

    Returns (I_flat, I_flicker) where

        I_flat    = integral of |H(f) / H_passband|^2 df
        I_flicker = integral of |H(f) / H_passband|^2 / f df

    The first is the equivalent noise bandwidth: what a flat noise density gets
    multiplied by. The second is what a 1/f density gets multiplied by and it
    converges at the low end only because the front end has a high pass in it,
    which is a fair description of why the high pass is there.

    Integrating the real response rather than assuming a rectangular 20 to
    450 Hz band matters in both directions. A two-pole roll-off passes noise
    above its corner that a brick wall would not, so the flat term is larger
    than the nominal band; the high pass removes noise below 20 Hz that a
    rectangular band would have counted, so the flicker term is smaller.
    """
    reference = components.total_gain
    flat = 0.0
    flicker = 0.0
    log_low, log_high = math.log10(low_hz), math.log10(high_hz)
    previous_f = low_hz
    previous_y = (magnitude(components, previous_f) / reference) ** 2
    for i in range(1, steps + 1):
        f = 10 ** (log_low + (log_high - log_low) * i / steps)
        y = (magnitude(components, f) / reference) ** 2
        width = f - previous_f
        flat += 0.5 * (y + previous_y) * width
        flicker += 0.5 * (y / f + previous_y / previous_f) * width
        previous_f, previous_y = f, y
    return flat, flicker


def noise_bandwidth(components: Components) -> float:
    """Equivalent noise bandwidth of the channel, in hertz."""
    return weighted_integrals(components)[0]


def amplifier_noise_nv(amplifier: "Amplifier", flat_hz: float, flicker_hz: float) -> float:
    """A device's voltage noise through this channel, in nanovolts RMS.

    e(f)^2 = e0^2 (1 + fc / f), integrated against the channel's response
    rather than against a rectangle.
    """
    return amplifier.voltage_noise_nv_rthz * math.sqrt(
        flat_hz + amplifier.voltage_noise_corner_hz * flicker_hz
    )


def thermal_noise_nv(resistance_ohms: float, bandwidth_hz: float,
                     temperature_k: float = ROOM_TEMPERATURE_K) -> float:
    return 1e9 * math.sqrt(4.0 * BOLTZMANN * temperature_k * resistance_ohms * bandwidth_hz)


@dataclass
class NoiseTerm:
    source: str
    microvolts_rms: float
    note: str = ""


@dataclass
class NoiseBudget:
    electrode: str
    band_hz: tuple
    noise_bandwidth_hz: float
    terms: list = field(default_factory=list)

    @property
    def total_microvolts_rms(self) -> float:
        return math.sqrt(sum(term.microvolts_rms ** 2 for term in self.terms))

    @property
    def dominant(self) -> NoiseTerm:
        return max(self.terms, key=lambda term: term.microvolts_rms)


def noise_budget(components: Components, electrode: ElectrodeInterface,
                 amplifier: Amplifier = INA333,
                 second_stage: Amplifier = TLV9062) -> NoiseBudget:
    """Every noise source in the channel, referred back to the electrode.

    Referred to input throughout, because that is the only place the numbers
    can be compared with the signal: surface EMG is 20 to 1000 microvolts RMS
    at the skin and a noise floor only means something next to that.
    """
    flat_hz, flicker_hz = weighted_integrals(components)
    terms: list[NoiseTerm] = []

    # The instrumentation amplifier's own voltage noise, with its 1/f tail.
    terms.append(NoiseTerm(
        f"{amplifier.name} voltage noise",
        amplifier_noise_nv(amplifier, flat_hz, flicker_hz) / 1000.0,
        f"{amplifier.voltage_noise_nv_rthz:.0f} nV/rtHz with a "
        f"{amplifier.voltage_noise_corner_hz:.0f} Hz corner",
    ))

    # Its current noise, through whatever the electrodes present.
    current_noise = Amplifier(
        amplifier.name,
        amplifier.current_noise_fa_rthz * 1e-15 * electrode.impedance_ohms * 1e9,
        amplifier.current_noise_corner_hz,
        0.0,
    )
    terms.append(NoiseTerm(
        f"{amplifier.name} current noise through the electrodes",
        amplifier_noise_nv(current_noise, flat_hz, flicker_hz) / 1000.0,
        f"{amplifier.current_noise_fa_rthz:.0f} fA/rtHz into "
        f"{electrode.impedance_ohms / 1e3:.0f} kohm",
    ))

    # Johnson noise of the electrode-skin interface. Not a property of the
    # board at all and on a dry electrode it is the largest term in the budget.
    terms.append(NoiseTerm(
        "electrode-skin thermal noise",
        thermal_noise_nv(electrode.impedance_ohms, flat_hz) / 1000.0,
        f"{electrode.impedance_ohms / 1e3:.0f} kohm at {ROOM_TEMPERATURE_K:.0f} K",
    ))

    # The protection resistors, in series with the signal.
    terms.append(NoiseTerm(
        "input series resistors",
        thermal_noise_nv(2.0 * components.r_series, flat_hz) / 1000.0,
        f"2 x {components.r_series / 1e3:.1f} kohm",
    ))

    # The gain-setting resistor, referred to input.
    terms.append(NoiseTerm(
        "gain-setting resistor",
        thermal_noise_nv(components.r_gain, flat_hz) / 1000.0 / components.stage1_gain,
        f"{components.r_gain / 1e3:.1f} kohm, divided by the stage gain",
    ))

    # The second stage, referred back through the first stage's gain.
    terms.append(NoiseTerm(
        f"{second_stage.name} second stage",
        amplifier_noise_nv(second_stage, flat_hz, flicker_hz) / 1000.0
        / components.stage1_gain,
        f"divided by the {components.stage1_gain:.1f} in front of it",
    ))

    # The converter, referred all the way back. This is the term that decides
    # the design.
    terms.append(NoiseTerm(
        "converter",
        components.adc_noise_lsb_rms * components.microvolts_per_count,
        f"{components.adc_noise_lsb_rms:.0f} LSB RMS at "
        f"{components.microvolts_per_count:.3f} uV per count",
    ))

    return NoiseBudget(electrode.name, (20.0, 450.0), flat_hz, terms)


def signal_to_noise_db(budget: NoiseBudget, signal_microvolts_rms: float) -> float:
    return 20.0 * math.log10(signal_microvolts_rms / budget.total_microvolts_rms)


# --- common-mode rejection --------------------------------------------------


def common_mode_rejection(components: Components,
                          electrode: ElectrodeInterface = ELECTRODE_CASES[0],
                          electrode_mismatch: float = 0.2,
                          resistor_tolerance: float = 0.001,
                          amplifier_cmrr_db: float = 100.0,
                          common_mode_input_ohms: float = 1.0e9) -> dict:
    """What the mains interference on the body actually gets attenuated by.

    Three limits and the amplifier's own specification is usually not the one
    that binds. Common mode reaches the two inputs through two different source
    impedances and any imbalance between them turns it into a difference
    before the amplifier sees any of it. In biopotential measurement that
    imbalance is the electrodes, not the board: two nominally identical
    electrodes on the same arm routinely differ by twenty percent and no
    resistor tolerance can help with that.

    The common-mode input impedance is quoted at 1 gigohm, which is the
    INA333's 3 pF of input capacitance at 50 Hz rather than its DC figure. At
    DC the number would be a hundred times better and the conclusion would be
    wrong.
    """
    # Electrode imbalance against the amplifier's common-mode input impedance.
    electrode_delta = electrode_mismatch * electrode.impedance_ohms
    electrode_conversion = electrode_delta / common_mode_input_ohms
    electrode_db = -20.0 * math.log10(electrode_conversion) if electrode_conversion > 0 else 300.0

    # The series protection resistors, matched to their tolerance.
    resistor_delta = 2.0 * resistor_tolerance * components.r_series
    resistor_conversion = resistor_delta / common_mode_input_ohms
    resistor_db = -20.0 * math.log10(resistor_conversion) if resistor_conversion > 0 else 300.0

    amplifier_conversion = 10 ** (-amplifier_cmrr_db / 20.0)

    total_conversion = math.sqrt(
        electrode_conversion ** 2 + resistor_conversion ** 2 + amplifier_conversion ** 2
    )
    combined_db = -20.0 * math.log10(total_conversion)

    limits = {
        "electrode imbalance": electrode_db,
        "series resistor tolerance": resistor_db,
        "amplifier specification": amplifier_cmrr_db,
    }
    binding = min(limits, key=limits.get)

    return {
        "electrode": electrode.name,
        "electrode_mismatch": electrode_mismatch,
        "common_mode_input_ohms": common_mode_input_ohms,
        "resistor_tolerance": resistor_tolerance,
        "limits_db": limits,
        "binding_limit": binding,
        "combined_db": combined_db,
        "amplifier_cmrr_db": amplifier_cmrr_db,
        "note": (
            "The driven right leg is fitted regardless of which limit binds. It "
            "reduces the common-mode voltage on the body rather than rejecting "
            "it afterwards, which is the only thing that helps when the "
            "interference is larger than the input range."
        ),
    }


def mains_rejection(components: Components, cmrr_db: float,
                    body_common_mode_v: float = 1.0,
                    drl_gain: float = 45.0) -> dict:
    """How much 50 Hz reaches the converter, in microvolts at the electrode.

    A person standing near mains wiring carries a volt or so of common mode.
    The driven right leg divides it by its loop gain, the amplifier rejects
    what is left and the digital notch takes another 30 dB. This is the chain
    that decides whether the 50 Hz term is above or below the noise floor.
    """
    after_drl = body_common_mode_v / (1.0 + drl_gain)
    after_cmrr = after_drl / (10 ** (cmrr_db / 20.0))
    # The firmware's notch, from the measured rejection in
    # firmware/test/test_dsp.cpp.
    after_notch = after_cmrr / (10 ** (34.0 / 20.0))
    return {
        "body_common_mode_v": body_common_mode_v,
        "driven_right_leg_gain": drl_gain,
        "after_drl_mv": after_drl * 1e3,
        "after_cmrr_uv": after_cmrr * 1e6,
        "after_digital_notch_uv": after_notch * 1e6,
        "cmrr_db": cmrr_db,
        "digital_notch_db": 34.0,
    }
