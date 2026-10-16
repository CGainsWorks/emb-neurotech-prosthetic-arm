// Board and system constants for the A1 sEMG and servo board, revision A.
//
// Every number here is a design decision with a reason attached and several
// of them appear in more than one place in the repository. Where that happens
// the value lives here and the other place derives it, so there is nowhere for
// two versions of the same constant to disagree.
//
// Nothing in this file has been checked against hardware. No board has been
// fabricated. See STATUS.md and firmware/docs/BRING_UP.md.
#pragma once

#include <cstddef>
#include <cstdint>

namespace pa::config {

// ---------------------------------------------------------------------------
// Microcontroller
// ---------------------------------------------------------------------------
// STM32H743VIT6: Cortex-M7 at 480 MHz, 2 MB flash, 1 MB RAM.
//
// The part is chosen by the classifier, not by this repository.
// ml-semg-gesture-classifier estimates its deployed int8 network at about
// 83 ms by arithmetic and 158 ms under Renode on a 100 MHz Cortex-M4F
// (STM32F411): it fits that part's flash and RAM but misses the 300 ms
// end-to-end budget. On this part the arithmetic estimate is about 13 ms,
// which makes it with room to spare. Its README recommends this part.
// Estimates both: no board has run either.
inline constexpr uint32_t kCpuHz = 480'000'000u;

// ---------------------------------------------------------------------------
// sEMG acquisition
// ---------------------------------------------------------------------------
inline constexpr std::size_t kSemgChannels = 8;

// 2 kHz per channel. Surface EMG has essentially no energy above 500 Hz, so
// 1 kHz of Nyquist covers the band with margin for the anti-alias skirt and
// oversampling by an order of magnitude over the 200 Hz the classifier wants
// is what makes the decimation filter cheap.
inline constexpr float kSemgSampleRateHz = 2000.0f;

// The rate frames leave the board at. It matches the Ninapro DB5 recordings
// the gesture classifier is trained on: two Myo armbands, sixteen channels at
// 200 Hz. docs/INTEGRATION.md
// covers what that costs.
inline constexpr float kSemgFrameRateHz = 200.0f;
inline constexpr std::size_t kSemgDecimation =
    static_cast<std::size_t>(kSemgSampleRateHz / kSemgFrameRateHz);

// Electrode half-cell potentials drift by tens of millivolts over minutes and
// would otherwise walk the gain stage into a rail. 5 Hz is below the lowest
// motor unit firing rate worth keeping.
inline constexpr float kDcBlockHz = 5.0f;

// UK mains. The instrumentation amplifier's common-mode rejection and the
// driven right leg do most of this; the notch is what is left over.
inline constexpr float kMainsNotchHz = 50.0f;
inline constexpr float kMainsNotchQ = 30.0f;

// Anti-alias before decimating by ten, as a sixth-order Butterworth at 80 Hz.
//
// The output rate is fixed at 200 Hz by the dataset the classifier is trained
// on, so Nyquist is 100 Hz and there is no room for a transition band above a
// wanted band that reaches 99 Hz. Something has to give. The corner is placed
// at 80 Hz and the order raised to six, which keeps the response inside a
// third of a decibel to 65 Hz and puts everything from 160 Hz upwards more
// than 37 dB down; what folds back from 100 to 160 Hz is attenuated by 12 to
// 37 dB and is the residual this design accepts.
//
// docs/INTEGRATION.md has the numbers and the honest conclusion: the clean fix
// is to retrain the classifier at the acquisition rate rather than to decimate
// to meet it.
inline constexpr float kAntiAliasHz = 80.0f;
inline constexpr std::size_t kAntiAliasSections = 3;  // sixth order

// Envelope detector for the direct proportional control path, which does not
// go through the classifier at all. 4 Hz is the usual compromise: fast enough
// that a wearer does not feel the lag, slow enough that the envelope is smooth.
inline constexpr float kEnvelopeHz = 4.0f;

// ---------------------------------------------------------------------------
// Analogue front end
// ---------------------------------------------------------------------------
// Two-stage gain: an INA333 instrumentation amplifier at 10.09, then a second
// stage at 30.30 after the high pass. Splitting it is what keeps the electrode
// half-cell offset from saturating the first stage.
//
// About 300 rather than the 500 a textbook would suggest: at 500 the
// full-scale input is 3 mV at the electrode and a strong contraction over
// forearm flexors reaches 5 mV peak. Clipping the largest signals is a worse
// fault than using two thirds of the converter's range, because a clipped
// contraction is exactly the one the classifier most needs to get right.
//
// The value is not round because the resistors are not: 1 + 100k/11k times
// 1M/33k, all E96 parts. hardware/semg-servo-board/analysis/afe_model.py
// computes it from the component values and a test there asserts this constant
// still agrees with them.
inline constexpr float kFrontEndGain = 305.785f;
inline constexpr float kAdcReferenceVolts = 3.0f;
inline constexpr std::size_t kAdcBits = 16;
inline constexpr int32_t kAdcFullScale = (1 << (kAdcBits - 1)) - 1;

// Volts at the electrode per ADC count, before any digital gain.
inline constexpr float kVoltsPerCount =
    kAdcReferenceVolts / (2.0f * static_cast<float>(kAdcFullScale) * kFrontEndGain);

// A channel spending this fraction of a window against a rail is not
// measuring anything; it is a lifted electrode or a saturated stage.
inline constexpr float kSaturationFraction = 0.98f;

// ---------------------------------------------------------------------------
// Servo drive
// ---------------------------------------------------------------------------
inline constexpr std::size_t kServoChannels = 6;

// 333 Hz frame rate: the fastest a digital servo of this class accepts and
// three times the 100 Hz the joint controller runs at, so a command is never
// more than one frame stale.
inline constexpr float kServoFrameRateHz = 333.0f;
inline constexpr float kServoMinPulseUs = 500.0f;
inline constexpr float kServoCentrePulseUs = 1500.0f;
inline constexpr float kServoMaxPulseUs = 2500.0f;

// Microseconds of pulse per radian at the servo horn, from the 1000 us over
// 180 degrees the class is specified for.
inline constexpr float kServoUsPerRadian = 1000.0f / 3.14159265358979323846f;

// ---------------------------------------------------------------------------
// Control and safety
// ---------------------------------------------------------------------------
inline constexpr float kJointControlRateHz = 100.0f;
inline constexpr float kImuRateHz = 100.0f;

// If no joint command arrives inside this window the arm holds position and
// then relaxes. Three joint control periods: long enough to ride out one lost
// packet, short enough that a dead host cannot leave the arm moving.
inline constexpr uint32_t kCommandTimeoutMs = 30;

// Two protections, at two speeds and they do different jobs.
//
// The current trip is the fast one. Every actuator stalled at once draws
// 10.2 A, a synchronised move of all six draws about 5.5 A and 8 A sits
// between them: a jam trips it inside a control period and no trajectory the
// planner can produce reaches it.
//
// The temperature trip is the slow one. The board can only carry about 6.4 A
// indefinitely before it reaches 85 C, which is below the current trip on
// purpose. A prosthesis moves in bursts and idles between them, so a limit
// that allowed nothing above the continuous figure would make the arm useless;
// the thermal trip is what catches the case where the bursts stop being
// bursts.
//
// hardware/semg-servo-board/docs/thermal_budget.md derives all three numbers.
inline constexpr float kBusCurrentLimitAmps = 8.0f;
inline constexpr float kBoardTemperatureLimitC = 85.0f;

// ---------------------------------------------------------------------------
// Link
// ---------------------------------------------------------------------------
inline constexpr uint32_t kUartBaud = 921'600u;

}  // namespace pa::config
