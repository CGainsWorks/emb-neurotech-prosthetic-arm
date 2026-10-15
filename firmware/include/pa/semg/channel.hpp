// One sEMG channel, from raw ADC counts to a decimated sample and an envelope.
//
// The chain, in order and why it is in that order:
//
//   1. DC block. The electrode half-cell potential is tens of millivolts and
//      drifts; at a gain of 500 it would walk the second stage into a rail.
//      It has to go first, before anything with memory sees it.
//   2. Mains notch. What the instrumentation amplifier's common-mode
//      rejection and the driven right leg leave behind. Before the anti-alias
//      filter, because 50 Hz is inside the band that survives decimation and
//      would otherwise land squarely in the classifier's input.
//   3. Anti-alias low pass, then decimate by ten. Everything above 90 Hz is
//      removed before the rate drops to 200 Hz, so nothing folds back.
//   4. Envelope, taken from the notched signal rather than the decimated one,
//      so the direct proportional control path keeps the full bandwidth and
//      does not inherit the decimation delay.
//
// No windowing, no feature extraction and no classification happen here. The
// board ships samples; ml-semg-gesture-classifier decides what they mean.
#pragma once

#include <cstdint>

#include "pa/config/board.hpp"
#include "pa/dsp/biquad.hpp"

namespace pa::semg {

struct ChannelConfig {
    float sample_rate_hz{config::kSemgSampleRateHz};
    float dc_block_hz{config::kDcBlockHz};
    float notch_hz{config::kMainsNotchHz};
    float notch_q{config::kMainsNotchQ};
    float anti_alias_hz{config::kAntiAliasHz};
    float envelope_hz{config::kEnvelopeHz};
    bool notch_enabled{true};
};

// What one 200 Hz output frame carries for one channel.
struct ChannelSample {
    int16_t value{0};        // conditioned sample, ADC counts at the same scale as the input
    float envelope{0.0f};    // rectified and smoothed, ADC counts
    bool saturated{false};   // this channel hit a rail somewhere in the last frame
};

class Channel {
  public:
    Channel() { configure(ChannelConfig{}); }

    void configure(const ChannelConfig& config);
    void reset();

    // Feed one raw sample at the acquisition rate. Returns true on the sample
    // that completes a decimation group, in which case `out` is filled.
    bool push(int16_t raw, ChannelSample& out);

    // The most recent envelope, available at the full rate rather than only on
    // frame boundaries, because the direct control path reads it whenever it
    // likes.
    float envelope() const { return envelope_; }

    bool saturated() const { return saturated_run_ > 0; }

    // Root mean square of the conditioned signal, exponentially weighted with
    // a quarter-second time constant. This is the quantity the bring-up
    // procedure compares against the predicted noise floor with the electrodes
    // shorted; see firmware/docs/BRING_UP.md. It has never been run on
    // hardware.
    //
    // Weighted over a quarter of a second rather than reported per decimation
    // group, because a group is ten samples and the root mean square of ten
    // samples of a periodic signal is not the root mean square of the signal:
    // averaging those short estimates biases low by a few percent, which is
    // the wrong side to be wrong on for a noise floor.
    float rms() const { return rms_; }

    const dsp::BiquadCascade<config::kAntiAliasSections>& anti_alias() const {
        return anti_alias_;
    }
    const dsp::Biquad& notch() const { return notch_; }

    // Every section designed at start-up is checked before the first sample is
    // taken. A filter designed for the wrong sample rate can come out
    // unstable and on a limb the first symptom would be movement.
    bool filters_stable() const;

  private:
    ChannelConfig config_{};
    dsp::DcBlock dc_block_{};
    dsp::Biquad notch_{};
    dsp::BiquadCascade<config::kAntiAliasSections> anti_alias_{};
    dsp::BiquadCascade<2> envelope_filter_{};

    uint32_t decimation_counter_{0};
    float envelope_{0.0f};
    float rms_{0.0f};
    float mean_square_{0.0f};
    float rms_alpha_{0.0f};
    float square_sum_{0.0f};
    uint32_t square_count_{0};
    uint32_t saturated_run_{0};
    float last_decimated_{0.0f};
};

}  // namespace pa::semg
