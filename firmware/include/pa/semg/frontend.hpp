// The eight-channel front end: the thing the acquisition interrupt talks to.
#pragma once

#include <array>
#include <cstdint>

#include "pa/config/board.hpp"
#include "pa/semg/channel.hpp"

namespace pa::semg {

// One 200 Hz frame: every channel, plus the state that goes with it.
struct Frame {
    uint32_t timestamp_us{0};
    uint32_t sequence{0};
    std::array<int16_t, config::kSemgChannels> samples{};
    std::array<float, config::kSemgChannels> envelopes{};
    uint8_t saturated_mask{0};   // bit i set when channel i is against a rail
    bool electrode_fault{false}; // any channel saturated, latched for this frame
};

class FrontEnd {
  public:
    FrontEnd() { configure(ChannelConfig{}); }

    void configure(const ChannelConfig& config);
    void reset();

    // Feed one sample per channel, taken at the same instant. Returns true
    // when a decimated frame is ready.
    //
    // Every channel decimates on the same schedule, so either all of them
    // produce a sample or none does. That is asserted rather than assumed:
    // channels drifting apart would put the classifier's electrodes out of
    // step with each other, which is exactly the kind of fault that looks like
    // poor accuracy rather than like a bug.
    bool push(const std::array<int16_t, config::kSemgChannels>& raw, uint32_t timestamp_us,
              Frame& frame);

    // Root mean square per channel over the last decimation group, in ADC
    // counts. Converted to microvolts at the electrode by
    // rms_microvolts_at_electrode().
    std::array<float, config::kSemgChannels> rms_counts() const;
    std::array<float, config::kSemgChannels> rms_microvolts_at_electrode() const;

    bool filters_stable() const;

    uint32_t frames_produced() const { return sequence_; }

    Channel& channel(std::size_t index) { return channels_[index]; }
    const Channel& channel(std::size_t index) const { return channels_[index]; }

  private:
    std::array<Channel, config::kSemgChannels> channels_{};
    uint32_t sequence_{0};
};

// ADC counts to microvolts referred to the electrode, using the front end gain
// and the reference in pa/config/board.hpp. This is the conversion the noise
// figures in hardware/semg-servo-board/docs/noise_analysis.md are quoted
// through, so it lives in one place.
constexpr float counts_to_microvolts(float counts) {
    return counts * config::kVoltsPerCount * 1e6f;
}

}  // namespace pa::semg
