#include "pa/semg/frontend.hpp"

namespace pa::semg {

void FrontEnd::configure(const ChannelConfig& config) {
    for (auto& channel : channels_) channel.configure(config);
    sequence_ = 0;
}

void FrontEnd::reset() {
    for (auto& channel : channels_) channel.reset();
    sequence_ = 0;
}

bool FrontEnd::filters_stable() const {
    for (const auto& channel : channels_)
        if (!channel.filters_stable()) return false;
    return true;
}

bool FrontEnd::push(const std::array<int16_t, config::kSemgChannels>& raw, uint32_t timestamp_us,
                    Frame& frame) {
    std::array<ChannelSample, config::kSemgChannels> samples{};
    std::size_t ready = 0;

    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        if (channels_[i].push(raw[i], samples[i])) ++ready;
    }

    // All or nothing. Every channel is driven from the same interrupt with the
    // same decimation counter, so a partial count means state has been
    // corrupted and shipping a frame with some channels one sample older than
    // others would show up as poor classification rather than as a fault.
    if (ready == 0) return false;
    if (ready != config::kSemgChannels) {
        reset();
        return false;
    }

    frame.timestamp_us = timestamp_us;
    frame.sequence = sequence_++;
    frame.saturated_mask = 0;
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) {
        frame.samples[i] = samples[i].value;
        frame.envelopes[i] = samples[i].envelope;
        if (samples[i].saturated) frame.saturated_mask |= static_cast<uint8_t>(1u << i);
    }
    frame.electrode_fault = frame.saturated_mask != 0;
    return true;
}

std::array<float, config::kSemgChannels> FrontEnd::rms_counts() const {
    std::array<float, config::kSemgChannels> out{};
    for (std::size_t i = 0; i < config::kSemgChannels; ++i) out[i] = channels_[i].rms();
    return out;
}

std::array<float, config::kSemgChannels> FrontEnd::rms_microvolts_at_electrode() const {
    std::array<float, config::kSemgChannels> out = rms_counts();
    for (auto& value : out) value = counts_to_microvolts(value);
    return out;
}

}  // namespace pa::semg
