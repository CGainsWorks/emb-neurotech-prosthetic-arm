#include "pa/semg/channel.hpp"

#include <algorithm>
#include <cmath>

namespace pa::semg {
namespace {

// Rail detection. The ADC is signed 16 bit, so a sample within a couple of
// percent of full scale either way is either a saturated stage or an electrode
// that has come off.
constexpr int32_t kSaturationCount =
    static_cast<int32_t>(config::kSaturationFraction * static_cast<float>(config::kAdcFullScale));

int16_t saturating_cast(float value) {
    const float limit = static_cast<float>(config::kAdcFullScale);
    return static_cast<int16_t>(std::max(-limit, std::min(limit, value)));
}

}  // namespace

void Channel::configure(const ChannelConfig& config) {
    config_ = config;
    dc_block_ = dsp::design_dc_block(config.dc_block_hz, config.sample_rate_hz);
    notch_ = dsp::design_notch(config.notch_hz, config.sample_rate_hz, config.notch_q);
    anti_alias_ = dsp::design_butterworth_lowpass<config::kAntiAliasSections>(
        config.anti_alias_hz, config.sample_rate_hz);
    // The envelope filter runs on the rectified signal, which is all positive,
    // so it is a plain low pass rather than a Butterworth: ripple in the stop
    // band of an envelope is not worth two extra sections.
    envelope_filter_ = dsp::design_butterworth_lowpass_4(config.envelope_hz, config.sample_rate_hz);

    // A quarter-second time constant, expressed in decimation groups because
    // that is how often the estimate is updated.
    const float groups_per_second = config.sample_rate_hz / static_cast<float>(config::kSemgDecimation);
    rms_alpha_ = std::exp(-1.0f / (0.25f * groups_per_second));

    reset();
}

void Channel::reset() {
    dc_block_.reset();
    notch_.reset();
    anti_alias_.reset();
    envelope_filter_.reset();
    decimation_counter_ = 0;
    envelope_ = 0.0f;
    rms_ = 0.0f;
    mean_square_ = 0.0f;
    square_sum_ = 0.0f;
    square_count_ = 0;
    saturated_run_ = 0;
    last_decimated_ = 0.0f;
}

bool Channel::filters_stable() const {
    return notch_.stable() && anti_alias_.stable() && envelope_filter_.stable() &&
           dc_block_.alpha > 0.0f && dc_block_.alpha < 1.0f;
}

bool Channel::push(int16_t raw, ChannelSample& out) {
    if (std::abs(static_cast<int32_t>(raw)) >= kSaturationCount) {
        ++saturated_run_;
    } else if (saturated_run_ > 0) {
        --saturated_run_;
    }

    float x = dc_block_.process(static_cast<float>(raw));
    if (config_.notch_enabled) x = notch_.process(x);

    // The envelope is taken here, before decimation, so the direct
    // proportional path keeps the full 1 kHz of bandwidth.
    envelope_ = envelope_filter_.process(std::fabs(x));

    square_sum_ += x * x;
    ++square_count_;

    last_decimated_ = anti_alias_.process(x);

    if (++decimation_counter_ < config::kSemgDecimation) return false;
    decimation_counter_ = 0;

    if (square_count_ > 0) {
        const float group = square_sum_ / static_cast<float>(square_count_);
        mean_square_ = rms_alpha_ * mean_square_ + (1.0f - rms_alpha_) * group;
        rms_ = std::sqrt(mean_square_);
    }
    square_sum_ = 0.0f;
    square_count_ = 0;

    out.value = saturating_cast(last_decimated_);
    out.envelope = envelope_;
    out.saturated = saturated_run_ > 0;
    return true;
}

}  // namespace pa::semg
