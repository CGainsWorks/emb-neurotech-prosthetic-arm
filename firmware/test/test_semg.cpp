#include <cmath>
#include <random>
#include <vector>

#include "pa/semg/frontend.hpp"
#include "test_framework.hpp"

using namespace pa;
using namespace pa::semg;

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kFs = config::kSemgSampleRateHz;

int16_t sine_counts(float hz, int n, float amplitude, float offset = 0.0f) {
    const float phase = 2.0f * kPi * hz * static_cast<float>(n) / kFs;
    const float value = offset + amplitude * std::sin(phase);
    return static_cast<int16_t>(std::max(-32767.0f, std::min(32767.0f, value)));
}

// Feed a channel a sine wave and report the amplitude of what comes out of the
// decimator, measured after the transients have gone.
float decimated_amplitude(Channel& channel, float hz, float amplitude) {
    channel.reset();
    ChannelSample sample;
    for (int n = 0; n < 40000; ++n) channel.push(sine_counts(hz, n, amplitude), sample);

    double sum = 0.0;
    int count = 0;
    for (int n = 40000; n < 60000; ++n) {
        if (channel.push(sine_counts(hz, n, amplitude), sample)) {
            sum += static_cast<double>(sample.value) * sample.value;
            ++count;
        }
    }
    return count > 0 ? static_cast<float>(std::sqrt(sum / count) * std::sqrt(2.0)) : 0.0f;
}

}  // namespace

TEST(a_channel_decimates_by_exactly_ten) {
    Channel channel;
    ChannelSample sample;
    int produced = 0;
    for (int n = 0; n < 10000; ++n)
        if (channel.push(sine_counts(60.0f, n, 1000.0f), sample)) ++produced;
    CHECK_EQ(produced, 1000);
    CHECK_EQ(static_cast<int>(config::kSemgDecimation), 10);
}

TEST(a_channel_passes_the_semg_band_and_stops_what_would_alias) {
    // End to end through the real chain, sine in and decimated samples out,
    // rather than through the transfer functions the sections were designed
    // from. It is the only way to catch a mistake in the order the stages are
    // applied, or in the decimator itself.
    Channel channel;
    // Inside the band the classifier reads.
    CHECK(decimated_amplitude(channel, 30.0f, 1000.0f) > 950.0f);
    CHECK(decimated_amplitude(channel, 65.0f, 1000.0f) > 900.0f);
    // Mains, notched out.
    CHECK(decimated_amplitude(channel, 50.0f, 1000.0f) < 60.0f);
    // Above the anti-alias corner, on its way down. This one folds to 50 Hz.
    CHECK(decimated_amplitude(channel, 150.0f, 1000.0f) < 40.0f);
    // Far enough up that it is gone entirely.
    CHECK(decimated_amplitude(channel, 400.0f, 1000.0f) < 5.0f);
    CHECK(decimated_amplitude(channel, 800.0f, 1000.0f) < 2.0f);
}

TEST(a_channel_removes_the_electrode_offset) {
    // Half a volt of half-cell potential at the electrode is a large offset by
    // the time it has been through a gain of 500. It must not appear in the
    // output at all.
    Channel channel;
    ChannelSample sample;
    for (int n = 0; n < 60000; ++n) channel.push(sine_counts(60.0f, n, 800.0f, 12000.0f), sample);

    double mean = 0.0;
    int count = 0;
    for (int n = 60000; n < 80000; ++n) {
        if (channel.push(sine_counts(60.0f, n, 800.0f, 12000.0f), sample)) {
            mean += sample.value;
            ++count;
        }
    }
    mean /= count;
    CHECK_NEAR(mean, 0.0, 20.0);
}

TEST(the_envelope_tracks_a_burst_of_activity) {
    // What the direct proportional control path reads: quiet, then a
    // contraction, then quiet again.
    Channel channel;
    ChannelSample sample;
    std::mt19937 rng(1);
    std::normal_distribution<float> noise(0.0f, 1.0f);

    auto run = [&](float amplitude, int samples) {
        for (int n = 0; n < samples; ++n) {
            // Band-limited noise is a better stand-in for sEMG than a sine:
            // it is what the signal actually is.
            const float value = amplitude * noise(rng);
            channel.push(static_cast<int16_t>(std::max(-32767.0f, std::min(32767.0f, value))),
                         sample);
        }
        return channel.envelope();
    };

    const float rest = run(20.0f, 8000);
    const float contraction = run(2000.0f, 8000);
    const float back_to_rest = run(20.0f, 8000);

    CHECK(contraction > rest * 10.0f);
    CHECK(back_to_rest < contraction * 0.2f);
    CHECK(rest > 0.0f);
}

TEST(a_channel_reports_a_rail_as_a_fault_rather_than_a_number) {
    // A lifted electrode drives the front end into a rail. The samples that
    // come out look like a large clean signal, which is exactly what a
    // classifier would happily assign a confident label to.
    Channel channel;
    ChannelSample sample;
    for (int n = 0; n < 400; ++n) channel.push(32700, sample);
    CHECK(channel.saturated());

    for (int n = 0; n < 4000; ++n) channel.push(sine_counts(60.0f, n, 500.0f), sample);
    CHECK(!channel.saturated());
}

TEST(the_root_mean_square_estimate_is_right_for_a_known_signal) {
    // The quantity firmware/docs/BRING_UP.md compares against the predicted
    // noise floor. A sine of amplitude A has an RMS of A over root two.
    Channel channel;
    ChannelSample sample;
    constexpr float kAmplitude = 1000.0f;
    for (int n = 0; n < 40000; ++n) channel.push(sine_counts(60.0f, n, kAmplitude), sample);

    double total = 0.0;
    int count = 0;
    for (int n = 40000; n < 60000; ++n) {
        if (channel.push(sine_counts(60.0f, n, kAmplitude), sample)) {
            total += channel.rms();
            ++count;
        }
    }
    CHECK_NEAR(total / count, kAmplitude / std::sqrt(2.0), 20.0);
}

TEST(counts_convert_to_microvolts_at_the_electrode) {
    // Full scale at the converter is half the reference, referred back through
    // the gain to the electrode: 1.5 V over 305.785 is 4.905 mV, so one count
    // is 150 nV. The whole noise budget is quoted through this conversion, so
    // it is worth pinning to a number that can be checked by hand.
    CHECK_NEAR(counts_to_microvolts(static_cast<float>(config::kAdcFullScale)), 4905.4f, 1.0);
    CHECK_NEAR(counts_to_microvolts(1.0f), 4905.4f / 32767.0f, 1e-4);
    CHECK_NEAR(counts_to_microvolts(0.0f), 0.0f, 1e-12);

    // And the full-scale input covers a strong contraction. Surface EMG over
    // forearm flexors reaches about 5 mV peak, which is what set the gain; the
    // front end clips a little under that and
    // hardware/semg-servo-board/docs/noise_analysis.md says so rather than
    // rounding it away.
    CHECK(counts_to_microvolts(static_cast<float>(config::kAdcFullScale)) > 4800.0f);
}

TEST(the_front_end_produces_one_frame_per_ten_samples_on_every_channel) {
    FrontEnd front_end;
    Frame frame;
    std::array<int16_t, config::kSemgChannels> raw{};

    int frames = 0;
    for (int n = 0; n < 20000; ++n) {
        for (std::size_t c = 0; c < config::kSemgChannels; ++c)
            raw[c] = sine_counts(60.0f + 5.0f * static_cast<float>(c), n, 500.0f);
        if (front_end.push(raw, static_cast<uint32_t>(n * 500), frame)) {
            CHECK_EQ(static_cast<int>(frame.sequence), frames);
            ++frames;
        }
    }
    CHECK_EQ(frames, 2000);
    CHECK_EQ(static_cast<int>(front_end.frames_produced()), 2000);
}

TEST(the_front_end_reports_which_electrode_lifted) {
    FrontEnd front_end;
    Frame frame;
    std::array<int16_t, config::kSemgChannels> raw{};

    for (int n = 0; n < 2000; ++n) {
        for (std::size_t c = 0; c < config::kSemgChannels; ++c)
            raw[c] = sine_counts(60.0f, n, 500.0f);
        raw[3] = 32760;  // channel three against a rail
        front_end.push(raw, static_cast<uint32_t>(n * 500), frame);
    }
    CHECK(frame.electrode_fault);
    CHECK_EQ(static_cast<int>(frame.saturated_mask), 1 << 3);
}

TEST(the_front_end_channels_stay_in_step) {
    // Every channel decimates on the same counter. If they ever came apart the
    // classifier would see electrodes sampled at different instants, which
    // shows up as poor accuracy rather than as a fault, so it is worth an
    // explicit check.
    FrontEnd front_end;
    Frame frame;
    std::array<int16_t, config::kSemgChannels> raw{};
    for (int n = 0; n < 10000; ++n) {
        for (std::size_t c = 0; c < config::kSemgChannels; ++c) raw[c] = static_cast<int16_t>(n % 97);
        const bool ready = front_end.push(raw, static_cast<uint32_t>(n), frame);
        CHECK_EQ(ready, (n % 10) == 9);
    }
}

TEST(every_filter_the_front_end_designs_is_stable) {
    FrontEnd front_end;
    CHECK(front_end.filters_stable());

    ChannelConfig config;
    config.sample_rate_hz = 500.0f;
    front_end.configure(config);
    CHECK(front_end.filters_stable());
}

TEST(turning_the_notch_off_lets_mains_through) {
    // A configuration option that does nothing is worse than no option, so
    // check the flag reaches the signal.
    Channel with_notch, without_notch;
    ChannelConfig config;
    config.notch_enabled = false;
    without_notch.configure(config);

    CHECK(decimated_amplitude(with_notch, 50.0f, 1000.0f) < 60.0f);
    CHECK(decimated_amplitude(without_notch, 50.0f, 1000.0f) > 800.0f);
}

TEST_MAIN()
