#include <cmath>
#include <vector>

#include "pa/config/board.hpp"
#include "pa/dsp/biquad.hpp"
#include "test_framework.hpp"

using namespace pa::dsp;

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kFs = pa::config::kSemgSampleRateHz;

// Drive a filter with a sine wave and measure what comes out. This is the
// measurement that matters: the closed-form magnitude response is derived from
// the coefficients, so checking one against the other only proves the algebra.
// Running the actual difference equation proves the filter.
template <typename Filter>
float measured_gain(Filter& filter, float hz, float sample_rate_hz) {
    filter.reset();
    const int settle = 20000;
    const int measure = 8000;

    for (int n = 0; n < settle; ++n) {
        filter.process(std::sin(2.0f * kPi * hz * static_cast<float>(n) / sample_rate_hz));
    }
    // Root mean square of the output against the known root mean square of a
    // unit sine, which is 1/sqrt(2).
    double sum = 0.0;
    for (int n = 0; n < measure; ++n) {
        const float phase = 2.0f * kPi * hz * static_cast<float>(settle + n) / sample_rate_hz;
        const float y = filter.process(std::sin(phase));
        sum += static_cast<double>(y) * y;
    }
    const double rms = std::sqrt(sum / measure);
    return static_cast<float>(rms * std::sqrt(2.0));
}

}  // namespace

TEST(a_lowpass_passes_dc_and_stops_nyquist) {
    Biquad filter = design_lowpass(100.0f, kFs, 0.7071f);
    CHECK_NEAR(filter.magnitude(0.0f, kFs), 1.0f, 1e-4);
    CHECK(filter.magnitude(kFs * 0.5f, kFs) < 1e-3f);
    CHECK_NEAR(filter.magnitude(100.0f, kFs), 0.70710678f, 1e-3);
}

TEST(a_highpass_stops_dc_and_passes_nyquist) {
    Biquad filter = design_highpass(100.0f, kFs, 0.7071f);
    CHECK(filter.magnitude(0.0f, kFs) < 1e-6f);
    CHECK_NEAR(filter.magnitude(kFs * 0.5f, kFs), 1.0f, 1e-4);
    CHECK_NEAR(filter.magnitude(100.0f, kFs), 0.70710678f, 1e-3);
}

TEST(a_notch_removes_its_centre_and_leaves_the_rest) {
    Biquad filter = design_notch(pa::config::kMainsNotchHz, kFs, pa::config::kMainsNotchQ);
    // The null is a true zero on the unit circle; what is left at the centre
    // is single-precision cancellation in the response evaluation, about
    // 76 dB down. The measured test below drives a real 50 Hz sine through the
    // difference equation and is the one that says what the filter does.
    CHECK(filter.magnitude(50.0f, kFs) < 5e-4f);
    CHECK_NEAR(filter.magnitude(0.0f, kFs), 1.0f, 1e-4);
    CHECK_NEAR(filter.magnitude(kFs * 0.5f, kFs), 1.0f, 1e-4);

    // The point of a high Q: a decade either side of the notch, the response
    // is essentially untouched. sEMG has real energy at 20 Hz and 100 Hz and
    // it has to survive.
    CHECK(filter.magnitude(20.0f, kFs) > 0.99f);
    CHECK(filter.magnitude(100.0f, kFs) > 0.99f);
}

TEST(the_measured_response_matches_the_designed_one) {
    // The difference equation against the transfer function it was designed
    // from. If the coefficient normalisation or the state update is wrong,
    // these part company.
    Biquad filter = design_lowpass(120.0f, kFs, 0.7071f);
    for (float hz : {10.0f, 60.0f, 120.0f, 300.0f, 600.0f}) {
        const float predicted = filter.magnitude(hz, kFs);
        const float measured = measured_gain(filter, hz, kFs);
        CHECK_NEAR(measured, predicted, 0.01);
    }
}

TEST(the_measured_notch_actually_rejects_mains) {
    Biquad filter = design_notch(50.0f, kFs, 30.0f);
    const float at_mains = measured_gain(filter, 50.0f, kFs);
    const float nearby = measured_gain(filter, 80.0f, kFs);
    CHECK(at_mains < 0.02f);
    CHECK(nearby > 0.95f);
    // At least 34 dB of rejection at the notch relative to the passband.
    CHECK(20.0f * std::log10(nearby / std::max(at_mains, 1e-9f)) > 34.0f);
}

TEST(a_butterworth_cascade_is_maximally_flat) {
    BiquadCascade<2> filter = design_butterworth_lowpass<2>(90.0f, kFs);

    CHECK_NEAR(filter.magnitude(0.0f, kFs), 1.0f, 1e-4);
    // Half power at the corner, whatever the order.
    CHECK_NEAR(filter.magnitude(90.0f, kFs), 0.70710678f, 5e-3);

    // Fourth order is 24 dB per octave. One octave above the corner the
    // response should be about a sixteenth of the passband, give or take the
    // frequency warping the bilinear transform introduces.
    const float octave = filter.magnitude(180.0f, kFs);
    CHECK(octave < 0.075f);
    CHECK(octave > 0.045f);

    // Monotone in the passband: no ripple. That is what distinguishes
    // Butterworth from Chebyshev and it is why it is used here, since ripple
    // in the band the classifier reads is an amplitude bias that varies with
    // frequency and would look like a property of the muscle.
    // The tolerance is single-precision rounding in the response evaluation,
    // not ripple. Chebyshev ripple at this order would be tenths of a decibel,
    // which is percent, so a bound of one part in ten thousand still tells the
    // two designs apart.
    float previous = filter.magnitude(0.0f, kFs);
    for (float hz = 1.0f; hz < 90.0f; hz += 1.0f) {
        const float gain = filter.magnitude(hz, kFs);
        CHECK(gain <= previous + 1e-4f);
        previous = gain;
    }
}

TEST(raising_the_order_steepens_the_skirt_and_leaves_the_passband_alone) {
    BiquadCascade<2> fourth = design_butterworth_lowpass<2>(80.0f, kFs);
    BiquadCascade<3> sixth = design_butterworth_lowpass<3>(80.0f, kFs);

    // Same corner.
    CHECK_NEAR(fourth.magnitude(80.0f, kFs), 0.70710678f, 5e-3);
    CHECK_NEAR(sixth.magnitude(80.0f, kFs), 0.70710678f, 5e-3);
    // Same passband.
    CHECK_NEAR(sixth.magnitude(20.0f, kFs), fourth.magnitude(20.0f, kFs), 0.01);
    // Steeper skirt.
    CHECK(sixth.magnitude(160.0f, kFs) < fourth.magnitude(160.0f, kFs) * 0.4f);
    CHECK(sixth.stable());
}

TEST(the_anti_alias_filter_does_what_the_decimator_needs_and_no_more) {
    // The design compromise this whole chain turns on, written down as a test.
    //
    // The output rate is 200 Hz because that is the rate Ninapro DB5 was
    // recorded at, so Nyquist is 100 Hz. Everything from 101 to 199 Hz folds
    // into the band the classifier reads and no filter can prevent that while
    // keeping a passband that reaches 99 Hz, because there is no transition
    // band left. What follows is what this design actually achieves.
    BiquadCascade<pa::config::kAntiAliasSections> filter =
        design_butterworth_lowpass<pa::config::kAntiAliasSections>(pa::config::kAntiAliasHz, kFs);

    // Flat where the signal is. sEMG power over forearm muscles peaks between
    // 50 and 150 Hz; the part of it below 65 Hz arrives untouched.
    for (float hz = 1.0f; hz <= 65.0f; hz += 1.0f) CHECK(filter.magnitude(hz, kFs) > 0.955f);
    CHECK_NEAR(filter.magnitude(pa::config::kAntiAliasHz, kFs), 0.70710678f, 5e-3);

    // Above 160 Hz, better than 37 dB down, so what folds back is below the
    // front end noise floor rather than on top of the signal.
    for (float hz = 160.0f; hz <= 1000.0f; hz += 5.0f) CHECK(filter.magnitude(hz, kFs) < 0.0142f);

    // Between 100 and 160 Hz is the residual this design accepts and it is
    // bounded rather than ignored: nothing in that range folds back at more
    // than a quarter of its amplitude.
    for (float hz = 100.0f; hz <= 160.0f; hz += 1.0f) CHECK(filter.magnitude(hz, kFs) < 0.26f);
}

TEST(every_designed_section_is_stable) {
    // Every filter the board designs at start-up, across every rate it could
    // plausibly be configured for.
    for (float fs : {500.0f, 1000.0f, 2000.0f, 4000.0f, 8000.0f}) {
        CHECK(design_butterworth_lowpass_4(pa::config::kAntiAliasHz, fs).stable());
        CHECK(design_butterworth_highpass_4(20.0f, fs).stable());
        CHECK(design_notch(50.0f, fs, 30.0f).stable());
        CHECK(design_lowpass(pa::config::kEnvelopeHz, fs, 0.7071f).stable());
    }
}

TEST(a_cutoff_at_or_above_nyquist_is_clamped_rather_than_exploding) {
    // The bilinear transform sends Nyquist to infinity. A misconfigured rate
    // must produce a usable filter, not a NaN, because on a limb the first
    // symptom of a NaN is movement.
    for (float cutoff : {1000.0f, 1500.0f, 100000.0f}) {
        Biquad filter = design_lowpass(cutoff, kFs, 0.7071f);
        CHECK(std::isfinite(filter.b0));
        CHECK(std::isfinite(filter.a1));
        CHECK(filter.stable());
    }
    // And below the useful range, where both poles converge on z = 1 and a
    // section stops being distinguishable from a marginally unstable one.
    for (float cutoff : {0.0f, -5.0f, 1e-6f}) {
        Biquad low = design_lowpass(cutoff, kFs, 0.7071f);
        CHECK(std::isfinite(low.b0));
        CHECK(std::isfinite(low.a2));
        CHECK(low.stable());
    }
}

TEST(an_impulse_response_decays_to_nothing) {
    BiquadCascade<2> filter = design_butterworth_lowpass_4(90.0f, kFs);
    filter.reset();
    float peak = std::fabs(filter.process(1.0f));
    for (int n = 1; n < 200; ++n) peak = std::max(peak, std::fabs(filter.process(0.0f)));

    float tail = 0.0f;
    for (int n = 0; n < 2000; ++n) tail = std::max(tail, std::fabs(filter.process(0.0f)));
    CHECK(tail < peak * 1e-5f);
}

TEST(the_dc_block_removes_an_offset_and_keeps_the_signal) {
    DcBlock block = design_dc_block(pa::config::kDcBlockHz, kFs);

    // A 100 Hz signal riding on a large offset: the offset is what an
    // electrode half-cell potential looks like at a gain of 500.
    constexpr float kOffset = 8000.0f;
    float settled = 0.0f;
    for (int n = 0; n < 20000; ++n) {
        const float phase = 2.0f * kPi * 100.0f * static_cast<float>(n) / kFs;
        settled = block.process(kOffset + 1000.0f * std::sin(phase));
    }

    float low = 0.0f, high = 0.0f;
    for (int n = 20000; n < 24000; ++n) {
        const float phase = 2.0f * kPi * 100.0f * static_cast<float>(n) / kFs;
        const float y = block.process(kOffset + 1000.0f * std::sin(phase));
        low = std::min(low, y);
        high = std::max(high, y);
    }
    CHECK(std::isfinite(settled));
    // Centred on zero: the offset is gone.
    CHECK_NEAR((high + low) * 0.5f, 0.0f, 20.0f);
    // And the signal is still there at close to full amplitude.
    CHECK_NEAR((high - low) * 0.5f, 1000.0f, 30.0f);
}

TEST(the_dc_block_does_not_eat_the_low_end_of_the_semg_band) {
    DcBlock block = design_dc_block(pa::config::kDcBlockHz, kFs);
    // 20 Hz is the bottom of the band the front end is specified for. Four
    // times the corner of a one-pole high pass leaves it 3 percent down.
    const float gain = measured_gain(block, 20.0f, kFs);
    CHECK(gain > 0.95f);
    CHECK(gain < 1.02f);
}

TEST_MAIN()
