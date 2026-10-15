#include "pa/dsp/biquad.hpp"

namespace pa::dsp {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// Guard both ends of the useful range. Clamping is the right answer rather
// than an assertion: filters are designed at start-up on a device with no
// console and a bad configuration has to produce a working filter rather than
// a halt.
//
// Above Nyquist the bilinear transform maps the cutoff to infinity. Below
// about a two-thousandth of the sample rate both poles land on top of z = 1
// and single precision can no longer tell the section from a marginally
// unstable one, so the floor is relative to the rate rather than absolute.
float clamp_normalised(float hz, float sample_rate_hz) {
    const float nyquist = sample_rate_hz * 0.5f;
    const float floor_hz = sample_rate_hz * 5e-4f;
    if (!(hz > floor_hz)) return floor_hz;
    if (hz >= nyquist) return nyquist * 0.999f;
    return hz;
}

Biquad normalise(float b0, float b1, float b2, float a0, float a1, float a2) {
    Biquad f;
    f.b0 = b0 / a0;
    f.b1 = b1 / a0;
    f.b2 = b2 / a0;
    f.a1 = a1 / a0;
    f.a2 = a2 / a0;
    return f;
}

}  // namespace

float Biquad::magnitude(float hz, float sample_rate_hz) const {
    const float w = 2.0f * kPi * hz / sample_rate_hz;
    const float cos1 = std::cos(w), sin1 = std::sin(w);
    const float cos2 = std::cos(2.0f * w), sin2 = std::sin(2.0f * w);

    const float numerator_real = b0 + b1 * cos1 + b2 * cos2;
    const float numerator_imag = -(b1 * sin1 + b2 * sin2);
    const float denominator_real = 1.0f + a1 * cos1 + a2 * cos2;
    const float denominator_imag = -(a1 * sin1 + a2 * sin2);

    const float numerator = std::sqrt(numerator_real * numerator_real +
                                      numerator_imag * numerator_imag);
    const float denominator = std::sqrt(denominator_real * denominator_real +
                                        denominator_imag * denominator_imag);
    return denominator > 0.0f ? numerator / denominator : 0.0f;
}

Biquad design_lowpass(float cutoff_hz, float sample_rate_hz, float q) {
    const float w0 = 2.0f * kPi * clamp_normalised(cutoff_hz, sample_rate_hz) / sample_rate_hz;
    const float cos_w0 = std::cos(w0);
    const float alpha = std::sin(w0) / (2.0f * q);

    const float b1 = 1.0f - cos_w0;
    return normalise(b1 * 0.5f, b1, b1 * 0.5f, 1.0f + alpha, -2.0f * cos_w0, 1.0f - alpha);
}

Biquad design_highpass(float cutoff_hz, float sample_rate_hz, float q) {
    const float w0 = 2.0f * kPi * clamp_normalised(cutoff_hz, sample_rate_hz) / sample_rate_hz;
    const float cos_w0 = std::cos(w0);
    const float alpha = std::sin(w0) / (2.0f * q);

    const float b0 = (1.0f + cos_w0) * 0.5f;
    return normalise(b0, -(1.0f + cos_w0), b0, 1.0f + alpha, -2.0f * cos_w0, 1.0f - alpha);
}

Biquad design_notch(float centre_hz, float sample_rate_hz, float q) {
    const float w0 = 2.0f * kPi * clamp_normalised(centre_hz, sample_rate_hz) / sample_rate_hz;
    const float cos_w0 = std::cos(w0);
    const float alpha = std::sin(w0) / (2.0f * q);

    return normalise(1.0f, -2.0f * cos_w0, 1.0f, 1.0f + alpha, -2.0f * cos_w0, 1.0f - alpha);
}

DcBlock design_dc_block(float cutoff_hz, float sample_rate_hz) {
    DcBlock block;
    // Pole at exp(-2 pi fc / fs), the matched-z equivalent of a one-pole RC.
    // The difference from the bilinear form is negligible three decades below
    // Nyquist, which is where this filter always sits.
    block.alpha = std::exp(-2.0f * kPi * cutoff_hz / sample_rate_hz);
    return block;
}

}  // namespace pa::dsp
