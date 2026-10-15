// Second-order IIR sections and the filters the front end is built from.
//
// Transposed direct form II throughout. On a Cortex-M7 with a single-precision
// FPU it is the same arithmetic cost as direct form I and needs half the
// state and its accumulation order keeps the rounding error small for the
// high-Q sections the mains notch needs.
//
// Coefficients follow the bilinear-transform designs in Robert Bristow
// Johnson's cookbook. They are computed on the host in the unit tests and on
// the target at start-up: at eight channels and four sections each this is a
// few hundred flops once, against the tens of kilobytes a coefficient table
// would cost in flash.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace pa::dsp {

// y[n] = b0 x[n] + s1, s1 = b1 x[n] - a1 y[n] + s2, s2 = b2 x[n] - a2 y[n].
// a0 is normalised to one at design time and never stored.
struct Biquad {
    float b0{1.0f}, b1{0.0f}, b2{0.0f};
    float a1{0.0f}, a2{0.0f};
    float s1{0.0f}, s2{0.0f};

    float process(float x) {
        const float y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        return y;
    }

    void reset() {
        s1 = 0.0f;
        s2 = 0.0f;
    }

    // A section is stable exactly when both poles are inside the unit circle,
    // which for real coefficients is this pair of inequalities. Checked at
    // start-up rather than assumed: a filter designed for a sample rate the
    // board is not actually running at can come out unstable and the first
    // symptom would be a limb moving.
    //
    // The margin is not decoration. As the cutoff falls towards DC both poles
    // walk towards z = 1 and the second inequality becomes an equality in
    // exact arithmetic; in single precision it crosses over and a section
    // that is only just inside the circle is one whose state grows for
    // thousands of samples before it decays. Requiring a margin rejects those
    // rather than shipping them.
    bool stable(float margin = 1e-6f) const {
        return std::fabs(a2) < 1.0f - margin && std::fabs(a1) < 1.0f + a2 - margin;
    }

    // Magnitude response at `hz`, evaluated on the unit circle. Used by the
    // tests and by the start-up self-check.
    float magnitude(float hz, float sample_rate_hz) const;
};

// A chain of sections, sized at compile time so nothing allocates.
template <std::size_t N>
struct BiquadCascade {
    Biquad sections[N]{};

    float process(float x) {
        for (std::size_t i = 0; i < N; ++i) x = sections[i].process(x);
        return x;
    }

    void reset() {
        for (std::size_t i = 0; i < N; ++i) sections[i].reset();
    }

    bool stable(float margin = 1e-6f) const {
        for (std::size_t i = 0; i < N; ++i)
            if (!sections[i].stable(margin)) return false;
        return true;
    }

    float magnitude(float hz, float sample_rate_hz) const {
        float gain = 1.0f;
        for (std::size_t i = 0; i < N; ++i) gain *= sections[i].magnitude(hz, sample_rate_hz);
        return gain;
    }

    static constexpr std::size_t size() { return N; }
};

Biquad design_lowpass(float cutoff_hz, float sample_rate_hz, float q);
Biquad design_highpass(float cutoff_hz, float sample_rate_hz, float q);
Biquad design_notch(float centre_hz, float sample_rate_hz, float q);

// Pole quality factor for section k of a Butterworth of order 2 * sections.
// The poles sit on a semicircle at angles pi (2k + 1) / (4 sections) and the
// Q of a conjugate pair at angle theta from the imaginary axis is
// 1 / (2 cos theta). Computed rather than pasted from a table, so changing the
// order is changing one number.
inline float butterworth_q(std::size_t k, std::size_t sections) {
    constexpr float kPi = 3.14159265358979323846f;
    const float angle = kPi * (2.0f * static_cast<float>(k) + 1.0f) /
                        (4.0f * static_cast<float>(sections));
    return 1.0f / (2.0f * std::cos(angle));
}

// Butterworth of order 2 * Sections, as a cascade of second-order sections.
// Maximally flat in the passband: no ripple, which matters here because ripple
// in the band the classifier reads is an amplitude bias that varies with
// frequency and it would look like a property of the muscle rather than of
// the filter.
template <std::size_t Sections>
BiquadCascade<Sections> design_butterworth_lowpass(float cutoff_hz, float sample_rate_hz) {
    BiquadCascade<Sections> cascade;
    for (std::size_t k = 0; k < Sections; ++k)
        cascade.sections[k] = design_lowpass(cutoff_hz, sample_rate_hz, butterworth_q(k, Sections));
    return cascade;
}

template <std::size_t Sections>
BiquadCascade<Sections> design_butterworth_highpass(float cutoff_hz, float sample_rate_hz) {
    BiquadCascade<Sections> cascade;
    for (std::size_t k = 0; k < Sections; ++k)
        cascade.sections[k] = design_highpass(cutoff_hz, sample_rate_hz, butterworth_q(k, Sections));
    return cascade;
}

inline BiquadCascade<2> design_butterworth_lowpass_4(float cutoff_hz, float sample_rate_hz) {
    return design_butterworth_lowpass<2>(cutoff_hz, sample_rate_hz);
}

inline BiquadCascade<2> design_butterworth_highpass_4(float cutoff_hz, float sample_rate_hz) {
    return design_butterworth_highpass<2>(cutoff_hz, sample_rate_hz);
}

// A one-pole high pass, used to strip the electrode half-cell potential before
// anything else sees it. One pole rather than two because the offset is a
// slowly varying DC term, not a band that needs rejecting and every extra
// order costs group delay inside the control loop.
struct DcBlock {
    float alpha{0.0f};
    float previous_input{0.0f};
    float previous_output{0.0f};

    float process(float x) {
        const float y = alpha * (previous_output + x - previous_input);
        previous_input = x;
        previous_output = y;
        return y;
    }

    void reset() {
        previous_input = 0.0f;
        previous_output = 0.0f;
    }
};

DcBlock design_dc_block(float cutoff_hz, float sample_rate_hz);

}  // namespace pa::dsp
