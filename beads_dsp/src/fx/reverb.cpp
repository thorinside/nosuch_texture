#include "reverb.h"
#include "../util/cosine_table.h"
#include <algorithm>
#include <cmath>

namespace beads {

namespace {

inline size_t ScaleLen(size_t ref_len, float sr_scale) {
    float scaled = static_cast<float>(ref_len) * sr_scale;
    long rounded = std::lroundf(scaled);
    if (rounded < 1) rounded = 1;
    return static_cast<size_t>(rounded);
}

}  // namespace

void Reverb::Init(float* buffer, size_t buffer_size, float sample_rate) {
    sample_rate_ = sample_rate;

    amount_ = 0.0f;
    prev_amount_ = 0.0f;
    decay_ = 0.5f;
    diffusion_ = 0.7f;

    lfo_phase_ = 0.0f;
    lfo_increment_ = (sample_rate_ > 0.0f) ? (kLfoHz / sample_rate_) : 0.0f;

    feedback_l_ = 0.0f;
    feedback_r_ = 0.0f;

    dc_estimate_l_ = 0.0f;
    dc_estimate_r_ = 0.0f;
    dc_block_coeff_ = (sample_rate > 0.0f)
        ? (kTwoPi * kDcBlockTargetHz / sample_rate)
        : 0.0005f;

    // Sample-rate scale relative to the reference design (48 kHz).
    const float sr_scale = (sample_rate_ > 0.0f)
        ? (sample_rate_ / kReferenceSampleRate)
        : 1.0f;

    // Modulation depth and headroom scale with sample rate so the LFO
    // produces the same fractional pitch deviation in real time at any SR.
    mod_depth_samples_ = kModDepthRef * sr_scale;
    const size_t mod_headroom = static_cast<size_t>(
        std::ceil(static_cast<float>(kModHeadroomRef) * sr_scale));

    // Compute scaled delay lengths.
    const size_t ap_in_1_len = ScaleLen(kApIn1Len, sr_scale);
    const size_t ap_in_2_len = ScaleLen(kApIn2Len, sr_scale);
    const size_t ap_in_3_len = ScaleLen(kApIn3Len, sr_scale);
    const size_t ap_in_4_len = ScaleLen(kApIn4Len, sr_scale);

    delay_l1_len_ = ScaleLen(kDelayL1Len, sr_scale);
    const size_t ap_l1_len  = ScaleLen(kApL1Len,  sr_scale);
    const size_t ap_l2_len  = ScaleLen(kApL2Len,  sr_scale);
    const size_t delay_l2_len = ScaleLen(kDelayL2Len, sr_scale);

    delay_r1_len_ = ScaleLen(kDelayR1Len, sr_scale);
    const size_t ap_r1_len  = ScaleLen(kApR1Len,  sr_scale);
    const size_t ap_r2_len  = ScaleLen(kApR2Len,  sr_scale);
    const size_t delay_r2_len = ScaleLen(kDelayR2Len, sr_scale);

    // Partition the shared buffer into 12 individual delay lines.  The two
    // modulated delay lines (delay_l1_, delay_r1_) need extra headroom for
    // LFO variation past the base length.
    initialized_ = true;
    float* ptr = buffer;
    size_t used = 0;

    auto alloc = [&](DelayLine& dl, size_t len) {
        if (buffer && used + len <= buffer_size) {
            dl.Init(ptr, len);
            ptr += len;
            used += len;
        } else {
            dl.Init(nullptr, 0);
            initialized_ = false;
        }
    };

    // Input diffusion allpasses
    alloc(ap_in_1_, ap_in_1_len);
    alloc(ap_in_2_, ap_in_2_len);
    alloc(ap_in_3_, ap_in_3_len);
    alloc(ap_in_4_, ap_in_4_len);

    // Left tank
    alloc(delay_l1_, delay_l1_len_ + mod_headroom);
    alloc(ap_l1_,    ap_l1_len);
    alloc(ap_l2_,    ap_l2_len);
    alloc(delay_l2_, delay_l2_len);

    // Right tank
    alloc(delay_r1_, delay_r1_len_ + mod_headroom);
    alloc(ap_r1_,    ap_r1_len);
    alloc(ap_r2_,    ap_r2_len);
    alloc(delay_r2_, delay_r2_len);

    // Butterworth LP biquads in the feedback path.  Default cutoff matches
    // the prior 0.7 alpha @ 48 kHz one-pole (~9.2 kHz) — call SetLpCutoffHz
    // to override per quality mode.
    lp_l_.Init();
    lp_r_.Init();
    lp_l_.SetQ(0.7071068f);
    lp_r_.SetQ(0.7071068f);
    lp_l_.SetFrequencyHz(9000.0f, sample_rate_);
    lp_r_.SetFrequencyHz(9000.0f, sample_rate_);
}

void Reverb::SetAmount(float amount) {
    amount_ = Clamp(amount, 0.0f, 1.0f);
}

void Reverb::SetDecay(float decay) {
    decay_ = Clamp(decay, 0.0f, 1.0f);
}

void Reverb::SetDiffusion(float diff) {
    diffusion_ = Clamp(diff, 0.0f, 1.0f);
}

void Reverb::SetLpCutoffHz(float hz) {
    // Clamp to a sensible range; SVF clamps internally too but be explicit.
    float clamped = Clamp(hz, 100.0f, 0.45f * sample_rate_);
    lp_l_.SetFrequencyHz(clamped, sample_rate_);
    lp_r_.SetFrequencyHz(clamped, sample_rate_);
}

void Reverb::ResetState() {
    feedback_l_ = 0.0f;
    feedback_r_ = 0.0f;
    dc_estimate_l_ = 0.0f;
    dc_estimate_r_ = 0.0f;
    lp_l_.Reset();
    lp_r_.Reset();
    ap_in_1_.Reset();
    ap_in_2_.Reset();
    ap_in_3_.Reset();
    ap_in_4_.Reset();
    delay_l1_.Reset();
    ap_l1_.Reset();
    ap_l2_.Reset();
    delay_l2_.Reset();
    delay_r1_.Reset();
    ap_r1_.Reset();
    ap_r2_.Reset();
    delay_r2_.Reset();
}

void Reverb::Process(float left_in, float right_in,
                     float* left_out, float* right_out) {
    // Fast path: reverb fully off.  On the >0 → 0 edge, scrub state so a
    // later re-engage doesn't dump a stale tank into the wet path.
    if (amount_ <= 0.0f) {
        if (prev_amount_ > 0.0f) {
            ResetState();
        }
        prev_amount_ = amount_;
        *left_out = left_in;
        *right_out = right_in;
        return;
    }
    prev_amount_ = amount_;

    // Bypass if allocation failed (insufficient buffer).  Without this guard
    // a null-buffer DelayLine would dereference inside ProcessAllpass.
    if (!initialized_) {
        *left_out = left_in;
        *right_out = right_in;
        return;
    }

    // Map decay knob (0-1) to feedback gain.
    // Non-linear: slow ramp then steep near 1.0 for long tails.
    float fb = 0.2f + decay_ * 0.75f;
    if (decay_ > 0.9f) {
        fb += (decay_ - 0.9f) * 0.5f;
    }
    fb = Clamp(fb, 0.0f, 0.9995f);

    // Diffusion coefficient for allpass stages
    float ap_coeff = diffusion_ * 0.75f;

    // ------------------------------------------------------------------
    // LFO for modulated delay taps (chorus-like effect in tank)
    // ------------------------------------------------------------------
    lfo_phase_ += lfo_increment_;
    if (lfo_phase_ >= 1.0f) lfo_phase_ -= 1.0f;

    float lfo_cos = CosLookup(lfo_phase_);
    float lfo_sin = CosLookup(lfo_phase_ - 0.25f);  // sin = cos(phase - pi/2)
    float mod_l = mod_depth_samples_ * lfo_sin;
    float mod_r = mod_depth_samples_ * lfo_cos;

    // ------------------------------------------------------------------
    // Input diffusion: 4 series allpass filters
    // Each has its own delay line, producing proper diffusion.
    // ------------------------------------------------------------------
    float input = (left_in + right_in) * 0.5f;

    float diffused = ProcessAllpass(ap_in_1_, input, ap_coeff);
    diffused = ProcessAllpass(ap_in_2_, diffused, ap_coeff);
    diffused = ProcessAllpass(ap_in_3_, diffused, ap_coeff);
    diffused = ProcessAllpass(ap_in_4_, diffused, ap_coeff);

    // ------------------------------------------------------------------
    // Tank: two cross-coupled feedback paths (Dattorro topology)
    //
    // Each path: modulated delay → allpass → allpass → LP → delay
    // Cross-coupling: left feedback feeds right input, and vice versa.
    // feedback_l_ and feedback_r_ carry values from the previous sample.
    // ------------------------------------------------------------------

    // Save previous-sample feedback for cross-coupling.
    // Both paths must use values from the PREVIOUS sample to maintain
    // the symmetric Dattorro figure-8 topology.
    float prev_fb_l = feedback_l_;
    float prev_fb_r = feedback_r_;

    // LEFT PATH --------------------------------------------------------
    float tank_l_in = diffused + fb * prev_fb_r;

    // Modulated delay L1: write input, read with modulation around base length
    delay_l1_.Write(tank_l_in);
    float dl1_out = delay_l1_.ReadInterpolated(
        static_cast<float>(delay_l1_len_) + mod_l);
    delay_l1_.Advance();

    // Two allpass filters in left tank
    float al1_out = ProcessAllpass(ap_l1_, dl1_out, ap_coeff);
    float al2_out = ProcessAllpass(ap_l2_, al1_out, ap_coeff);

    // Butterworth biquad LP in feedback path (frequency-dependent decay).
    float lp_out_l = lp_l_.ProcessLP(al2_out);

    // DC blocker: subtract slowly-tracked DC estimate to prevent
    // low-frequency buildup at high decay settings.
    ONE_POLE(dc_estimate_l_, lp_out_l, dc_block_coeff_);
    float tank_l_dc_blocked = lp_out_l - dc_estimate_l_;

    // Feedback delay L2
    float dl2_out = delay_l2_.ReadOldest();
    delay_l2_.Write(tank_l_dc_blocked);
    delay_l2_.Advance();

    // Safety: prevent unbounded energy accumulation
    feedback_l_ = SoftClip(dl2_out);

    // RIGHT PATH -------------------------------------------------------
    float tank_r_in = diffused + fb * prev_fb_l;

    // Modulated delay R1
    delay_r1_.Write(tank_r_in);
    float dr1_out = delay_r1_.ReadInterpolated(
        static_cast<float>(delay_r1_len_) + mod_r);
    delay_r1_.Advance();

    // Two allpass filters in right tank
    float ar1_out = ProcessAllpass(ap_r1_, dr1_out, ap_coeff);
    float ar2_out = ProcessAllpass(ap_r2_, ar1_out, ap_coeff);

    // Butterworth biquad LP
    float lp_out_r = lp_r_.ProcessLP(ar2_out);

    // DC blocker
    ONE_POLE(dc_estimate_r_, lp_out_r, dc_block_coeff_);
    float tank_r_dc_blocked = lp_out_r - dc_estimate_r_;

    // Feedback delay R2
    float dr2_out = delay_r2_.ReadOldest();
    delay_r2_.Write(tank_r_dc_blocked);
    delay_r2_.Advance();

    feedback_r_ = SoftClip(dr2_out);

    // ------------------------------------------------------------------
    // Output: tap from multiple points for decorrelated stereo
    // ------------------------------------------------------------------
    float wet_l = dl1_out * 0.6f + dr2_out * 0.4f;
    float wet_r = dr1_out * 0.6f + dl2_out * 0.4f;

    *left_out  = left_in  + amount_ * (wet_l - left_in);
    *right_out = right_in + amount_ * (wet_r - right_in);
}

} // namespace beads
