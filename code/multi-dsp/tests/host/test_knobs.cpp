// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Knob + hardware-constant tests.
//   1. Pim's WAVE SHAPE (P3)      skews the continuous shapes, leaves GATE alone, centre = unchanged
//   2. Ringo MODEL MORPH (P5)     0 = selected model, 1 = next model, bounded in between
//   3. Tiddly SMOOTHNESS (P3)      slews the envelope / CV out 1 monotonically
//   4. Hardware constants         SAMPLE_RATE matches i2s.pio, PCM3060 de-emphasis is off, scope labels match the rate
#include "host_common.inc"
#define main fw_main
#include "../../main.cpp"
#undef main
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("   FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void set_params(const float* p) {
    for (int i = 0; i < 7; i++) { g_param[i].store(p[i]); sp[i] = sp_tgt[i] = p[i]; sp_step[i] = 0; }
}
static void run_silence(int chunks) {
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    for (int c = 0; c < chunks; c++) { fake_us += 100; process_audio_chunk(in, out); }
}

// ---------------------------------------------------------------- 1. Pim's WAVE SHAPE
// CV out 0 is written once per chunk. Fraction of the cycle spent RISING == the skew point for a triangle.
static std::vector<int> pims_trace(int shape, float wave_knob, int chunks) {
    for (int c = 0; c < NUM_CV_OUTPUTS; c++) { cv_channels[c].mult_div = 0; cv_channels[c].phase = 0; cv_channels[c].current_val = 0; }
    cv_channels[0].mult_div = 1; cv_channels[0].shape = shape;
    for (int c = 0; c < 4; c++) pims_slew_state[c] = 0;
    master_bpm.store(250);                        // 250 BPM -> ~4.2 cycles/s -> ~44 chunks per cycle
    float p[7] = {0.5f, 0.5f, wave_knob, 0.0f, 0.0f, 0.0f, 0.5f};   // no slew, no phase offset, no chop
    set_params(p);
    active_mode.store(MODE_PIMS);
    std::vector<int> v;
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    for (int c = 0; c < chunks; c++) { fake_us += 100; process_audio_chunk(in, out); v.push_back(g_dac_out[0].load()); }
    return v;
}
static float rising_fraction(const std::vector<int>& v) {
    int up = 0, down = 0;
    for (size_t i = 100; i + 1 < v.size(); i++) { int d = v[i + 1] - v[i]; if (d > 20) up++; else if (d < -20) down++; }
    return (up + down) ? (float)up / (float)(up + down) : -1.0f;
}
// Peak position of a sine trace: chunks from an upward mid-level crossing to the maximum, divided by the period.
static float peak_position(const std::vector<int>& v) {
    std::vector<int> up;
    for (size_t i = 100; i + 1 < v.size(); i++) if (v[i] < 2048 && v[i + 1] >= 2048) up.push_back((int)i);
    if (up.size() < 3) return -1.0f;
    int a = up[1], b = up[2], best = a;
    for (int i = a; i < b; i++) if (v[i] > v[best]) best = i;
    return (float)(best - a) / (float)(b - a);
}
static float mean_level(const std::vector<int>& v) {
    double s = 0; int n = 0; for (size_t i = 100; i < v.size(); i++) { s += v[i]; n++; }
    return (float)(s / n / 4095.0);
}
static void test_pims() {
    printf("1) Pim's WAVE SHAPE (P3) - fraction of a triangle cycle spent rising:\n");
    float fr[5]; const float ks[5] = {0.0f, 0.2f, 0.5f, 0.8f, 1.0f};
    for (int i = 0; i < 5; i++) { fr[i] = rising_fraction(pims_trace(1, ks[i], 1400)); printf("   knob %.1f -> rising %.2f\n", ks[i], fr[i]); }
    CHECK(fabsf(fr[2] - 0.5f) < 0.06f, "centre must be a symmetric triangle (got %.2f)", fr[2]);
    CHECK(fr[0] < 0.15f && fr[1] < fr[2] && fr[3] > fr[2] && fr[4] > 0.85f, "skew must grow monotonically from fast-rise to slow-rise");
    // SINE: still half the cycle rising, but the PEAK moves (expected at s/2 of the cycle). SAW: the mean level moves.
    float pk_lo = peak_position(pims_trace(2, 0.1f, 1400)), pk_hi = peak_position(pims_trace(2, 0.9f, 1400));
    printf("   sine peak position (fraction of cycle after the rising mid-crossing): knob 0.1 -> %.2f, knob 0.9 -> %.2f\n", pk_lo, pk_hi);
    CHECK(pk_lo >= 0 && pk_hi > pk_lo + 0.25f, "sine peak should move later as the knob goes up");
    float m_lo = mean_level(pims_trace(5, 0.1f, 1400)), m_hi = mean_level(pims_trace(5, 0.9f, 1400)), m_mid = mean_level(pims_trace(5, 0.5f, 1400));
    printf("   saw mean level: knob 0.1 -> %.2f, 0.5 -> %.2f, 0.9 -> %.2f\n", m_lo, m_mid, m_hi);
    CHECK(m_lo > m_mid + 0.1f && m_mid > m_hi + 0.1f && fabsf(m_mid - 0.5f) < 0.06f, "saw mean must fall as the knob rises (0.69 / 0.5 / 0.31 expected)");
    auto g_lo = pims_trace(0, 0.1f, 600), g_hi = pims_trace(0, 0.9f, 600);
    CHECK(g_lo == g_hi, "GATE timing must not depend on WAVE SHAPE");
    // Output stays inside the DAC range for every shape / knob combination
    int bad = 0;
    for (int shp = 0; shp < 10; shp++) for (float k : {0.0f, 0.3f, 0.5f, 0.7f, 1.0f})
        for (int v : pims_trace(shp, k, 300)) if (v < 0 || v > 4095) bad++;
    CHECK(bad == 0, "%d DAC values out of range", bad);
    cv_channels[0] = {1, 0, 0, 0}; cv_channels[1] = {2, 1, 0, 0}; cv_channels[2] = {4, 2, 0, 0}; cv_channels[3] = {-4, 7, 0, 0};
}

// ---------------------------------------------------------------- 2. Ringo MODEL MORPH
static std::vector<float> ringo_strike(int model, float morph, bool with_input, unsigned* clips, int chunks = 200) {
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    for (int m = 0; m < MAX_MODES; m++) ringo_modes[m] = {0, 0};
    ringo_fb = 0; ringo_env = 0; dc_offset = 0; ringo_ratio_init = false; rng_state = 0x1234ABCDu; fm_phase = 0;
    float p[7] = {0.4f, 0.5f, 0.7f, 0.5f, morph, 0.5f, 0.9f};
    set_params(p);
    ringo_model_idx.store(model); is_frozen.store(false); active_mode.store(MODE_RINGO);
    g_clip_count.store(0);
    std::vector<float> y; long t = 0;
    for (int c = 0; c < chunks; c++) {
        for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
            float x = with_input ? 0.3f * (2.0f * fmodf(220.0f * t / (float)SAMPLE_RATE, 1.0f) - 1.0f) : 0.0f;
            in[i] = in[i + 1] = (int32_t)(x * 2147483000.0f); t++;
        }
        if (c == 2) trigger_strike.store(true);
        fake_us += 100; process_audio_chunk(in, out);
        for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) y.push_back(out[i] / 2147483648.0f);
    }
    if (clips) *clips = g_clip_count.load();
    return y;
}
static double rel_diff(const std::vector<float>& a, const std::vector<float>& b) {
    double d = 0, e = 0; size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; i++) { d += (a[i] - b[i]) * (a[i] - b[i]); e += a[i] * a[i]; }
    return e > 0 ? sqrt(d / e) : 1e9;
}
static void test_ringo_morph() {
    printf("\n2) Ringo MODEL MORPH (P5):\n");
    for (int model = 0; model < 6; model++) {
        auto a0 = ringo_strike(model, 0.0f, false, nullptr);
        auto a1 = ringo_strike(model, 1.0f, false, nullptr);
        auto bn = ringo_strike((model + 1) % 6, 0.0f, false, nullptr);
        double d_knob = rel_diff(a0, a1), d_next = rel_diff(a1, bn);
        printf("   model %d: |morph0 - morph1| = %.2f   |morph1 - model+1| = %.4f\n", model, d_knob, d_next);
        CHECK(d_knob > 0.1, "model %d: MODEL MORPH has no audible effect (%.3f)", model, d_knob);
        CHECK(d_next < 0.05, "model %d: morph=1 should equal the next model (%.4f)", model, d_next);
    }
    // bounded across the whole range, with and without an input signal and heavy feedback
    int overs = 0, runs = 0;
    for (int model = 0; model < 6; model++) for (float mo : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) for (int inp = 0; inp < 2; inp++) {
        unsigned clips = 0; auto y = ringo_strike(model, mo, inp, &clips, 300); runs++;
        float pk = 0; for (float v : y) pk = std::max(pk, fabsf(v));
        if (clips || !(pk < 1.0f)) { overs++; printf("   over: model %d morph %.2f input %d clips %u peak %.2f\n", model, mo, inp, clips, pk); }
    }
    printf("   %d runs, %d with overs\n", runs, overs);
    CHECK(overs == 0, "%d morph runs exceeded full scale before the limiter", overs);
}

// ---------------------------------------------------------------- 3. Tiddly SMOOTHNESS
static float tiddly_p2p(int mode_sel, float smooth) {
    tiddly_env_s = 0; tiddly_phase = 0;
    tiddly_shape_mode.store(mode_sel);
    float p[7] = {0.5f, 0.5f, smooth, 0.0f, 0.0f, 0.0f, 0.5f};
    set_params(p);
    active_mode.store(MODE_TIDDLY);
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    int lo = 4095, hi = 0;
    for (int c = 0; c < 400; c++) { fake_us += 100; process_audio_chunk(in, out); if (c > 100) { int v = g_dac_out[0].load(); lo = std::min(lo, v); hi = std::max(hi, v); } }
    return (hi - lo) / 4095.0f;
}
static void test_tiddly() {
    printf("\n3) Tiddly SMOOTHNESS (P3) - peak-to-peak of CV out 1 (stepped mode):\n");
    float a = tiddly_p2p(2, 0.0f), b = tiddly_p2p(2, 0.5f), c = tiddly_p2p(2, 1.0f);
    printf("   smoothness 0.0 -> %.2f   0.5 -> %.2f   1.0 -> %.2f\n", a, b, c);
    CHECK(a > b && b > c, "CV swing must shrink monotonically as smoothness rises");
    CHECK(c < 0.6f * a, "smoothness=1 should visibly slew the CV (%.2f vs %.2f)", c, a);
    float ad = tiddly_p2p(0, 0.0f), ad1 = tiddly_p2p(0, 1.0f);
    printf("   AD mode: 0.0 -> %.2f   1.0 -> %.2f\n", ad, ad1);
    CHECK(ad1 < ad, "AD envelope should also be slewed");
    // audio stays bounded
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    g_clip_count.store(0);
    for (float sm : {0.0f, 0.5f, 1.0f}) for (float fold : {0.0f, 1.0f}) {
        float p[7] = {0.8f, 0.5f, sm, fold, 1.0f, 0.5f, 1.0f}; set_params(p); active_mode.store(MODE_TIDDLY);
        for (int c = 0; c < 200; c++) { for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) in[i] = in[i + 1] = (int32_t)(0.4f * 2147483000.0f * sinf(0.05f * (c * 256 + i))); fake_us += 100; process_audio_chunk(in, out); }
    }
    CHECK(g_clip_count.load() == 0, "Tiddly clipped before the limiter (%u)", g_clip_count.load());
}

// ---------------------------------------------------------------- 4. Hardware constants
static std::string slurp(const char* path) { std::ifstream f(path); std::stringstream s; s << f.rdbuf(); return s.str(); }
static void test_hw() {
    printf("\n4) Hardware constants:\n");
    // 4a. SAMPLE_RATE must equal what i2s.pio generates: sysclk / clkdiv / (2 PIO cycles per bit * 32 bits * 2 channels)
    std::string pio = slurp("../../i2s.pio");
    if (pio.empty()) { printf("   SKIP i2s.pio not found\n"); }
    else {
        std::vector<double> divs; size_t pos = 0;
        while ((pos = pio.find("sm_config_set_clkdiv(&c,", pos)) != std::string::npos) { divs.push_back(atof(pio.c_str() + pos + 24)); pos += 10; }
        CHECK(divs.size() == 2 && divs[0] == divs[1], "expected the same clkdiv for TX and RX");
        if (!divs.empty()) {
            double fs = 144e6 / divs[0] / 128.0;
            printf("   i2s.pio: clkdiv %.4f -> fs = %.1f Hz   (SAMPLE_RATE = %d)\n", divs[0], fs, SAMPLE_RATE);
            CHECK(fabs(fs - SAMPLE_RATE) < 0.5, "SAMPLE_RATE does not match i2s.pio");
            printf("   MCLK 24 MHz / fs = %.2f (PCM3060 needs 256/384/512/768)\n", 24e6 / fs);
            CHECK(fabs(24e6 / fs - 512.0) < 0.01, "MCLK/fs must be exactly 512");
        }
    }
    // 4b. PCM3060 register 0x45: bit 4 = DMC (de-emphasis). Must be off.
    std::string src = slurp("../../main.cpp");
    size_t k = src.find("{0x45, 0x");
    CHECK(k != std::string::npos, "codec_init has no register 0x45 write");
    if (k != std::string::npos) {
        int v = (int)strtol(src.c_str() + k + 9, nullptr, 16);
        printf("   codec reg 0x45 = 0x%02X  (de-emphasis %s)\n", v, (v & 0x10) ? "ON" : "off");
        CHECK(!(v & 0x10), "PCM3060 de-emphasis is switched on");
    }
    // 4b2. DAC attenuation registers 0x41 / 0x42 must be 0 dB (0xFF)
    for (const char* reg : {"{0x41, 0x", "{0x42, 0x"}) {
        size_t q = src.find(reg);
        CHECK(q != std::string::npos, "codec_init has no write for %s", reg);
        if (q != std::string::npos) { int v = (int)strtol(src.c_str() + q + 9, nullptr, 16); printf("   codec reg %.5s = 0x%02X  (%.1f dB)\n", reg + 1, v, 0.5 * (v - 255)); CHECK(v == 0xFF, "DAC attenuation is not 0 dB (0x%02X)", v); }
    }
    // 4c. Scope timebase labels agree with the true rate
    for (int i = 0; i < NUM_TIMEBASES; i++) {
        double ms = tb_stride[i] * 256000.0 / SAMPLE_RATE;
        double lab = atof(tb_names[i]);
        printf("   timebase %d: stride %2d = %6.2f ms  label %s\n", i, tb_stride[i], ms, tb_names[i]);
        CHECK(fabs(ms - lab) < 0.6, "label %s vs %.2f ms", tb_names[i], ms);
    }
}

int main() {
    init_hann_table(); ui_init_tables();
    test_pims();
    test_ringo_morph();
    test_tiddly();
    test_hw();
    printf(g_fail ? "\nRESULT: FAIL (%d)\n" : "\nRESULT: PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
