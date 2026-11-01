// SPDX-License-Identifier: GPL-3.0-or-later
// MULTI-DSP firmware for the pico2-eurorack development board
// Copyright (C) 2026 Louis Brooks
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// ============================================================================
//  RP2350 MULTI-DSP  -  revision 3.3 (audio-quality pass + UI overhaul + hardware-clock and CV-scaling fixes)
//
//  What changed vs. the previous revision (short version):
//   * Full 24-bit audio path (no more & 0xFFFF0000 truncation in or out)
//   * Transparent output limiter instead of the 1.5x-gain cubic soft_clip
//   * Per-SAMPLE parameter ramps (was: one step per 256-sample chunk = zipper)
//   * Filtered/oversampled pot + CV reads, continuous CV dead-zone
//   * PolyBLEP oscillators (Plaids / Brains), ADAA wavefolder (Wraps),
//     pitch-tracked FM index to tame aliasing
//   * Clods: grain scheduling moved into the audio engine (no cross-core
//     race), interpolated grain reads, Hann table, bounded grain length,
//     smoothed grain-count normalisation, glided loop length
//   * Ringo: proper per-mode decay times, level normalisation, working
//     feedback, correct magic-circle tuning
//   * Tiddly fold bug, Plaids chord double-increment, Mumbles pitch scaling,
//     Pims wrap detection, Brains trigger flag, FX pluck button all fixed
//   * Hermite-interpolated delay / chorus, 8-comb + 4-allpass reverb
//   * Quantizer now quantises CV (not audio amplitude) and drives a voice
//   * NaN guard on the output, xorshift RNG in the audio path,
//     bigger core-1 stack, non-blocking button debounce,
//     peak CPU + clip counter on the diagnostics page
//  Rev 3.1 UI / diagnostics:
//   * CPU readout: windowed average + worst case (was one random chunk), xrun counter
//   * Views: PARAMETERS / SCOPE / SPECTRUM / SCOPE+SPECTRUM  (hold encoder to cycle)
//   * Menu: module list + SETTINGS + SYSTEM INFO, toasts, parameter overlay, screen saver
//   * Interrupt-driven encoder (no more lost detents), long-press, page-diff OLED updates
//   * Crossfade when switching modules (no pop)
//   * Ringo: input AGC + level-consistent strike (tonal input no longer slams the limiter)
//   * Quirks: bounded gain staging, DC removal, anti-aliased |x|, exponential S&H rate, LPF direction fixed
//  Rev 3.2 hardware / knobs:
//   * SAMPLE_RATE 48000 -> 46875: the rate i2s.pio really produces (pitch was ~41 cents flat, delays/times 2.4% long)
//   * PCM3060 register 0x45 was 0xD7 (de-emphasis ON at 32 kHz = treble cut); now 0x00 (off, sharp filter)
//   * The three dead knobs now work: Pims WAVE SHAPE (skew), Ringo MODEL MORPH (hybrid of model and next model),
//     Tiddly SMOOTHNESS (envelope/CV slew). Ringo mode ratios now ramp per sample (no steps on STRUCTURE/MORPH).
//   * Scope timebase labels follow the true rate (5/11/22/33/49/66/87/131 ms)
//   * PCM3060 DAC attenuation 0xF3 (-6 dB) -> 0xFF (0 dB): the output is 6 dB louder than before
//   * Encoder: own cycle-position decoder (quarter-step total, one count per full cycle), replaces the table-driven one
//  Rev 3.3 CV scaling (matches the schematic of LB-SYNTH-PROTO-A v0.2, see hardware/README.md):
//   * CV inputs are inverting stages: cv_from_counts() now flips the sign (+5 V at the jack = +1) and clamps to +/-1
//   * CV Out C and D are inverting with a +5 V offset: dac_hw_code() maps the logical 0..4095 written by the modules
//     to 0 ... +4.9 V on those two channels (A and B unchanged, 0 ... ~8 V); cvout_cd_code_from_volts() allows bipolar use
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <atomic>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "i2s.pio.h"

// ========================================================
// 1. GLOBAL SYSTEM & HARDWARE DEFINITIONS
// ========================================================
// TRUE sample rate. It is set by i2s.pio, not by this constant:
//   PIO clock = 144 MHz / clkdiv 24 = 6 MHz;  one stereo frame = 128 PIO cycles  ->  6 MHz / 128 = 46,875 Hz.
//   (BCLK = 3 MHz = 64 x fs, MCLK 24 MHz = 512 x fs: a valid, synchronous PCM3060 clock ratio.)
// Every oscillator, filter and delay is scaled from this value, so it must match the hardware.
// If you change clkdiv in i2s.pio, the MCLK divider, or the system clock, change this too.
#define SAMPLE_RATE 46875
#define CHUNK_SAMPLES 256
#define CHUNK_32BIT_WORDS (CHUNK_SAMPLES * 2)
#define CHUNK_DEADLINE_US ((float)CHUNK_SAMPLES * 1000000.0f / (float)SAMPLE_RATE)
#define TWO_PI_F 6.28318530718f

#define OLED_I2C_HZ 400000   // try 1000000 if your display tolerates it (frees core 1 time)

#define MCP4728_ADDR 0x60
#define OLED_ADDR 0x3C

#define MCP3208_SPI spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#define PIN_BTN_SELECT 0
#define PIN_BTN_EDIT   1
#define PIN_ENC_SW     20
#define PIN_ENC_B      21
#define PIN_ENC_A      22

enum ModuleMode {
    MODE_PIMS = 0,
    MODE_RINGO = 1,
    MODE_CLODS = 2,
    MODE_QUANTIZER = 3,
    MODE_QUIRKS = 4,
    MODE_PLAIDS = 5,
    MODE_BRAINS = 6,
    MODE_MUMBLES = 7,
    MODE_WRAPS = 8,
    MODE_TIDDLY = 9,
    MODE_FX = 10,
    NUM_MODES = 11
};

std::atomic<int> active_mode{MODE_CLODS};

// Real-time diagnostics metrics
std::atomic<float> g_cpu_load_pct{0.0f};
std::atomic<float> g_cpu_peak_pct{0.0f};
std::atomic<uint32_t> g_clip_count{0};
uint32_t g_sram_static_bytes = 0;

// ---- Audio health + visualisation data (written by the audio ISR, read by the UI core) ----
//  g_cpu_load_pct : average load over the last CPU_WINDOW_CHUNKS chunks (~340 ms)
//  g_cpu_peak_pct : worst single chunk inside that same window (UI adds its own hold)
//  g_xrun_count   : chunks that used >97% of their time budget (a glitch is imminent/audible)
//  g_out_peak/rms : output level (linear, 1.0 = full scale)
//  g_scope_ring   : output, mono, decimated 2:1 (23.4 kHz) - feeds scope + spectrum
#define CPU_WINDOW_CHUNKS 64
#define SCOPE_RING_LEN    4096                  // power of two; 175 ms @ 23.4 kHz (longest scope timebase needs ~135 ms)
#define SCOPE_FS          (SAMPLE_RATE / 2.0f)
std::atomic<uint32_t> g_xrun_count{0};
std::atomic<float>    g_out_peak{0.0f};
std::atomic<float>    g_out_rms{0.0f};
static int16_t               g_scope_ring[SCOPE_RING_LEN];
static std::atomic<uint32_t> g_scope_wr{0};

const char* mode_titles[NUM_MODES] = {
    "1. PIM'S CV", "2. RINGO", "3. CLODS", "4. QUANTIZER", "5. QUIRKS", "6. PLAIDS",
    "7. BRAINS", "8. MUMBLES", "9. WRAPS", "10. TIDDLY", "11. MULTI-FX"
};

const char* mode_short_names[NUM_MODES] = {
    "PIM'S", "RINGO", "CLODS", "QUANT", "QUIRKS",
    "PLAIDS", "BRAINS", "MUMBLES", "WRAPS", "TIDDLY", "MULTI-FX"
};

const char* mode_param_names[NUM_MODES][7] = {
    {"GATE WIDTH",  "EUCLID STEP", "WAVE SHAPE",  "PHASE OFFSET", "SLEW SMOOTH", "AUDIO CHOP",  "BPM / TEMPO"},
    {"FREQUENCY",   "STRUCTURE",   "BRIGHTNESS",  "POSITION",     "MODEL MORPH", "FEEDBACK",    "WET/DRY MIX"},
    {"POSITION",    "GRAIN SIZE",  "PITCH",       "DENSITY",      "TEXTURE",     "FEEDBACK",    "WET/DRY MIX"},
    {"TRANSPOSE",   "CHORD SPREAD","OCTAVE RNG",  "GLIDE SLEW",   "OSC DETUNE",  "FM TIMBRE",   "WET/DRY MIX"},
    {"S&H RATE",    "DRIVE",       "BLEND",       "NOISE LEVEL",  "LPF CUTOFF",  "THRESHOLD",   "WET/DRY MIX"},
    {"PITCH",       "HARMONICS",   "TIMBRE",      "MORPH",        "FM AMOUNT",   "LPG DECAY",   "OUTPUT LVL"},
    {"PITCH",       "TIMBRE",      "COLOR",       "MOD DEPTH",    "MOD RATE",    "DRIVE CRUSH", "OUTPUT LVL"},
    {"CLOCK RATE",  "V-SPREAD",    "DEJA VU",     "ROOT BIAS",    "QUANT SCALE", "GATE DECAY",  "OUTPUT LVL"},
    {"CARRIER FRQ", "FOLD DEPTH",  "MOD GAIN",    "TIMBRE",       "INT/EXT CAR", "FEEDBACK",    "WET/DRY MIX"},
    {"FREQ RATE",   "SLOPE SHAPE", "SMOOTHNESS",  "WAVE FOLD",    "FM DEPTH",    "AUDIO MOD",   "OUTPUT LVL"},
    {"CHORUS DPTH", "CHORUS RATE", "ECHO TIME",   "ECHO FDBK",    "REVERB DECY", "REVERB DAMP", "WET/DRY MIX"}
};

std::atomic<float> g_param[7]{0.5f, 0.3f, 0.5f, 0.5f, 0.5f, 0.3f, 0.5f};
std::atomic<float> g_cv[4]{0.0f, 0.0f, 0.0f, 0.0f};

std::atomic<uint16_t> g_dac_out[4]{2048, 2048, 2048, 2048};
std::atomic<bool> g_dac_dirty{false};

static float dc_offset = 0.0f;


// --- MODE 0: PIM'S PRO STATE ---
#define NUM_CV_OUTPUTS 4
struct ModChannel {
    int mult_div;
    int shape;
    float phase;
    float current_val;
};
ModChannel cv_channels[NUM_CV_OUTPUTS] = {
    { 1, 0, 0.0f, 0.0f },
    { 2, 1, 0.0f, 0.0f },
    { 4, 2, 0.0f, 0.0f },
    { -4, 7, 0.0f, 0.0f }
};
std::atomic<int> selected_cv_out{0};
std::atomic<int> master_bpm{120};
const char* shape_names[] = {
    "01. GATE / TRIG", "02. TRIANGLE LFO", "03. SINE LFO",
    "04. EXP ENV",     "05. LOG DECAY",    "06. SAW RAMP",
    "07. TRAPEZOID",   "08. RANDOM S&H",   "09. RANDOM WANDER",
    "10. EUCLIDEAN"
};

// --- MODE 1: RINGO STATE ---
#define MAX_MODES 4
struct ResonatorMode { float state1; float state2; };
ResonatorMode ringo_modes[MAX_MODES] = {{0,0}, {0,0}, {0,0}, {0,0}};
std::atomic<int> ringo_model_idx{0};
std::atomic<bool> trigger_strike{false};
std::atomic<bool> is_frozen{false};
const char* ringo_model_names[6] = {
    "MODAL STRINGS", "SYMPATHETIC", "RESONANT TUBE",
    "INHARMONIC BELL", "FM KARPLUS", "STRING CHORD"
};

// --- MODE 2: CLODS STATE ---
#define BUFFER_SECONDS 2
#define MAX_BUFFER_LEN (SAMPLE_RATE * BUFFER_SECONDS)
float record_buffer[MAX_BUFFER_LEN] = {0.0f};
int write_head = 0;                       // audio-ISR only
std::atomic<bool> trigger_dsp_clear{false};

#define MAX_GRAINS 16
struct Grain {
    bool active;
    int size_samples;
    int current_sample;
    float pitch_ratio;
    float playback_head;
};
Grain grains[MAX_GRAINS];                 // audio-ISR only (core 0)
std::atomic<int> clods_submode{0};
const char* clods_mode_names[3] = {"GRANULAR", "LOOP / FREEZE", "PITCH WASH"};

// --- MODE 3: QUANTIZER STATE ---
std::atomic<int> selected_chord_type{0};
std::atomic<int> root_note{0};
const int chord_intervals[8][7] = {
    { 0, 2, 4, 5, 7, 9, 11 },
    { 0, 2, 3, 5, 7, 8, 10 },
    { 0, 2, 4, 7, 9, 0, 0 },
    { 0, 3, 5, 7, 10, 0, 0 },
    { 0, 4, 7, 11, 0, 0, 0 },
    { 0, 3, 7, 10, 0, 0, 0 },
    { 0, 4, 7, 10, 0, 0, 0 },
    { 0, 5, 7, 0, 0, 0, 0 }
};
const int chord_lengths[8] = { 7, 7, 5, 5, 4, 4, 4, 3 };
const char* chord_names[8] = {
    "MAJOR SCALE", "MINOR SCALE", "MAJ PENTATONIC", "MIN PENTATONIC",
    "MAJOR 7TH",   "MINOR 7TH",   "DOMINANT 7TH",   "SUS4 CHORD"
};
const char* note_names[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

// --- MODE 4: QUIRKS STATE ---
std::atomic<int> quirks_audio_route{0};
std::atomic<bool> quirks_sh_trig{false};
std::atomic<bool> quirks_sh_use_audio{false};
const char* quirks_route_names[3] = { "SIGN: RECT/INV", "LOGIC: MAX/MIN", "S&H + NOISE" };

// --- MODE 5: PLAIDS STATE ---
std::atomic<int> plaids_model{0};
std::atomic<bool> plaids_lpg_mode{false};
const char* plaids_model_names[4] = { "VA MORPH", "PHASE DIST", "2-OP FM", "CHORD MACHINE" };

// --- MODE 6: BRAINS STATE ---
std::atomic<int> brains_model{0};
std::atomic<bool> brains_bitcrush{false};
const char* brains_model_names[4] = { "CSAW", "SUPERSAW SWARM", "VOWEL FORMANT", "METALLIC BELL" };

// --- MODE 7: MUMBLES STATE ---
std::atomic<int> mumbles_loop_len{8};
float mumbles_step_cache[16] = {0.2f, 0.5f, 0.7f, 0.4f, 0.8f, 0.3f, 0.6f, 0.5f};
int mumbles_step_ptr = 0;
static float mumbles_clock_ph = 0.0f;
static float mumbles_osc_ph = 0.0f;
static float mumbles_env = 1.0f;
static float mumbles_amp_s = 1.0f;

// --- MODE 8: WRAPS STATE ---
std::atomic<int> wraps_algo{0};
const char* wraps_algo_names[4] = { "WAVEFOLDER", "THRU-ZERO FM", "RING MODULATOR", "VOCODER FILTER" };
static float wraps_carrier_ph = 0.0f;

// --- MODE 9: TIDDLY STATE ---
static float tiddly_phase = 0.0f;
static float tiddly_audio_ph = 0.0f;
static float tiddly_env_s = 0.0f;
std::atomic<int> tiddly_shape_mode{0};
const char* tiddly_mode_names[3] = { "ENVELOPE AD/AR", "SLOPE RAMP LFO", "STEP FUNCTION" };

// --- MODE 10: MULTI-FX STATE ---
std::atomic<int> fx_submode{0};
const char* fx_mode_names[4] = { "CHOR+ECHO+REV", "ENSEMBLE CHORUS", "TAPE ECHO", "HALL REVERB" };

#define CHORUS_BUF_LEN 2048
float chorus_buf[CHORUS_BUF_LEN] = {0.0f};
int chorus_write_idx = 0;
static float chorus_lfo_ph = 0.0f;
static float echo_smooth_delay = 12000.0f;
static float echo_lp_state = 0.0f;

// Freeverb-style hall: 8 parallel damped combs -> 4 series allpasses
#define NUM_COMBS 8
#define NUM_APS   4
#define REV_COMB_TOTAL 12000
#define REV_AP_TOTAL   1701
static const int comb_off[NUM_COMBS + 1] = { 0, 1215, 2508, 3898, 5374, 6922, 8545, 10240, 12000 };
static const int ap_off[NUM_APS + 1]     = { 0, 605, 1085, 1456, 1701 };
float rev_comb_mem[REV_COMB_TOTAL] = {0.0f};
float rev_ap_mem[REV_AP_TOTAL]     = {0.0f};
static int   comb_idx[NUM_COMBS] = {0};
static float comb_lp[NUM_COMBS]  = {0.0f};
static int   ap_idx[NUM_APS]     = {0};

std::atomic<bool> fx_trigger_pluck{false};
static float fx_pluck_env  = 0.0f;
static float fx_pluck_ph   = 0.0f;
static float fx_pluck_freq = 261.63f;
static int   fx_arp_idx    = 0;
static const float fx_arp_freqs[6] = { 261.63f, 311.13f, 392.00f, 466.16f, 587.33f, 783.99f };

// ========================================================
// 2. MATH / AUDIO HELPERS & OLED DRIVER
// ========================================================
inline float clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Transparent below the knee, smoothly approaches +/-1 above it.
// (Replaces the old cubic soft_clip that had 1.5x gain at zero.)
inline float soft_limit(float x) {
    const float knee = 0.7f;
    float a = fabsf(x);
    if (a <= knee) return x;
    float o = (a - knee) / (1.0f - knee);
    float y = knee + (1.0f - knee) * (o / (1.0f + o));
    return (x < 0.0f) ? -y : y;
}

// Deliberate saturator (unity slope at 0, bounded to +/-1)
inline float fast_tanh(float x) {
    if (x >= 3.0f) return 1.0f;
    if (x <= -3.0f) return -1.0f;
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// xorshift32 - fast, no locks, audio-ISR only (do NOT call from core 1)
static uint32_t rng_state = 0x1234ABCDu;
static inline uint32_t xrand() {
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static inline float rnd01()  { return (float)(xrand() >> 8) * (1.0f / 16777216.0f); }
static inline float rnd_bi() { return rnd01() * 2.0f - 1.0f; }

// Equal-power dry/wet crossfade
static inline void xfade(float m, float &dry, float &wet) {
    m = clamp(m, 0.0f, 1.0f);
    dry = sqrtf(1.0f - m);
    wet = sqrtf(m);
}

static inline float dc_block(float x) {
    dc_offset += 0.0002f * (x - dc_offset);
    return x - dc_offset;
}

// Full 24-bit-capable conversion (codec ignores the low 8 bits if it is 24-bit)
static inline float in_to_float(int32_t w) { return (float)w * (1.0f / 2147483648.0f); }

static inline void write_stereo(int32_t* out_buf, int i, float x) {
    if (x != x) x = 0.0f;                                   // NaN guard
    if (fabsf(x) > 1.0f) g_clip_count.fetch_add(1, std::memory_order_relaxed);
    int32_t w = (int32_t)(soft_limit(x) * 2147483392.0f);   // stays inside int32
    out_buf[i]     = w;
    out_buf[i + 1] = w;
}

static inline int wrap_idx(int i, int len) {
    if (i < 0) i += len;
    else if (i >= len) i -= len;
    return i;
}

// 4-point Hermite read 'delay' samples behind write index w (delay < len-4)
static inline float read_behind_hermite(const float* buf, int len, int w, float delay) {
    int   di = (int)delay;
    float t  = delay - (float)di;
    int   i0 = w - di;
    float ym1 = buf[wrap_idx(i0 + 1, len)];
    float y0  = buf[wrap_idx(i0,     len)];
    float y1  = buf[wrap_idx(i0 - 1, len)];
    float y2  = buf[wrap_idx(i0 - 2, len)];
    float c1 = 0.5f * (y1 - ym1);
    float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return ((c3 * t + c2) * t + c1) * t + y0;
}

// PolyBLEP residual for a unit step discontinuity at phase 0
static inline float poly_blep(float t, float dt) {
    if (t < dt)         { t /= dt;               return t + t - t * t - 1.0f; }
    if (t > 1.0f - dt)  { t = (t - 1.0f) / dt;   return t * t + t + t + 1.0f; }
    return 0.0f;
}

// Band-limited square (+1 for p<pw)
static inline float blep_square(float p, float dt, float pw) {
    float s = (p < pw) ? 1.0f : -1.0f;
    s += poly_blep(p, dt);
    float t = p - pw; if (t < 0.0f) t += 1.0f;
    s -= poly_blep(t, dt);
    return s;
}

// Hann window table
#define HANN_N 512
static float hann_tab[HANN_N + 1];
static void init_hann_table() {
    for (int i = 0; i <= HANN_N; i++)
        hann_tab[i] = 0.5f * (1.0f - cosf(TWO_PI_F * (float)i / (float)HANN_N));
}
static inline float hann_lookup(float ph) {      // ph in [0,1)
    float idx = ph * (float)HANN_N;
    int k = (int)idx;
    float f = idx - (float)k;
    return hann_tab[k] * (1.0f - f) + hann_tab[k + 1] * f;
}

// ---- Hardware access ----
uint16_t read_mcp3208(uint channel) {
    uint8_t tx[3], rx[3];
    tx[0] = 0x06 | (channel >> 2);
    tx[1] = (channel & 0x03) << 6;
    tx[2] = 0x00;
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(MCP3208_SPI, tx, rx, 3);
    gpio_put(PIN_CS, 1);
    return ((rx[1] & 0x0F) << 8) | rx[2];
}

// Averaged read (counts, 0..4095, as float)
static float read_mcp3208_avg(uint channel, int n) {
    uint32_t acc = 0;
    for (int k = 0; k < n; k++) acc += read_mcp3208(channel);
    return (float)acc / (float)n;
}

static float read_adc_avg(int input, int n) {
    adc_select_input(input);
    sleep_us(10);
    (void)adc_read();                       // discard first sample after mux switch
    uint32_t acc = 0;
    for (int k = 0; k < n; k++) acc += adc_read();
    return (float)acc / ((float)n * 4095.0f);
}

// ---------------------------------------------------------------------------------------------
// CV input scaling (board LB-SYNTH-PROTO-A v0.2, hardware/README.md section 8.2)
// Each CV input is an INVERTING TL074 stage: Vadc = 1.5 V - 0.3 x Vin, read by the 12-bit MCP3208
// against a 3.0 V reference, so  counts = 2048 - 409.6 x Vin  (+5 V -> 0, 0 V -> mid, -5 V -> 4095).
// The mid-scale point comes from a 240k resistor to the -12 V rail, so it moves with that rail
// (about 2005 for -11.75 V, 2048 for -12.0 V). Calibrate by reading the counts with the jacks
// unpatched (an unpatched jack is shorted to ground) and set CV_IN_MID_COUNTS to that value.
// ---------------------------------------------------------------------------------------------
#define CV_IN_MID_COUNTS 2048.0f

// Counts -> normalised CV in [-1, +1] where +1 is +5 V at the jack (positive voltage = positive value).
// Continuous dead-zone: no jump at the edge of the zone.
static inline float cv_from_counts(float c) {
    const float dz = 30.0f;
    const float d = CV_IN_MID_COUNTS - c;           // inverting input stage: higher voltage -> lower count
    float r = 0.0f;
    if (d > dz)       r = (d - dz) / (2048.0f - dz);
    else if (d < -dz) r = (d + dz) / (2048.0f - dz);
    return fminf(1.0f, fmaxf(-1.0f, r));
}

// ---------------------------------------------------------------------------------------------
// CV output scaling (hardware/README.md section 8.1)
// The MCP4728 runs from its VDD (3.3 V) reference, so Vdac = DAC_VDD_V x code / 4096.
//   CV Out A, B: non-inverting x2.43 stage -> 0 ... ~8.0 V, code passes straight through.
//   CV Out C, D: inverting summing stage   -> Vout = CVOUT_CD_OFFSET_V - CVOUT_CD_GAIN x Vdac,
//                so code 0 is about +5 V and a high code goes NEGATIVE. These two channels are mapped here
//                so that the rest of the firmware can keep writing one convention to g_dac_out[]:
//                0 ... 4095 = 0 V ... +CVOUT_CD_OFFSET_V (about +4.9 V).
// CVOUT_CD_OFFSET_V is |-12V-F| x 100k/240k, about 4.9 V in practice (the -12 V rail sits a Schottky drop
// below the bus). To calibrate: command logical 0 on channel C or D, measure the jack, and adjust this
// value until it reads 0 V.
// ---------------------------------------------------------------------------------------------
#define DAC_VDD_V          3.3f
#define CVOUT_CD_OFFSET_V  4.9f
#define CVOUT_CD_GAIN      5.0f

// Hardware code for CV Out C/D that produces `volts` at the jack (bipolar capable: about -5 ... +4.9 V is
// the useful range; larger magnitudes clip against the op-amp's output swing). Clamped to the DAC range.
static inline uint16_t cvout_cd_code_from_volts(float volts) {
    const float vdac = (CVOUT_CD_OFFSET_V - volts) / CVOUT_CD_GAIN;
    const float code = vdac * (4096.0f / DAC_VDD_V);
    return (uint16_t)(fminf(4095.0f, fmaxf(0.0f, code + 0.5f)));
}

// Logical value written by the modules (0 ... 4095) -> code to send to the MCP4728 for that channel.
static inline uint16_t dac_hw_code(uint8_t channel, uint16_t logical) {
    if (logical > 4095) logical = 4095;
    if (channel < 2) return logical;                               // A, B: unipolar pass-through
    return cvout_cd_code_from_volts((float)logical * (CVOUT_CD_OFFSET_V / 4095.0f));   // C, D: inverted and offset
}

void update_mcp4728_channel(uint8_t channel, uint16_t value) {
    uint8_t buf[3];
    uint8_t dev_cmd = 0x40 | ((channel & 0x03) << 1);
    if (value > 4095) value = 4095;
    buf[0] = dev_cmd;
    buf[1] = (value >> 8) & 0x0F;
    buf[2] = value & 0xFF;
    i2c_write_blocking(i2c0, MCP4728_ADDR, buf, 3, false);
}

uint8_t oled_buf[1024];
const uint8_t font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
    0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
    0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x08,0x2A,0x1C,0x2A,0x08, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
    0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
    0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
    0x00,0x08,0x14,0x22,0x41, 0x14,0x14,0x14,0x14,0x14, 0x41,0x22,0x14,0x08,0x00, 0x02,0x01,0x51,0x09,0x06,
    0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x01,0x01, 0x3E,0x41,0x41,0x51,0x32,
    0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
    0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x04,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
    0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x7F,0x20,0x18,0x20,0x7F,
    0x63,0x14,0x08,0x14,0x63, 0x03,0x04,0x78,0x04,0x03, 0x61,0x51,0x49,0x45,0x43
};

void oled_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(i2c1, OLED_ADDR, buf, 2, false);
}

void oled_init() {
    uint8_t init_cmds[] = {
        0xAE, 0x20, 0x00, 0x40, 0xA1, 0xA8, 63, 0xC8, 0xD3, 0x00,
        0xDA, 0x12, 0xD5, 0x80, 0xD9, 0xF1, 0xDB, 0x30, 0x81, 0xFF,
        0xA4, 0xA6, 0x8D, 0x14, 0xAF
    };
    for (int i = 0; i < (int)sizeof(init_cmds); i++) oled_cmd(init_cmds[i]);
    memset(oled_buf, 0, sizeof(oled_buf));
}

// ---- OLED drawing primitives -------------------------------------------------
// g_ink = 1 sets pixels, g_ink = 0 clears them (used for inverted text on a lit bar).
static uint8_t g_ink = 1;

static inline int iclamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline void oled_px(int x, int y) {
    if ((unsigned)x >= 128u || (unsigned)y >= 64u) return;
    uint8_t m = (uint8_t)(1u << (y & 7));
    uint8_t &b = oled_buf[(y >> 3) * 128 + x];
    if (g_ink) b |= m; else b &= (uint8_t)~m;
}
static void oled_hline(int x0, int x1, int y) { for (int x = x0; x <= x1; x++) oled_px(x, y); }
static void oled_vline(int x, int y0, int y1) { for (int y = y0; y <= y1; y++) oled_px(x, y); }
static void oled_fill_rect(int x, int y, int w, int h) {
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) oled_px(xx, yy);
}
static void oled_rect(int x, int y, int w, int h) {
    if (w <= 0 || h <= 0) return;
    oled_hline(x, x + w - 1, y);
    oled_hline(x, x + w - 1, y + h - 1);
    oled_vline(x, y, y + h - 1);
    oled_vline(x + w - 1, y, y + h - 1);
}

void oled_draw_char(int x, int y, char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);      // the font table is upper-case only
    if (c < 32 || c > 90) return;
    int font_idx = (c - 32) * 5;
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[font_idx + i];
        for (int j = 0; j < 7; j++)
            if (line & (1 << j)) oled_px(x + i, y + j);
    }
}

static void oled_draw_char2x(int x, int y, char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    if (c < 32 || c > 90) return;
    int font_idx = (c - 32) * 5;
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[font_idx + i];
        for (int j = 0; j < 7; j++)
            if (line & (1 << j)) oled_fill_rect(x + i * 2, y + j * 2, 2, 2);
    }
}

void oled_print(int x, int y, const char* str) {
    while (*str) { oled_draw_char(x, y, *str++); x += 6; }
}
static void oled_print_n(int x, int y, const char* str, int max_chars) {
    while (*str && max_chars-- > 0) { oled_draw_char(x, y, *str++); x += 6; }
}
static void oled_print2x(int x, int y, const char* str) {
    while (*str) { oled_draw_char2x(x, y, *str++); x += 12; }
}
static void oled_print_right(int x_right, int y, const char* str) {
    oled_print(x_right - (int)strlen(str) * 6, y, str);
}
static void oled_print_center(int x_center, int y, const char* str) {
    oled_print(x_center - (int)strlen(str) * 3, y, str);
}

// Only the 8-pixel pages that changed since the last frame are sent, so static
// screens (menus, parameter pages) cost almost no I2C time and the animated ones
// are no worse than before.
static uint8_t oled_prev[1024];
static bool    oled_force_full = true;

void oled_show() {
    static uint8_t cmd_buf[7];
    static uint8_t tx_buf[129];
    for (int p = 0; p < 8; p++) {
        const uint8_t* cur = &oled_buf[p * 128];
        if (!oled_force_full && memcmp(cur, &oled_prev[p * 128], 128) == 0) continue;
        cmd_buf[0] = 0x00;                               // all following bytes are commands
        cmd_buf[1] = 0x21; cmd_buf[2] = 0;   cmd_buf[3] = 127;   // column range
        cmd_buf[4] = 0x22; cmd_buf[5] = (uint8_t)p; cmd_buf[6] = (uint8_t)p;  // page range
        i2c_write_blocking(i2c1, OLED_ADDR, cmd_buf, 7, false);
        tx_buf[0] = 0x40;
        memcpy(&tx_buf[1], cur, 128);
        i2c_write_blocking(i2c1, OLED_ADDR, tx_buf, 129, false);
        memcpy(&oled_prev[p * 128], cur, 128);
    }
    oled_force_full = false;
}

static void oled_set_contrast(uint8_t v) { oled_cmd(0x81); oled_cmd(v); }
static void oled_power(bool on)          { oled_cmd(on ? 0xAF : 0xAE); if (on) oled_force_full = true; }

// Outlined horizontal bar with a 1 px inset fill
static void oled_hbar(int x, int y, int w, int h, float v) {
    oled_rect(x, y, w, h);
    int fw = (int)((float)(w - 2) * clamp(v, 0.0f, 1.0f));
    oled_fill_rect(x + 1, y + 1, fw, h - 2);
}

// ========================================================
// 3. USER INTERFACE: VIEWS, MENUS, SCOPE / SPECTRUM
//    (everything in this section runs on core 1 only)
// ========================================================
#define FW_VERSION "3.3"

enum View   { VIEW_PARAMS = 0, VIEW_SCOPE, VIEW_SPECTRUM, VIEW_COMBO, NUM_VIEWS };
enum Screen { SCR_PLAY = 0, SCR_MENU, SCR_SETTINGS, SCR_SYSTEM };

static const char* view_names[NUM_VIEWS] = { "VIEW: PARAMETERS", "VIEW: SCOPE", "VIEW: SPECTRUM", "VIEW: SCOPE+SPECTRUM" };

// What SELECT / EDIT / the encoder do in each module (shown in the menu footer + on entry)
static const char* mode_hints[NUM_MODES] = {
    "SEL:CHAN EDIT:SHAPE",   // Pims
    "SEL:STRIKE EDIT:FRZ",   // Ringo
    "SEL:RANDOM EDIT:FRZ",   // Clods
    "SEL:ROOT ENC:SCALE",    // Quantizer
    "SEL:S&H EDIT:SRC",      // Quirks
    "SEL:TRIG EDIT:LPG",     // Plaids
    "SEL:ACCENT EDIT:CRUSH", // Brains
    "SEL:TRIG ENC:STEPS",    // Mumbles
    "ENC:ALGORITHM",         // Wraps
    "ENC:MODE",              // Tiddly
    "SEL:PLUCK EDIT:FRZ"     // Multi-FX
};
static const char* shape_short[10] = { "GATE", "TRI", "SINE", "EXP", "LOG", "SAW", "TRAP", "S&H", "WANDER", "EUCLID" };
// Scope timebases: samples per pixel of the 2:1-decimated ring -> time across the 128 px screen = stride * 128 * 2 / SAMPLE_RATE
// (5.46 ms per unit of stride at 46,875 Hz; test_knobs checks the labels against this formula)
#define NUM_TIMEBASES 8
static const int         tb_stride[NUM_TIMEBASES] = { 1, 2, 4, 6, 9, 12, 16, 24 };
static const char*       tb_names[NUM_TIMEBASES]  = { "5MS", "11MS", "22MS", "33MS", "49MS", "66MS", "87MS", "131MS" };

// ---- settings (RAM only for now; see notes about persisting to flash) ----
struct Settings {
    uint8_t brightness;   // 0..4
    uint8_t saver;        // 0 off, 1 30 s, 2 1 min, 3 5 min
    uint8_t timebase;     // index into tb_stride / tb_names
    uint8_t trig;         // 0 rising-edge trigger, 1 free-run
    uint8_t smooth;       // spectrum fall speed: 0 fast, 1 medium, 2 slow
    uint8_t peakhold;     // spectrum peak caps
    uint8_t enc;          // encoder speed: 0 normal (1 step per detent), 1 slow (1 per 2), 2 slower (1 per 3)
};
static Settings cfg = { 3, 2, 4, 0, 1, 1, 0 };   // timebase 4 = 48 ms
static const uint8_t  contrast_tab[5] = { 0x10, 0x40, 0x80, 0xC0, 0xFF };
static const uint32_t saver_ms[4]     = { 0, 30000, 60000, 300000 };
static const int      enc_div_tab[3]  = { 1, 2, 3 };

// ---- UI state ----
static Screen   ui_screen = SCR_PLAY;
static int      ui_view = VIEW_PARAMS;
static int      ui_focus = 0;                 // last pot touched (PARAMS view highlight + overlay)
static int      menu_cur = 0, set_cur = 0, list_first = 0;
static bool     set_editing = false;
static char     toast_msg[24] = "";
static uint32_t toast_t = 0, param_ov_t = 0;
static bool     toast_on = false, param_ov_on = false;

// smoothed copies of the audio-thread statistics
static float    ui_cpu_avg = 0.0f, ui_cpu_peak = 0.0f, ui_lvl_hold = 0.0f;
static uint32_t ui_cpu_peak_t = 0, ui_xrun_seen = 0, ui_xrun_t = 0, ui_clip_seen = 0, ui_clip_t = 0;
static bool     ui_xrun_ever = false;

static void ui_toast(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vsnprintf(toast_msg, sizeof(toast_msg), fmt, ap);   // 23 chars max; screen holds 20
    va_end(ap);
    toast_msg[20] = 0;
    toast_t = to_ms_since_boot(get_absolute_time());
    toast_on = true;
}
static void ui_param_touched(int idx, uint32_t now) { ui_focus = idx; param_ov_t = now; param_ov_on = true; }

static inline float lin_to_db(float x) { return 8.685889638f * logf(x + 1e-9f); }

// ---------------------------------------------------------------------------
// FFT (512 point radix-2) for the spectrum
// ---------------------------------------------------------------------------
#define FFT_N 512
#define FFT_LOG2 9
static float    fft_re[FFT_N], fft_im[FFT_N];
static float    fft_win[FFT_N], fft_cos[FFT_N / 2], fft_sin[FFT_N / 2];
static uint16_t fft_rev[FFT_N];

#define SPEC_BARS 42
#define SPEC_F_LO 60.0f
#define SPEC_F_HI 12000.0f
#define SPEC_FLOOR_DB -72.0f
static float   spec_db[FFT_N / 2 + 2];
static float   spec_lo_bin[SPEC_BARS], spec_hi_bin[SPEC_BARS];
static float   bar_db[SPEC_BARS], bar_pk[SPEC_BARS];
static uint8_t bar_hold[SPEC_BARS];
static float   spec_peak_hz = 0.0f, spec_peak_db = -120.0f;
static float   scope_gain = 1.0f;

static void ui_init_tables() {
    for (int i = 0; i < FFT_N; i++) {
        fft_win[i] = 0.5f * (1.0f - cosf(TWO_PI_F * (float)i / (float)(FFT_N - 1)));
        int r = 0;
        for (int b = 0; b < FFT_LOG2; b++) if (i & (1 << b)) r |= 1 << (FFT_LOG2 - 1 - b);
        fft_rev[i] = (uint16_t)r;
    }
    for (int i = 0; i < FFT_N / 2; i++) {
        fft_cos[i] =  cosf(TWO_PI_F * (float)i / (float)FFT_N);
        fft_sin[i] = -sinf(TWO_PI_F * (float)i / (float)FFT_N);
    }
    const float binw  = (float)SCOPE_FS / (float)FFT_N;
    const float ratio = powf(SPEC_F_HI / SPEC_F_LO, 1.0f / (float)SPEC_BARS);
    float f = SPEC_F_LO;
    for (int b = 0; b < SPEC_BARS; b++) {
        spec_lo_bin[b] = f / binw;
        f *= ratio;
        spec_hi_bin[b] = f / binw;
        bar_db[b] = bar_pk[b] = SPEC_FLOOR_DB;
    }
}

static void fft_run() {
    for (int i = 0; i < FFT_N; i++) {
        int j = fft_rev[i];
        if (j > i) {
            float t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;
            t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t;
        }
    }
    for (int len = 2; len <= FFT_N; len <<= 1) {
        const int half = len >> 1, step = FFT_N / len;
        for (int i = 0; i < FFT_N; i += len) {
            for (int k = 0; k < half; k++) {
                const float wr = fft_cos[k * step], wi = fft_sin[k * step];
                const int a = i + k, b = a + half;
                const float tr = fft_re[b] * wr - fft_im[b] * wi;
                const float ti = fft_re[b] * wi + fft_im[b] * wr;
                fft_re[b] = fft_re[a] - tr;  fft_im[b] = fft_im[a] - ti;
                fft_re[a] += tr;             fft_im[a] += ti;
            }
        }
    }
}

static void spectrum_update() {
    const uint32_t end = g_scope_wr.load(std::memory_order_acquire);
    for (int i = 0; i < FFT_N; i++) {
        fft_re[i] = (float)g_scope_ring[(end - FFT_N + (uint32_t)i) & (SCOPE_RING_LEN - 1)] * (1.0f / 32768.0f) * fft_win[i];
        fft_im[i] = 0.0f;
    }
    fft_run();

    const float norm = 4.0f / (float)FFT_N;        // Hann (coherent gain 0.5): sine of amplitude A -> A
    float best = -200.0f; int bi = 2;
    for (int k = 0; k <= FFT_N / 2; k++) {
        float m = sqrtf(fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]) * norm;
        float db = lin_to_db(m);
        spec_db[k] = db;
        if (k >= 2 && k < FFT_N / 2 && db > best) { best = db; bi = k; }
    }
    spec_db[FFT_N / 2 + 1] = spec_db[FFT_N / 2];
    // parabolic peak interpolation -> frequency readout
    {
        float a = spec_db[bi - 1], b = spec_db[bi], c = spec_db[bi + 1];
        float den = a - 2.0f * b + c;
        float d = (fabsf(den) > 1e-6f) ? 0.5f * (a - c) / den : 0.0f;
        spec_peak_hz = ((float)bi + clamp(d, -0.5f, 0.5f)) * (float)SCOPE_FS / (float)FFT_N;
        spec_peak_db = best;
    }

    static const float fall[3] = { 0.65f, 0.35f, 0.18f };
    const float f_coef = fall[cfg.smooth > 2 ? 2 : cfg.smooth];
    for (int b = 0; b < SPEC_BARS; b++) {
        float lo = spec_lo_bin[b], hi = spec_hi_bin[b], db;
        if (hi - lo < 1.0f) {                                   // narrow bar: interpolate between bins
            float c = 0.5f * (lo + hi);
            int i0 = (int)c; float fr = c - (float)i0;
            if (i0 > FFT_N / 2) i0 = FFT_N / 2;
            db = spec_db[i0] * (1.0f - fr) + spec_db[i0 + 1] * fr;
        } else {                                                // wide bar: strongest bin inside it
            int i0 = (int)ceilf(lo), i1 = (int)floorf(hi);
            if (i1 > FFT_N / 2) i1 = FFT_N / 2;
            if (i0 > i1) i0 = i1;
            db = spec_db[i0];
            for (int k = i0 + 1; k <= i1; k++) if (spec_db[k] > db) db = spec_db[k];
        }
        if (db < SPEC_FLOOR_DB) db = SPEC_FLOOR_DB;
        if (db > bar_db[b]) bar_db[b] += 0.85f * (db - bar_db[b]);
        else                bar_db[b] += f_coef * (db - bar_db[b]);
        if (bar_db[b] >= bar_pk[b]) { bar_pk[b] = bar_db[b]; bar_hold[b] = 22; }
        else if (bar_hold[b])       { bar_hold[b]--; }
        else { bar_pk[b] -= 1.5f; if (bar_pk[b] < bar_db[b]) bar_pk[b] = bar_db[b]; }
    }
}

// ---------------------------------------------------------------------------
// Drawing: scope, spectrum, level meter
// ---------------------------------------------------------------------------
static void draw_scope(int x, int y, int w, int h) {
    const int stride = tb_stride[cfg.timebase % NUM_TIMEBASES];   // samples per pixel of the scope ring
    const uint32_t span = (uint32_t)(w * stride) + 1;
    const uint32_t search = 512;
    const uint32_t end = g_scope_wr.load(std::memory_order_acquire);
    const uint32_t start = end - (span + search);
#define SC(k) ((int)g_scope_ring[(uint32_t)(k) & (SCOPE_RING_LEN - 1)])

    int pk = 1;
    for (uint32_t k = 0; k < span + search; k += 2) { int v = abs(SC(start + k)); if (v > pk) pk = v; }

    uint32_t off = 0;                                       // rising-edge trigger with hysteresis
    if (cfg.trig == 0) {
        const int hyst = pk / 8;
        bool armed = false;
        for (uint32_t k = 0; k < search; k++) {
            int v = SC(start + k);
            if (v < -hyst) armed = true;
            else if (armed && v >= 0) { off = k; break; }
        }
    }

    float target = 0.85f * 32768.0f / (float)pk;            // gentle auto-gain (boost only)
    if (target > 16.0f) target = 16.0f;
    if (target < 1.0f)  target = 1.0f;
    scope_gain += 0.15f * (target - scope_gain);

    const int yc = y + h / 2;
    const float k_amp = scope_gain * (float)(h / 2 - 1) / 32768.0f;
    for (int px = 0; px < w; px += 4) oled_px(x + px, yc);  // dotted centre line

    int prev_lo = yc, prev_hi = yc;
    for (int px = 0; px < w; px++) {
        const uint32_t i0 = start + off + (uint32_t)px * (uint32_t)stride;
        int lo = SC(i0), hi = lo;
        for (int s = 1; s <= stride; s++) { int v = SC(i0 + (uint32_t)s); if (v < lo) lo = v; if (v > hi) hi = v; }
        int y_hi = iclamp(yc - (int)((float)hi * k_amp), y, y + h - 1);
        int y_lo = iclamp(yc - (int)((float)lo * k_amp), y, y + h - 1);
        const int c_hi = y_hi, c_lo = y_lo;
        if (px > 0) {                                       // join to the previous column
            if (y_lo < prev_hi) y_lo = prev_hi;
            if (y_hi > prev_lo) y_hi = prev_lo;
        }
        oled_vline(x + px, y_hi, y_lo);
        prev_hi = c_hi; prev_lo = c_lo;
    }
#undef SC
}

static void draw_spectrum(int x, int y, int h) {            // bars occupy y .. y+h-1, baseline at y+h
    const int pitch = 3;
    const int x0 = x + (128 - SPEC_BARS * pitch) / 2;
    for (int b = 0; b < SPEC_BARS; b++) {
        float v = clamp((bar_db[b] - SPEC_FLOOR_DB) / (-SPEC_FLOOR_DB), 0.0f, 1.0f);
        int hp = (int)(v * (float)h + 0.5f);
        if (hp > 0) oled_fill_rect(x0 + b * pitch, y + h - hp, 2, hp);
        if (cfg.peakhold) {
            float vp = clamp((bar_pk[b] - SPEC_FLOOR_DB) / (-SPEC_FLOOR_DB), 0.0f, 1.0f);
            int pp = (int)(vp * (float)h + 0.5f);
            if (pp > hp + 1) oled_hline(x0 + b * pitch, x0 + b * pitch + 1, y + h - pp);
        }
    }
    oled_hline(x, x + 127, y + h);
}

static void draw_spec_axis(int y_tick) {
    static const struct { float hz; const char* s; } T[3] = { {100.0f, "100"}, {1000.0f, "1K"}, {10000.0f, "10K"} };
    const float span = logf(SPEC_F_HI / SPEC_F_LO);
    for (int t = 0; t < 3; t++) {
        float pos = logf(T[t].hz / SPEC_F_LO) / span;
        int xp = 1 + (int)(pos * (float)(SPEC_BARS * 3));
        oled_vline(xp, y_tick, y_tick + 1);
        int len = (int)strlen(T[t].s);
        oled_print(iclamp(xp - len * 3, 0, 128 - len * 6), y_tick + 3, T[t].s);
    }
}

static float level_pos(float lin) { return clamp((lin_to_db(lin) + 48.0f) / 48.0f, 0.0f, 1.0f); }

static void draw_level_bar(int x, int y, int w, int h) {
    float pk = g_out_peak.load(std::memory_order_relaxed);
    ui_lvl_hold = (pk > ui_lvl_hold) ? pk : ui_lvl_hold * 0.93f;
    oled_rect(x, y, w, h);
    int fw = (int)((float)(w - 2) * level_pos(g_out_rms.load(std::memory_order_relaxed) * 1.4f));
    oled_fill_rect(x + 1, y + 1, fw, h - 2);
    int tx = x + 1 + (int)((float)(w - 3) * level_pos(ui_lvl_hold));
    oled_vline(tx, y, y + h - 1);
}

// ---------------------------------------------------------------------------
// Header + play-screen views
// ---------------------------------------------------------------------------
static void mode_sub_label(int mode, char* out, size_t n) {
    static const char* ringo_s[6]   = { "STRINGS", "SYMPATH", "TUBE", "BELL", "FM KARP", "CHORD" };
    static const char* clods_s[3]  = { "GRAIN", "LOOP", "WASH" };
    static const char* chord_s[8]   = { "MAJOR", "MINOR", "MAJ PENT", "MIN PENT", "MAJ7", "MIN7", "DOM7", "SUS4" };
    static const char* quirks_s[3]   = { "RECT/INV", "MAX/MIN", "S&H" };
    static const char* plaids_s[4]  = { "VA", "PHASE DIST", "2-OP FM", "CHORD" };
    static const char* brains_s[4]  = { "CSAW", "SWARM", "VOWEL", "BELL" };
    static const char* wraps_s[4]   = { "FOLD", "THRU-0 FM", "RING MOD", "VOCODER" };
    static const char* tiddly_s[3]   = { "AD/AR", "SLOPE", "STEPS" };
    static const char* fx_s[4]      = { "ALL", "CHORUS", "ECHO", "REVERB" };
    switch (mode) {
        case MODE_PIMS:      snprintf(out, n, "%d BPM", master_bpm.load()); break;
        case MODE_RINGO:     snprintf(out, n, "%s", ringo_s[ringo_model_idx.load() % 6]); break;
        case MODE_CLODS:    snprintf(out, n, "%s", clods_s[clods_submode.load() % 3]); break;
        case MODE_QUANTIZER: snprintf(out, n, "%s %s", note_names[root_note.load() % 12], chord_s[selected_chord_type.load() % 8]); break;
        case MODE_QUIRKS:     snprintf(out, n, "%s", quirks_s[quirks_audio_route.load() % 3]); break;
        case MODE_PLAIDS:    snprintf(out, n, "%s%s", plaids_s[plaids_model.load() % 4], plaids_lpg_mode.load() ? " LPG" : ""); break;
        case MODE_BRAINS:    snprintf(out, n, "%s%s", brains_s[brains_model.load() % 4], brains_bitcrush.load() ? " CRUSH" : ""); break;
        case MODE_MUMBLES:   snprintf(out, n, "%d STEPS", mumbles_loop_len.load()); break;
        case MODE_WRAPS:     snprintf(out, n, "%s", wraps_s[wraps_algo.load() % 4]); break;
        case MODE_TIDDLY:     snprintf(out, n, "%s", tiddly_s[tiddly_shape_mode.load() % 3]); break;
        case MODE_FX:        snprintf(out, n, "%s", fx_s[fx_submode.load() % 4]); break;
        default:             out[0] = 0; break;
    }
}

static void ui_chip(int x_right, int y, const char* s, bool inverted) {   // right-aligned tag; returns via x
    int w = (int)strlen(s) * 6 + 3;
    int x = x_right - w;
    if (inverted) {
        g_ink = 1; oled_fill_rect(x, y, w, 9);
        g_ink = 0; oled_print(x + 2, y + 1, s);
        g_ink = 1;
    } else {
        oled_print(x + 2, y + 1, s);
    }
}

static void ui_update_stats(uint32_t now) {
    ui_cpu_avg += 0.35f * (g_cpu_load_pct.load(std::memory_order_relaxed) - ui_cpu_avg);
    float m = g_cpu_peak_pct.load(std::memory_order_relaxed);
    if (m >= ui_cpu_peak) { ui_cpu_peak = m; ui_cpu_peak_t = now; }
    else if (now - ui_cpu_peak_t > 3000) ui_cpu_peak += 0.1f * (m - ui_cpu_peak);   // hold 3 s, then ease down
    uint32_t xr = g_xrun_count.load(std::memory_order_relaxed);
    if (xr != ui_xrun_seen) { ui_xrun_seen = xr; ui_xrun_t = now; ui_xrun_ever = true; }
    uint32_t cl = g_clip_count.load(std::memory_order_relaxed);
    if (cl != ui_clip_seen) { ui_clip_seen = cl; ui_clip_t = now; }
}

static void draw_header(int mode, uint32_t now) {
    // module name chip
    const char* nm = mode_short_names[mode];
    int bw = (int)strlen(nm) * 6 + 5;
    g_ink = 1; oled_fill_rect(0, 0, bw, 10);
    g_ink = 0; oled_print(3, 1, nm);
    g_ink = 1;

    // right side, built from the edge inwards: CPU, then FRZ, then CLIP
    char cpu[8];
    snprintf(cpu, sizeof(cpu), "%d%%", (int)(ui_cpu_avg + 0.5f));
    bool warn = (ui_cpu_avg > 85.0f) || (now - ui_xrun_t < 3000 && ui_xrun_ever);
    int xr = 128;
    ui_chip(xr, 0, cpu, warn);
    xr -= (int)strlen(cpu) * 6 + 3 + 2;
    if (is_frozen.load() && (mode == MODE_RINGO || mode == MODE_CLODS || mode == MODE_FX)) {
        ui_chip(xr, 0, "FRZ", true);
        xr -= 3 * 6 + 3 + 2;
    }
    if (now - ui_clip_t < 1500 && ui_clip_seen != 0) {
        ui_chip(xr, 0, "CLIP", true);
        xr -= 4 * 6 + 3 + 2;
    }

    char sub[24];
    mode_sub_label(mode, sub, sizeof(sub));
    int avail = (xr - (bw + 4)) / 6;
    if (avail > 0) oled_print_n(bw + 4, 1, sub, avail);
}

static void draw_view_pims() {
    const int sel = selected_cv_out.load();
    for (int ch = 0; ch < NUM_CV_OUTPUTS; ch++) {
        const int y = 14 + ch * 11;
        char buf[12];
        if (ch == sel) oled_print(0, y + 1, ">");
        snprintf(buf, sizeof(buf), "CV%d", ch + 1);
        oled_print(8, y + 1, buf);
        int m = cv_channels[ch].mult_div;
        snprintf(buf, sizeof(buf), m > 0 ? "X%d" : "/%d", abs(m));
        oled_print(34, y + 1, buf);
        oled_print_n(62, y + 1, shape_short[iclamp(cv_channels[ch].shape, 0, 9)], 6);
        oled_hbar(100, y, 26, 9, (float)g_dac_out[ch].load() * (1.0f / 4095.0f));
    }
}

static void draw_view_params(int mode) {
    if (mode == MODE_PIMS) { draw_view_pims(); return; }
    char buf[20];
    oled_print_n(2, 13, mode_param_names[mode][ui_focus], 14);
    snprintf(buf, sizeof(buf), "%d%%", (int)(g_param[ui_focus].load() * 100.0f + 0.5f));
    oled_print_right(126, 13, buf);
    for (int i = 0; i < 7; i++) {
        const int x0 = 4 + i * 18;
        const float v = clamp(g_param[i].load(), 0.0f, 1.0f);
        oled_rect(x0, 23, 11, 30);
        const int fh = (int)(28.0f * v + 0.5f);
        oled_fill_rect(x0 + 1, 23 + 29 - fh, 9, fh);
        snprintf(buf, sizeof(buf), "%d", i + 1);
        if (i == ui_focus) {
            g_ink = 1; oled_fill_rect(x0 - 1, 55, 13, 9);
            g_ink = 0; oled_print(x0 + 3, 56, buf);
            g_ink = 1;
        } else {
            oled_print(x0 + 3, 56, buf);
        }
    }
}

static void draw_view_scope() {
    draw_scope(0, 12, 128, 40);
    char buf[24];
    snprintf(buf, sizeof(buf), "%s X%d", tb_names[cfg.timebase % NUM_TIMEBASES], (int)(scope_gain + 0.5f));
    oled_print(2, 55, buf);
    draw_level_bar(84, 54, 43, 8);
}

static void draw_view_spectrum() {
    spectrum_update();
    draw_spectrum(0, 12, 41);
    draw_spec_axis(54);
    if (spec_peak_db > -55.0f) {                                    // peak frequency read-out
        char buf[12];
        if (spec_peak_hz >= 1000.0f) snprintf(buf, sizeof(buf), "%.1fKHZ", (double)(spec_peak_hz * 0.001f));
        else                         snprintf(buf, sizeof(buf), "%dHZ", (int)(spec_peak_hz + 0.5f));
        int w = (int)strlen(buf) * 6 + 3;
        g_ink = 0; oled_fill_rect(128 - w - 1, 12, w + 1, 9);
        g_ink = 1; oled_print(128 - w + 1, 13, buf);
    }
}

static void draw_view_combo() {
    spectrum_update();
    draw_scope(0, 12, 128, 21);
    for (int px = 0; px < 128; px += 2) oled_px(px, 34);
    draw_spectrum(0, 36, 27);
}

static void draw_toast_bar() {
    g_ink = 1; oled_fill_rect(0, 50, 128, 14);
    g_ink = 0; oled_print_center(64, 54, toast_msg);
    g_ink = 1;
}

static void draw_param_overlay(int mode, int idx) {
    g_ink = 0; oled_fill_rect(0, 45, 128, 19);
    g_ink = 1; oled_hline(0, 127, 45);
    const float v = clamp(g_param[idx].load(), 0.0f, 1.0f);
    char buf[12];
    oled_print_n(3, 48, mode_param_names[mode][idx], 15);
    snprintf(buf, sizeof(buf), "%d%%", (int)(v * 100.0f + 0.5f));
    oled_print_right(125, 48, buf);
    oled_hbar(3, 57, 122, 5, v);
}

static void draw_play(int mode, uint32_t now) {
    draw_header(mode, now);
    switch (ui_view) {
        case VIEW_SCOPE:    draw_view_scope();    break;
        case VIEW_SPECTRUM: draw_view_spectrum(); break;
        case VIEW_COMBO:    draw_view_combo();    break;
        default:            draw_view_params(mode); break;
    }
    if (toast_on && now - toast_t < 1400)               draw_toast_bar();
    else if (param_ov_on && now - param_ov_t < 1500)    draw_param_overlay(mode, ui_focus);
}

// ---------------------------------------------------------------------------
// Menus (module list + settings share one list renderer)
// ---------------------------------------------------------------------------
static void ui_title_bar(const char* title, const char* right) {
    g_ink = 1; oled_fill_rect(0, 0, 128, 10);
    g_ink = 0; oled_print(3, 1, title);
    if (right) oled_print_right(125, 1, right);
    g_ink = 1;
}

typedef const char* (*label_fn)(int);
typedef void        (*value_fn)(int, char*, size_t);

static void draw_list(const char* title, int count, int cursor, bool editing,
                      label_fn label, value_fn value, int mark_idx, const char* footer) {
    const int ROWS = 4, ROW_H = 10, Y0 = 11;
    if (cursor < list_first) list_first = cursor;
    if (cursor >= list_first + ROWS) list_first = cursor - ROWS + 1;
    if (list_first < 0) list_first = 0;

    char pg[24];
    snprintf(pg, sizeof(pg), "%d/%d", cursor + 1, count);
    ui_title_bar(title, pg);

    for (int r = 0; r < ROWS; r++) {
        const int idx = list_first + r;
        if (idx >= count) break;
        const int y = Y0 + r * ROW_H;
        const bool sel = (idx == cursor);
        char val[16] = "";
        if (value) value(idx, val, sizeof(val));

        if (sel) {
            if (!editing) { g_ink = 1; oled_fill_rect(0, y, 124, ROW_H - 1); g_ink = 0; }
            else          { g_ink = 1; oled_rect(0, y, 124, ROW_H - 1); }
        }
        oled_print(4, y + 1, label(idx));
        if (idx == mark_idx) oled_print(val[0] ? 90 : 110, y + 1, "*");
        if (val[0]) {
            const int vw = (int)strlen(val) * 6;
            if (sel && editing) {
                g_ink = 1; oled_fill_rect(120 - vw - 3, y + 1, vw + 5, ROW_H - 3);
                g_ink = 0; oled_print_right(120, y + 1, val);
            } else {
                oled_print_right(120, y + 1, val);
            }
        }
        g_ink = 1;
    }

    if (count > ROWS) {                                         // scroll bar
        const int total = ROWS * ROW_H - 1;
        for (int yy = Y0; yy < Y0 + total; yy += 2) oled_px(127, yy);
        int th = total * ROWS / count; if (th < 6) th = 6;
        int ty = Y0 + (total - th) * list_first / (count - ROWS);
        oled_fill_rect(126, ty, 2, th);
    }
    oled_hline(0, 127, 52);
    oled_print_n(2, 55, footer, 21);
}

// -- module menu --
#define MENU_N (NUM_MODES + 2)
static const char* menu_label(int i)  { return i < NUM_MODES ? mode_titles[i] : (i == NUM_MODES ? "SETTINGS" : "SYSTEM INFO"); }
static const char* menu_footer(int i) {
    if (i < NUM_MODES) return mode_hints[i];
    return i == NUM_MODES ? "DISPLAY, SCOPE, ..." : "CPU, LEVEL, MEMORY";
}

// -- settings --
#define SET_N 10
enum { SET_ENC = 6, SET_RESET = 7, SET_CLEAR = 8, SET_BACK = 9 };
static const char* set_labels[SET_N] = { "BRIGHTNESS", "SCREEN SAVER", "SCOPE TIME", "SCOPE TRIG", "SPECTRUM",
                                         "PEAK HOLD", "ENCODER", "RESET STATS", "CLEAR BUFFERS", "BACK" };
static const char* set_label(int i) { return set_labels[i]; }
static void set_value(int i, char* o, size_t n) {
    static const char* saver_s[4] = { "OFF", "30 SEC", "1 MIN", "5 MIN" };
    static const char* trig_s[2]  = { "RISING", "FREE" };
    static const char* smooth_s[3] = { "FAST", "MEDIUM", "SLOW" };
    static const char* enc_s[3]    = { "NORMAL", "SLOW", "SLOWER" };
    switch (i) {
        case 0: snprintf(o, n, "%d/5", cfg.brightness + 1); break;
        case 1: snprintf(o, n, "%s", saver_s[cfg.saver & 3]); break;
        case 2: snprintf(o, n, "%s", tb_names[cfg.timebase % NUM_TIMEBASES]); break;
        case 3: snprintf(o, n, "%s", trig_s[cfg.trig & 1]); break;
        case 4: snprintf(o, n, "%s", smooth_s[cfg.smooth > 2 ? 2 : cfg.smooth]); break;
        case 5: snprintf(o, n, "%s", cfg.peakhold ? "ON" : "OFF"); break;
        case SET_ENC: snprintf(o, n, "%s", enc_s[cfg.enc > 2 ? 2 : cfg.enc]); break;
        default: o[0] = 0; break;
    }
}
static bool set_is_action(int i) { return i >= SET_RESET; }
static void set_adjust(int i, int dir) {
    auto wrap = [&](uint8_t v, int count) { return (uint8_t)((v + dir + count) % count); };
    switch (i) {
        case 0: cfg.brightness = (uint8_t)iclamp(cfg.brightness + dir, 0, 4); oled_set_contrast(contrast_tab[cfg.brightness]); break;
        case 1: cfg.saver    = wrap(cfg.saver, 4); break;
        case 2: cfg.timebase = wrap(cfg.timebase, NUM_TIMEBASES); break;
        case 3: cfg.trig     = wrap(cfg.trig, 2); break;
        case 4: cfg.smooth   = wrap(cfg.smooth, 3); break;
        case 5: cfg.peakhold = cfg.peakhold ? 0 : 1; break;
        case SET_ENC: cfg.enc = wrap(cfg.enc, 3); break;
        default: break;
    }
}

// -- system info --
static void draw_system(uint32_t now) {
    ui_title_bar("SYSTEM INFO", "FW " FW_VERSION);
    char buf[48];

    // CPU: average bar with a peak tick
    oled_print(2, 12, "CPU");
    oled_rect(24, 11, 60, 9);
    oled_fill_rect(25, 12, (int)(58.0f * clamp(ui_cpu_avg * 0.01f, 0.0f, 1.0f)), 7);
    oled_vline(25 + (int)(57.0f * clamp(ui_cpu_peak * 0.01f, 0.0f, 1.0f)), 11, 19);
    snprintf(buf, sizeof(buf), "%d/%d%%", (int)(ui_cpu_avg + 0.5f), (int)(ui_cpu_peak + 0.5f));
    oled_print_right(127, 12, buf);

    // level
    float pk = g_out_peak.load(std::memory_order_relaxed);
    oled_print(2, 22, "OUT");
    draw_level_bar(24, 21, 60, 9);
    snprintf(buf, sizeof(buf), "%dDB", (int)lroundf(fmaxf(lin_to_db(pk), -99.0f)));
    oled_print_right(127, 22, buf);

    snprintf(buf, sizeof(buf), "XRUN %lu  CLIP %lu", (unsigned long)g_xrun_count.load(), (unsigned long)g_clip_count.load());
    oled_print(2, 33, buf);
    snprintf(buf, sizeof(buf), "RAM %luK/520K", (unsigned long)(g_sram_static_bytes / 1024));
    oled_print(2, 41, buf);
    snprintf(buf, sizeof(buf), "LATENCY %.1fMS %.1fK", (double)(2.0f * CHUNK_DEADLINE_US * 0.001f), (double)SAMPLE_RATE / 1000.0);
    oled_print(2, 49, buf);
    uint32_t s = now / 1000;
    snprintf(buf, sizeof(buf), "UP %02lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
    oled_print(2, 57, buf);
    oled_print_right(127, 57, "EDIT:RST");
}

static void draw_splash(int step, int steps) {
    memset(oled_buf, 0, sizeof(oled_buf));
    g_ink = 1;
    oled_print2x(10, 14, "MULTI-DSP");
    oled_print_center(64, 38, "FW " FW_VERSION "  RP2350");
    oled_hbar(14, 54, 100, 7, (float)step / (float)steps);
    oled_show();
}

// ========================================================
// 4. DSP ENGINES
// ========================================================
int32_t rx_buf_0[CHUNK_32BIT_WORDS];
int32_t rx_buf_1[CHUNK_32BIT_WORDS];
int32_t tx_buf_0[CHUNK_32BIT_WORDS];
int32_t tx_buf_1[CHUNK_32BIT_WORDS];
int dma_rx_0, dma_rx_1, dma_tx_0, dma_tx_1;

// ---------------------------------------------------------------------------
// Per-sample parameter ramps.
//   sp[i]      : value for the CURRENT sample (advanced by params_tick())
//   sp_tgt[i]  : value this chunk ramps toward (use for chunk-rate decisions)
// The target itself is lag-filtered once per chunk (kills pot/ADC jitter),
// then each sample moves linearly toward it, so there are no 187 Hz steps.
// ---------------------------------------------------------------------------
static float sp[7]      = {0.5f, 0.3f, 0.5f, 0.5f, 0.5f, 0.3f, 0.5f};
static float sp_tgt[7]  = {0.5f, 0.3f, 0.5f, 0.5f, 0.5f, 0.3f, 0.5f};
static float sp_step[7] = {0.0f};

static void params_begin_chunk() {
    for (int i = 0; i < 7; i++) {
        float raw = g_param[i].load();
        sp_tgt[i] += 0.35f * (raw - sp_tgt[i]);
        sp_step[i] = (sp_tgt[i] - sp[i]) * (1.0f / (float)CHUNK_SAMPLES);
    }
}
static inline void params_tick() {
    for (int i = 0; i < 7; i++) sp[i] += sp_step[i];
}

// Helper: exponential pitch law used by several oscillators
static inline float pitch_hz(float base, float range, float p) { return base * powf(range, p); }

// ---- Quantiser core (shared by QUANT and MUMBLES) ----
// Nearest note in the chosen scale/chord, searching adjacent octaves too.
static int quantize_semitone(int semis_in, int root, int chord, int max_semi) {
    int len = chord_lengths[chord];
    int oct = semis_in / 12;
    int best = (semis_in < 0) ? 0 : ((semis_in > max_semi) ? max_semi : semis_in);
    int best_d = 1000;
    for (int o = oct - 1; o <= oct + 1; o++) {
        for (int i = 0; i < len; i++) {
            int cand = o * 12 + root + chord_intervals[chord][i];
            if (cand < 0 || cand > max_semi) continue;
            int d = abs(cand - semis_in);
            if (d < best_d) { best_d = d; best = cand; }
        }
    }
    return best;
}

// --- ENGINE 0: PIM'S PRO CV ---
static float pims_slew_state[4] = {0.0f};
void process_pims_chunk(int32_t* in_buf, int32_t* out_buf) {
    int bpm = master_bpm.load();
    float phase_step = ((float)bpm / 60.0f) / (float)SAMPLE_RATE;

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float gate_width = 0.05f + sp[0] * 0.90f;
        int   euclid_steps = 1 + (int)(sp[1] * 7.99f);
        // WAVE SHAPE (P3): skews the continuous shapes (TRI, SINE, EXP, LOG, SAW, TRAP) by moving the
        // half-way point of the cycle. 12 o'clock (small dead-zone) = unchanged; left = fast first half
        // (sharp attack, long tail), right = slow first half. Gate/S&H/Wander/Euclid keep their own timing.
        float skew_d = sp[2] - 0.5f;
        skew_d = (fabsf(skew_d) < 0.03f) ? 0.0f : (skew_d > 0.0f ? skew_d - 0.03f : skew_d + 0.03f);
        const float skew = clamp(0.5f + skew_d * 1.02f, 0.02f, 0.98f);
        const float sk_a = 0.5f / skew;
        const float sk_b = 0.5f / (1.0f - skew);
        float phase_off  = sp[3];
        float slew_coeff = 1.0f - (sp[4] * 0.995f);
        float chop_depth = sp[5];

        float in_sample = dc_block(in_to_float(in_buf[i]));

        for (int ch = 0; ch < NUM_CV_OUTPUTS; ch++) {
            int m = cv_channels[ch].mult_div;
            if (m == 0) continue;
            float ch_freq_mult = (m > 0) ? (float)m : (1.0f / (float)(-m));
            cv_channels[ch].phase += phase_step * ch_freq_mult;
            bool wrapped = false;
            if (cv_channels[ch].phase >= 1.0f) { cv_channels[ch].phase -= 1.0f; wrapped = true; }

            float p = cv_channels[ch].phase + (ch > 0 ? phase_off : 0.0f);
            if (p >= 1.0f) p -= 1.0f;

            const int shp = cv_channels[ch].shape;
            if (shp >= 1 && shp <= 6)
                p = (p < skew) ? (p * sk_a) : (0.5f + (p - skew) * sk_b);   // piecewise-linear phase warp: 0->0, skew->0.5, 1->1

            float val = 0.0f;
            switch (shp) {
                case 0: val = (p < gate_width) ? 1.0f : 0.0f; break;
                case 1: val = (p < 0.5f) ? (2.0f * p) : (2.0f * (1.0f - p)); break;
                case 2: val = 0.5f + 0.5f * sinf(TWO_PI_F * p); break;
                case 3: val = expf(-4.0f * p); break;
                case 4: val = 1.0f - (p * p); break;
                case 5: val = p; break;
                case 6: val = (p < 0.2f) ? (p * 5.0f) : ((p > 0.8f) ? ((1.0f - p) * 5.0f) : 1.0f); break;
                case 7:
                    if (wrapped) cv_channels[ch].current_val = rnd01();
                    val = cv_channels[ch].current_val;
                    break;
                case 8:
                    if (wrapped) {
                        float delta = rnd_bi() * 0.25f;
                        cv_channels[ch].current_val = clamp(cv_channels[ch].current_val + delta, 0.0f, 1.0f);
                    }
                    val = cv_channels[ch].current_val;
                    break;
                case 9: {
                    int step = (int)(p * 8.0f) % 8;
                    val = (((step * euclid_steps) % 8) < euclid_steps && (fmodf(p * 8.0f, 1.0f) < gate_width)) ? 1.0f : 0.0f;
                    break;
                }
            }
            pims_slew_state[ch] += slew_coeff * (val - pims_slew_state[ch]);
            if (i == 0) {
                g_dac_out[ch].store((uint16_t)(clamp(pims_slew_state[ch], 0.0f, 1.0f) * 4095.0f));
            }
        }

        float mod_gain = (1.0f - chop_depth) + (chop_depth * pims_slew_state[0]);
        write_stereo(out_buf, i, in_sample * mod_gain);
    }
    g_dac_dirty.store(true);
}

// --- ENGINE 1: RINGO ---
static float fm_phase = 0.0f;
static float ringo_fb = 0.0f;
static float ringo_env = 0.0f;           // follower on the resonator output, drives the input AGC
static float ringo_ratio_prev[MAX_MODES] = {1.0f, 2.0f, 3.0f, 4.0f};   // last chunk's mode ratios (for the per-sample ramp)
static bool  ringo_ratio_init = false;

// Mode-frequency ratios (relative to the fundamental) for each of the 6 models; f_struct = STRUCTURE knob 0..1.
static void ringo_model_ratios(int model, float f_struct, float* r) {
    switch (model) {
        case 0:  r[0]=1.0f; r[1]=2.0f+f_struct*0.1f;   r[2]=3.0f+f_struct*0.2f;  r[3]=4.0f+f_struct*0.3f;   break;
        case 1:  r[0]=1.0f; r[1]=1.5f+f_struct*0.05f;  r[2]=2.0f;                r[3]=2.5f+f_struct*0.1f;   break;
        case 2:  r[0]=1.0f; r[1]=3.0f*f_struct;        r[2]=5.0f*f_struct;       r[3]=7.0f*f_struct;        break;
        case 3:  r[0]=1.0f; r[1]=1.414f*f_struct;      r[2]=1.732f*f_struct;     r[3]=2.236f*f_struct;      break;
        case 4:  r[0]=1.0f; r[1]=1.005f+f_struct*0.5f; r[2]=2.01f;               r[3]=3.02f+f_struct;       break;
        default: r[0]=1.0f; r[1]=1.25f;                r[2]=1.5f;                r[3]=1.875f+f_struct*0.125f; break;
    }
}
void process_ringo_chunk(int32_t* in_buf, int32_t* out_buf) {
    int  model  = ringo_model_idx.load();
    bool frozen = is_frozen.load();
    const bool struck = trigger_strike.exchange(false);

    // chunk-rate decisions use the target (end-of-chunk) values
    float f_struct = sp_tgt[1];
    float f_bright = sp_tgt[2];

    // MODEL MORPH (P5): crossfade the mode ratios (and the FM wobble) from this model toward the NEXT one
    // (wrapping 5 -> 0). 0 = the selected model exactly; 1 = the next model; in between = inharmonic hybrids.
    const float f_morph = sp_tgt[4];
    const int   model_b = (model + 1) % 6;
    float ra[MAX_MODES], rb[MAX_MODES], ratios[MAX_MODES];
    ringo_model_ratios(model,   f_struct, ra);
    ringo_model_ratios(model_b, f_struct, rb);
    for (int m = 0; m < MAX_MODES; m++) ratios[m] = ra[m] + f_morph * (rb[m] - ra[m]);
    const float fm_depth = 0.15f * (((model == 4) ? (1.0f - f_morph) : 0.0f) + ((model_b == 4) ? f_morph : 0.0f));
    if (!ringo_ratio_init) { for (int m = 0; m < MAX_MODES; m++) ringo_ratio_prev[m] = ratios[m]; ringo_ratio_init = true; }

    // Per-mode decay: brightness = longer ring, higher modes die faster.
    float tau0 = 0.05f * powf(80.0f, f_bright);          // 0.05 .. 4 s
    float fa = pitch_hz(32.0f, 32.0f, sp[0]);            // freq at chunk start
    float fb = pitch_hz(32.0f, 32.0f, sp_tgt[0]);        // freq at chunk end

    float damp[MAX_MODES], in_gain[MAX_MODES];
    float c_cur[MAX_MODES], c_step[MAX_MODES];
    for (int m = 0; m < MAX_MODES; m++) {
        float tau_m = tau0 / (1.0f + (float)m * (1.0f - f_bright) * 1.5f);
        damp[m] = frozen ? 0.99999f : expf(-1.0f / (tau_m * (float)SAMPLE_RATE));

        float mf0 = fminf(fa * ringo_ratio_prev[m], 12000.0f);   // ramp from last chunk's ratio: no steps when STRUCTURE/MORPH move
        float mf1 = fminf(fb * ratios[m], 12000.0f);
        // magic-circle coefficient: 2*sin(pi*f/fs)  (the old sin(2*pi*f/fs) detuned the top end)
        float c0 = 2.0f * sinf((float)M_PI * mf0 / (float)SAMPLE_RATE);
        float c1 = 2.0f * sinf((float)M_PI * mf1 / (float)SAMPLE_RATE);
        c_cur[m]  = c0;
        c_step[m] = (c1 - c0) * (1.0f / (float)CHUNK_SAMPLES);

        // Level normalisation: broadband input gives roughly the same output
        // level whatever the decay time (tune kRingoInGain to taste).
        const float kRingoInGain = 0.7f;
        in_gain[m] = sqrtf(1.0f - damp[m]) / fmaxf(c1, 0.02f) * kRingoInGain;
    }

    for (int m = 0; m < MAX_MODES; m++) ringo_ratio_prev[m] = ratios[m];

    // STRIKE: put energy straight into the modes. Level is the same whatever the decay time
    // (the old noise burst went through the decay-dependent input gain, so long ring-outs were
    // almost silent and short ones were loud). Brightness tilts the energy toward the upper modes.
    if (struck) {
        const float tilt = 0.5f + (1.0f - f_bright) * 2.0f;
        for (int m = 0; m < MAX_MODES; m++)
            ringo_modes[m].state1 += 0.18f * powf(1.0f / (float)(m + 1), tilt) * (0.6f + 0.4f * rnd01());
    }

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float f_pos  = sp[3];
        float f_fdbk = sp[5];
        float f_mix  = sp[6];

        float in_sample = dc_block(in_to_float(in_buf[i]));

        // INPUT AGC. A high-Q resonator driven at its own frequency has a gain of ~1/sqrt(1-damp)
        // (hundreds x for long decays), so any note that matched a mode used to slam into the
        // limiter. The follower on the resonator output pulls the input gain down steeply once
        // the modes get loud; quiet signals are untouched. The feedback path goes through the
        // same gain, so it can sustain and self-oscillate but can no longer run away.
        const float e  = ringo_env * (1.0f / 0.05f);        // ringo_env follows |x| (~0.64 x peak), so this is a ~0.08 peak knee
        const float e2 = e * e;
        const float agc = 1.0f / (1.0f + e2 * e2);
        float excitation = (in_sample + ringo_fb * f_fdbk * 0.5f) * agc;
        excitation = soft_limit(excitation);

        float fm_mul = 1.0f;
        if (fm_depth > 0.001f) {
            fm_phase += 0.001f * (1.0f + f_pos * 10.0f);
            if (fm_phase > 1.0f) fm_phase -= 1.0f;
            fm_mul = 1.0f + fm_depth * sinf(TWO_PI_F * fm_phase);
        }

        float pos_gain = 0.25f + f_pos * 0.75f;
        float modal_sum = 0.0f;
        for (int m = 0; m < MAX_MODES; m++) {
            c_cur[m] += c_step[m];
            float c = c_cur[m] * fm_mul;
            ringo_modes[m].state1 += c * (excitation * pos_gain * in_gain[m] - ringo_modes[m].state2);
            ringo_modes[m].state2 += c * ringo_modes[m].state1;
            ringo_modes[m].state1 *= damp[m];
            ringo_modes[m].state2 *= damp[m];
            modal_sum += ringo_modes[m].state2 * (1.0f / (float)(m + 1));
        }
        {
            const float a = fabsf(modal_sum);
            ringo_env += ((a > ringo_env) ? 0.02f : 0.0004f) * (a - ringo_env);   // ~1 ms attack, ~50 ms release
        }
        if (!(fabsf(modal_sum) < 8.0f)) {                       // safety net: NaN / blow-up -> reset the modes
            for (int m = 0; m < MAX_MODES; m++) ringo_modes[m].state1 = ringo_modes[m].state2 = 0.0f;
            modal_sum = 0.0f; ringo_env = 0.0f;
        }
        ringo_fb = soft_limit(modal_sum);

        float dg, wg;
        xfade(f_mix, dg, wg);
        write_stereo(out_buf, i, in_sample * dg + (modal_sum * 2.0f) * wg);
    }
}

// --- ENGINE 2: CLODS ---
static int   clods_spawn_countdown = 0;
static float clods_norm = 1.0f;
static float clods_loop_len_s = 24000.0f;
static float clods_last_wet = 0.0f;

static void spawn_grain(float pos, float size_param, float pitch_param, float texture_param, int submode, bool frozen) {
    for (int i = 0; i < MAX_GRAINS; i++) {
        if (grains[i].active) continue;

        float size_ms = (submode == 2) ? (120.0f + size_param * 600.0f) : (30.0f + (size_param * 570.0f));
        int size = (int)((size_ms / 1000.0f) * SAMPLE_RATE);

        float semitones = (pitch_param - 0.5f) * 24.0f;
        float jitter_scale = (submode == 2) ? (texture_param * 12.0f) : (texture_param * 4.0f);
        float jitter = rnd_bi() * 0.5f * jitter_scale;
        float ratio = powf(2.0f, (semitones + jitter) / 12.0f);

        int offset = (int)(pos * (MAX_BUFFER_LEN - 4800)) + 2400;

        // Keep the read head from overtaking the write head (click) or from
        // running past the oldest data in the buffer.
        if (!frozen) {
            float max_len = (float)size;
            if (ratio > 1.001f)      max_len = (float)(offset - 64) / (ratio - 1.0f);
            else if (ratio < 0.999f) max_len = (float)(MAX_BUFFER_LEN - 64 - offset) / (1.0f - ratio);
            if ((float)size > max_len) size = (int)max_len;
        }
        if (size < 64) size = 64;

        int start_idx = wrap_idx(write_head - offset, MAX_BUFFER_LEN);

        grains[i].size_samples   = size;
        grains[i].current_sample = 0;
        grains[i].pitch_ratio    = ratio;
        grains[i].playback_head  = (float)start_idx;
        grains[i].active         = true;          // set LAST
        break;
    }
}

void process_clods_chunk(int32_t* in_buf, int32_t* out_buf) {
    if (trigger_dsp_clear.load()) {
        memset(record_buffer, 0, sizeof(record_buffer));
        for (int g = 0; g < MAX_GRAINS; g++) grains[g].active = false;
        write_head = 0;
        dc_offset = 0.0f;
        clods_last_wet = 0.0f;
        trigger_dsp_clear.store(false);
    }

    int  submode = clods_submode.load();
    bool frozen  = is_frozen.load();

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float p_size = sp[1];
        float p_fdbk = sp[5];
        float p_mix  = sp[6];

        float in_sample = dc_block(in_to_float(in_buf[i]));

        // grain scheduler lives in the audio engine now (sample-accurate, no cross-core race)
        if (--clods_spawn_countdown <= 0) {
            float interval_ms = 10.0f + (1.0f - sp[3]) * 120.0f;
            clods_spawn_countdown = (int)(interval_ms * 0.001f * (float)SAMPLE_RATE * (0.7f + 0.6f * rnd01()));
            spawn_grain(sp[0], sp[1], sp[2], sp[4], submode, frozen);
        }

        float write_val = soft_limit(in_sample + (clods_last_wet * p_fdbk * 0.75f));
        if (!frozen) {
            record_buffer[write_head] = write_val;
            if (++write_head >= MAX_BUFFER_LEN) write_head = 0;
        }

        float granular_out = 0.0f;
        int active_count = 0;

        if (submode == 1) {
            float target_len = 2400.0f + p_size * (float)(MAX_BUFFER_LEN - 4800);
            clods_loop_len_s += 0.0005f * (target_len - clods_loop_len_s);   // glide, no zipper
            granular_out = read_behind_hermite(record_buffer, MAX_BUFFER_LEN, write_head, clods_loop_len_s);
            active_count = 1;
        }

        for (int g = 0; g < MAX_GRAINS; g++) {
            if (!grains[g].active) continue;

            float phase  = (float)grains[g].current_sample / (float)grains[g].size_samples;
            float window = hann_lookup(phase);

            float head = grains[g].playback_head;
            int   i0 = (int)head;
            float fr = head - (float)i0;
            int   i1 = i0 + 1; if (i1 >= MAX_BUFFER_LEN) i1 = 0;
            float sample = record_buffer[i0] * (1.0f - fr) + record_buffer[i1] * fr;

            granular_out += sample * window;
            active_count++;

            head += grains[g].pitch_ratio;
            if (head >= (float)MAX_BUFFER_LEN) head -= (float)MAX_BUFFER_LEN;
            grains[g].playback_head = head;

            if (++grains[g].current_sample >= grains[g].size_samples) grains[g].active = false;
        }

        // smoothed 1/sqrt(N) normalisation (the instant version amplitude-modulated the output)
        float norm_target = (active_count > 0) ? 1.0f / sqrtf((float)active_count) : 1.0f;
        clods_norm += 0.002f * (norm_target - clods_norm);
        granular_out *= clods_norm;
        if (granular_out != granular_out) granular_out = 0.0f;

        clods_last_wet = granular_out;

        float dg, wg;
        xfade(p_mix, dg, wg);
        write_stereo(out_buf, i, in_sample * dg + granular_out * wg);
    }
}

// --- ENGINE 3: QUANTIZER (quantises CV 1/2, drives a small voice) ---
static float quant_pitch_s = 12.0f;
static float quant_ph[2] = {0.0f, 0.0f};
static float quant_fm_ph = 0.0f;
static int   quant_last_note = -1;
static int   quant_gate_timer = 0;
void process_quantizer_chunk(int32_t* in_buf, int32_t* out_buf) {
    int root  = (root_note.load() + (int)(sp_tgt[0] * 11.99f)) % 12;
    int chord = selected_chord_type.load();
    int max_semi = 12 + (int)floorf(sp_tgt[2] * 3.0f) * 12;     // 12..48 semitones
    int clen = chord_lengths[chord];

    // CV inputs are bipolar -1..+1 -> 0..max_semi
    float cv_a = clamp((g_cv[0].load() + 1.0f) * 0.5f, 0.0f, 1.0f);
    float cv_b = clamp((g_cv[1].load() + 1.0f) * 0.5f, 0.0f, 1.0f);
    int note_a = quantize_semitone((int)roundf(cv_a * (float)max_semi), root, chord, max_semi);
    int note_b = quantize_semitone((int)roundf(cv_b * (float)max_semi), root, chord, max_semi);

    if (note_a != quant_last_note) {
        quant_last_note = note_a;
        quant_gate_timer = SAMPLE_RATE / 100;                   // 10 ms gate/trigger
    }

    // CHORD SPREAD picks which chord/scale degree is stacked on top
    int degree = (int)(sp_tgt[1] * ((float)clen - 0.01f));
    float interval2 = (float)chord_intervals[chord][degree];

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float glide_k = 1.0f / (1.0f + sp[3] * sp[3] * 20000.0f);
        float detune  = 1.0f + (sp[4] - 0.5f) * 0.02f;
        float fm_idx  = sp[5] * 3.0f;
        float mix     = sp[6];

        float in_sample = dc_block(in_to_float(in_buf[i]));

        quant_pitch_s += glide_k * ((float)note_a - quant_pitch_s);
        float f1 = 32.7f * exp2f(quant_pitch_s * (1.0f / 12.0f));
        float f2 = f1 * exp2f(interval2 * (1.0f / 12.0f)) * detune;

        quant_fm_ph += (f1 * 2.0f) / (float)SAMPLE_RATE;  if (quant_fm_ph >= 1.0f) quant_fm_ph -= 1.0f;
        quant_ph[0] += f1 / (float)SAMPLE_RATE;           if (quant_ph[0] >= 1.0f) quant_ph[0] -= 1.0f;
        quant_ph[1] += f2 / (float)SAMPLE_RATE;           if (quant_ph[1] >= 1.0f) quant_ph[1] -= 1.0f;

        float v1 = sinf(TWO_PI_F * quant_ph[0] + fm_idx * sinf(TWO_PI_F * quant_fm_ph));
        float v2 = sinf(TWO_PI_F * quant_ph[1]);
        float voice = 0.35f * (v1 + v2);

        float dg, wg;
        xfade(mix, dg, wg);
        write_stereo(out_buf, i, in_sample * dg + voice * wg);
    }

    if (quant_gate_timer > 0) quant_gate_timer -= CHUNK_SAMPLES;
    g_dac_out[0].store((uint16_t)(clamp((float)note_a / 48.0f, 0.0f, 1.0f) * 4095.0f));
    g_dac_out[1].store((uint16_t)(clamp((float)note_b / 48.0f, 0.0f, 1.0f) * 4095.0f));
    g_dac_out[2].store(quant_gate_timer > 0 ? 4095 : 0);
    g_dac_out[3].store((uint16_t)(clamp(quant_pitch_s / 48.0f, 0.0f, 1.0f) * 4095.0f));
    g_dac_dirty.store(true);
}

// --- ENGINE 4: QUIRKS ---
//  in -> DC block -> DRIVE (bounded saturator) -> y
//    route 0  SIGN : crossfade  -y  <->  |y|            (BLEND)
//    route 1  LOGIC: crossfade  min(y,thr) <-> max(y,thr)  (BLEND, THRESHOLD)
//    route 2  S&H  : sample & hold of y (or noise), blended with noise (NOISE LEVEL)
//  wet -> remove the DC the rectifier creates (routes 0/1) -> soft limit -> LPF -> LINEAR mix with dry.
//  Every stage is bounded to +/-1, so the output can only exceed full scale if the input does.
//  (The old version multiplied by up to 3.5x with nothing compensating, let the rectifier's DC
//   through, used max+min which is simply a+b, added the noise twice, and summed correlated
//   dry+wet with an equal-power law = +3 dB.)
static float quirks_held_val = 0.0f;
static float quirks_clock_ph = 0.0f;
static float quirks_lpf = 0.0f;
static float quirks_wdc = 0.0f;          // DC estimate of the wet path
static float quirks_u1 = 0.0f;           // previous input of the |.| stage (anti-aliasing state)
static float quirks_cut_s = 0.5f;        // smoothed LPF coefficient

// |u| with first-order antiderivative anti-aliasing (F(u) = u|u|/2). The sharp corner of
// |x| used to alias badly on anything bright.
static inline float adaa_abs(float u, float u1) {
    const float d = u - u1;
    if (fabsf(d) < 1e-4f) return fabsf(0.5f * (u + u1));
    return (u * fabsf(u) - u1 * fabsf(u1)) / (2.0f * d);
}

void process_quirks_chunk(int32_t* in_buf, int32_t* out_buf) {
    const int  route     = quirks_audio_route.load();
    const bool use_audio = quirks_sh_use_audio.load();

    // S&H clock: exponential 0.1 Hz .. 2 kHz (was linear 1..2001 Hz: nearly the whole knob was fast)
    const float sh_rate = 0.1f * powf(20000.0f, sp_tgt[0]);
    // LPF: knob up = brighter now (it was inverted). 200 Hz .. 20 kHz, exponential.
    const float fc = 200.0f * powf(100.0f, sp_tgt[4]);
    const float cut_target = fminf(1.0f - expf(-TWO_PI_F * fc / (float)SAMPLE_RATE), 0.95f);

    uint16_t dac_rect = 2048, dac_max = 2048, dac_min = 2048, dac_sh = 2048;

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        const float drive      = sp[1];
        const float rect_blend = sp[2];
        const float noise_lvl  = sp[3];
        const float thr        = (sp[5] - 0.5f) * 0.5f;           // +/-0.25
        const float mix        = sp[6];

        const float x = dc_block(in_to_float(in_buf[i]));

        // DRIVE: clean at 0, increasingly saturated toward 1; bounded to +/-1 for any input in range
        const float g   = 1.0f + 3.0f * drive;
        const float sat = fast_tanh(g * x) / fast_tanh(g);
        const float y   = x + (sat - x) * drive;

        // |.| stage shared by routes 0 and 1:  max = (y+t+|y-t|)/2,  min = (y+t-|y-t|)/2
        const float t  = (route == 1) ? thr : 0.0f;
        const float u  = y - t;
        const float au = adaa_abs(u, quirks_u1);
        quirks_u1 = u;

        // S&H clock + manual trigger
        quirks_clock_ph += sh_rate * (1.0f / (float)SAMPLE_RATE);
        bool fire = false;
        if (quirks_clock_ph >= 1.0f) { quirks_clock_ph -= 1.0f; fire = true; }
        if (quirks_sh_trig.load(std::memory_order_relaxed) && quirks_sh_trig.exchange(false)) fire = true;
        if (fire) {
            const float nz = rnd_bi();
            // noise is added once, bounded: audio source crossfades toward noise, internal source is noise
            quirks_held_val = use_audio ? (y * (1.0f - noise_lvl) + nz * noise_lvl * 0.7f)
                                       : (nz * noise_lvl * 0.5f);
        }

        if (i == 0) {                                             // CV outputs (once per chunk, as before)
            const float cvt = (sp[5] - 0.5f) * 0.5f;
            dac_rect = (uint16_t)(clamp(fabsf(y), 0.0f, 1.0f) * 4095.0f);
            dac_max  = (uint16_t)(clamp((fmaxf(y, cvt) + 1.0f) * 0.5f, 0.0f, 1.0f) * 4095.0f);
            dac_min  = (uint16_t)(clamp((fminf(y, cvt) + 1.0f) * 0.5f, 0.0f, 1.0f) * 4095.0f);
            dac_sh   = (uint16_t)(clamp((quirks_held_val + 1.0f) * 0.5f, 0.0f, 1.0f) * 4095.0f);
        }

        float wet;
        if (route == 0) {
            wet = -y * (1.0f - rect_blend) + au * rect_blend;
        } else if (route == 1) {
            const float lo = 0.5f * (y + t - au), hi = 0.5f * (y + t + au);
            wet = lo * (1.0f - rect_blend) + hi * rect_blend;
        } else {
            wet = quirks_held_val;
        }
        if (route != 2) {                                         // rectifier / clipper create DC -> remove it
            quirks_wdc += 0.0015f * (wet - quirks_wdc);             // ~11 Hz high-pass
            wet = (wet - quirks_wdc) * 1.25f;                      // AC part of |x| is quieter; small makeup
        }
        wet = soft_limit(wet);

        quirks_cut_s += 0.01f * (cut_target - quirks_cut_s);
        quirks_lpf += quirks_cut_s * (wet - quirks_lpf);

        write_stereo(out_buf, i, x * (1.0f - mix) + quirks_lpf * mix);   // linear: dry and wet are correlated
    }
    g_dac_out[0].store(dac_rect);
    g_dac_out[1].store(dac_max);
    g_dac_out[2].store(dac_min);
    g_dac_out[3].store(dac_sh);
    g_dac_dirty.store(true);
}

// --- ENGINE 5: PLAIDS ---
static float plaids_ph[2]  = {0.0f, 0.0f};     // [0] main, [1] FM modulator
static float plaids_cph[4] = {0.0f};           // chord voices (own phases now)
static float plaids_env = 1.0f;
static float plaids_amp_s = 1.0f;
void process_plaids_chunk(int32_t* in_buf, int32_t* out_buf) {
    float f_decay = 0.9990f + sp_tgt[5] * 0.00095f;
    int   model   = plaids_model.load();
    bool  use_lpg = plaids_lpg_mode.load();

    if (trigger_strike.exchange(false)) plaids_env = 1.0f;

    float freq  = pitch_hz(32.7f, 32.0f, sp[0]);
    float freqB = pitch_hz(32.7f, 32.0f, sp_tgt[0]);
    float f_inc = (freqB - freq) * (1.0f / (float)CHUNK_SAMPLES);

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float f_harm  = sp[1];
        float f_timb  = sp[2];
        float f_morph = sp[3];
        float f_fm    = sp[4];
        float f_level = sp[6];

        freq += f_inc;
        float dt = freq / (float)SAMPLE_RATE;

        float in_sample = dc_block(in_to_float(in_buf[i]));

        float dt_eff = clamp(dt * (1.0f + in_sample * f_fm * 2.0f), 0.00001f, 0.45f);
        plaids_ph[0] += dt_eff;
        if (plaids_ph[0] >= 1.0f) plaids_ph[0] -= 1.0f;

        float p = plaids_ph[0];
        float sig = 0.0f;

        if (model == 0) {
            float tri = (p < 0.5f) ? (4.0f * p - 1.0f) : (3.0f - 4.0f * p);
            float saw = (2.0f * p - 1.0f) - poly_blep(p, dt_eff);
            float pw  = 0.1f + f_timb * 0.8f;
            float sq  = blep_square(p, dt_eff, pw);
            sig = (tri * (1.0f - f_harm) + saw * f_harm) * (1.0f - f_morph) + sq * f_morph;
        } else if (model == 1) {
            float d  = 0.05f + (1.0f - f_timb) * 0.45f;
            float pd = (p < d) ? (0.5f * p / d) : (0.5f + 0.5f * (p - d) / (1.0f - d));
            sig = sinf(TWO_PI_F * pd);
        } else if (model == 2) {
            float ratio = 1.0f + floorf(f_harm * 7.0f) + f_morph * 0.5f;
            plaids_ph[1] += dt_eff * ratio;
            if (plaids_ph[1] >= 1.0f) plaids_ph[1] -= 1.0f;
            // FM index falls with pitch so high notes don't alias into garbage
            float idx = f_timb * 6.0f / (1.0f + freq * 0.004f);
            sig = sinf(TWO_PI_F * p + sinf(TWO_PI_F * plaids_ph[1]) * idx);
        } else {
            const float c_ratios[4] = { 1.0f, 1.25f + f_harm * 0.08f, 1.5f, 1.875f + f_morph * 0.125f };
            for (int c = 0; c < 4; c++) {
                plaids_cph[c] += dt * c_ratios[c];
                if (plaids_cph[c] >= 1.0f) plaids_cph[c] -= 1.0f;
                sig += sinf(TWO_PI_F * plaids_cph[c]) * 0.25f;
            }
        }

        plaids_env *= f_decay;
        float amp = use_lpg ? plaids_env : 1.0f;
        plaids_amp_s += 0.02f * (amp - plaids_amp_s);      // ~1 ms: removes retrigger click
        write_stereo(out_buf, i, sig * plaids_amp_s * (0.2f + f_level * 0.8f));
    }
}

// --- ENGINE 6: BRAINS ---
static float brains_ph[5] = {0.0f};
static float brains_lfo_ph = 0.0f;
static float brains_accent = 0.0f;
void process_brains_chunk(int32_t* in_buf, int32_t* out_buf) {
    (void)in_buf;
    int  model = brains_model.load();
    bool crush = brains_bitcrush.load();

    if (trigger_strike.exchange(false)) brains_accent = 1.0f;   // consume flag; short level accent

    float base = pitch_hz(32.7f, 32.0f, sp[0]);
    float baseB = pitch_hz(32.7f, 32.0f, sp_tgt[0]);
    float b_inc = (baseB - base) * (1.0f / (float)CHUNK_SAMPLES);

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float f_timb  = sp[1];
        float f_color = sp[2];
        float f_mod_d = sp[3];
        float f_mod_r = 0.5f + sp[4] * 25.0f;
        float f_drive = sp[5];
        float f_level = sp[6];

        base += b_inc;

        brains_lfo_ph += f_mod_r / (float)SAMPLE_RATE;
        if (brains_lfo_ph >= 1.0f) brains_lfo_ph -= 1.0f;
        float lfo = sinf(TWO_PI_F * brains_lfo_ph) * f_mod_d * 0.15f;

        float freq = base * (1.0f + lfo);
        float dt   = clamp(freq / (float)SAMPLE_RATE, 0.00001f, 0.2f);

        brains_ph[0] += dt;
        if (brains_ph[0] >= 1.0f) brains_ph[0] -= 1.0f;
        float dt_sub = dt * 0.5f;
        brains_ph[4] += dt_sub;
        if (brains_ph[4] >= 1.0f) brains_ph[4] -= 1.0f;

        float p = brains_ph[0];
        float sig = 0.0f;

        if (model == 0) {
            float notch = 0.05f + clamp(f_timb + lfo, 0.0f, 1.0f) * 0.4f;
            float lo = -1.0f + f_color;
            sig = (p < notch) ? lo : (2.0f * p - 1.0f);
            // band-limit the wrap edge and the notch edge
            sig -= 0.5f * (2.0f - f_color) * poly_blep(p, dt);
            float tn = p - notch; if (tn < 0.0f) tn += 1.0f;
            sig += 0.5f * (2.0f * notch - f_color) * poly_blep(tn, dt);
        } else if (model == 1) {
            sig = (2.0f * p - 1.0f) - poly_blep(p, dt);
            for (int s = 1; s <= 3; s++) {
                float dts = dt * (1.0f + (float)s * 0.008f * (0.1f + f_timb));
                brains_ph[s] += dts;
                if (brains_ph[s] >= 1.0f) brains_ph[s] -= 1.0f;
                sig += (2.0f * brains_ph[s] - 1.0f) - poly_blep(brains_ph[s], dts);
            }
            sig *= 0.25f;
        } else if (model == 2) {
            // continuous ratios (the old floorf() steps clicked when you turned the knob);
            // the (1-p) window keeps the waveform continuous at the wrap.
            float r1 = 2.0f + f_timb * 6.0f;
            float r2 = 5.0f + f_color * 10.0f;
            sig = (1.0f - p) * (sinf(TWO_PI_F * p * r1) * 0.6f + sinf(TWO_PI_F * p * r2) * 0.4f);
        } else {
            brains_ph[1] += dt * (1.414f + f_color * 2.0f);
            if (brains_ph[1] >= 1.0f) brains_ph[1] -= 1.0f;
            float idx = f_timb * 8.0f / (1.0f + freq * 0.004f);
            sig = sinf(TWO_PI_F * p + sinf(TWO_PI_F * brains_ph[1]) * idx);
        }

        float sub_sig = blep_square(brains_ph[4], dt_sub, 0.5f);
        sig = sig * 0.75f + sub_sig * 0.25f;
        sig = fast_tanh(sig * (1.0f + f_drive * 2.0f));
        if (crush) sig = floorf(sig * 8.0f) / 8.0f;

        brains_accent *= 0.9995f;
        float out_mono = sig * (0.2f + f_level * 0.8f) * (1.0f + 0.5f * brains_accent);
        write_stereo(out_buf, i, out_mono);
    }
}

// --- ENGINE 7: MUMBLES ---
void process_mumbles_chunk(int32_t* in_buf, int32_t* out_buf) {
    (void)in_buf;
    int   scale_idx = (int)(sp_tgt[4] * 7.99f);
    float decay     = 0.9990f + sp_tgt[5] * 0.00095f;
    int   loop_len  = mumbles_loop_len.load();

    uint16_t dac_pitch = 2048;
    float osc_dt = 0.0f;

    // semitone-accurate pitch (the old (q+1)*1.5 mapping compressed the scale to 75%)
    auto update_note = [&]() {
        float active_val = mumbles_step_cache[mumbles_step_ptr % loop_len];
        int q = quantize_semitone((int)roundf(clamp(active_val, 0.0f, 1.0f) * 36.0f), 0, scale_idx, 36);
        float note_freq = 65.4f * exp2f((float)q * (1.0f / 12.0f));
        osc_dt = note_freq / (float)SAMPLE_RATE;
        dac_pitch = (uint16_t)(clamp((float)q / 48.0f, 0.0f, 1.0f) * 4095.0f);
    };
    update_note();

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float rate    = 0.5f + sp[0] * 20.0f;
        float spread  = sp[1];
        float deja_vu = sp[2];
        float bias    = sp[3];
        float level   = sp[6];

        mumbles_clock_ph += rate / (float)SAMPLE_RATE;
        if (mumbles_clock_ph >= 1.0f || trigger_strike.load()) {
            if (mumbles_clock_ph >= 1.0f) mumbles_clock_ph -= 1.0f;
            trigger_strike.store(false);
            mumbles_env = 1.0f;

            if (rnd01() > deja_vu) {
                float r = (rnd01() - 0.5f) * spread * 2.0f + bias;
                mumbles_step_cache[mumbles_step_ptr % 16] = clamp(r, 0.0f, 1.0f);
            }
            mumbles_step_ptr = (mumbles_step_ptr + 1) % loop_len;
            update_note();                                   // pitch changes on the trigger, not next chunk
        }

        mumbles_osc_ph += osc_dt;
        if (mumbles_osc_ph >= 1.0f) mumbles_osc_ph -= 1.0f;
        mumbles_env *= decay;
        mumbles_amp_s += 0.02f * (mumbles_env - mumbles_amp_s);   // ~1 ms attack: no retrigger click

        float voice = sinf(TWO_PI_F * mumbles_osc_ph + 1.5f * mumbles_amp_s * sinf(2.0f * TWO_PI_F * mumbles_osc_ph));
        write_stereo(out_buf, i, voice * mumbles_amp_s * (0.2f + level * 0.8f));
    }
    uint16_t gate_dac = (mumbles_clock_ph < 0.5f) ? 4095 : 0;
    g_dac_out[0].store(dac_pitch);
    g_dac_out[1].store(dac_pitch);
    g_dac_out[2].store(gate_dac);
    g_dac_out[3].store((uint16_t)(clamp(mumbles_env, 0.0f, 1.0f) * 4095.0f));
    g_dac_dirty.store(true);
}

// --- ENGINE 8: WRAPS ---
static float wraps_fb = 0.0f;
static float wraps_x1 = 0.0f, wraps_c1 = 1.0f, wraps_s2_1 = 0.0f;   // ADAA history
void process_wraps_chunk(int32_t* in_buf, int32_t* out_buf) {
    int algo = wraps_algo.load();

    float cf  = pitch_hz(32.7f, 32.0f, sp[0]);
    float cfB = pitch_hz(32.7f, 32.0f, sp_tgt[0]);
    float cf_inc = (cfB - cf) * (1.0f / (float)CHUNK_SAMPLES);

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float fold_depth  = 1.0f + sp[1] * 5.0f;
        float mod_gain    = 0.5f + sp[2] * 2.5f;
        float timbre      = sp[3];
        float car_balance = sp[4];
        float fdbk        = sp[5] * 0.4f;
        float mix         = sp[6];

        cf += cf_inc;
        float in_sample = dc_block(in_to_float(in_buf[i]));

        wraps_carrier_ph += cf / (float)SAMPLE_RATE;
        if (wraps_carrier_ph >= 1.0f) wraps_carrier_ph -= 1.0f;

        float int_osc = sinf(TWO_PI_F * wraps_carrier_ph);
        float carrier = int_osc * (1.0f - car_balance) + in_sample * car_balance;
        float modulator = (in_sample * mod_gain) + (wraps_fb * fdbk);

        float out_sig = 0.0f;
        if (algo == 0) {
            // First-order ADAA sine/cos folder: far less aliasing than sinf(x) directly.
            float x  = (carrier + modulator) * fold_depth;
            float c0 = cosf(x);
            float s2 = sinf(2.0f * x);
            float d  = x - wraps_x1;
            if (fabsf(d) > 1e-3f) {
                out_sig = ((-c0 + wraps_c1) + timbre * 0.5f * (s2 - wraps_s2_1)) / d;
            } else {
                float xm = 0.5f * (x + wraps_x1);
                out_sig = sinf(xm) + timbre * cosf(2.0f * xm);
            }
            wraps_x1 = x; wraps_c1 = c0; wraps_s2_1 = s2;
        } else if (algo == 1) {
            float idx = clamp(modulator * fold_depth * 2.0f, -8.0f, 8.0f);   // bounded FM index
            out_sig = sinf(TWO_PI_F * wraps_carrier_ph + idx);
        } else if (algo == 2) {
            out_sig = carrier * (modulator + (1.0f - timbre) * 0.5f) * fold_depth;
        } else {
            out_sig = (carrier + modulator) * sinf(TWO_PI_F * wraps_carrier_ph * (2.0f + timbre * 3.0f));
        }

        wraps_fb = fast_tanh(out_sig * 0.75f);
        float dg, wg;
        xfade(mix, dg, wg);
        write_stereo(out_buf, i, in_sample * dg + wraps_fb * wg);
    }
}

// --- ENGINE 9: TIDDLY ---
void process_tiddly_chunk(int32_t* in_buf, int32_t* out_buf) {
    int mode = tiddly_shape_mode.load();

    float af  = pitch_hz(55.0f, 16.0f, sp[0]);
    float afB = pitch_hz(55.0f, 16.0f, sp_tgt[0]);
    float af_inc = (afB - af) * (1.0f / (float)CHUNK_SAMPLES);

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float rate       = 0.2f + sp[0] * 25.0f;
        float shape      = sp[1];
        float fold_amt   = sp[3];
        float fm_depth   = sp[4];
        float audio_mode = sp[5];
        float level      = sp[6];
        // SMOOTHNESS (P3): one-pole slew on the envelope that drives the VCA, the FM and CV out 1.
        // 0 = ~0.4 ms (just de-clicks), 1 = ~45 ms (steps become ramps, a fast LFO turns into a soft swell).
        const float smooth_a = 0.05f * expf(-4.6f * sp[2]);

        af += af_inc;
        float in_sample = dc_block(in_to_float(in_buf[i]));

        tiddly_phase += rate / (float)SAMPLE_RATE;
        if (tiddly_phase >= 1.0f) tiddly_phase -= 1.0f;

        float env = 0.0f;
        float split = 0.05f + shape * 0.90f;
        if (mode == 0) {
            if (tiddly_phase < split) env = tiddly_phase / split;
            else { float r = 1.0f - ((tiddly_phase - split) / (1.0f - split)); env = r * r; }
        } else if (mode == 1) {
            env = (tiddly_phase < split) ? (tiddly_phase / split) : (1.0f - ((tiddly_phase - split) / (1.0f - split)));
        } else {
            env = floorf(tiddly_phase * 8.0f) / 8.0f;
        }

        tiddly_env_s += smooth_a * (env - tiddly_env_s);     // smoothed envelope (also de-clicks the stepped mode)

        if (i == 0) {
            for (int c = 0; c < 4; c++) {
                float ph_c = fmodf(tiddly_phase + (float)c * 0.25f, 1.0f);
                g_dac_out[c].store((uint16_t)(clamp((c == 0) ? tiddly_env_s : ph_c, 0.0f, 1.0f) * 4095.0f));
            }
        }

        tiddly_audio_ph += (af * (1.0f + tiddly_env_s * fm_depth)) / (float)SAMPLE_RATE;
        if (tiddly_audio_ph >= 1.0f) tiddly_audio_ph -= 1.0f;

        // Fold is waveshaping now. The old sinf(phase*fold) was discontinuous at every
        // phase wrap for non-integer fold -> harsh buzz at the oscillator frequency.
        float s = sinf(TWO_PI_F * tiddly_audio_ph);
        float shaped = s + fold_amt * (sinf(s * (1.0f + fold_amt * 5.0f)) - s);
        float sheep_osc = shaped * tiddly_env_s;
        float vca_audio = in_sample * tiddly_env_s * (1.0f + fold_amt);
        float sig = sheep_osc * (1.0f - audio_mode) + vca_audio * audio_mode;

        write_stereo(out_buf, i, sig * (0.2f + level * 0.8f));
    }
    g_dac_dirty.store(true);
}

// --- ENGINE 10: MULTI-FX ---
static float tape_wow_ph = 0.0f;
void process_fx_chunk(int32_t* in_buf, int32_t* out_buf) {
    if (trigger_dsp_clear.load()) {
        memset(record_buffer, 0, sizeof(record_buffer));
        memset(chorus_buf, 0, sizeof(chorus_buf));
        memset(rev_comb_mem, 0, sizeof(rev_comb_mem));
        memset(rev_ap_mem, 0, sizeof(rev_ap_mem));
        for (int k = 0; k < NUM_COMBS; k++) { comb_idx[k] = 0; comb_lp[k] = 0.0f; }
        for (int k = 0; k < NUM_APS; k++)   { ap_idx[k] = 0; }
        write_head = 0;
        chorus_write_idx = 0;
        echo_lp_state = 0.0f;
        dc_offset = 0.0f;
        trigger_dsp_clear.store(false);
    }

    if (fx_trigger_pluck.exchange(false)) {
        fx_pluck_env  = 1.0f;
        fx_pluck_freq = fx_arp_freqs[fx_arp_idx];
        fx_arp_idx    = (fx_arp_idx + 1) % 6;
    }

    int  submode = fx_submode.load();
    bool frozen  = is_frozen.load();

    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
        params_tick();
        float chor_depth = sp[0];
        float lfo_inc    = (0.2f + sp[1] * 4.8f) / (float)SAMPLE_RATE;
        float echo_time  = sp[2];
        float echo_fdbk  = 0.15f + sp[3] * 0.70f;
        float rev_decay  = 0.45f + sp[4] * 0.45f;
        float rev_damp   = 0.20f + sp[5] * 0.65f;
        float mix        = sp[6];
        if (frozen) { echo_fdbk = 0.995f; rev_decay = 0.995f; }

        float target_delay = 1200.0f + (echo_time * echo_time) * 70800.0f;

        float in_sample = dc_block(in_to_float(in_buf[i]));

        if (fx_pluck_env > 0.0005f) {
            fx_pluck_env *= 0.9985f;
            fx_pluck_ph  += fx_pluck_freq / (float)SAMPLE_RATE;
            if (fx_pluck_ph >= 1.0f) fx_pluck_ph -= 1.0f;

            float env_sq = fx_pluck_env * fx_pluck_env;
            float fm_mod = sinf(2.0f * TWO_PI_F * fx_pluck_ph) * (env_sq * 2.5f);
            float pluck_body = sinf(TWO_PI_F * fx_pluck_ph + fm_mod) * env_sq;
            in_sample += pluck_body * 0.6f;
        }

        float dry = in_sample;
        float fx_sig = frozen ? 0.0f : in_sample;

        // 1. CHORUS STAGE (Hermite taps)
        if (submode == 0 || submode == 1) {
            chorus_buf[chorus_write_idx] = fx_sig;
            chorus_lfo_ph += lfo_inc;
            if (chorus_lfo_ph >= 1.0f) chorus_lfo_ph -= 1.0f;

            float lfo1 = sinf(TWO_PI_F * chorus_lfo_ph);
            float lfo2 = -lfo1;

            float d1 = 600.0f + lfo1 * (chor_depth * 200.0f);
            float d2 = 900.0f + lfo2 * (chor_depth * 200.0f);

            float tap1 = read_behind_hermite(chorus_buf, CHORUS_BUF_LEN, chorus_write_idx, d1);
            float tap2 = read_behind_hermite(chorus_buf, CHORUS_BUF_LEN, chorus_write_idx, d2);

            if (++chorus_write_idx >= CHORUS_BUF_LEN) chorus_write_idx = 0;

            float chor_wet = (tap1 + tap2) * 0.4f * chor_depth;
            fx_sig = fx_sig * (1.0f - chor_depth * 0.35f) + chor_wet;
        }

        // 2. TAPE ECHO STAGE (Hermite read, integer/fraction kept separate)
        if (submode == 0 || submode == 2) {
            echo_smooth_delay += 0.0005f * (target_delay - echo_smooth_delay);

            tape_wow_ph += 0.65f / (float)SAMPLE_RATE;
            if (tape_wow_ph >= 1.0f) tape_wow_ph -= 1.0f;
            float wow_mod = sinf(TWO_PI_F * tape_wow_ph) * 10.0f;

            float delay = clamp(echo_smooth_delay + wow_mod, 8.0f, (float)(MAX_BUFFER_LEN - 8));
            float delayed = read_behind_hermite(record_buffer, MAX_BUFFER_LEN, write_head, delay);

            echo_lp_state += (1.0f - rev_damp * 0.6f) * (delayed - echo_lp_state);

            if (!frozen) {
                record_buffer[write_head] = soft_limit(fx_sig + echo_lp_state * echo_fdbk);
            }
            if (++write_head >= MAX_BUFFER_LEN) write_head = 0;

            fx_sig = fx_sig * 0.7f + echo_lp_state * 0.6f;
        }

        // 3. HALL REVERB STAGE (8 combs -> 4 allpasses)
        if (submode == 0 || submode == 3) {
            float rev_in = fx_sig * 0.15f;
            float comb_sum = 0.0f;
            for (int k = 0; k < NUM_COMBS; k++) {
                float* b = &rev_comb_mem[comb_off[k]];
                int len = comb_off[k + 1] - comb_off[k];
                float o = b[comb_idx[k]];
                comb_lp[k] = o * (1.0f - rev_damp) + comb_lp[k] * rev_damp;
                b[comb_idx[k]] = rev_in + comb_lp[k] * rev_decay;
                if (++comb_idx[k] >= len) comb_idx[k] = 0;
                comb_sum += o;
            }
            float x = comb_sum * (1.0f / (float)NUM_COMBS);
            for (int k = 0; k < NUM_APS; k++) {
                float* b = &rev_ap_mem[ap_off[k]];
                int len = ap_off[k + 1] - ap_off[k];
                float bo = b[ap_idx[k]];
                float y = -x + bo;
                b[ap_idx[k]] = x + bo * 0.5f;
                if (++ap_idx[k] >= len) ap_idx[k] = 0;
                x = y;
            }
            fx_sig = fx_sig * 0.55f + x * 0.75f;
        }

        write_stereo(out_buf, i, dry * (1.0f - mix * 0.4f) + fx_sig * mix);
    }
}

// --- AUDIO CHUNK WRAPPER: CPU PROFILING, LEVEL METERING, SCOPE CAPTURE ---
//
// Why the old CPU readout was "all over the place": it showed the load of ONE chunk,
// sampled whenever the OLED happened to redraw, and its peak decayed by 0.5% *per
// chunk* (so it fell to ~nothing within a second). Load is genuinely bursty (Clods grain
// count, mode-switch buffer clears, flash-cache misses from the other core), so a single
// sample is meaningless. Now: average + worst-case over a ~340 ms window, the first
// chunks after a mode change are excluded, and chunks that blow the time budget are
// counted as xruns.
void __not_in_flash_func(process_audio_chunk)(int32_t* in_buf, int32_t* out_buf) {
    static int      last_mode   = -1;
    static int      warmup      = 0;
    static uint32_t acc_us      = 0, max_us = 0, n_win = 0;
    static int32_t  last_out    = 0;
    static float    pk_env      = 0.0f, rms_env = 0.0f;

    const int mode = active_mode.load();
    const bool changed = (mode != last_mode);
    const bool first_ever = (last_mode < 0);
    if (changed) { last_mode = mode; warmup = 6; }

    const uint32_t t0 = time_us_32();

    params_begin_chunk();
    switch (mode) {
        case MODE_PIMS:      process_pims_chunk(in_buf, out_buf);      break;
        case MODE_RINGO:     process_ringo_chunk(in_buf, out_buf);     break;
        case MODE_CLODS:    process_clods_chunk(in_buf, out_buf);    break;
        case MODE_QUANTIZER: process_quantizer_chunk(in_buf, out_buf); break;
        case MODE_QUIRKS:     process_quirks_chunk(in_buf, out_buf);     break;
        case MODE_PLAIDS:    process_plaids_chunk(in_buf, out_buf);    break;
        case MODE_BRAINS:    process_brains_chunk(in_buf, out_buf);    break;
        case MODE_MUMBLES:   process_mumbles_chunk(in_buf, out_buf);   break;
        case MODE_WRAPS:     process_wraps_chunk(in_buf, out_buf);     break;
        case MODE_TIDDLY:     process_tiddly_chunk(in_buf, out_buf);     break;
        case MODE_FX:        process_fx_chunk(in_buf, out_buf);        break;
        default:             process_clods_chunk(in_buf, out_buf);    break;
    }

    const uint32_t dt = time_us_32() - t0;

    // Mode change: crossfade from the last sample of the previous engine over this chunk,
    // so switching modules no longer produces a step/pop.
    if (changed && !first_ever) {
        const float inv = 1.0f / (float)CHUNK_SAMPLES;
        const float lo  = (float)last_out;
        for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
            float t = (float)(i >> 1) * inv;
            int32_t v = (int32_t)((float)out_buf[i] * t + lo * (1.0f - t));
            out_buf[i] = v;
            out_buf[i + 1] = v;
        }
    }
    last_out = out_buf[CHUNK_32BIT_WORDS - 2];

    // Output metering + scope capture (2:1 decimation by averaging adjacent samples)
    uint32_t wr = g_scope_wr.load(std::memory_order_relaxed);
    float pk = 0.0f, sumsq = 0.0f;
    for (int i = 0; i < CHUNK_32BIT_WORDS; i += 4) {
        const int32_t a = out_buf[i], b = out_buf[i + 2];
        g_scope_ring[wr & (SCOPE_RING_LEN - 1)] = (int16_t)((a >> 17) + (b >> 17));
        wr++;
        const float fa = (float)a * (1.0f / 2147483648.0f);
        const float fb = (float)b * (1.0f / 2147483648.0f);
        const float aa = fabsf(fa), ab = fabsf(fb);
        if (aa > pk) pk = aa;
        if (ab > pk) pk = ab;
        sumsq += fa * fa + fb * fb;
    }
    g_scope_wr.store(wr, std::memory_order_release);
    pk_env  = (pk > pk_env * 0.97f) ? pk : pk_env * 0.97f;        // ~170 ms release
    rms_env += 0.3f * (sqrtf(sumsq * (1.0f / (float)CHUNK_SAMPLES)) - rms_env);
    g_out_peak.store(pk_env, std::memory_order_relaxed);
    g_out_rms.store(rms_env, std::memory_order_relaxed);

    // CPU statistics
    if (dt > (uint32_t)(CHUNK_DEADLINE_US * 0.97f)) g_xrun_count.fetch_add(1, std::memory_order_relaxed);
    if (warmup > 0) {
        warmup--;
    } else {
        acc_us += dt;
        if (dt > max_us) max_us = dt;
        if (++n_win >= CPU_WINDOW_CHUNKS) {
            const float scale = 100.0f / CHUNK_DEADLINE_US;
            g_cpu_load_pct.store((float)acc_us / (float)n_win * scale, std::memory_order_relaxed);
            g_cpu_peak_pct.store((float)max_us * scale, std::memory_order_relaxed);
            acc_us = max_us = n_win = 0;
        }
    }
}

void __isr __not_in_flash_func(dma_audio_irq_handler)() {
    if (dma_hw->ints0 & (1u << dma_rx_0)) {
        dma_hw->ints0 = (1u << dma_rx_0);
        dma_channel_set_write_addr(dma_rx_0, rx_buf_0, false);
        dma_channel_set_read_addr(dma_tx_0, tx_buf_0, false);
        process_audio_chunk(rx_buf_0, tx_buf_0);
    }
    if (dma_hw->ints0 & (1u << dma_rx_1)) {
        dma_hw->ints0 = (1u << dma_rx_1);
        dma_channel_set_write_addr(dma_rx_1, rx_buf_1, false);
        dma_channel_set_read_addr(dma_tx_1, tx_buf_1, false);
        process_audio_chunk(rx_buf_1, tx_buf_1);
    }
}

// ========================================================
// 5. CORE 1: UI, INPUT & CONTROL SCANNING
// ========================================================
// Core 1 gets its own (bigger) stack: snprintf + OLED code overflowed the small default one.
static uint32_t core1_stack[2048];     // 8 KB

#define DEBOUNCE_MS      25
#define LONG_PRESS_MS    600
#define MENU_TIMEOUT_MS  30000

// ---- Encoder: interrupt driven, cycle-position decoder ---------------------------------
// History: polling every ~2 ms lost detents while the OLED blocked core 1 (~23 ms per refresh);
// the first interrupt version counted every falling edge of A, which is far too sensitive on an
// encoder with contact bounce (one detent gave several steps, and B was often read mid-bounce so
// some went the wrong way).
//
// This decoder watches BOTH pins on BOTH edges and tracks WHERE we are in the quadrature cycle.
// With pull-ups the detent rest state is A=1,B=1. One clockwise detent walks the pins through
//         11 -> 01 -> 00 -> 10 -> 11        (A falling while B is high = clockwise)
// and counter-clockwise walks it the other way. We number the four states 0..3 along that
// clockwise path (rest = 0) and, for every pin change, work out how far we moved:
//   +1 / -1  a normal quarter step      -> add to a running total `enc_q`
//   0        same state again           -> ignore (e.g. both pin IRQs firing for one edge)
//   2        both pins changed between two samples (a missed edge). If we were already moving we
//            assume we kept going the same way (+2/-2); if not, the excursion is marked bad.
// Arriving back at rest, a count is emitted only if the total is exactly +4 or -4, i.e. one
// complete cycle in one direction. Contact bounce just goes back and forth (+1,-1,...) and nets
// out; half-turns that go back where they came from net to 0; a bad excursion is dropped and the
// next clean detent counts normally. So: exactly one step per detent.
static const uint8_t enc_cycle_pos[4] = { 2, 1, 3, 0 };   // index (A<<1)|B: 00->2, 01->1, 10->3, 11->0
static volatile int32_t enc_accum = 0;     // finished detents since the last enc_take()
static uint8_t          enc_pos   = 0;     // current position in the cycle (0 = rest)
static int8_t           enc_q     = 0;     // net quarter-steps since we were last at rest
static bool             enc_bad   = false; // this excursion contained an unexplainable jump

// (re)start the decoder from the real pin levels
static void enc_reset(bool a, bool b) {
    enc_pos = enc_cycle_pos[(a ? 2 : 0) | (b ? 1 : 0)];
    enc_q = 0; enc_bad = false;
}

// feed one sample of the two pin levels into the decoder
static inline void enc_feed(bool a, bool b) {
    const uint8_t np = enc_cycle_pos[(a ? 2 : 0) | (b ? 1 : 0)];
    const int d = (int)((np - enc_pos) & 3);
    if (d == 0) return;
    int step;
    if      (d == 1) step = 1;
    else if (d == 3) step = -1;
    else if (enc_q != 0 && !enc_bad) step = (enc_q > 0) ? 2 : -2;
    else { enc_bad = true; step = 0; }
    enc_pos = np;
    if (np == 0) {                                   // back at the detent
        enc_q += step;
        if (!enc_bad) { if (enc_q == 4) enc_accum = enc_accum + 1; else if (enc_q == -4) enc_accum = enc_accum - 1; }
        enc_q = 0; enc_bad = false;
    } else {
        int q = enc_q + step;
        enc_q = (int8_t)(q > 8 ? 8 : (q < -8 ? -8 : q));
    }
}
static void enc_gpio_cb(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    enc_feed(gpio_get(PIN_ENC_A), gpio_get(PIN_ENC_B));
}
static int enc_take() {
    uint32_t irq = save_and_disable_interrupts();
    int d = enc_accum;
    enc_accum = 0;
    restore_interrupts(irq);
    return d;
}

// ---- Buttons: debounced, with press / release / long-press events --------------
struct Btn {
    uint     pin;
    bool     stable;
    bool     raw_last;
    uint32_t t_raw;
    uint32_t t_down;
    bool     long_fired;
};
enum { BE_PRESS = 1, BE_RELEASE = 2, BE_LONG = 4 };

static void btn_init(Btn& b, uint pin) {
    b.pin = pin; b.stable = true; b.raw_last = true; b.t_raw = 0; b.t_down = 0; b.long_fired = false;
}
static int btn_update(Btn& b, uint32_t now) {
    int ev = 0;
    const bool raw = gpio_get(b.pin);
    if (raw != b.raw_last) { b.raw_last = raw; b.t_raw = now; }
    if (raw != b.stable && (now - b.t_raw) >= DEBOUNCE_MS) {
        b.stable = raw;
        if (!raw) { ev |= BE_PRESS; b.t_down = now; b.long_fired = false; }
        else      { ev |= BE_RELEASE; }
    }
    if (!b.stable && !b.long_fired && (now - b.t_down) >= LONG_PRESS_MS) {
        b.long_fired = true;
        ev |= BE_LONG;
    }
    return ev;
}

// ---- Module switching ---------------------------------------------------------
static void switch_module(int next) {
    const int mode = active_mode.load();
    if (next == mode) return;
    if (next == MODE_CLODS || next == MODE_FX) trigger_dsp_clear.store(true);
    trigger_strike.store(false);                    // don't leak a stale trigger into the next engine
    active_mode.store(next);
    ui_toast("%s", mode_hints[next]);
}

// ---- Per-module encoder behaviour (cw = true for clockwise) --------------------
static void module_encoder(int mode, bool cw) {
    switch (mode) {
        case MODE_PIMS: {
            int sel = selected_cv_out.load();
            int v = iclamp(cv_channels[sel].mult_div + (cw ? 1 : -1), -32, 32);
            if (v == 0) v = cw ? 1 : -1;
            cv_channels[sel].mult_div = v;
            break;
        }
        case MODE_RINGO: {
            int m = (ringo_model_idx.load() + (cw ? 1 : 5)) % 6;
            ringo_model_idx.store(m);
            ui_toast("MODEL: %s", ringo_model_names[m]);
            break;
        }
        case MODE_CLODS: {
            int m = (clods_submode.load() + (cw ? 1 : 2)) % 3;
            clods_submode.store(m);
            ui_toast("MODE: %s", clods_mode_names[m]);
            break;
        }
        case MODE_QUANTIZER: {
            int c = (selected_chord_type.load() + (cw ? 1 : 7)) % 8;
            selected_chord_type.store(c);
            ui_toast("TYPE: %s", chord_names[c]);
            break;
        }
        case MODE_QUIRKS: {
            int r = (quirks_audio_route.load() + (cw ? 1 : 2)) % 3;
            quirks_audio_route.store(r);
            ui_toast("OUT: %s", quirks_route_names[r]);
            break;
        }
        case MODE_PLAIDS: {
            int m = (plaids_model.load() + (cw ? 1 : 3)) % 4;
            plaids_model.store(m);
            ui_toast("MODEL: %s", plaids_model_names[m]);
            break;
        }
        case MODE_BRAINS: {
            int m = (brains_model.load() + (cw ? 1 : 3)) % 4;
            brains_model.store(m);
            ui_toast("MODEL: %s", brains_model_names[m]);
            break;
        }
        case MODE_MUMBLES: {
            int l = mumbles_loop_len.load();
            l = cw ? (l >= 16 ? 16 : l + 1) : (l <= 2 ? 2 : l - 1);
            mumbles_loop_len.store(l);
            ui_toast("LOOP: %d STEPS", l);
            break;
        }
        case MODE_WRAPS: {
            int a = (wraps_algo.load() + (cw ? 1 : 3)) % 4;
            wraps_algo.store(a);
            ui_toast("ALGO: %s", wraps_algo_names[a]);
            break;
        }
        case MODE_TIDDLY: {
            int sm = (tiddly_shape_mode.load() + (cw ? 1 : 2)) % 3;
            tiddly_shape_mode.store(sm);
            ui_toast("MODE: %s", tiddly_mode_names[sm]);
            break;
        }
        case MODE_FX: {
            int fm = (fx_submode.load() + (cw ? 1 : 3)) % 4;
            fx_submode.store(fm);
            ui_toast("FX: %s", fx_mode_names[fm]);
            break;
        }
        default: break;
    }
}

// Renders the whole current screen into oled_buf and pushes it to the display.
static void ui_render(int mode, uint32_t now) {
    ui_update_stats(now);
    memset(oled_buf, 0, sizeof(oled_buf));
    g_ink = 1;
    switch (ui_screen) {
        case SCR_PLAY:
            draw_play(mode, now);
            break;
        case SCR_MENU:
            draw_list("SELECT MODULE", MENU_N, menu_cur, false, menu_label, nullptr, mode, menu_footer(menu_cur));
            break;
        case SCR_SETTINGS:
            draw_list("SETTINGS", SET_N, set_cur, set_editing, set_label, set_value, -1,
                      set_editing ? "TURN:CHANGE PRESS:OK" : (set_is_action(set_cur) ? "PRESS:DO SEL:BACK" : "PRESS:EDIT SEL:BACK"));
            break;
        case SCR_SYSTEM:
            draw_system(now);
            break;
    }
    if (ui_screen != SCR_PLAY && toast_on && (now - toast_t) < 1400) draw_toast_bar();
}

// ---- Main UI / control loop ---------------------------------------------------
void core1_ui_loop() {
    i2c_init(i2c1, OLED_I2C_HZ);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    oled_init();
    oled_set_contrast(contrast_tab[cfg.brightness]);

    spi_init(MCP3208_SPI, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    gpio_init(PIN_BTN_SELECT); gpio_set_dir(PIN_BTN_SELECT, GPIO_IN); gpio_pull_up(PIN_BTN_SELECT);
    gpio_init(PIN_BTN_EDIT);   gpio_set_dir(PIN_BTN_EDIT,   GPIO_IN); gpio_pull_up(PIN_BTN_EDIT);
    gpio_init(PIN_ENC_A);      gpio_set_dir(PIN_ENC_A,      GPIO_IN); gpio_pull_up(PIN_ENC_A);
    gpio_init(PIN_ENC_B);      gpio_set_dir(PIN_ENC_B,      GPIO_IN); gpio_pull_up(PIN_ENC_B);
    gpio_init(PIN_ENC_SW);     gpio_set_dir(PIN_ENC_SW,     GPIO_IN); gpio_pull_up(PIN_ENC_SW);

    // IRQ is registered from this function, so it is serviced on core 1 and never touches the audio core
    gpio_set_irq_enabled_with_callback(PIN_ENC_A, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &enc_gpio_cb);
    gpio_set_irq_enabled(PIN_ENC_B, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    enc_reset(gpio_get(PIN_ENC_A), gpio_get(PIN_ENC_B));

    ui_init_tables();

    // boot splash
    for (int s = 0; s <= 12; s++) { draw_splash(s, 12); sleep_ms(70); }

    Btn b_enc, b_sel, b_edit;
    btn_init(b_enc, PIN_ENC_SW);
    btn_init(b_sel, PIN_BTN_SELECT);
    btn_init(b_edit, PIN_BTN_EDIT);

    float    param_target[7] = {0.5f, 0.3f, 0.5f, 0.5f, 0.5f, 0.3f, 0.5f};
    float    pot_baseline[7] = {0.0f};
    bool     pot_latched[7]  = {false, false, false, false, false, false, false};
    float    last_val[7]     = {0.0f};
    float    pot_f[7]        = {0.0f};
    float    pot_raw[7]      = {0.0f};
    bool     pot_f_init      = false;

    int      enc_resid = 0;
    uint32_t enc_last_t = 0;
    bool     fx_sel_down = false;           // FX: tap SELECT = test pluck, hold = randomise
    bool     asleep = false;
    uint32_t last_input = to_ms_since_boot(get_absolute_time());
    uint32_t last_frame = 0;
    uint32_t last_loop_us = time_us_32();
    bool     need_redraw = true;

    auto randomize_params = [&]() {
        for (int i = 0; i < 7; i++) {
            param_target[i] = (float)(rand() % 1000) / 1000.0f;
            pot_baseline[i] = pot_raw[i];
            pot_latched[i]  = true;
        }
        ui_toast("RANDOMIZED!");
    };

    ui_toast("%s", mode_hints[active_mode.load()]);

    while (true) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        const uint32_t now_us = time_us_32();
        const float dt_ms = (float)(now_us - last_loop_us) * 0.001f;
        last_loop_us = now_us;
        const int mode = active_mode.load();
        bool any_input = false;

        // ---------------- read pots & CV (oversampled; the ADC is noisy and pitch pots reveal it) ----------------
        float p1 = 1.0f - read_adc_avg(0, 8);
        float p2 = 1.0f - read_adc_avg(1, 8);
        float p3 = 1.0f - read_adc_avg(2, 8);

        float cv1 = cv_from_counts(read_mcp3208_avg(0, 4));
        float cv2 = cv_from_counts(read_mcp3208_avg(1, 4));
        float cv3 = cv_from_counts(read_mcp3208_avg(2, 4));
        float cv4 = cv_from_counts(read_mcp3208_avg(3, 4));
        g_cv[0].store(cv1); g_cv[1].store(cv2); g_cv[2].store(cv3); g_cv[3].store(cv4);

        float p4 = 1.0f - (read_mcp3208_avg(4, 4) / 4095.0f);
        float p5 = 1.0f - (read_mcp3208_avg(5, 4) / 4095.0f);
        float p6 = 1.0f - (read_mcp3208_avg(6, 4) / 4095.0f);
        float p7 = 1.0f - (read_mcp3208_avg(7, 4) / 4095.0f);

        const float pot_in[7] = {p1, p2, p3, p4, p5, p6, p7};
        // time-aware one-pole smoothing: same feel whether the loop runs at 3 ms (menus) or 30 ms (scope)
        const float a_pot = 1.0f - expf(-dt_ms / 15.0f);
        for (int i = 0; i < 7; i++) {
            if (!pot_f_init) pot_f[i] = pot_in[i];
            pot_f[i] += a_pot * (pot_in[i] - pot_f[i]);
            pot_raw[i] = clamp(pot_f[i], 0.0f, 1.0f);
        }
        pot_f_init = true;

        // ---------------- inputs ----------------
        const int enc_raw = enc_take();
        if (enc_raw) enc_last_t = now;
        else if ((now - enc_last_t) > 300) enc_resid = 0;       // forget a half-turned setting after a pause
        enc_resid += enc_raw;
        const int enc_div = enc_div_tab[cfg.enc > 2 ? 2 : cfg.enc];
        const int enc_delta = enc_resid / enc_div;               // ENCODER setting: 1 step per 1/2/3 detents
        enc_resid -= enc_delta * enc_div;
        const int ev_enc  = btn_update(b_enc, now);
        const int ev_sel  = btn_update(b_sel, now);
        const int ev_edit = btn_update(b_edit, now);
        if (enc_raw || ev_enc || ev_sel || ev_edit) any_input = true;

        // wake the display on the first input after the screen saver kicked in; swallow that input
        bool swallow = false;
        if (asleep && any_input) {
            asleep = false;
            oled_power(true);
            swallow = true;
            need_redraw = true;
        }

        if (!swallow) {
            const bool enc_short = (ev_enc & BE_RELEASE) && !b_enc.long_fired;
            const bool enc_long  = (ev_enc & BE_LONG) != 0;

            switch (ui_screen) {
            // ======================= PLAY =======================
            case SCR_PLAY:
                if (enc_short) {
                    menu_cur = mode; list_first = 0;
                    ui_screen = SCR_MENU;
                } else if (enc_long) {
                    ui_view = (ui_view + 1) % NUM_VIEWS;
                    ui_toast("%s", view_names[ui_view]);
                } else if (enc_delta) {
                    const bool cw = enc_delta > 0;
                    for (int k = 0; k < abs(enc_delta); k++) module_encoder(mode, cw);
                }

                if (ev_sel & BE_PRESS) {
                    if (mode == MODE_PIMS) {
                        selected_cv_out.store((selected_cv_out.load() + 1) % NUM_CV_OUTPUTS);
                    } else if (mode == MODE_CLODS) {
                        randomize_params();
                    } else if (mode == MODE_FX) {
                        fx_sel_down = true;                          // decided on hold / release
                    } else if (mode == MODE_RINGO || mode == MODE_PLAIDS || mode == MODE_BRAINS || mode == MODE_MUMBLES) {
                        trigger_strike.store(true);
                        ui_toast("* STRIKE / TRIG *");
                    } else if (mode == MODE_QUANTIZER) {
                        int r = (root_note.load() + 1) % 12;
                        root_note.store(r);
                        ui_toast("ROOT: %s", note_names[r]);
                    } else if (mode == MODE_QUIRKS) {
                        quirks_sh_trig.store(true);
                        ui_toast("S&H TRIGGERED");
                    }
                }
                if (fx_sel_down) {
                    if (ev_sel & BE_LONG) { fx_sel_down = false; randomize_params(); }
                    else if (ev_sel & BE_RELEASE) {
                        fx_sel_down = false;
                        if (mode == MODE_FX) { fx_trigger_pluck.store(true); ui_toast("* TEST PLUCK *"); }
                    }
                }

                if (ev_edit & BE_PRESS) {
                    if (mode == MODE_PIMS) {
                        int sel = selected_cv_out.load();
                        cv_channels[sel].shape = (cv_channels[sel].shape + 1) % 10;
                    } else if (mode == MODE_RINGO || mode == MODE_CLODS || mode == MODE_FX) {
                        bool frz = !is_frozen.load();
                        is_frozen.store(frz);
                        ui_toast(frz ? "FROZEN!" : "UNFROZEN");
                    } else if (mode == MODE_QUIRKS) {
                        bool ua = !quirks_sh_use_audio.load();
                        quirks_sh_use_audio.store(ua);
                        ui_toast(ua ? "SRC: AUDIO IN" : "SRC: INT NOISE");
                    } else if (mode == MODE_PLAIDS) {
                        bool lpg = !plaids_lpg_mode.load();
                        plaids_lpg_mode.store(lpg);
                        if (lpg) trigger_strike.store(true);         // retrigger via the flag, not by poking ISR state
                        ui_toast(lpg ? "MODE: LPG TRIG" : "MODE: DRONE ON");
                    } else if (mode == MODE_BRAINS) {
                        bool cr = !brains_bitcrush.load();
                        brains_bitcrush.store(cr);
                        ui_toast(cr ? "BITCRUSH: ON" : "BITCRUSH: OFF");
                    }
                }
                break;

            // ======================= MODULE MENU =======================
            case SCR_MENU:
                if (enc_delta) menu_cur = ((menu_cur + enc_delta) % MENU_N + MENU_N) % MENU_N;
                if (ev_sel & BE_PRESS) {                              // SELECT = back
                    ui_screen = SCR_PLAY;
                } else if (enc_short) {
                    if (menu_cur < NUM_MODES) { switch_module(menu_cur); ui_screen = SCR_PLAY; }
                    else if (menu_cur == NUM_MODES) { ui_screen = SCR_SETTINGS; set_cur = 0; set_editing = false; list_first = 0; }
                    else { ui_screen = SCR_SYSTEM; }
                }
                break;

            // ======================= SETTINGS =======================
            case SCR_SETTINGS:
                if (enc_delta) {
                    if (set_editing) {
                        for (int k = 0; k < abs(enc_delta); k++) set_adjust(set_cur, enc_delta > 0 ? 1 : -1);
                    } else {
                        set_cur = ((set_cur + enc_delta) % SET_N + SET_N) % SET_N;
                    }
                }
                if (ev_sel & BE_PRESS) {                              // SELECT = back one level
                    if (set_editing) set_editing = false;
                    else { ui_screen = SCR_MENU; menu_cur = NUM_MODES; list_first = 0; }
                } else if (enc_short) {
                    if (set_is_action(set_cur)) {
                        if (set_cur == SET_RESET) {
                            g_clip_count.store(0); g_xrun_count.store(0);
                            ui_cpu_peak = 0.0f; ui_xrun_ever = false; ui_clip_seen = 0; ui_xrun_seen = 0;
                            ui_toast("STATS RESET");
                        } else if (set_cur == SET_CLEAR) {
                            trigger_dsp_clear.store(true);
                            ui_toast("BUFFERS CLEARED");
                        } else {
                            ui_screen = SCR_MENU; menu_cur = NUM_MODES; list_first = 0;
                        }
                    } else {
                        set_editing = !set_editing;
                    }
                }
                break;

            // ======================= SYSTEM INFO =======================
            case SCR_SYSTEM:
                if ((ev_sel & BE_PRESS) || enc_short) { ui_screen = SCR_MENU; menu_cur = NUM_MODES + 1; list_first = 0; }
                if (ev_edit & BE_PRESS) {
                    g_clip_count.store(0); g_xrun_count.store(0);
                    ui_cpu_peak = 0.0f; ui_xrun_ever = false; ui_clip_seen = 0; ui_xrun_seen = 0;
                    ui_toast("STATS RESET");
                }
                break;
            }
        }

        if (any_input) { last_input = now; need_redraw = true; }

        // ---------------- pot pickup / latch + CV summing ----------------
        float param_vals[7];
        for (int i = 0; i < 7; i++) {
            if (pot_latched[i]) {
                if (fabsf(pot_raw[i] - pot_baseline[i]) > 0.05f) pot_latched[i] = false;
            }
            float base_val = pot_latched[i] ? param_target[i] : pot_raw[i];
            if (i == 0)      param_vals[i] = clamp(base_val + cv1, 0.0f, 1.0f);
            else if (i == 1) param_vals[i] = clamp(base_val + cv2, 0.0f, 1.0f);
            else if (i == 2) param_vals[i] = clamp(base_val + cv3, 0.0f, 1.0f);
            else if (i == 3) param_vals[i] = clamp(base_val + cv4, 0.0f, 1.0f);
            else             param_vals[i] = base_val;
            g_param[i].store(param_vals[i]);
        }
        if (mode == MODE_PIMS) master_bpm.store(40 + (int)(param_vals[6] * 210.0f));

        // pot moved -> parameter overlay (and wake the screen)
        for (int i = 0; i < 7; i++) {
            if (!pot_latched[i] && fabsf(pot_raw[i] - last_val[i]) > 0.02f) {
                last_val[i] = pot_raw[i];
                last_input = now;
                if (asleep) { asleep = false; oled_power(true); }
                if (ui_screen == SCR_PLAY) { ui_param_touched(i, now); toast_on = false; }
                need_redraw = true;
            }
        }

        // ---------------- timeouts ----------------
        if (ui_screen != SCR_PLAY && (now - last_input) > MENU_TIMEOUT_MS) { ui_screen = SCR_PLAY; set_editing = false; need_redraw = true; }
        if (!asleep && cfg.saver && (now - last_input) > saver_ms[cfg.saver & 3]) { asleep = true; oled_power(false); }

        // ---------------- render ----------------
        if (!asleep) {
            const bool animated = (ui_screen == SCR_PLAY && ui_view != VIEW_PARAMS) || ui_screen == SCR_SYSTEM;
            const bool toast_live = toast_on && (now - toast_t) < 1500;
            const bool ov_live = param_ov_on && (now - param_ov_t) < 1600;
            const uint32_t interval = animated ? 30 : 60;
            if (need_redraw || (now - last_frame) >= interval || toast_live || ov_live) {
                if ((now - last_frame) >= 20) {            // never flood the I2C bus
                    last_frame = now;
                    need_redraw = false;
                    ui_render(mode, now);
                    oled_show();
                }
            }
        }

        sleep_ms(2);
    }
}

// ========================================================
// 6. MAIN SYSTEM STARTUP
// ========================================================
int main() {
    set_sys_clock_khz(144000, true);
    stdio_init_all();

    init_hann_table();

    g_sram_static_bytes = sizeof(record_buffer) + sizeof(chorus_buf) +
                          sizeof(rev_comb_mem) + sizeof(rev_ap_mem) +
                          sizeof(rx_buf_0) + sizeof(rx_buf_1) + sizeof(tx_buf_0) + sizeof(tx_buf_1) +
                          sizeof(hann_tab) + sizeof(core1_stack) + sizeof(grains) + sizeof(oled_buf) + sizeof(oled_prev) +
                          sizeof(g_scope_ring) + sizeof(fft_re) * 2 + sizeof(fft_win) + sizeof(fft_cos) * 2 +
                          sizeof(fft_rev) + sizeof(spec_db);

    gpio_init(23);
    gpio_set_dir(23, GPIO_OUT);
    gpio_put(23, 1);

    multicore_launch_core1_with_stack(core1_ui_loop, core1_stack, sizeof(core1_stack));

    // Codec master clock (SCKI): 144 MHz / 6 = 24.000 MHz = 512 x fs for fs = 46,875 Hz (the rate i2s.pio generates).
    // PCM3060 needs SCKI = 256/384/512/768 x fs, max 36.864 MHz, and SCKI/BCK/LRCK frequency-locked: all three come from the same 144 MHz clock.
    gpio_set_function(14, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(14);
    pwm_set_wrap(slice_num, 5);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(14), 3);
    pwm_set_enabled(slice_num, true);

    i2c_init(i2c0, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // PCM3060 register writes {register, value}. Register map per the TI PCM3060 datasheet:
    //   0x40 (64)  power/reset : 0x00 = reset mode regs, ADC+DAC powered; final 0xC0 = MRST,SRST released
    //   0x43 (67)  DAC clock/format : 0x00 = slave, 24-bit I2S
    //   0x44 (68)  DAC oversampling/mute/phase : 0x00 = x64 OSR, no mute, normal phase
    //   0x41/0x42  DAC L/R attenuation : 0xFF = 0 dB (0.5 dB steps: dB = 0.5 x (code-255); the old 0xF3 was -6 dB).
    //              0 dB keeps all DAC bits; set the output level in the analog stage, or lower these codes for a trim.
    //   0x45 (69)  DAC filter + DE-EMPHASIS : 0x00 = sharp roll-off, de-emphasis OFF.
    //              (It used to be 0xD7, which switched ON the 32 kHz de-emphasis = a treble roll-off (typically ~-10 dB at the top): dull / "lo-fi".
    //               0xD7 is the ADC attenuation default for register 0x46 - probably written to the wrong register.)
    uint8_t codec_init[][2] = {
        {0x40, 0x00}, {0x43, 0x00}, {0x44, 0x00},
        {0x41, 0xFF}, {0x42, 0xFF}, {0x45, 0x00}, {0x40, 0xC0}
    };
    for (int i = 0; i < 7; i++) {
        i2c_write_blocking(i2c0, 0x46, codec_init[i], 2, false);
        sleep_ms(1);
    }

    PIO pio = pio0;
    uint sm_tx = 0, sm_rx = 1;
    uint offset_tx = pio_add_program(pio, &i2s_tx_program);
    uint offset_rx = pio_add_program(pio, &i2s_rx_program);

    i2s_tx_program_init(pio, sm_tx, offset_tx, 13, 10);
    i2s_rx_program_init(pio, sm_rx, offset_rx, 12, 10);

    dma_tx_0 = dma_claim_unused_channel(true);
    dma_tx_1 = dma_claim_unused_channel(true);
    dma_rx_0 = dma_claim_unused_channel(true);
    dma_rx_1 = dma_claim_unused_channel(true);

    dma_channel_config c;

    c = dma_channel_get_default_config(dma_tx_0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_tx, true));
    channel_config_set_chain_to(&c, dma_tx_1);
    dma_channel_configure(dma_tx_0, &c, &pio->txf[sm_tx], tx_buf_0, CHUNK_32BIT_WORDS, false);

    c = dma_channel_get_default_config(dma_tx_1);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_tx, true));
    channel_config_set_chain_to(&c, dma_tx_0);
    dma_channel_configure(dma_tx_1, &c, &pio->txf[sm_tx], tx_buf_1, CHUNK_32BIT_WORDS, false);

    c = dma_channel_get_default_config(dma_rx_0);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_rx, false));
    channel_config_set_chain_to(&c, dma_rx_1);
    dma_channel_configure(dma_rx_0, &c, rx_buf_0, &pio->rxf[sm_rx], CHUNK_32BIT_WORDS, false);

    c = dma_channel_get_default_config(dma_rx_1);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_rx, false));
    channel_config_set_chain_to(&c, dma_rx_0);
    dma_channel_configure(dma_rx_1, &c, rx_buf_1, &pio->rxf[sm_rx], CHUNK_32BIT_WORDS, false);

    dma_channel_set_irq0_enabled(dma_rx_0, true);
    dma_channel_set_irq0_enabled(dma_rx_1, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_audio_irq_handler);
    irq_set_priority(DMA_IRQ_0, 0);                 // audio gets top priority
    irq_set_enabled(DMA_IRQ_0, true);

    dma_start_channel_mask((1u << dma_tx_0) | (1u << dma_rx_0));
    pio_enable_sm_mask_in_sync(pio, (1u << sm_tx) | (1u << sm_rx));

    uint16_t last_dac[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    while (true) {
        if (g_dac_dirty.load()) {
            g_dac_dirty.store(false);
            for (uint8_t ch = 0; ch < 4; ch++) {
                uint16_t val = g_dac_out[ch].load();
                if (val != last_dac[ch]) {
                    update_mcp4728_channel(ch, dac_hw_code(ch, val));
                    last_dac[ch] = val;
                }
            }
        }
        tight_loop_contents();
    }
}
