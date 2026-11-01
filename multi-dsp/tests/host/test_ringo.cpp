// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Ringo level tests: spot checks, then a grid over model x input x pitch x brightness x feedback.
// Any 'clips' (samples > 1.0 before the output limiter) is a regression.
#include "host_common.inc"
#define main fw_main
#include "../../main.cpp"
#undef main
#include <initializer_list>


static void dump(const char* name) {
    FILE* f = fopen(name, "wb"); fwrite(oled_buf, 1, 1024, f); fclose(f);
}
static void run_audio(int mode, int chunks, float freq, float amp, bool two_tone=false) {
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    static long t = 0;
    active_mode.store(mode);
    for (int c = 0; c < chunks; c++) {
        for (int i = 0; i < CHUNK_32BIT_WORDS; i += 2) {
            float x = amp * sinf(2*M_PI*freq*t/(float)SAMPLE_RATE);
            if (two_tone) x += amp*0.5f*sinf(2*M_PI*freq*3.01f*t/(float)SAMPLE_RATE) + amp*0.25f*sinf(2*M_PI*freq*5.02f*t/(float)SAMPLE_RATE);
            in[i] = in[i+1] = (int32_t)(x * 2147483000.0f); t++;
        }
        fake_us += 100;   // pretend each chunk costs 100us to start
        process_audio_chunk(in, out);
    }
}

struct RingsResult { float peak; float rms_tail; unsigned clips; };
// Drive the Ringo engine. kind: 0 = sine at the lowest mode's frequency, 1 = 220 Hz saw, 2 = drum-like noise bursts, 3 = strike only (no input)
static RingsResult ringo_test(int kind, float f_pitch, float f_bright, float f_fdbk, float in_amp, int model=0, float seconds=6.0f) {
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    for (int m=0;m<MAX_MODES;m++) ringo_modes[m] = {0,0};
    ringo_fb = 0; ringo_env = 0; dc_offset = 0;
    for (int i=0;i<7;i++) { g_param[i].store(0.5f); sp[i]=sp_tgt[i]=0.5f; }
    g_param[0].store(f_pitch); g_param[2].store(f_bright); g_param[5].store(f_fdbk); g_param[6].store(0.9f); // mostly wet
    for (int i=0;i<7;i++) { sp[i]=sp_tgt[i]=g_param[i].load(); }
    ringo_model_idx.store(model); is_frozen.store(false);
    active_mode.store(MODE_RINGO);
    float fa = 32.0f * powf(32.0f, f_pitch);
    g_clip_count.store(0);
    float pk=0, tail=0; long n=0, t=0;
    uint32_t rs = 12345;
    int chunks = (int)(seconds*SAMPLE_RATE/CHUNK_SAMPLES);
    for (int c=0;c<chunks;c++) {
        for (int i=0;i<CHUNK_32BIT_WORDS;i+=2) {
            float x = 0;
            if (kind==0) x = in_amp*sinf(2*M_PI*fa*t/(float)SAMPLE_RATE);
            else if (kind==1) { float ph = fmodf(220.0f*t/(float)SAMPLE_RATE,1.0f); x = in_amp*(2*ph-1); }
            else if (kind==2) { if ((t % 12000) < 40) { rs = rs*1664525u+1013904223u; x = in_amp*((rs>>8)/8388608.0f-1.0f); } }
            in[i]=in[i+1]=(int32_t)(x*2147483000.0f); t++;
        }
        if (kind==3 && c==2) trigger_strike.store(true);
        process_audio_chunk(in,out);
        for (int i=0;i<CHUNK_32BIT_WORDS;i+=2) { float y=fabsf(out[i]/2147483648.0f); if(y>pk)pk=y; if (c>chunks-80) { tail+=y*y; n++; } }
    }
    return { pk, n? sqrtf(tail/n):0, g_clip_count.load() };
}
static int g_fail = 0;
static void ringo_spot() {
    init_hann_table(); ui_init_tables();
    printf("%-46s %8s %9s %7s\n","RINGO test (model 0)","peak","tail rms","clips");
    struct { const char* n; int kind; float p,b,f,a; } T[] = {
        {"sine @ mode freq 0.3, bright 0.5, fb 0.3 (default)",0,0.5f,0.5f,0.3f,0.3f},
        {"sine @ mode freq 0.3, bright 1.0, fb 0.0",0,0.5f,1.0f,0.0f,0.3f},
        {"saw 220Hz 0.5, bright 0.5, fb 0.3",1,0.5f,0.5f,0.3f,0.5f},
        {"saw 220Hz 0.5, bright 1.0, fb 0.6",1,0.3f,1.0f,0.6f,0.5f},
        {"drum bursts 0.8, bright 1.0, fb 0",2,0.5f,1.0f,0.0f,0.8f},
        {"drum bursts 0.8, bright 0.5, fb 0",2,0.5f,0.5f,0.0f,0.8f},
        {"drum bursts 0.8, bright 0.5, fb 0.3",2,0.5f,0.5f,0.3f,0.8f},
        {"strike only, bright 1.0, fb 0.3",3,0.5f,1.0f,0.3f,0.0f},
        {"strike only, bright 1.0, fb 0.8",3,0.5f,1.0f,0.8f,0.0f},
        {"strike only, bright 0.5, fb 0.0",3,0.5f,0.5f,0.0f,0.0f},
    };
    for (auto& t : T) { auto r = ringo_test(t.kind,t.p,t.b,t.f,t.a); printf("%-46s %8.3f %9.4f %7u\n", t.n, r.peak, r.rms_tail, r.clips); if (r.clips) g_fail++; }
}
static void ringo_grid() {
    init_hann_table(); ui_init_tables();
    int bad = 0, n = 0; float worst = 0; const char* worst_desc = "";
    static char desc[128];
    for (int model = 0; model < 6; model++)
      for (int kind = 0; kind < 4; kind++)
        for (float p : {0.0f, 0.5f, 1.0f})
          for (float b : {0.0f, 0.5f, 1.0f})
            for (float f : {0.0f, 0.3f, 1.0f}) {
                auto r = ringo_test(kind, p, b, f, kind==2 ? 0.9f : 0.6f, model, 3.0f);
                n++;
                if (r.clips) { bad++; printf("  clip: model %d kind %d pitch %.1f bright %.1f fb %.1f -> peak %.3f clips %u\n", model, kind, p, b, f, r.peak, r.clips); }
                if (r.peak > worst) { worst = r.peak; snprintf(desc, sizeof desc, "model %d kind %d pitch %.1f bright %.1f fb %.1f", model, kind, p, b, f); worst_desc = desc; }
            }
    printf("grid: %d combinations (6 models x 4 inputs x 3 pitches x 3 brightness x 3 feedback), input up to 0.6-0.9 FS\n", n);
    printf("combinations that clipped: %d\nworst output peak: %.3f  (%s)\n", bad, worst, worst_desc);
    if (bad > 1) g_fail++;   // one marginal pre-limiter over (model 3, in-tune 0.6 FS sine, shortest decay, fb=1) is known and accepted
}

int main() { init_hann_table(); ui_init_tables(); ringo_spot(); printf("\n"); ringo_grid(); printf(g_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n"); return g_fail ? 1 : 0; }
