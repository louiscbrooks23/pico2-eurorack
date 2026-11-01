// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Quirks tests: clipping grid, DC, transparency at mix=0, S&H rate mapping, LPF direction, rectifier aliasing.
#include "host_common.inc"
#define main fw_main
#include "../../main.cpp"
#undef main


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

#include <initializer_list>
#include <vector>
struct KR { float peak, dc, rms; unsigned clips; };
static void quirks_reset() { quirks_lpf=0; quirks_held_val=0; quirks_wdc=0; quirks_u1=0; quirks_cut_s=0.5f; quirks_clock_ph=0; dc_offset=0; g_clip_count.store(0); }
static void set_params(float p0,float p1,float p2,float p3,float p4,float p5,float p6) {
    float v[7]={p0,p1,p2,p3,p4,p5,p6};
    for (int i=0;i<7;i++) { g_param[i].store(v[i]); sp[i]=sp_tgt[i]=v[i]; }
}
static std::vector<float> run_quirks(int route, bool use_audio, const float* p, float in_amp, float f_in, int chunks, int skip) {
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    quirks_audio_route.store(route); quirks_sh_use_audio.store(use_audio);
    set_params(p[0],p[1],p[2],p[3],p[4],p[5],p[6]);
    quirks_reset(); active_mode.store(MODE_QUIRKS);
    std::vector<float> o; long t=0; uint32_t rs=99;
    for (int c=0;c<chunks;c++) {
        for (int i=0;i<CHUNK_32BIT_WORDS;i+=2) {
            float x;
            if (f_in > 0) x = in_amp*sinf(2*M_PI*f_in*t/(float)SAMPLE_RATE);
            else { rs=rs*1664525u+1013904223u; x = in_amp*((rs>>8)/8388608.0f-1.0f); }   // white noise input
            in[i]=in[i+1]=(int32_t)(x*2147483000.0f); t++;
        }
        process_audio_chunk(in,out);
        if (c>=skip) for (int i=0;i<CHUNK_32BIT_WORDS;i+=2) o.push_back(out[i]/2147483648.0f);
    }
    return o;
}
static KR stats(const std::vector<float>& o) {
    double sum=0,sq=0; float pk=0; for (float y: o) { sum+=y; sq+=y*y; if (fabsf(y)>pk) pk=fabsf(y); }
    return { pk, (float)(sum/o.size()), (float)sqrt(sq/o.size()), g_clip_count.load() };
}
int main() {
    int fails = 0;
    init_hann_table(); ui_init_tables();
    const char* rn[3] = {"RECT/INV ","MAX/MIN  ","S&H+NOISE"};

    // ---- 1. same scenarios as the "before" measurement ----
    printf("1) Same scenarios as before (220 Hz sine). clips = samples > 1.0 before the output limiter\n");
    printf("%-10s %-22s %6s %6s %7s %7s\n", "route", "drive/blend/noise/mix", "peak", "DC", "rms", "clips");
    for (float in_amp : {0.3f, 0.6f}) {
        printf("--- input amplitude %.1f FS ---\n", in_amp);
        for (int route=0; route<3; route++) for (float drive : {0.0f,0.5f,1.0f}) {
            float blend=(route==0)?1.0f:0.5f, noise=(route==2)?0.5f:0.3f;
            float p[7]={0.2f,drive,blend,noise,0.0f,0.5f,0.5f};
            auto o = run_quirks(route, route==2, p, in_amp, 220.0f, 400, 100); auto r = stats(o);
            char d[64]; snprintf(d,sizeof d,"drv %.1f bl %.1f ns %.1f mx .5",drive,blend,noise);
            printf("%-10s %-22s %6.3f %+6.3f %7.3f %7u\n", rn[route], d, r.peak, r.dc, r.rms, r.clips);
            if (r.clips || fabsf(r.dc) > 0.2f && route != 2) fails++;
        }
    }

    // ---- 2. exhaustive extreme grid with a hot input ----
    { long combos=0, bad=0; float worst=0; unsigned total_clips=0; double worst_dc=0;
      float lv[3]={0.0f,0.5f,1.0f};
      for (int route=0; route<3; route++) for (int ua=0; ua<2; ua++)
        for (float d: lv) for (float b: lv) for (float n: lv) for (float c: lv) for (float th: lv) for (float m: lv) for (float r0 : {0.1f, 0.9f}) {
            if (route!=2 && ua==1) continue;
            float p[7]={r0,d,b,n,c,th,m};
            auto o = run_quirks(route, ua==1, p, 0.95f, 440.0f, 60, 20); auto r = stats(o);
            combos++; if (r.clips) { bad++; total_clips+=r.clips; } if (r.peak>worst) worst=r.peak; if (fabs(r.dc)>worst_dc) worst_dc=fabs(r.dc);
      }
      printf("\n2) Extreme grid, 0.95 FS input: %ld combinations (3 routes x drive/blend/noise/cutoff/threshold/mix 0..1 x 2 S&H rates)\n", combos);
      printf("   combinations that clipped before the limiter: %ld   worst output peak %.3f   worst |DC| %.3f\n", bad, worst, worst_dc); if (bad) fails++; }

    // ---- 3. transparency: mix = 0 must pass the (DC-blocked) input unchanged ----
    { float p[7]={0.5f,1.0f,1.0f,1.0f,1.0f,1.0f,0.0f}; double maxerr=0;
      auto o = run_quirks(0,false,p,0.5f,1000.0f,200,50);
      for (size_t k=0;k<o.size();k++) { long t = 50L*CHUNK_SAMPLES + (long)k; float ref=0.5f*sinf(2*M_PI*1000.0f*t/(float)SAMPLE_RATE); double e=fabs(o[k]-ref); if (e>maxerr) maxerr=e; }
      printf("\n3) mix=0 with every other knob at max: max deviation from the input = %.5f\n", maxerr); if (maxerr > 0.005) fails++; }

    // ---- 4. S&H rate mapping (measured: count the steps in the output) ----
    printf("\n4) S&H rate vs knob (internal noise source, steps counted in the output over 20 s):\n");
    for (float k : {0.0f,0.25f,0.5f,0.75f,1.0f}) {
        float p[7]={k,0,0.5f,1.0f,1.0f,0.5f,1.0f};
        auto o = run_quirks(2,false,p,0.0f,220.0f,(int)(20*SAMPLE_RATE/CHUNK_SAMPLES),0);
        int events=0; int quiet=100;
        for (size_t i=1;i<o.size();i++) { float d=fabsf(o[i]-o[i-1]); if (d>0.004f && quiet>=3) { events++; quiet=0; } else quiet++; }
        printf("   knob %.2f -> expected %8.2f Hz   measured %8.2f Hz\n", k, 0.1f*powf(20000.0f,k), events/20.0f);
        { float e = 0.1f*powf(20000.0f,k), m = events/20.0f; if (fabsf(m - e) > 0.15f*e) fails++; }
    }

    // ---- 5. LPF direction: white noise in, route 1 clean-ish, compare HF energy at cutoff knob 0 vs 1 ----
    { float lo_p[7]={0.5f,0,0.5f,0,0.0f,0.5f,1.0f}, hi_p[7]={0.5f,0,0.5f,0,1.0f,0.5f,1.0f};
      auto a = run_quirks(1,false,lo_p,0.5f,0,200,50), b = run_quirks(1,false,hi_p,0.5f,0,200,50);
      auto hf=[&](const std::vector<float>& o){ double e=0; for (size_t i=1;i<o.size();i++){ double d=o[i]-o[i-1]; e+=d*d; } return sqrt(e/o.size()); };
      printf("\n5) LPF knob direction (white noise in, wet only): HF energy knob=0: %.4f   knob=1: %.4f  -> %s\n", hf(a), hf(b), hf(b)>hf(a)*3 ? "knob up = brighter (correct)" : "UNEXPECTED"); if (!(hf(b)>hf(a)*3)) fails++; }

    // ---- 6. anti-aliasing of the rectifier: 5 kHz sine, wet only, LPF wide open ----
    { const int N=4096; float p[7]={0.5f,0,1.0f,0,1.0f,0.5f,1.0f};
      auto o = run_quirks(0,false,p,0.8f,5000.0f,64,8);
      // naive reference: |x| with the same DC removal and gain, no ADAA
      std::vector<float> ref; double mean=0; std::vector<float> a; for (int k=0;k<N;k++) { long t=8L*CHUNK_SAMPLES+k; a.push_back(fabsf(0.8f*sinf(2*M_PI*5000.0f*t/(float)SAMPLE_RATE))); mean+=a.back(); } mean/=N;
      for (int k=0;k<N;k++) ref.push_back((a[k]-(float)mean)*1.25f);
      auto spec_alias=[&](const std::vector<float>& x, int off){ // energy outside the even harmonics (10k,20k) +-4 bins, 1k..22k
          std::vector<double> re(N), im(N,0.0); for (int k=0;k<N;k++) re[k]=x[off+k]*(0.5-0.5*cos(2*M_PI*k/(N-1)));
          double tot=0, harm=0;
          for (int bin=(int)(1000.0/SAMPLE_RATE*N); bin<(int)(22000.0/SAMPLE_RATE*N); bin++) { double sr=0, si=0; for (int k=0;k<N;k++){ double ph=2*M_PI*bin*k/N; sr+=re[k]*cos(ph); si-=re[k]*sin(ph);} double pw=sr*sr+si*si; tot+=pw;
              double f=bin*(double)SAMPLE_RATE/N; if (fabs(f-10000)<350 || fabs(f-20000)<350) harm+=pw; }
          return 10*log10((tot-harm)/harm + 1e-30); };
      double a_fw = spec_alias(o, 0), a_ref = spec_alias(ref, 0);
      printf("\n6) Rectifier aliasing (5 kHz sine, energy NOT on the 10k/20k harmonics relative to them): naive |x| %.1f dB, firmware %.1f dB\n", a_ref, a_fw); if (a_fw > a_ref - 3.0) fails++; }
    printf(fails ? "RESULT: FAIL (%d)\n" : "RESULT: PASS\n", fails);
    return fails ? 1 : 0;
}
