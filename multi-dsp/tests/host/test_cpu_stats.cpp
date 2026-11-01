// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Verifies the windowed CPU statistic, xrun counting and mode-switch warm-up.
#include "host_common.inc"
#define main fw_main
#include "../../main.cpp"
#undef main
#include <cstdio>

int main() {
    init_hann_table(); ui_init_tables();
    // ---- CPU statistic test ----
    printf("\nCPU statistic: 500 windows, simulated cost = 40%% +/- 25%% jitter, 1-in-200 chunks spike to 90%%\n");
    srand(1);
    active_mode.store(MODE_FX);
    static int32_t in[CHUNK_32BIT_WORDS], out[CHUNK_32BIT_WORDS];
    double mn_old=1e9, mx_old=-1e9, mn_new=1e9, mx_new=-1e9; float last_pub=-1; int publishes=0;
    for (int c = 0; c < 64*500; c++) {
        float load = 0.40f + 0.25f*((rand()%2001)/1000.0f - 1.0f);
        if (rand()%200==0) load = 0.90f;
        next_cost = (uint32_t)(load * CHUNK_DEADLINE_US);
        // what the OLD display would have shown: the single-chunk load sampled at an arbitrary moment
        if (c % 17 == 0) { double o = load*100; if(o<mn_old)mn_old=o; if(o>mx_old)mx_old=o; }
        process_audio_chunk(in, out);
        float p = g_cpu_load_pct.load();
        if (c>64*3 && p != last_pub) { last_pub = p; publishes++; if(p<mn_new)mn_new=p; if(p>mx_new)mx_new=p; }
    }
    int fails = 0;
    if (mn_new < 30 || mx_new > 50) { printf("FAIL: windowed average should sit between 30%% and 50%%\n"); fails++; }
    if (g_xrun_count.load() != 0) { printf("FAIL: a 90%% spike must not count as an xrun\n"); fails++; }
    printf("  OLD readout range (sampled single chunks): %.0f%% .. %.0f%%\n", mn_old, mx_old);
    printf("  NEW readout range (windowed average):      %.1f%% .. %.1f%%   (%d published windows)\n", mn_new, mx_new, publishes);
    printf("  last window max: %.0f%%   xruns counted: %u (expect 0: spike 90%% < 97%%)\n", g_cpu_peak_pct.load(), (unsigned)g_xrun_count.load());
    // force an overrun
    next_cost = (uint32_t)(1.05f*CHUNK_DEADLINE_US); process_audio_chunk(in,out);
    printf("  after one 105%% chunk: xruns=%u\n", (unsigned)g_xrun_count.load());
    if (g_xrun_count.load() != 1) { printf("FAIL: a 105%% chunk must count exactly one xrun\n"); fails++; }
    // mode switch warm-up: a huge spike on the first chunk after a switch must not pollute the average
    g_cpu_load_pct.store(0);
    next_cost = (uint32_t)(0.3f*CHUNK_DEADLINE_US);
    for (int c=0;c<64;c++) process_audio_chunk(in,out);
    active_mode.store(MODE_CLODS); next_cost=(uint32_t)(0.9f*CHUNK_DEADLINE_US); process_audio_chunk(in,out);
    next_cost=(uint32_t)(0.3f*CHUNK_DEADLINE_US); for (int c=0;c<64;c++) process_audio_chunk(in,out);
    printf("  30%% load with a 90%% spike on mode switch -> published avg %.1f%% (spike excluded by warm-up)\n", g_cpu_load_pct.load());
    if (fabsf(g_cpu_load_pct.load() - 30.0f) > 1.0f) { printf("FAIL: warm-up should exclude the switch spike\n"); fails++; }
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return fails ? 1 : 0;
}
