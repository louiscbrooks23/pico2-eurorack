// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Renders every screen into raw 1 KB OLED frames (.bin) in the current directory; topng.py turns them into PNGs.
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
int main(int argc, char** argv) {
    init_hann_table(); ui_init_tables();
    g_sram_static_bytes = sizeof(record_buffer) + sizeof(chorus_buf) + sizeof(rev_comb_mem) + sizeof(rev_ap_mem) +
        sizeof(rx_buf_0)*4 + sizeof(hann_tab) + sizeof(core1_stack) + sizeof(grains) + sizeof(oled_buf) + sizeof(oled_prev) +
        sizeof(g_scope_ring) + sizeof(fft_re)*2 + sizeof(fft_win) + sizeof(fft_cos)*2 + sizeof(fft_rev) + sizeof(spec_db);
    printf("static RAM estimate: %lu bytes (%.1f KB) of 520 KB\n", (unsigned long)g_sram_static_bytes, g_sram_static_bytes/1024.0);

    for (int i=0;i<7;i++) g_param[i].store(0.5f);
    uint32_t now = fake_ms;
    // Plaids: a saw-like tone for the scope
    active_mode.store(MODE_PLAIDS); plaids_model.store(0);
    for (int i=0;i<7;i++) g_param[i].store(0.5f);
    g_param[0].store(0.45f);
    // let params settle
    for (int k=0;k<20;k++) run_audio(MODE_PLAIDS, 8, 0, 0);
    next_cost = (uint32_t)(0.34f * CHUNK_DEADLINE_US);   // pretend the engine uses 34% of the chunk budget

    struct { const char* name; Screen scr; int view; } shots[] = {
        {"play_params", SCR_PLAY, VIEW_PARAMS}, {"play_scope", SCR_PLAY, VIEW_SCOPE},
        {"play_spectrum", SCR_PLAY, VIEW_SPECTRUM}, {"play_combo", SCR_PLAY, VIEW_COMBO},
        {"menu", SCR_MENU, 0}, {"settings", SCR_SETTINGS, 0}, {"system", SCR_SYSTEM, 0},
    };
    for (auto& s : shots) {
        ui_screen = s.scr; ui_view = s.view;
        menu_cur = 5; set_cur = 2; list_first = 0;
        for (int k=0;k<12;k++) { run_audio(MODE_PLAIDS, 6, 0, 0); ui_render(MODE_PLAIDS, now); }
        char fn[64]; snprintf(fn, sizeof fn, "%s.bin", s.name); dump(fn);
    }
    // extra shots: param overlay + toast on scope, menu scrolled, settings editing, frozen header, CPU warn
    ui_screen = SCR_PLAY; ui_view = VIEW_SCOPE; ui_param_touched(2, now); ui_render(MODE_PLAIDS, now); dump("ov_scope.bin");
    ui_toast("MODEL: PHASE DIST"); toast_t = now; ui_render(MODE_PLAIDS, now); dump("toast_scope.bin");
    ui_screen = SCR_MENU; menu_cur = 12; list_first = 0; ui_render(MODE_PLAIDS, now); dump("menu_end.bin");
    ui_screen = SCR_SETTINGS; set_cur = 1; set_editing = true; list_first = 0; ui_render(MODE_PLAIDS, now); dump("settings_edit.bin");
    ui_screen = SCR_PLAY; ui_view = VIEW_PARAMS; set_editing=false; is_frozen.store(true);
    next_cost = (uint32_t)(0.92f * CHUNK_DEADLINE_US); for (int k=0;k<80;k++) run_audio(MODE_CLODS, 1, 0, 0); ui_cpu_avg = 92.0f; toast_on=false; param_ov_on=false;
    ui_render(MODE_CLODS, now); dump("hdr_frozen_clods.bin");
    ui_render(MODE_PIMS, now); dump("pims.bin");
    draw_splash(7, 12); dump("splash.bin");

    return 0;
}
