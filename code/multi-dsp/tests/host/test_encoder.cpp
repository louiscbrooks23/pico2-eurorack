// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Encoder decoder tests: clean detents, contact bounce, missed edges, recovery. Compared with the older edge-counting scheme.
#include "host_common.inc"
#define main fw_main
#include "../../main.cpp"
#undef main



#include <vector>
#include <random>
struct P { bool a, b; };
// one detent in the "A falling while B high = clockwise" direction (matches the previous firmware)
static std::vector<P> detent(bool cw) {
    if (cw) return { {0,1}, {0,0}, {1,0}, {1,1} };
    else    return { {1,0}, {0,0}, {0,1}, {1,1} };
}
// inject contact bounce: at each transition, flap between old and new level a few times
static std::vector<P> with_bounce(const std::vector<P>& clean, std::mt19937& rng, P start, int max_flaps) {
    std::vector<P> out; P prev = start;
    for (auto p : clean) {
        int flaps = rng() % (max_flaps + 1);
        for (int f = 0; f < flaps; f++) { out.push_back(p); out.push_back(prev); }
        out.push_back(p); prev = p;
    }
    return out;
}
static int run_new(const std::vector<P>& seq) {
    enc_reset(true, true); enc_accum = 0;
    for (auto p : seq) enc_feed(p.a, p.b);
    return enc_accum;
}
static int run_old(const std::vector<P>& seq, P start) {      // previous interrupt version: count every falling edge of A, direction from B
    int acc = 0; bool la = start.a;
    for (auto p : seq) { if (la && !p.a) acc += p.b ? 1 : -1; la = p.a; }
    return acc;
}
static int g_fail = 0;
static void expect(int got, int want) { if (got != want) g_fail++; }
int main() {
    std::mt19937 rng(7);
    const P rest{1,1};
    printf("%-52s %8s %8s\n", "scenario", "OLD irq", "NEW");
    { auto s = detent(true);  printf("%-52s %8d %8d\n", "1 clean clockwise detent", run_old(s,rest), run_new(s)); expect(run_new(s), 1); }
    { auto s = detent(false); printf("%-52s %8d %8d\n", "1 clean counter-clockwise detent", run_old(s,rest), run_new(s)); expect(run_new(s), -1); }
    { std::vector<P> s = { {0,1},{1,1},{0,1},{1,1},{0,1},{0,0},{1,0},{1,1} };
      printf("%-52s %8d %8d\n", "1 CW detent, A bounces 3x on the first edge", run_old(s,rest), run_new(s)); expect(run_new(s), 1); }
    { std::vector<P> s = { {0,1},{0,0},{1,0},{1,1},{1,0},{1,1} };
      printf("%-52s %8d %8d\n", "1 CW detent then B chatter at rest", run_old(s,rest), run_new(s)); }
    { std::vector<P> s = { {0,1},{0,0},{0,1},{0,0},{0,1},{1,1} };
      printf("%-52s %8d %8d\n", "start CW, back off to rest (no detent)", run_old(s,rest), run_new(s)); expect(run_new(s), 0); }
    for (int flaps : {0, 2, 5}) {
        int cw_old=0, cw_new=0, ccw_old=0, ccw_new=0;
        std::vector<P> all_cw, all_ccw;
        for (int i=0;i<100;i++) { auto b = with_bounce(detent(true), rng, rest, flaps); all_cw.insert(all_cw.end(), b.begin(), b.end()); }
        for (int i=0;i<100;i++) { auto b = with_bounce(detent(false), rng, rest, flaps); all_ccw.insert(all_ccw.end(), b.begin(), b.end()); }
        char name[80];
        snprintf(name, sizeof name, "100 CW detents, up to %d bounces per edge", flaps);  printf("%-52s %8d %8d\n", name, run_old(all_cw,rest), run_new(all_cw)); expect(run_new(all_cw), 100);
        snprintf(name, sizeof name, "100 CCW detents, up to %d bounces per edge", flaps); printf("%-52s %8d %8d\n", name, run_old(all_ccw,rest), run_new(all_ccw)); expect(run_new(all_ccw), -100);
    }
    // fast spin: 1000 detents alternating direction runs
    { std::vector<P> s; int expect_total = 0;
      for (int run = 0; run < 20; run++) { bool cw = run % 2 == 0; int n = 10 + (rng()%40); for (int i=0;i<n;i++){ auto b = with_bounce(detent(cw), rng, rest, 3); s.insert(s.end(), b.begin(), b.end()); expect_total += cw ? 1 : -1; } }
      printf("%-52s %8d %8d   (expect %d)\n", "20 alternating spins, mixed directions, bounce", run_old(s,rest), run_new(s), expect_total); expect(run_new(s), expect_total); }

    // ---- harder cases ----
    // heavy bounce: up to 8 flaps on every edge, 2000 detents in random directions
    { std::vector<P> s; int total = 0;
      for (int i = 0; i < 2000; i++) { bool cw = rng() & 1; auto b = with_bounce(detent(cw), rng, rest, 8); s.insert(s.end(), b.begin(), b.end()); total += cw ? 1 : -1; }
      printf("%-52s %8d %8d   (expect %d)\n", "2000 random detents, up to 8 bounces per edge", run_old(s,rest), run_new(s), total); expect(run_new(s), total); }
    // long chatter around the rest position must never produce a count
    { std::vector<P> s; for (int i = 0; i < 1000; i++) { s.push_back({1,0}); s.push_back({1,1}); s.push_back({0,1}); s.push_back({1,1}); }
      printf("%-52s %8d %8d\n", "4000 edges of chatter at rest (no detent)", run_old(s,rest), run_new(s)); expect(run_new(s), 0); }
    // reversal in the middle of a detent, then a normal detent: only the normal one counts
    { std::vector<P> s = { {0,1},{0,0},{0,1},{1,1}  /* back off */, {1,0},{0,0},{0,1},{1,1} /* one CCW detent */ };
      printf("%-52s %8d %8d\n", "reverse half-way, then 1 CCW detent", run_old(s,rest), run_new(s)); expect(run_new(s), -1); }
    // missed edge in the middle of a turn (both pins changed between two samples): direction is inferred
    { std::vector<P> s = { {0,1},{1,0},{1,1} };                         // 11 -> 01 -> (00 missed) -> 10 -> 11
      printf("%-52s %8d %8d\n", "CW detent with the 00 state missed", run_old(s,rest), run_new(s)); expect(run_new(s), 1); }
    { std::vector<P> s = { {1,0},{0,1},{1,1} };                         // 11 -> 10 -> (00 missed) -> 01 -> 11
      printf("%-52s %8d %8d\n", "CCW detent with the 00 state missed", run_old(s,rest), run_new(s)); expect(run_new(s), -1); }
    // a jump from rest that cannot be explained (11 -> 00): drop that excursion, recover on the next one
    { std::vector<P> s = { {0,0},{1,0},{1,1} };
      printf("%-52s %8d %8d\n", "unexplained 11->00 jump (dropped)", run_old(s,rest), run_new(s)); expect(run_new(s), 0); }
    { std::vector<P> s = { {0,0},{1,0},{1,1} }; auto d = detent(true); s.insert(s.end(), d.begin(), d.end());
      printf("%-52s %8d %8d\n", "...and the next clean CW detent still counts", run_old(s,rest), run_new(s)); expect(run_new(s), 1); }
    // duplicate samples (both pin IRQs firing for one edge) must not change anything
    { std::vector<P> s; for (auto p : detent(true)) { s.push_back(p); s.push_back(p); s.push_back(p); }
      printf("%-52s %8d %8d\n", "1 CW detent, every sample repeated 3x", run_old(s,rest), run_new(s)); expect(run_new(s), 1); }
    // starting mid-detent (decoder initialised while the knob is between detents) must not count a phantom step
    { enc_reset(false, false); enc_accum = 0; for (auto p : std::vector<P>{ {1,0},{1,1} }) enc_feed(p.a, p.b);
      printf("%-52s %8s %8d\n", "power-up with the knob between detents", "-", (int)enc_accum); expect(enc_accum, 0); }
    printf(g_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return g_fail ? 1 : 0;
}
