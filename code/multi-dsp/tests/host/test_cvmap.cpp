// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// CV scaling tests: the firmware's CV input and output mapping is checked against a model of the analog stages
// in the schematic (LB-SYNTH-PROTO-A v0.2; see hardware/README.md sections 8.1 and 8.2).
#include "host_common.inc"
#define main fw_main
#include "../../main.cpp"
#undef main

#include <cmath>
#include <initializer_list>
static int g_fail = 0;
static void check(bool ok, const char* what, double a = 0, double b = 0) {
    if (!ok) { g_fail++; printf("  FAIL: %s (%.4f vs %.4f)\n", what, a, b); }
}

// ---- model of the board ---------------------------------------------------------------------------
// MCP4728 from VDD (3.3 V): Vdac = 3.3 * code / 4096
static double vdac(unsigned code) { return 3.3 * (double)code / 4096.0; }
// CV Out A/B: non-inverting, 1 + 14.3k/10k
static double hw_out_ab(unsigned code) { return vdac(code) * (1.0 + 14.3 / 10.0); }
// CV Out C/D: inverting summing, -100k/20k on the signal plus 100k/240k from the -12V-F rail
static double hw_out_cd(unsigned code, double rail_abs = 11.75) { return rail_abs * 100.0 / 240.0 - 5.0 * vdac(code); }
// CV inputs: inverting, Vadc = |rail| * 30/240 - 0.3 * Vin, 12-bit against 3.0 V
static double hw_cv_counts(double vin, double rail_abs = 12.0) {
    double vadc = rail_abs * 30.0 / 240.0 - 0.3 * vin;
    double c = std::floor(vadc / 3.0 * 4096.0 + 0.5);
    return std::fmin(4095.0, std::fmax(0.0, c));
}

int main() {
    printf("CV output mapping\n");
    // A and B pass straight through
    for (unsigned l = 0; l <= 4095; l += 17) {
        check(dac_hw_code(0, l) == l, "CV out A passes through", dac_hw_code(0, l), l);
        check(dac_hw_code(1, l) == l, "CV out B passes through", dac_hw_code(1, l), l);
    }
    check(dac_hw_code(0, 9999) == 4095, "A clamps above range");
    printf("  A: 0 -> %.2f V, 4095 -> %.2f V\n", hw_out_ab(dac_hw_code(0, 0)), hw_out_ab(dac_hw_code(0, 4095)));
    check(hw_out_ab(dac_hw_code(0, 0)) < 0.01 && std::fabs(hw_out_ab(dac_hw_code(0, 4095)) - 8.0) < 0.05, "A spans 0 to about 8 V");

    // C and D: the logical value must come out as a rising 0 ... ~4.9 V ramp, in step with the logical value
    double worst = 0, prev = -1e9; bool mono = true;
    for (unsigned l = 0; l <= 4095; l++) {
        double v = hw_out_cd(dac_hw_code(2, l), 11.75);          // the rail the constant was calibrated for
        double want = (double)l / 4095.0 * CVOUT_CD_OFFSET_V;
        worst = std::fmax(worst, std::fabs(v - want));
        if (v < prev - 1e-9) mono = false;
        prev = v;
    }
    printf("  C/D: logical 0 -> %.3f V, 4095 -> %.3f V, worst error %.1f mV\n",
           hw_out_cd(dac_hw_code(2, 0)), hw_out_cd(dac_hw_code(2, 4095)), worst * 1000.0);
    check(mono, "C/D output rises monotonically with the logical value");
    // one C/D DAC step is 4.03 mV; the model rail (11.75 V -> 4.896 V) also differs 4 mV from the 4.9 V constant
    check(worst < 0.008, "C/D tracks the logical value within about 2 DAC steps after the mapping", worst, 0.008);
    check(dac_hw_code(2, 0) == dac_hw_code(3, 0) && dac_hw_code(2, 3000) == dac_hw_code(3, 3000), "C and D use the same mapping");
    // gate idles low, goes high
    check(hw_out_cd(dac_hw_code(2, 0)) < 0.02, "a low gate on C is about 0 V, not +5 V", hw_out_cd(dac_hw_code(2, 0)), 0);
    check(hw_out_cd(dac_hw_code(2, 4095)) > 4.8, "a high gate on C is about +4.9 V, not negative", hw_out_cd(dac_hw_code(2, 4095)), 4.8);
    // a different rail changes the zero point by about |dV| x 0.42 only; it must stay small
    printf("  rail sensitivity: -12.0 V rail -> %.3f V at logical 0; -11.5 V rail -> %.3f V\n",
           hw_out_cd(dac_hw_code(2, 0), 12.0), hw_out_cd(dac_hw_code(2, 0), 11.5));
    check(std::fabs(hw_out_cd(dac_hw_code(2, 0), 12.0)) < 0.15 && std::fabs(hw_out_cd(dac_hw_code(2, 0), 11.5)) < 0.15, "zero point stays within 150 mV over a +/-0.25 V rail change");

    // bipolar helper
    printf("  bipolar: ");
    for (double v : {-5.0, -2.5, 0.0, 2.5, 4.9}) {
        unsigned c = cvout_cd_code_from_volts((float)v);
        double got = hw_out_cd(c);
        printf("%+.1f V -> code %u -> %+.3f V   ", v, c, got);
        check(std::fabs(got - v) < 0.006, "cvout_cd_code_from_volts hits the requested voltage", got, v);
    }
    printf("\n");
    check(cvout_cd_code_from_volts(-50.0f) == 4095 && cvout_cd_code_from_volts(50.0f) == 0, "bipolar helper clamps to the DAC range");

    printf("CV input mapping\n");
    check(cv_from_counts(2048.0f) == 0.0f, "mid-scale reads zero");
    check(cv_from_counts(2048.0f + 25.0f) == 0.0f && cv_from_counts(2048.0f - 25.0f) == 0.0f, "dead zone holds zero");
    check(cv_from_counts(0.0f) > 0.99f && cv_from_counts(0.0f) <= 1.0f, "count 0 (+5 V) reads about +1", cv_from_counts(0.0f), 1.0);
    check(cv_from_counts(4095.0f) < -0.99f && cv_from_counts(4095.0f) >= -1.0f, "count 4095 (-5 V) reads about -1", cv_from_counts(4095.0f), -1.0);
    // monotonic decreasing with counts, continuous at the dead-zone edge
    double pc = 2.0; bool mono2 = true; double maxjump = 0;
    for (int c = 0; c <= 4095; c++) { double v = cv_from_counts((float)c); if (v > pc + 1e-9) mono2 = false; if (c) maxjump = std::fmax(maxjump, std::fabs(v - pc)); pc = v; }
    check(mono2, "CV value falls as counts rise (inverting input stage)");
    check(maxjump < 0.002, "no jump at the dead-zone edge", maxjump, 0.002);
    // end to end through the modelled input stage: Vin / 5 within a few percent
    double werr = 0;
    for (double vin = -5.0; vin <= 5.0001; vin += 0.25) {
        double cv = cv_from_counts((float)hw_cv_counts(vin, 12.0));
        werr = std::fmax(werr, std::fabs(cv - vin / 5.0));
    }
    printf("  end-to-end (-12.0 V rail): worst |cv - Vin/5| = %.3f\n", werr);
    check(werr < 0.025, "cv ~ Vin/5 (positive volts give positive values)", werr, 0.025);
    // with a rail 0.3 V low the unpatched (grounded) jack should still be near zero
    double z = cv_from_counts((float)hw_cv_counts(0.0, 11.7));
    printf("  unpatched jack with a -11.7 V rail reads %.4f\n", z);
    check(std::fabs(z) < 0.02, "unpatched jack stays near zero with a slightly low rail", z, 0.02);
    check(cv_from_counts(0.0f) > 0 && cv_from_counts(4095.0f) < 0, "sign: +5 V positive, -5 V negative");

    printf("\nRESULT: %s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
