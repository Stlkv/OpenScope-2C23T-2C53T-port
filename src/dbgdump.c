#include "dbgdump.h"

#include "fpga.h"
#include "fw_update.h"
#include "hw.h"
#include "scope.h"
#include "ui.h"

#ifndef FPGA53_BITSTREAM_EXTERN
#define FPGA53_BITSTREAM_EXTERN 1
#endif

/* Host-triggered telemetry dump: the host drops an empty DBGREQ file on the
 * USB volume, the idle scan spots it, deletes it and (re)writes DBG.TXT with
 * this text. Everything the LCD debug overlay shows, plus raw GPIO/SPI state,
 * readable without buttons or screenshots. */

/* Sweep builds trade the roll and pin-state lines for the sweep table.
 * The flags themselves live in fpga.h — see the note there for what happened
 * when this file kept its own copy of their defaults. */

static uint16_t put_str(char *dst, uint16_t cap, uint16_t p, const char *s) {
    while (*s && p < cap) {
        dst[p++] = *s++;
    }
    return p;
}

static uint16_t put_hex(char *dst, uint16_t cap, uint16_t p, uint32_t v, uint8_t digits) {
    while (digits--) {
        if (p < cap) {
            dst[p++] = "0123456789ABCDEF"[(v >> (4u * digits)) & 0xFu];
        }
    }
    return p;
}

static uint16_t put_dec(char *dst, uint16_t cap, uint16_t p, uint32_t v) {
    char tmp[10];
    uint8_t n = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && n < sizeof(tmp));
    while (n-- && p < cap) {
        dst[p++] = tmp[n];
    }
    return p;
}

uint16_t dbgdump_render(char *dst, uint16_t cap) {
    fpga53_diag_t dg;
    uint16_t p = 0;

    fpga53_get_diag(&dg);

    p = put_str(dst, cap, p, "OpenScope 2C53T DBG dump v2\n");

    /* Seam table first, ahead of everything else: the dump is capped at one
     * FAT cluster and this is the only part of it a seam build is run for.
     * One line per fresh frame, oldest first: r2, the first crossing's index,
     * then every crossing-to-crossing gap as a byte. A clean capture is a flat
     * run of half-periods; a glitch is a gap far below them. GLITCH sums that
     * up across every frame since boot, which is what a skip threshold has to
     * be chosen from — twelve rows are a sample, not a bound. */
#if FPGA53_SEAM_LOG
    {
        uint8_t rows = fpga53_seam_rows();

        uint8_t vmin = 0, vmax = 0, hi = 0, lo = 0;
        const uint8_t *strip = fpga53_seam_strip();

        p = put_str(dst, cap, p, "SEAM r1r2b0b1b2b3:first:gaps x");
        p = put_dec(dst, cap, p, rows);
        p = put_str(dst, cap, p, "\n");
        for (uint8_t i = 0; i < rows; ++i) {
            const fpga53_seam_row_t *r = fpga53_seam_row(i);

            for (uint8_t k = 0; k < 6u; ++k) {
                p = put_hex(dst, cap, p, r->pre[k], 2);
            }
            p = put_str(dst, cap, p, ":");
            p = put_hex(dst, cap, p, r->first, 3);
            p = put_str(dst, cap, p, ":");
            for (uint8_t g = 0; g < r->count; ++g) {
                p = put_hex(dst, cap, p, r->gap[g], 2);
            }
            p = put_str(dst, cap, p, "\n");
        }

        /* The window itself, every 16th sample, and the band the analyser cut
         * out of it. A table of zeroes says only "no periodic content found";
         * this says whether that is because the window is flat, because it is
         * noise, or because two outlier samples stretched the band past
         * everything real in between. */
        fpga53_seam_band(&vmin, &vmax, &hi, &lo);
        {
            uint16_t gmax = 0, gframes = 0, frames = 0;

            fpga53_seam_stats(&gmax, &gframes, &frames);
            uint16_t r1nz = 0, jump = 0;

            fpga53_seam_pre_stats(&r1nz, &jump);
            p = put_str(dst, cap, p, "PRE r1nz=");
            p = put_dec(dst, cap, p, r1nz);
            p = put_str(dst, cap, p, " jump=");
            p = put_dec(dst, cap, p, jump);
            p = put_str(dst, cap, p, "\n");
            p = put_str(dst, cap, p, "GLITCH max=");
            p = put_dec(dst, cap, p, gmax);
            p = put_str(dst, cap, p, " hit=");
            p = put_dec(dst, cap, p, gframes);
            p = put_str(dst, cap, p, "/");
            p = put_dec(dst, cap, p, frames);
            p = put_str(dst, cap, p, "\n");
        }
        p = put_str(dst, cap, p, "BAND ");
        p = put_hex(dst, cap, p, vmin, 2);
        p = put_str(dst, cap, p, "-");
        p = put_hex(dst, cap, p, vmax, 2);
        p = put_str(dst, cap, p, " hi=");
        p = put_hex(dst, cap, p, hi, 2);
        p = put_str(dst, cap, p, " lo=");
        p = put_hex(dst, cap, p, lo, 2);
        p = put_str(dst, cap, p, "\nWIN16 ");
        for (uint8_t i = 0; i < FPGA53_SEAM_STRIP; ++i) {
            p = put_hex(dst, cap, p, strip[i], 2);
        }
        /* Raw and undecimated, because the whole defect fits inside it.
         * BADH is the same window head from the last frame with a glitch in
         * it, prefixed by that frame's first crossing and leading gaps. */
        {
            const uint8_t *head = fpga53_seam_head();
            const uint8_t *bad;
            const uint8_t *bgap = 0;
            uint16_t bfirst = 0;

            p = put_str(dst, cap, p, "\nHEAD ");
            for (uint8_t i = 0; i < FPGA53_SEAM_HEAD; ++i) {
                p = put_hex(dst, cap, p, head[i], 2);
            }
            bad = fpga53_seam_bad_head(&bfirst, &bgap);
            p = put_str(dst, cap, p, "\nBADH ");
            p = put_hex(dst, cap, p, bfirst, 3);
            p = put_str(dst, cap, p, ":");
            for (uint8_t i = 0; i < 4u; ++i) {
                p = put_hex(dst, cap, p, bgap[i], 2);
            }
            p = put_str(dst, cap, p, ":");
            for (uint8_t i = 0; i < FPGA53_SEAM_HEAD; ++i) {
                p = put_hex(dst, cap, p, bad[i], 2);
            }
        }
        p = put_str(dst, cap, p, "\n");
    }
#endif

    /* The updater's own state. Without it a self-flash that silently refused
     * to apply is indistinguishable from one that applied — the file leaves
     * the volume either way, and the frame counters only tell you a boot is
     * old or young, not which image it is running. st: 0 idle, 1 staging,
     * 2 ready, 3 error, 4 applying; err: 1 range, 2 vector, 3 order. */
    {
        fw_update_status_t fw;

        fw_update_status(&fw);
        p = put_str(dst, cap, p, "FW st=");
        p = put_dec(dst, cap, p, fw.state);
        p = put_str(dst, cap, p, " err=");
        p = put_dec(dst, cap, p, fw.error);
        p = put_str(dst, cap, p, " b=");
        p = put_dec(dst, cap, p, fw.bytes);
        p = put_str(dst, cap, p, "/");
        p = put_dec(dst, cap, p, fw.expected_size);
        p = put_str(dst, cap, p, " seq=");
        p = put_dec(dst, cap, p, fw.sequence);
        p = put_str(dst, cap, p, "\n");
    }

    p = put_str(dst, cap, p, "I");
    p = put_dec(dst, cap, p, dg.inited);
    p = put_str(dst, cap, p, " P");
    p = put_dec(dst, cap, p, dg.pc0);
    p = put_str(dst, cap, p, " R");
    p = put_dec(dst, cap, p, dg.reads);
    p = put_str(dst, cap, p, " C");
    p = put_dec(dst, cap, p, dg.forced);
    p = put_str(dst, cap, p, " S");
    p = put_dec(dst, cap, p, scope_hw_last_status());
    p = put_str(dst, cap, p, " F");
    p = put_dec(dst, cap, p, scope_hw_frame_count());
    p = put_str(dst, cap, p, " W");
    p = put_dec(dst, cap, p, scope_hw_wait_count());
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "r0=");
    p = put_hex(dst, cap, p, dg.r0, 2);
    p = put_str(dst, cap, p, " r1=");
    p = put_hex(dst, cap, p, dg.r1, 2);
    p = put_str(dst, cap, p, " r2=");
    p = put_hex(dst, cap, p, dg.r2, 2);
    p = put_str(dst, cap, p, " ch1=");
    p = put_hex(dst, cap, p, dg.smin, 2);
    p = put_str(dst, cap, p, "-");
    p = put_hex(dst, cap, p, dg.smax, 2);
    p = put_str(dst, cap, p, " ch2=");
    p = put_hex(dst, cap, p, dg.smin2, 2);
    p = put_str(dst, cap, p, "-");
    p = put_hex(dst, cap, p, dg.smax2, 2);
    p = put_str(dst, cap, p, " D");
    p = put_dec(dst, cap, p, dg.dup);
    p = put_str(dst, cap, p, "\n");

#if HW_TARGET_2C53T
    {
        uint8_t now = 0, want = 0;
        uint32_t ns = 0;
        uint16_t calls = 0, skips = 0, sent = 0;

        fpga53_tb_debug(&now, &want, &ns, &calls, &skips, &sent);
        p = put_str(dst, cap, p, "TB now=");
        p = put_hex(dst, cap, p, now, 2);
        p = put_str(dst, cap, p, " want=");
        p = put_hex(dst, cap, p, want, 2);
        p = put_str(dst, cap, p, " ns=");
        p = put_dec(dst, cap, p, ns);
        p = put_str(dst, cap, p, " calls=");
        p = put_dec(dst, cap, p, calls);
        p = put_str(dst, cap, p, " skip=");
        p = put_dec(dst, cap, p, skips);
        p = put_str(dst, cap, p, " sent=");
        p = put_dec(dst, cap, p, sent);
        p = put_str(dst, cap, p, "\n");
    }
#endif
    /* Both channels, measured the same way, in the same run. Everything this
     * port measured after CH2 was proven independent on 2026-08-14 came off
     * the CH1 window alone, so CH2's health after the DMA read, the head skip
     * and the sample-rate work is untested — and comparing a fresh CH2 number
     * against a CH1 number from another build's journal entry is exactly the
     * kind of eyeballing that has cost this project days.
     *
     *   w   = trimmed window range this frame (ADC offset already removed)
     *   env = envelope across frames since the previous dump; the liveness
     *         test on inputs slower than the 166 us window
     *   e/T16 = rising crossings and mean period x16, window samples
     *   G   = glitch reach: furthest start / frames hit / fresh frames */
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        uint16_t gmax = 0, hit = 0, frames = 0;
        uint8_t emin = 0, emax = 0;

        fpga53_glitch_stats(ch, &gmax, &hit, &frames);
        fpga53_window_envelope(ch, &emin, &emax);
        p = put_str(dst, cap, p, ch ? "C2 w=" : "C1 w=");
        p = put_hex(dst, cap, p, ch ? dg.wmin2 : dg.wmin, 2);
        p = put_str(dst, cap, p, "-");
        p = put_hex(dst, cap, p, ch ? dg.wmax2 : dg.wmax, 2);
        p = put_str(dst, cap, p, " env=");
        p = put_hex(dst, cap, p, emin, 2);
        p = put_str(dst, cap, p, "-");
        p = put_hex(dst, cap, p, emax, 2);
        p = put_str(dst, cap, p, " e=");
        p = put_dec(dst, cap, p, ch ? dg.win_edges2 : dg.win_edges);
        p = put_str(dst, cap, p, " T16=");
        p = put_dec(dst, cap, p, ch ? dg.win_period2 : dg.win_period);
        p = put_str(dst, cap, p, " G=");
        p = put_dec(dst, cap, p, gmax);
        p = put_str(dst, cap, p, "/");
        p = put_dec(dst, cap, p, hit);
        p = put_str(dst, cap, p, "/");
        p = put_dec(dst, cap, p, frames);
        p = put_str(dst, cap, p, "\n");
    }

    p = put_str(dst, cap, p, "Q=");
    for (uint8_t i = 0; i < 5u; ++i) {
        p = put_hex(dst, cap, p, dg.cst[i], 2);
        if (i < 4u) {
            p = put_str(dst, cap, p, " ");
        }
    }
    /* A= (pre-command sweep) and X=/Y= (frontend pattern indices) dropped
     * 2026-08-15 for flash room: both belong to hunts that finished — the
     * preamble sweep and the CH2 GPIO search — and the flash bought the
     * PC0-gated read instead. */
    p = put_str(dst, cap, p, "\n");

    /* Window shape + the MCU-paced slow timebase. edges/period are measured
     * in window samples (period x16), so they answer "did the sample rate
     * change?" without a photograph. psc/pr are what TMR1 actually got.
     * Not in sweep builds — those run on a fast timebase where roll is idle,
     * and the flash they free is what the sweep table costs. */
#if !FPGA53_SWEEP_TIMING
    {
        uint16_t psc = 0;
        uint16_t pr = 0;
        uint16_t cost = 0;
        uint16_t over = 0;

        scope_hw_slow_timer_debug(&psc, &pr, &cost, &over);
        p = put_str(dst, cap, p, "WIN e=");
        p = put_dec(dst, cap, p, dg.win_edges);
        p = put_str(dst, cap, p, " T16=");
        p = put_dec(dst, cap, p, dg.win_period);
        p = put_str(dst, cap, p, " ROLL n=");
        p = put_dec(dst, cap, p, dg.slow_points);
        p = put_str(dst, cap, p, " p=");
        p = put_hex(dst, cap, p, dg.slow_min, 2);
        p = put_str(dst, cap, p, "-");
        p = put_hex(dst, cap, p, dg.slow_max, 2);
        p = put_str(dst, cap, p, dg.slow_full ? " FULL" : " short");
        p = put_str(dst, cap, p, dg.dma_fail ? " POLLED" : " dma");
        p = put_str(dst, cap, p, " psc=");
        p = put_dec(dst, cap, p, psc);
        p = put_str(dst, cap, p, " pr=");
        p = put_dec(dst, cap, p, pr);
        p = put_str(dst, cap, p, " cost=");
        p = put_dec(dst, cap, p, cost);
        p = put_str(dst, cap, p, " over=");
        p = put_dec(dst, cap, p, over);
        p = put_str(dst, cap, p, "\n");
    }
#endif

    /* Timing-register sweep table: one line per register, one field per value
     * as <spread><edges><period16> in hex. A register that divides the sample
     * rate shows edges climbing and period falling along its line. Compiled
     * only into sweep builds — the app is within a few hundred bytes of the
     * 224 KB self-update ceiling. */
#if FPGA53_SWEEP_TIMING
    {
        uint8_t row_count = 0;
        const fpga53_tsweep_row_t *rows = fpga53_tsweep_table(&row_count);
        const fpga53_tsweep_row_t *base = fpga53_tsweep_baseline();
        const uint8_t *regs = 0;
        const uint8_t *vals = 0;
        uint8_t reg_count = 0;
        uint8_t val_count = 0;

        fpga53_tsweep_axes(&regs, &reg_count, &vals, &val_count);
        if (rows && row_count && base) {
            p = put_str(dst, cap, p, "TSW row=");
            p = put_dec(dst, cap, p, dg.tsweep_row);
            p = put_str(dst, cap, p, dg.tsweep_done ? "! now=" : ". now=");
            p = put_hex(dst, cap, p, dg.tsweep_reg, 2);
            p = put_str(dst, cap, p, ":");
            p = put_hex(dst, cap, p, dg.tsweep_val, 2);
            p = put_str(dst, cap, p, " base=");
            p = put_hex(dst, cap, p, base->spread, 2);
            p = put_hex(dst, cap, p, base->edges, 2);
            p = put_hex(dst, cap, p, base->period, 4);
            p = put_str(dst, cap, p, "\n");
            for (uint8_t r = 0; r < reg_count; ++r) {
                p = put_hex(dst, cap, p, regs[r], 2);
                for (uint8_t v = 0; v < val_count; ++v) {
                    uint8_t idx = (uint8_t)(r * val_count + v);
                    p = put_str(dst, cap, p, " ");
                    if (idx < dg.tsweep_row) {
                        p = put_hex(dst, cap, p, rows[idx].spread, 2);
                        p = put_hex(dst, cap, p, rows[idx].edges, 2);
                        p = put_hex(dst, cap, p, rows[idx].period, 4);
                    } else {
                        p = put_str(dst, cap, p, "--------");
                    }
                }
                p = put_str(dst, cap, p, "\n");
            }
        }
    }
#endif

    p = put_str(dst, cap, p, "V=");
    p = put_hex(dst, cap, p, dg.v04_id, 8);
    p = put_str(dst, cap, p, " B=");
    p = put_hex(dst, cap, p, dg.v04_stb, 8);
    p = put_str(dst, cap, p, " A=");
    p = put_hex(dst, cap, p, dg.v04_sta, 8);
    /* WARM=1: this boot skipped configuration on purpose (the RAM token said
     * the FPGA already carries this build's bitstream), so V/B/A are zero
     * because nothing was read, not because the port went silent. */
    p = put_str(dst, cap, p, " WARM=");
    p = put_hex(dst, cap, p, dg.warm, 1);
#if FPGA53_BITSTREAM_EXTERN
    /* BS=0 means the bitstream store is empty or corrupt: nothing was
     * uploaded, on purpose. Drop the store file on the volume to fix it.
     * Meaningless in the provisioning image, which carries its own payload. */
    p = put_str(dst, cap, p, " BS=");
    p = put_hex(dst, cap, p, dg.bs_ok, 1);
#endif
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "calls init=");
    p = put_dec(dst, cap, p, dg.init_calls);
    p = put_str(dst, cap, p, " cfg=");
    p = put_dec(dst, cap, p, dg.cfg_calls);
    p = put_str(dst, cap, p, " poll=");
    p = put_dec(dst, cap, p, dg.poll_calls);
    p = put_str(dst, cap, p, "\n");

#if !FPGA53_SWEEP_TIMING
    p = put_str(dst, cap, p, "IDR A=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOA_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " B=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOB_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " C=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOC_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " E=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOE_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " D=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOD_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, "\n");

    /* The frontend, as pins rather than as intent. crh0 is GPIOD CRH at boot,
     * before we wrote anything: nibbles 4 and 5 (PD12, PD13) reading 8 or B
     * mean alternate function, where EXMC owns the pin and our level writes go
     * nowhere — upstream's unit boots 0xBB4BBBBB and is AC-coupled forever
     * because of it. crh is the same register now. r2 is the relay code this
     * build wrote to CH2's bank; the four pin levels after it are read back
     * from IDR, so a bank that did not take is visible rather than assumed. */
    {
        uint32_t a = GPIO_IDR(GPIOA_BASE);
        uint32_t b = GPIO_IDR(GPIOB_BASE);
        uint32_t c = GPIO_IDR(GPIOC_BASE);
        uint32_t e = GPIO_IDR(GPIOE_BASE);
        uint32_t d = GPIO_IDR(GPIOD_BASE);

        p = put_str(dst, cap, p, "FE crh0=");
        p = put_hex(dst, cap, p, dg.crh_boot, 8);
        p = put_str(dst, cap, p, " crh=");
        p = put_hex(dst, cap, p, GPIO_CRH(GPIOD_BASE), 8);
        p = put_str(dst, cap, p, " r2=");
        p = put_hex(dst, cap, p, dg.fe_ch2, 2);
        p = put_str(dst, cap, p, " ch1[c12,e4,e5,e6]=");
        p = put_dec(dst, cap, p, (c >> 12) & 1u);
        p = put_dec(dst, cap, p, (e >> 4) & 1u);
        p = put_dec(dst, cap, p, (e >> 5) & 1u);
        p = put_dec(dst, cap, p, (e >> 6) & 1u);
        p = put_str(dst, cap, p, " ch2[a15,b11,b10,a10]=");
        p = put_dec(dst, cap, p, (a >> 15) & 1u);
        p = put_dec(dst, cap, p, (b >> 11) & 1u);
        p = put_dec(dst, cap, p, (b >> 10) & 1u);
        p = put_dec(dst, cap, p, (a >> 10) & 1u);
        p = put_str(dst, cap, p, " cpl[d12,d13]=");
        p = put_dec(dst, cap, p, (d >> 12) & 1u);
        p = put_dec(dst, cap, p, (d >> 13) & 1u);
        p = put_str(dst, cap, p, "\n");

        /* What PB11 was doing while config and the arm ran, and which build
         * this is: without the flag in the dump, a run that behaved normally
         * cannot be told from a run of the ordinary build. */
#if FPGA53_OP0A_PROBE
        /* Both passes, all six bytes, and the 16-bit value stock would have
         * assembled from them. Printed raw so a reader can disagree with our
         * assembly without reflashing anything. */
        for (uint8_t pass = 0; pass < 2u; ++pass) {
            p = put_str(dst, cap, p, "OP0A pass");
            p = put_dec(dst, cap, p, pass);
            p = put_str(dst, cap, p, " f09=");
            for (uint8_t i = 0; i < 3u; ++i) {
                p = put_hex(dst, cap, p, dg.op09_bytes[pass][i], 2);
            }
            p = put_str(dst, cap, p, " f0A=");
            for (uint8_t i = 0; i < 3u; ++i) {
                p = put_hex(dst, cap, p, dg.op0a_bytes[pass][i], 2);
            }
            p = put_str(dst, cap, p, " v=");
            p = put_hex(dst, cap, p,
                        ((uint32_t)dg.op09_bytes[pass][2] << 8) |
                            dg.op0a_bytes[pass][2],
                        4);
            p = put_str(dst, cap, p, "\n");
        }
#endif

        p = put_str(dst, cap, p, "PB11 cfglow=");
        p = put_dec(dst, cap, p, (uint32_t)FPGA53_PB11_LOW_AT_CONFIG);
        p = put_str(dst, cap, p, " idrb_arm=");
        p = put_hex(dst, cap, p, dg.idrb_arm, 4);
        p = put_str(dst, cap, p, " b11_arm=");
        p = put_dec(dst, cap, p, (dg.idrb_arm >> 11) & 1u);
        p = put_str(dst, cap, p, "\n");
    }

#if FPGA53_RELAY_SWEEP
    /* One line per relay row: the code and the envelope CH2 showed under it.
     * Read with a fixed input on the CH2 probe, this is the attenuation
     * ladder — the thing the volts/div knob needs before it can mean volts. */
    {
        uint8_t pass = 0;

        p = put_str(dst, cap, p, "RSW row:code:min-max\n");
        for (uint8_t i = 0; i < 10u; ++i) {
            uint8_t code = 0, mn = 0, mx = 0;

            fpga53_relay_sweep_get(i, &code, &mn, &mx, &pass);
            p = put_dec(dst, cap, p, i);
            p = put_str(dst, cap, p, ":");
            p = put_hex(dst, cap, p, code, 2);
            p = put_str(dst, cap, p, ":");
            p = put_hex(dst, cap, p, mn, 2);
            p = put_str(dst, cap, p, "-");
            p = put_hex(dst, cap, p, mx, 2);
            p = put_str(dst, cap, p, i == 9u ? "\n" : " ");
        }
        p = put_str(dst, cap, p, "RSW pass=");
        p = put_dec(dst, cap, p, pass);
        p = put_str(dst, cap, p, "\n");
    }
#endif

    /* The windows themselves, decimated: 64 raw samples stepped across the
     * trimmed region each channel's renderer draws from. This is the line that
     * says whether a window carries the periods its edge count claims. */
    for (uint8_t ch = 0; ch < 2u; ++ch) {
        uint8_t len = 0, step = 0;
        const uint8_t *s = fpga53_window_strip(ch, &len, &step);

        p = put_str(dst, cap, p, ch ? "S2/" : "S1/");
        p = put_dec(dst, cap, p, step);
        p = put_str(dst, cap, p, " ");
        for (uint8_t i = 0; i < len; ++i) {
            p = put_hex(dst, cap, p, s[i], 2);
        }
        p = put_str(dst, cap, p, "\n");
    }

    /* The ODR line (what our code wrote, against IDR's actual pin levels) was
     * dropped 2026-08-15 for flash room. It existed for the frontend-pose
     * defect of 2026-08-14, which is fixed; IDR still shows the levels, and
     * POSE= counts the re-applications. Bring it back with pose work. */

    /* The CR (pin mode) line lived here until 2026-08-15. It did its job —
     * the frontend pose defect it was added for is fixed and IDR/ODR still
     * show the levels — and the app has under 500 bytes of flash left, so it
     * is gone rather than the roll telemetry. Bring it back with the pose
     * work, not before. */
#endif

    p = put_str(dst, cap, p, "SPI3 CTRL1=");
    p = put_hex(dst, cap, p, SPI_CTRL1(SPI3_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " MODE=");
    p = put_hex(dst, cap, p, ui_debug_mode_byte(), 2);
    p = put_str(dst, cap, p, " POSE=");
    p = put_dec(dst, cap, p, dg.pose_calls);
    p = put_str(dst, cap, p, "\n");

    return p;
}
