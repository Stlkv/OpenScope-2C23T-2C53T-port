#pragma once

#include <stdint.h>

enum {
    FPGA_SAMPLE_COUNT = 2048,
    FPGA_SCOPE_BUFFER_BYTES = FPGA_SAMPLE_COUNT * 2,
};

#ifndef HW_TARGET_2C53T
#define HW_TARGET_2C53T 0
#endif

#if HW_TARGET_2C53T
typedef struct {
    uint8_t inited;
    uint8_t pc0;
    uint16_t reads;
    uint16_t forced;
    /* CH1 read preamble. Not three status bytes, whatever this used to be
     * called: measured 2026-08-16 over 193 frames, r1 was 00 in every single
     * one, and r2 tracked the window's first sample continuously in 96% of
     * them — the rest being windows that opened on a steep edge, where one
     * sample step legitimately reaches 0x3B. So r2 IS the first sample and we
     * discard it. That costs one sample in 1024, inside the head we now skip
     * anyway, which is why the read is left alone. */
    uint8_t r0, r1, r2;
    uint8_t smin, smax;   /* raw CH1 sample range, last read */
    uint16_t init_calls;  /* fpga_init_once entries */
    uint16_t cfg_calls;   /* scope_hw_configure_channels entries */
    uint16_t poll_calls;  /* scope_poll_frame entries (ui) */
    uint8_t cst[5];       /* SPI3 status-read 0x03 reply (stock: 00 01 42 2E 2E) */
    uint8_t smin2, smax2; /* raw CH2-window sample range, last read */
    uint8_t dup;          /* 1 = CH2 window byte-identical to CH1 window */
    uint8_t fe_idx;       /* frontend experiment pattern A (PC12/PE4/PE5/PE6), 0xFF = untouched */
    uint8_t fe_idx_b;     /* frontend experiment pattern B (PA15/PA10/PB9/PA6), 0xFF = untouched */
    uint32_t v04_id;      /* V0.4 config attempt: IDCODE readback */
    uint32_t v04_stb;     /* status before upload */
    uint32_t v04_sta;     /* status after upload */
    uint8_t warm;         /* 1 = warm boot: config skipped, FPGA kept as-is */
    uint8_t bs_ok;        /* 1 = bitstream store holds a whole, matching payload */
    uint8_t sweep_val;    /* auto-sweep: current pre-cmd byte */
    uint8_t sweep_hit;    /* auto-sweep: 1 = CH2 spread detected, sweep frozen */
    uint16_t pose_calls;  /* fpga53_scope_pose_reapply entries (scope-mode entries) */
    uint16_t win_edges;   /* CH1 window: rising crossings of mid level, last read */
    uint16_t win_period;  /* CH1 window: mean edge-to-edge distance, samples x16 */
    /* 32-bit on purpose: at the rates this sampler actually reaches a 16-bit
     * counter wraps in seconds, which makes "points per wall-clock second"
     * measured from two dumps ambiguous. */
    uint32_t slow_points;
    uint16_t slow_busy;   /* slow points skipped because the SPI3 bus was busy */
    uint8_t slow_min;     /* CH1 slow points: span of the last completed block */
    uint8_t slow_max;     /* — a flat span means the points carry no signal */
    uint8_t slow_full;    /* 1 = each point drains a whole window (re-arm test) */
    uint8_t dma_fail;     /* 1 = DMA transfers never completed, fell back to polling */
    uint8_t tsweep_reg;   /* timing sweep: register being written now */
    uint8_t tsweep_val;   /* timing sweep: value being written now */
    uint8_t tsweep_row;   /* timing sweep: rows recorded so far */
    uint8_t tsweep_done;  /* timing sweep: 1 = table complete */
} fpga53_diag_t;

/* Leading and trailing window samples to drop: the engine settles for the
 * first part of every acquisition, and drops samples while it does.
 *
 * Measured 2026-08-16 with the raw heads and the GLITCH counter in DBG.TXT.
 * A glitch is a sample or several falling off the plateau followed by a
 * ~10-sample exponential recovery — a signal cannot fall in one sample and
 * rise in ten, so it is the acquisition and not the input. Over 168 fresh
 * frames 124 carried one, and the furthest started at sample 91; with the
 * glitch width and its recovery that puts the contaminated head at roughly
 * the first 130 samples, or the first 26 us of a 205 us window. Past that the
 * gap list is a perfect grid in every frame — which is why four earlier
 * explanations (PC0 gating, ping-pong, the read/write race, ring addressing)
 * and a fifth (rotation by r2) all came back negative: every one of them was
 * a statement about the whole window.
 *
 * A head skip of 160 took the glitch rate from 124/168 to 3/171, and those
 * three sat at sample 999 — the other end. Hence the tail trim.
 *
 * Why the engine needs this long is still open, and the obvious guess is
 * wrong: stock does NOT drop a head. Its normal-timebase renderer transforms
 * adc_buf_ch1[0..300] — the first 301 samples of its 1024-sample buffer, head
 * included (stock RE, scope_render_monsters_annotated.c). What stock does and
 * we do not is bracket the read: a precursor that sends the timebase index and
 * refuses to read until the engine has accumulated enough samples, then the
 * bulk read, then a re-arm. So this skip may be standing in for a handshake we
 * never perform, rather than for a settling time that has to exist.
 *
 * These live in the header because the renderer has to agree with them: the
 * window is no longer 1023 samples, and a UI that still asks for the whole
 * frame gets a flat line where the trimmed part used to be. */
#ifndef FPGA53_HEAD_SKIP
#define FPGA53_HEAD_SKIP 160u
#endif
#ifndef FPGA53_TAIL_SKIP
#define FPGA53_TAIL_SKIP 32u
#endif
enum {
    FPGA53_WINDOW_SAMPLES = 1023,
    /* Window samples that survive the trim, and the frame slots they fill
     * after the renderer's 2x stretch. */
    FPGA53_WINDOW_VALID =
        FPGA53_WINDOW_SAMPLES - (int)FPGA53_HEAD_SKIP - (int)FPGA53_TAIL_SKIP,
    FPGA53_FRAME_VALID = FPGA53_WINDOW_VALID * 2,
};

/* Seam hunt. The default lives here, in the header both fpga.c and dbgdump.c
 * include, on purpose: the sweep flags kept a private copy of their default in
 * each file, the two drifted apart, and a bench run then executed the sweep
 * while printing nothing — which reads exactly like a negative result
 * (2026-08-15). One definition, no mirror to forget. */
#ifndef FPGA53_SEAM_LOG
#define FPGA53_SEAM_LOG 0
#endif

#if FPGA53_SEAM_LOG
/* Where the frame's discontinuity sits, measured rather than eyeballed.
 * Everything outside the data has been falsified (PC0 gating, ping-pong, the
 * read/write race, ring addressing in the renderer, and — on 2026-08-16 — a
 * rotation following r2). What the gap lists then showed is that the defect
 * never moves: the tail of every window is a perfect grid and only the first
 * period or two is wrong.
 *
 * The whole gap list, not a summary of it. The first cut recorded only the
 * single worst interval, and on the bench that interval turned out to sit in
 * the first period of every frame — an artefact of where the hysteresis state
 * starts — which is exactly the shape that would hide a real seam further in.
 * A summary can only answer the question it was built around; the list lets
 * the next question be asked without another flash cycle. */
enum { FPGA53_SEAM_GAPS = 24 };

typedef struct {
    uint16_t first;                   /* window index of the first crossing */
    uint8_t r2;                       /* third byte of the read preamble */
    uint8_t count;                    /* gaps stored below */
    /* r1, r2 and the first four raw window samples of the same read.
     *
     * Stock's bulk read sends ONE echo byte and then takes 1024 data bytes;
     * ours sends the opcode and then discards two more bytes as status before
     * taking 1023. If those two are really samples, we throw away the start of
     * every window — and r2 has never once left the signal's own range, while
     * r1 has never been anything but 00. Six consecutive bytes settle it: a
     * sample sequence has to stay inside what the input can do, which on this
     * capture is a six-sample fall and a ten-sample rise. */
    uint8_t pre[6];
    uint8_t gap[FPGA53_SEAM_GAPS];    /* crossing-to-crossing, samples, 255 = over */
} fpga53_seam_row_t;

/* Oldest first; 0 past the end. */
const fpga53_seam_row_t *fpga53_seam_row(uint8_t i);
uint8_t fpga53_seam_rows(void);

/* Every-16th sample of the last analysed window, plus the band the analyser
 * derived from it. Without this a row of zeroes is ambiguous — a window with
 * no periodic content and a window the hysteresis band simply failed to
 * straddle look identical, and guessing between them costs a bench cycle. */
enum { FPGA53_SEAM_STRIP = 64, FPGA53_SEAM_HEAD = 80 };
const uint8_t *fpga53_seam_strip(void);
/* The first FPGA53_SEAM_HEAD raw samples, undecimated and unclamped. */
const uint8_t *fpga53_seam_head(void);
/* Same, from the most recent frame the analyser found a glitch in, with that
 * frame's first crossing and leading gaps. */
const uint8_t *fpga53_seam_bad_head(uint16_t *first, const uint8_t **gaps);
void fpga53_seam_band(uint8_t *vmin, uint8_t *vmax, uint8_t *hi, uint8_t *lo);
/* Glitch reach since boot: furthest start index, frames hit, frames seen. */
void fpga53_seam_stats(uint16_t *gmax, uint16_t *gframes, uint16_t *frames);
/* Read-preamble evidence since boot: frames with a non-zero r1, and frames
 * where r2 could not have been the sample before the window's first. */
void fpga53_seam_pre_stats(uint16_t *r1nz, uint16_t *jump);
#endif

/* One timing-sweep row: what the window looked like while <reg> held <val>.
 * period is samples x16 (0 = no periodic content measurable). */
typedef struct {
    uint8_t spread;
    uint8_t edges;
    uint16_t period;
} fpga53_tsweep_row_t;

/* Timing sweep table (FPGA53_SWEEP_TIMING builds; count is 0 otherwise).
 * Row order is register-major, matching fpga53_tsweep_axes(). */
const fpga53_tsweep_row_t *fpga53_tsweep_table(uint8_t *rows);
void fpga53_tsweep_axes(const uint8_t **regs,
                        uint8_t *reg_count,
                        const uint8_t **vals,
                        uint8_t *val_count);
const fpga53_tsweep_row_t *fpga53_tsweep_baseline(void);
void fpga53_get_diag(fpga53_diag_t *d);
void fpga53_note_configure(void);
void fpga53_note_poll(void);
void fpga53_fe_cycle(void);
void fpga53_fe_cycle_b(void);
/* Re-apply the scope analog posture (relays, gain keys, PC1/PC2/PC11 selector)
 * and the TMR13 CH2 trigger reference. Call on scope-mode entry: the meter
 * leaves its own posture behind and takes PA6 back as a GPIO. */
void fpga53_scope_pose_reapply(void);
/* Slow-point read length. Short reads (0) release CS after a few samples;
 * full reads (1) clock out the whole 1023-sample window, which is what a
 * normal capture read does — the open question is whether the engine only
 * refreshes its window once one has been consumed. The roll pacer picks the
 * mode by interval: a full read is ~1.4 ms of SPI and needs room. */
void fpga53_slow_point_set_full(uint8_t full);
#endif

void fpga_init_once(void);
uint8_t fpga_ready(void);
void fpga_write_timing(uint32_t tuning_word, uint32_t span);
void fpga_write_scope_timing(uint32_t span);
void fpga_write_signal_buffer(const uint8_t *data, uint16_t len);
void fpga_capture_latch(void);
uint8_t fpga_capture_ready(void);
void fpga_capture_ready_irq_handler(void);
uint8_t fpga_capture_read(uint8_t *dst, uint16_t len);
uint8_t fpga_capture_read_slow_point(uint8_t sample[2]);
