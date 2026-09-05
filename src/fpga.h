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
    /* The same two for CH2, plus the trimmed range of both windows.
     *
     * Every number this port has measured since CH2 was proven independent on
     * 2026-08-14 — the seam, the head skip, the sample-rate ladder — came off
     * the CH1 window alone: the metric ran on the 0x04 buffer and the seam
     * analyser only snapshotted opcode 0x04. CH2 had min/max over the whole
     * untrimmed window and nothing else, which cannot tell a live second
     * channel from a flat one that happens to sit at a different baseline.
     *
     * smin/smax above stay what they were, the raw range of the WHOLE window,
     * dirty head included. wmin/wmax are the trimmed window the renderer
     * actually draws, after the ADC offset subtraction — so the two channels'
     * baselines and swings are comparable within one run, which is the only
     * comparison worth making (journal numbers came off other builds). */
    uint16_t win_edges2;
    uint16_t win_period2;
    uint8_t wmin, wmax;
    uint8_t wmin2, wmax2;
    /* Frontend state, for the CH2 hunt. crh_boot is GPIOD CRH sampled at
     * fpga_init_once entry, BEFORE anything of ours writes PD12/PD13: if it
     * reads 0xBB4BBBBB our unit boots those pins in EXMC alternate function
     * like upstream's, where ODR is ignored and the input stays AC-coupled no
     * matter what we write. fe_ch2 is the relay code last written to CH2's
     * bank, so a dump says which row produced the numbers next to it. */
    uint32_t crh_boot;
    uint8_t fe_ch2;
    /* The op 0x09 / 0x0A pair, read twice back to back. Two passes because a
     * value that differs between them is a counter, not a constant, and that
     * distinction costs nothing to make here and cannot be made later from one
     * number. All three bytes of each frame are kept: upstream saw byte 0 of
     * the op-09 frame read 0x80 on five tries of six, which is either a
     * ready flag or noise, and only the raw bytes can ever say which. */
    /* Eight passes: four on the working read clock (/8) and four on /256, the
     * divider the arm writes and the status read already require. Four rather
     * than two per clock because one repeat cannot tell a value that settles
     * from a value that alternates. */
    uint8_t op09_bytes[8][3];
    uint8_t op0a_bytes[8][3];
    /* GPIOB sampled once, right after the arm writes and before the pose runs.
     * Bit 11 of it is the only honest answer to "what level did config and arm
     * see on PB11", since the relay row overwrites the pin milliseconds later. */
    uint32_t idrb_arm;
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
/* Where a channel sits with nothing on the probe, in samples as the renderer
 * sees them (the ADC offset is already subtracted by the read). Measured
 * 2026-08-16 across the ladder rows: CH2 held 0x28 on every unclipped row, CH1
 * read a few counts higher on the off-ladder code it used to be pinned to.
 * This is the zero the volts conversion refers to — 128 is what a signed ADC
 * would use and this one is not that. A per-channel calibration belongs here
 * eventually; one constant is what today's numbers support. */
#ifndef FPGA53_ZERO_COUNT
#define FPGA53_ZERO_COUNT 40
#endif

enum {
    FPGA53_WINDOW_SAMPLES = 1023,
    /* Window samples that survive the trim, and the frame slots they fill
     * after the renderer's 2x stretch. */
    FPGA53_WINDOW_VALID =
        FPGA53_WINDOW_SAMPLES - (int)FPGA53_HEAD_SKIP - (int)FPGA53_TAIL_SKIP,
    FPGA53_FRAME_VALID = FPGA53_WINDOW_VALID * 2,
};

/* Sweep flags. Same rule as the seam flag below: defined once, in the header
 * both fpga.c and dbgdump.c include. fpga.c and dbgdump.c each used to carry a
 * private copy of FPGA53_SWEEP_TIMING's default, the two drifted, and a bench
 * run executed the sweep while printing nothing (2026-08-15).
 *
 * FPGA53_SWEEP_TB01 walks the TWO-BYTE command `01 <idx>` over the timebase
 * indices 0x00-0x13. Stock's own auto-timebase does exactly this — hold an
 * index for LUT[idx]+0x32 ticks, then step — with the dwell table extracted
 * from the stock image at 0x0804D833 (base 0x08007000): 1 for indices 0-13,
 * then 3, 5, 9, 21, 41, 82. Our 2026-08-15 sweep sent the index as a BARE
 * byte with no command in front of it and measured a no-op, which is the
 * expected result for a malformed command. */
#ifndef FPGA53_SWEEP_TBIDX
#define FPGA53_SWEEP_TBIDX 0
#endif
#ifndef FPGA53_SWEEP_TB01
#define FPGA53_SWEEP_TB01 0
#endif
#ifndef FPGA53_SWEEP_TIMING
#define FPGA53_SWEEP_TIMING (FPGA53_SWEEP_TBIDX || FPGA53_SWEEP_TB01)
#endif

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

/* Point a channel's relay bank at the row its volts/div step calls for. The
 * stock relay table turned out to BE the volts/div ladder — ten 1-2-5 steps at
 * 25 counts per division, measured on the bench 2026-08-16 — and our nine
 * steps start one row in, so the mapping is index + 1 and nothing else.
 * ch: 0 = CH1, 1 = CH2. Writes only when the row changes; these are relays. */
void fpga53_set_channel_range(uint8_t ch, uint8_t vdiv_idx);

/* What one sample count is worth at a given volts/div, in microvolts. Falls
 * out of the same ladder: a division is 25 counts, so the setting alone fixes
 * it. Microvolts because the sensitive steps are under a millivolt a count. */
uint16_t fpga53_range_uv_per_count(uint8_t vdiv_idx, uint32_t vdiv_mv);

/* Glitch reach per channel: furthest sample a short gap started at, fresh
 * frames carrying one, fresh frames seen. Same rule the CH1-only seam build
 * used on 2026-08-16 (a crossing-to-crossing gap under 30 samples), but always
 * compiled in and counted for both windows, so "is CH2 as clean as CH1" is a
 * question one dump answers instead of two flash cycles. The 30-sample
 * threshold is tied to a 50 kHz input at 5 MSa/s, where the grid runs 48-52;
 * on a slower input the window holds no crossings at all and the counters just
 * stay at zero. ch: 0 = CH1 (opcode 0x04), 1 = CH2 (0x05). */
void fpga53_glitch_stats(uint8_t ch,
                         uint16_t *gmax,
                         uint16_t *hit,
                         uint16_t *frames);

/* Trimmed-window envelope across frames, per channel — min of the minima, max
 * of the maxima — cleared by the read. A single window is 166 us and holds a
 * flat level on any slow input, so this is the number that says whether a
 * channel follows a signal the window cannot contain. */
void fpga53_window_envelope(uint8_t ch, uint8_t *emin, uint8_t *emax);

/* Decimated raw samples of each channel's last window — the trimmed part, the
 * one the renderer draws, before the ADC offset subtraction. Returns the
 * buffer and reports its length and the sample step it was taken at. Summary
 * numbers cannot distinguish "the window holds eight periods" from "it holds
 * two and then a rail"; this can. */
const uint8_t *fpga53_window_strip(uint8_t ch, uint8_t *len, uint8_t *step);

/* Read stock's op 0x09 / 0x0A pair once the engine is armed (issue #18,
 * 2026-08-16). Upstream decoded the pair from the stock dispatch table — op
 * 0x09's handler reads a byte, toggles CS and opens a second frame with an
 * opcode nothing else reaches — and reads a stable 0x0089 on their bench unit,
 * meaning unknown. A second unit's value is the cheapest thing that separates
 * "constant of the design" from "something per-device", and no amount of
 * reading the image can produce it. Off by default: this sends opcodes our
 * port has never sent to a configured part. */
#ifndef FPGA53_OP0A_PROBE
#define FPGA53_OP0A_PROBE 0
#endif

/* Hold PB11 LOW through config and the arm writes (issue #18 discriminator,
 * see the note at its use in fpga.c). Lives here because the dump prints it,
 * and a flag whose default is written twice eventually disagrees with itself. */
#ifndef FPGA53_PB11_LOW_AT_CONFIG
#define FPGA53_PB11_LOW_AT_CONFIG 0
#endif

#ifndef FPGA53_RELAY_SWEEP
#define FPGA53_RELAY_SWEEP 0
#endif
#if FPGA53_RELAY_SWEEP
/* Relay-ladder sweep results: for each of the stock table's ten rows, the code
 * written and the envelope CH2's window showed while it was held. With a fixed
 * input on the CH2 probe these spans are the attenuation ladder, which is what
 * a volts/div knob has to be bound to. pass = how many complete laps have been published (the table is
 * always a whole pass, never a mix of two). */
void fpga53_relay_sweep_get(uint8_t row, uint8_t *code, uint8_t *mn, uint8_t *mx,
                            uint8_t *pass);
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
/* CH2 vertical-offset reference: TMR13 CH1 PWM on PA6 through the board's RC
 * filter, NOT DAC2 — this board's CH2 comparator does not sit on a DAC at all.
 * `set` clamps to 12 bits and arms TMR13 if the boot path has not; `armed`
 * says whether the timer is actually running (false on builds without
 * FPGA53_TMR13_REF, where `set` is a no-op and `get` only reports the default).
 * The centering code is a measured quantity — see FPGA53_TMR13_REF_CODE. */
void fpga53_ch2_ref_set(uint16_t code);
/* Adopt a stored code without arming — for the boot path, before the owner of
 * PA6 is decided. Values above 12 bits are ignored, which is how the settings
 * store says "never measured on this unit". */
void fpga53_ch2_ref_preset(uint16_t code);
/* CH1's vertical-offset reference: DAC1 on PA4. Same contract as CH2's, and
 * the same per-unit story — mid-scale is not centre. */
void fpga53_ch1_ref_set(uint16_t code);
void fpga53_ch1_ref_preset(uint16_t code);
uint16_t fpga53_ch1_ref_get(void);
uint8_t fpga53_ch1_ref_armed(void);
/* Write one scope-engine register over SPI3, at the divider and framing the arm
 * sequence uses. Returns 0 if the FPGA is not up. Register 0x08 is the
 * candidate digital trigger level (arm value 0xAD); the effect of changing it
 * has not been measured. */
uint8_t fpga53_scope_reg_write(uint8_t reg, uint8_t value);
uint8_t fpga53_trig_level_get(void);
uint16_t fpga53_ch2_ref_get(void);
uint8_t fpga53_ch2_ref_armed(void);
/* Slow-point read length. Short reads (0) release CS after a few samples;
 * full reads (1) clock out the whole 1023-sample window, which is what a
 * normal capture read does — the open question is whether the engine only
 * refreshes its window once one has been consumed. The roll pacer picks the
 * mode by interval: a full read is ~1.4 ms of SPI and needs room. */
void fpga53_slow_point_set_full(uint8_t full);

/* Sample-rate ladder, measured 2026-08-16 (EXPERIMENT-LOG, "THE SAMPLE-RATE
 * DIVIDER FOUND"). The engine takes the two-byte SPI3 command `01 <idx>`;
 * against a 50 kHz square the indices came out as a 1-2-5 ladder on round
 * sample counts:
 *
 *   idx 07  12.5 MSa/s     idx 0A  1.25 MSa/s
 *   idx 08   5.00          idx 0B  0.50
 *   idx 09   2.50          idx 0C  0.247
 *
 * Set the rate for a UI timebase step, and ask what a frame-buffer entry is
 * worth in nanoseconds — the renderer needs the second to size the visible
 * window, and it was a hardcoded 100 while there was only one rate. */
void fpga53_set_timebase(uint8_t ui_timebase);
uint32_t fpga53_frame_entry_ns(void);
/* What the setter has actually done: where it thinks the engine is, what was
 * last asked for, and how many times it ran, bailed out and wrote. */
void fpga53_tb_debug(uint8_t *now, uint8_t *want, uint32_t *entry_ns,
                     uint16_t *calls, uint16_t *skips, uint16_t *sent);
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
