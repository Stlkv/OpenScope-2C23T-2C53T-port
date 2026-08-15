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
    uint8_t r0, r1, r2;   /* CH1 frame status bytes */
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
