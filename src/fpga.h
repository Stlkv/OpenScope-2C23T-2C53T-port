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
    uint8_t sweep_val;    /* auto-sweep: current pre-cmd byte */
    uint8_t sweep_hit;    /* auto-sweep: 1 = CH2 spread detected, sweep frozen */
} fpga53_diag_t;
void fpga53_get_diag(fpga53_diag_t *d);
void fpga53_note_configure(void);
void fpga53_note_poll(void);
void fpga53_fe_cycle(void);
void fpga53_fe_cycle_b(void);
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
