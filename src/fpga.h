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
} fpga53_diag_t;
void fpga53_get_diag(fpga53_diag_t *d);
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
