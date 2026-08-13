#pragma once

#include <stdint.h>

void dmm_init(void);
void dmm_pause(void);
void dmm_hw4_set_mode_gate(uint8_t active);
void dmm_reenter(uint8_t mode_index);
void dmm_set_mode(uint8_t mode_index);
uint8_t dmm_poll(void);
void dmm_tick(uint32_t elapsed_ms);
uint8_t dmm_has_reading(void);
uint8_t dmm_value_is_numeric(void);
int32_t dmm_value_milli_units(void);
const char *dmm_value_text(void);
const char *dmm_unit_text(void);
const char *dmm_status_text(void);
uint8_t dmm_reading_is_real(void);
uint8_t dmm_live_wire_active(void);
uint8_t dmm_diode_continuity_active(void);
void dmm_uart_irq_handler(void);

/* 2C53T only (dmm53.c): small-font debug overlay lines for the meter
 * screen, scope-telemetry style. idx 0..2. */
const char *dmm53_debug_line(uint8_t idx);
