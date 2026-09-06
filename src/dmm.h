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
 * screen, scope-telemetry style.
 *   0 — baud candidate, transition queue position, frame counters
 *   1 — last 12-byte data frame, raw
 *   2 — decoder state: submode, raw BCD, decimal, class, frame[6] rotation
 *   3 — transition plan: selector/apply words, planned vs live frontend mask
 *   4 — firmware-update path state (not drawn; CDC `fwstat` covers it) */
const char *dmm53_debug_line(uint8_t idx);
/* Bench only: queue one raw meter command word (see dmm53.c). */
void dmm53_debug_send(uint8_t hi, uint8_t lo);
