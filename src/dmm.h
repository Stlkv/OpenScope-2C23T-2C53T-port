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
/* Bench only: drive the frontend to stock mux arms (ce, ab), 0-9 each. */
uint8_t dmm53_debug_pose(uint8_t ce, uint8_t ab);
/* Bench only: USART2 TX frame counter, to prove a word went out. */
uint16_t dmm53_debug_tx_count(void);
/* Bench only: steady-state poll period, ms; 0 stops the (0x00,0x09) poll. */
void dmm53_debug_poll_period(uint16_t ms);
/* Bench only: TX-to-pin probe, returns (PA2 low samples << 16) | samples. */
uint32_t dmm53_debug_tx_probe(void);
/* Bench only: TX frame header bytes [0],[1], byte [4], checksum mode. */
void dmm53_debug_set_header(uint8_t b0, uint8_t b1, uint8_t b4, uint8_t cs_mode);
/* Bench only: sweep header pairs from `start`, one frame per tick, until an
 * echo frame (AA 55) arrives; results via dmm53_debug_scan_status. */
void dmm53_debug_scan(uint8_t on, uint16_t start);
void dmm53_debug_scan_status(uint8_t *on, uint16_t *idx, uint16_t *hits, uint16_t *first);
