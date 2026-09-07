#pragma once

#include <stdint.h>

enum {
    KEY_MOVE  = 1u << 0,
    KEY_F2    = 1u << 1,
    KEY_F3    = 1u << 2,
    KEY_F4    = 1u << 3,
    KEY_AUTO  = 1u << 4,
    KEY_MENU  = 1u << 5,
    KEY_LEFT  = 1u << 6,
    KEY_RIGHT = 1u << 7,
    KEY_UP    = 1u << 8,
    KEY_DOWN  = 1u << 9,
    KEY_OK    = 1u << 10,
    KEY_CH1   = 1u << 11,
    KEY_CH2   = 1u << 12,
    KEY_SAVE  = 1u << 13,
    KEY_MOVE_LONG = 1u << 14,
    KEY_F2_LONG = 1u << 15,
    KEY_F3_LONG = 1u << 16,
    KEY_POWER = 1u << 17,
    KEY_REPEAT = 1u << 18,
    KEY_AUTO_LONG = 1u << 19,
    KEY_SAVE_LONG = 1u << 20,
    KEY_F4_LONG = 1u << 21,
    KEY_CH1_LONG = 1u << 22,
};

void delay_ms(uint32_t ms);
void gpio_config_mask(uint32_t base, uint16_t mask, uint8_t cfg);
void load_counter_init(void);
uint32_t load_counter_read(void);
uint32_t load_counter_elapsed(uint32_t start, uint32_t end);

void board_init(void);
void power_key_exti_init(void);
void power_key_irq_handler(void);
uint8_t board_power_off_requested(void);
void board_power_off(void);
void board_backlight_set(uint8_t on);
void board_backlight_set_level(uint8_t percent);
void board_buzzer_init(void);
void board_buzzer_set(uint8_t on);
void board_buzzer_set_full(uint8_t on);
void board_buzzer_set_percent(uint8_t on, uint8_t percent);
void board_buzzer_set_volume(uint8_t percent);
uint8_t board_dmm_beep_active(void);
uint8_t board_dmm_beep_edge_seen(void);
void board_dmm_beep_irq_arm(uint8_t enabled);
void board_dmm_beep_irq_force_full(uint8_t enabled);
/* 2C53T only: the piezo is PB9/TMR11_CH1 and the SoC's beep request is PC3. */
void board_buzzer_set_tone_div(uint16_t div);
uint16_t board_buzzer_tone_div(void);
void board_buzzer_set_min_beep_ms(uint16_t ms);
uint16_t board_buzzer_min_beep_ms(void);
void board_dmm_beep_irq_handler(void);
void board_dmm_beep_probe(uint32_t *rise, uint32_t *fall, uint8_t *level);
void board_dmm_beep_pulse_probe(uint32_t *edges, uint32_t *high,
                                uint32_t *samples, uint32_t *cycles);
void board_probe_watch_tick(void);
void board_pin_sweep_clear(void);
void board_beep_stat_inc(uint8_t slot);
uint32_t board_beep_stat(uint8_t slot);
uint32_t board_beep_last_on_ticks(void);
uint16_t board_pin_sweep_read(uint8_t port, uint8_t bit);
uint32_t board_pin_sweep_samples(void);
void board_probe_watch_read(uint8_t *pc0, uint32_t *pc0_edges,
                            uint8_t *pc7, uint32_t *pc7_edges);
void board_dmm_beep_chain_probe(uint32_t *noarm, uint32_t *irq_start,
                                uint32_t *on, uint32_t *off,
                                uint32_t *last_cycles, uint32_t *seen_taken,
                                uint8_t *armed);
void battery_init(void);
void battery_update(void);
void battery_update_charging_status(void);
uint16_t battery_millivolts(void);
uint8_t battery_percent(void);
uint8_t battery_is_charging(void);

void input_init(void);
uint32_t input_read_keys(void);
uint32_t input_pressed_events(void);
