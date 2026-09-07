#pragma once

#include <stdint.h>

enum {
    SETTINGS_START_MENU,
    SETTINGS_START_DMM,
    SETTINGS_START_SCOPE,
    SETTINGS_START_GEN,
    /* Boot into whatever screen was showing last time: the mode, and the
     * mode menu if it was open over it. The default since 2026-09-06. */
    SETTINGS_START_LAST,
    SETTINGS_START_COUNT,
    SETTINGS_LEVEL_COUNT = 5,
    SETTINGS_SLEEP_COUNT = 4,
    SETTINGS_SCOPE_CHANNEL_COUNT = 2,
    SETTINGS_SCOPE_RANGE_COUNT = 9,
    SETTINGS_SCOPE_TIMEBASE_DEFAULT = 4,
    SETTINGS_SCOPE_TIMEBASE_COUNT = 26,
    SETTINGS_SCOPE_DISPLAY_COUNT = 3,
    SETTINGS_SCOPE_CURSOR_COUNT = 3,
    SETTINGS_SCOPE_TRIGGER_COUNT = 3,
    SETTINGS_SCOPE_MEASURE_COUNT = 6,
    SETTINGS_SIGGEN_WAVE_COUNT = 16,
    SETTINGS_SIGGEN_PARAM_COUNT = 4,
    SETTINGS_SIGGEN_FREQ_UNIT_COUNT = 3,
    SETTINGS_SIGGEN_DEFAULT_WAVE = 0,
    SETTINGS_SIGGEN_DEFAULT_PARAM = 1,
    SETTINGS_SIGGEN_DEFAULT_DUTY = 50,
    SETTINGS_SIGGEN_DEFAULT_AMP_TENTHS = 33,
    SETTINGS_SIGGEN_DEFAULT_FREQ_HZ = 1000,
    SETTINGS_SIGGEN_MAX_FREQ_HZ = 2000000,
    SETTINGS_SIGGEN_FM_MAX_HZ = 10,
    SETTINGS_SIGGEN_SWEEP_MODE_COUNT = 3, // 0 = OFF, 1 = LINEAR, 2 = LOG
    SETTINGS_SIGGEN_FM_MODE_COUNT = 2, // 0 = OFF, 1 = ON
    SETTINGS_SIGGEN_FM_SOURCE_COUNT = 3, // 0 = SINE, 1 = TRIANGLE, 2 = SQUARE
    /* CH2's vertical-offset reference on the 2C53T is a TMR13 PWM code, not a
     * DAC code, so its centering value is nothing like CH1's and nothing like
     * the 2C23T default below. Measured on bench unit #2, 2026-09-05: 2501
     * centers the channel (2048 -> ADC 70.5, 3072 -> 201.0, slope 0.1274 per
     * code). It is per-unit — upstream's unit #1 wants 2544 — so it is only a
     * default here; the calibrated value lives in scope_bias[1][range] like
     * every other per-unit offset. */
    SETTINGS_SCOPE_BIAS_CH2_2C53T_DEFAULT = 2501,
    /* CH1's reference IS a true 12-bit DAC, and mid-scale still is not centre:
     * on bench unit #2, 2048 leaves the channel at ADC 75. Measured 2026-09-06
     * the same way CH2's was — 2048 -> 75.5, 2560 -> 140.5, 3072 -> 205.5, a
     * slope of 0.1270 ADC per code (CH2's is 0.1274, the same front end) — and
     * 2467 reads 128.0/128.0/129.0 over three runs. Per-unit, like CH2's. */
    SETTINGS_SCOPE_BIAS_CH1_2C53T_DEFAULT = 2467,
};

typedef struct {
    uint8_t dmm_mode;
    uint8_t beep_level;
    uint8_t brightness_level;
    uint8_t startup_screen;
    uint8_t last_screen;   /* 0 DMM, 1 SCOPE, 2 GEN — the mode last shown */
    uint8_t last_in_menu;  /* the mode menu was open over it */
    uint8_t sleep_enabled;
    uint8_t scope_timebase;
    uint8_t scope_display;
    uint8_t scope_cursor_mode;
    uint8_t scope_trigger_source;
    uint8_t scope_trigger_mode;
    uint8_t scope_trigger_edge;
    uint8_t scope_trigger_level;
    uint8_t scope_afterglow;
    uint8_t scope_active_ch;
    uint8_t scope_measure_param;
    uint8_t scope_measure_visible;
    uint8_t scope_ch_enabled[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_probe_x10[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_vdiv[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_coupling_dc[SETTINGS_SCOPE_CHANNEL_COUNT];
    int8_t scope_ch_pos[SETTINGS_SCOPE_CHANNEL_COUNT];
    uint8_t scope_measure_mask[SETTINGS_SCOPE_CHANNEL_COUNT];
    int8_t scope_h_value_pos;
    uint8_t scope_h_value_timebase;
    uint8_t siggen_wave;
    uint8_t siggen_param;
    uint8_t siggen_duty_percent;
    uint8_t siggen_amp_tenths_v;
    uint8_t siggen_freq_unit;
    uint8_t siggen_running;
    uint32_t siggen_freq_hz;
    uint16_t scope_bias[SETTINGS_SCOPE_CHANNEL_COUNT][SETTINGS_SCOPE_RANGE_COUNT];
    uint16_t scope_bias_rate[SETTINGS_SCOPE_CHANNEL_COUNT][SETTINGS_SCOPE_RANGE_COUNT];
    uint8_t scope_math_mode;     // 0 = OFF, 1 = MATH (A±B)
    uint8_t scope_math_op;       // 0 = CH1 + CH2, 1 = CH1 - CH2, 2 = CH2 - CH1
    uint8_t scope_fft_src;       // 0 = OFF, 1 = CH1, 2 = CH2, 3 = XY mode, 4 = Bode
    uint8_t scope_math_selected; // 0..5, selected row in the scope math menu
    uint8_t scope_fft_window;    // 0 = HANN, 1 = HAMMING, 2 = BLACKMAN, 3 = RECTANGLE
    uint8_t scope_fft_display;   // 0 = NORMAL, 1 = AVERAGE, 2 = MAX HOLD
    uint8_t scope_hide_traces;   // 0=NONE, 1=CH1, 2=CH2, 3=ALL
    uint8_t siggen_sweep_mode;
    uint32_t siggen_sweep_ms;    // Range: 100ms to 10000ms (10 seconds)
    uint32_t siggen_sweep_start_hz;
    uint32_t siggen_sweep_stop_hz;
    uint8_t siggen_fm_mode;
    uint8_t siggen_fm_source;
    uint32_t siggen_fm_freq_hz;  // Low-rate MCU modulation: 1Hz to 10Hz
    uint32_t bode_start_hz;
    uint32_t bode_stop_hz;
    uint8_t bode_steps;

} settings_state_t;

uint8_t settings_load(settings_state_t *settings);
void settings_note(const settings_state_t *settings);
uint8_t settings_load_dmm_mode(uint8_t *mode);
void settings_note_dmm_mode(uint8_t mode);
void settings_flush(void);
