#include "board.h"
#include "app_config.h"
#include "cdc_shell.h"
#include "dmm.h"
#include "display.h"
#include "fw_update.h"
#include "settings.h"
#include "siggen.h"
#include "ui.h"
#include "usb_msc.h"

#include <stdint.h>

static uint16_t diode_beep_hold_ms;
static uint16_t live_beep_hold_ms;
static uint16_t live_beep_phase_ms;
static uint16_t ui_beep_ms;
static uint8_t diode_beep_ready;
static uint8_t diode_beep_was_enabled;

enum {
    DIODE_BEEP_HOLD_MS = 30,
    KEY_BEEP_MS = 28,
    SETTINGS_BEEP_PREVIEW_MS = 90,
    LIVE_BEEP_HOLD_MS = 450,
    LIVE_BEEP_PERIOD_MS = 180,
    LIVE_BEEP_ON_MS = 45,
    LIVE_BEEP_VOLUME_PERCENT = 70,
    STARTUP_BEEP_MS = 55,
    SHUTDOWN_BEEP_MS = 70,
    POWER_HOLD_MS = 1000,
    STARTUP_INPUT_SETTLE_MS = 200,
};

static void shutdown_now(uint8_t beep) {
    board_dmm_beep_irq_arm(0);
    ui_note_runtime_settings();
    siggen_shutdown();
    if (beep) {
        board_buzzer_set(1);
        delay_ms(SHUTDOWN_BEEP_MS);
    }
    board_buzzer_set(0);
    settings_flush();
    board_power_off();
}

#if HW_TARGET_2C53T
/* POWER switches off only when held for POWER_HOLD_MS; a shorter press does
 * nothing. The UI is frozen while the key is down, but USB keeps being served
 * so a host session does not drop. */
static uint8_t power_hold_confirmed(void) {
    for (uint16_t ms = 0; ms < POWER_HOLD_MS; ms = (uint16_t)(ms + 10u)) {
        if (!input_power_held()) {
            ui_power_hold_cancel();
            return 0;
        }
        ui_power_hold_progress(ms, POWER_HOLD_MS);
        for (uint8_t i = 0; i < 10u; ++i) {
            usb_msc_poll();
            cdc_shell_service();
            usb_msc_cdc_pump();
            delay_ms(1);
        }
    }
    ui_power_hold_progress(POWER_HOLD_MS, POWER_HOLD_MS);
    ui_power_off_collapse();
    return 1;
}
#endif

#if HW_TARGET_2C53T
/* Bench: how the last continuity tone ended, in beep-service ticks (~0.77 ms).
 * [0] ticks the tone sounded after the last PC3 edge, [1] of those, ticks the
 * frame source alone held it on, [2] tones ended, [3] the longest gap between
 * PC3 edges inside the tone -- what the keep-alive hold has to bridge. Read
 * with `mem`. */
volatile uint16_t cont_tone_tail[4];
static uint16_t cont_gap_max;
static uint16_t cont_tail_run;
static uint16_t cont_frame_run;
static uint8_t cont_tone_was_on;
#endif

static void dmm_beep_service(uint8_t elapsed_ms) {
    uint8_t dmm_beep_seen;
#if HW_TARGET_2C53T
    uint8_t cont_edge = 0;
    uint8_t cont_frame = 0;
#endif
    uint8_t buzzer_on = 0;
    uint8_t force_full = 0;
    uint8_t fixed_volume = 0;
    uint8_t diode_mode = ui_diode_beep_enabled();
    uint8_t live_mode = ui_live_beep_enabled();

    /* Bench watch on the two probe-ish pins, sampled here because this is the
     * fastest loop the port has. Read only; it decides nothing. */
    board_probe_watch_tick();
    board_beep_stat_inc(0);

    if (diode_mode && !diode_beep_was_enabled) {
        diode_beep_ready = 0;
        diode_beep_hold_ms = 0;
        board_dmm_beep_irq_arm(0);
        (void)board_dmm_beep_edge_seen();
    } else if (!diode_mode) {
        diode_beep_ready = 0;
    }
    diode_beep_was_enabled = diode_mode;

    if (ui_beep_ms && !diode_mode && !live_mode) {
        board_dmm_beep_irq_arm(0);
        if (elapsed_ms >= ui_beep_ms) {
            ui_beep_ms = 0;
            board_buzzer_set(0);
        } else {
            ui_beep_ms = (uint16_t)(ui_beep_ms - elapsed_ms);
            board_buzzer_set(1);
        }
        return;
    }

    if (diode_mode) {
        board_dmm_beep_irq_force_full(1);
        if (!diode_beep_ready && dmm_reading_is_real()) {
            diode_beep_ready = 1;
            (void)board_dmm_beep_edge_seen();
        }
#if HW_TARGET_2C53T
        /* Measured 2026-09-07: the old gate left the beep dead for 2-4 s after
         * entering continuity (armed=0 at 2.0 s, armed=1 at 4.1 s), because
         * diode_beep_ready waits for the first parsed frame while the mode
         * transition is still draining frames. That gate exists to stop a
         * beep from stale data at mode entry -- which cannot happen through
         * the line, since a PC3 edge is a physical event. So the line arms as
         * soon as we are in the mode; only the frame source stays behind
         * diode_beep_ready. */
        /* Light-only continuity keeps the line armed, so edges are still
         * seen; the mute only stops the interrupt from starting the tone. */
        board_dmm_beep_irq_arm(1);
        board_dmm_beep_irq_mute(!ui_cont_sound_enabled());
        cont_edge = board_dmm_beep_edge_seen();
        cont_frame = (uint8_t)(diode_beep_ready && dmm_diode_continuity_active());
        dmm_beep_seen = (uint8_t)(cont_edge || cont_frame);
#else
        if (diode_beep_ready) {
            /* PC3 pulses while the SoC wants a tone (measured 2026-09-07:
             * nothing at all until the probes are shorted in continuity, then
             * a train of tens of thousands of edges). Stock re-arms the tone
             * on every pulse and lets a timeout kill it; here every pulse
             * sets edge_seen and edge_seen refreshes DIODE_BEEP_HOLD_MS, which
             * is the same shape. The frame's own continuity mark stays OR'd in
             * as an independent second source -- on this unit the SoC prints
             * "0.170 Ohm" in continuity instead of the stock BCD mark, so the
             * frame path alone stayed silent. */
#if HW_TARGET_HW40
            board_dmm_beep_irq_arm(0);
            dmm_beep_seen = dmm_diode_continuity_active();
#else
            board_dmm_beep_irq_arm(1);
            dmm_beep_seen = (uint8_t)(board_dmm_beep_active() || board_dmm_beep_edge_seen());
#endif
        } else {
            board_dmm_beep_irq_arm(0);
            (void)board_dmm_beep_edge_seen();
            dmm_beep_seen = 0;
        }
#endif
        if (dmm_beep_seen) {
            /* A tap can be a single PC3 pulse: the first pulse after silence
             * latches the minimum audible length, later pulses only refresh
             * the keep-alive tail. board_buzzer_min_beep_ms() is 0 on the
             * 2C23T, which leaves that board's behaviour as it was. */
            uint16_t hold = board_buzzer_min_beep_ms();
            if (hold < DIODE_BEEP_HOLD_MS) {
                hold = DIODE_BEEP_HOLD_MS;
            }
            if (diode_beep_hold_ms == 0) {
                diode_beep_hold_ms = hold;
                board_beep_stat_inc(1);
            } else {
                /* The latch is a minimum counted from the first pulse, so it
                 * keeps running down while pulses continue. It used to freeze
                 * here, which turned it into a 246 ms tail after every long
                 * contact (measured 2026-10-06: 320 ticks after the last PC3
                 * edge on 5 of 5 tones). */
                diode_beep_hold_ms = diode_beep_hold_ms > elapsed_ms ?
                    (uint16_t)(diode_beep_hold_ms - elapsed_ms) : 0u;
                if (diode_beep_hold_ms < DIODE_BEEP_HOLD_MS) {
                    diode_beep_hold_ms = DIODE_BEEP_HOLD_MS;
                }
            }
        } else if (diode_beep_hold_ms > elapsed_ms) {
            diode_beep_hold_ms = (uint16_t)(diode_beep_hold_ms - elapsed_ms);
        } else {
            diode_beep_hold_ms = 0;
        }
        live_beep_hold_ms = 0;
        live_beep_phase_ms = 0;
        buzzer_on = diode_beep_hold_ms ? 1u : 0u;
        force_full = 1;
#if HW_TARGET_2C53T
        if (buzzer_on) {
            if (cont_edge && cont_tail_run > cont_gap_max) {
                cont_gap_max = cont_tail_run;
            }
            cont_tail_run = cont_edge ? 0u : (uint16_t)(cont_tail_run + 1u);
            if (cont_edge) {
                cont_frame_run = 0;
            } else if (cont_frame) {
                cont_frame_run++;
            }
        } else if (cont_tone_was_on) {
            cont_tone_tail[0] = cont_tail_run;
            cont_tone_tail[1] = cont_frame_run;
            cont_tone_tail[2]++;
            cont_tone_tail[3] = cont_gap_max;
            cont_gap_max = 0;
            cont_tail_run = 0;
            cont_frame_run = 0;
        }
        cont_tone_was_on = buzzer_on;
        ui_cont_contact(buzzer_on);
        if (!ui_cont_sound_enabled()) {
            buzzer_on = 0;
        }
#endif
    } else if (live_mode) {
        board_dmm_beep_irq_force_full(0);
        board_dmm_beep_irq_arm(0);
        (void)board_dmm_beep_edge_seen();
        dmm_beep_seen = dmm_live_wire_active();
        if (dmm_beep_seen) {
            if (!live_beep_hold_ms) {
                live_beep_phase_ms = 0;
            }
            live_beep_hold_ms = LIVE_BEEP_HOLD_MS;
        } else if (live_beep_hold_ms > elapsed_ms) {
            live_beep_hold_ms = (uint16_t)(live_beep_hold_ms - elapsed_ms);
        } else {
            live_beep_hold_ms = 0;
        }

        if (live_beep_hold_ms) {
            live_beep_phase_ms = (uint16_t)(live_beep_phase_ms + elapsed_ms);
            if (live_beep_phase_ms >= LIVE_BEEP_PERIOD_MS) {
                live_beep_phase_ms = (uint16_t)(live_beep_phase_ms % LIVE_BEEP_PERIOD_MS);
            }
        } else {
            live_beep_phase_ms = 0;
        }
        diode_beep_hold_ms = 0;
        buzzer_on = (live_beep_hold_ms && live_beep_phase_ms < LIVE_BEEP_ON_MS) ? 1u : 0u;
        fixed_volume = LIVE_BEEP_VOLUME_PERCENT;
    } else {
        board_dmm_beep_irq_arm(0);
        diode_beep_hold_ms = 0;
        live_beep_hold_ms = 0;
        live_beep_phase_ms = 0;
        (void)board_dmm_beep_edge_seen();
    }

    ui_set_live_wire_detected((uint8_t)(live_mode && live_beep_hold_ms));

    if (ui_beep_ms && !buzzer_on) {
        if (elapsed_ms >= ui_beep_ms) {
            ui_beep_ms = 0;
            board_buzzer_set(0);
        } else {
            ui_beep_ms = (uint16_t)(ui_beep_ms - elapsed_ms);
            board_buzzer_set(1);
        }
    } else if (force_full) {
        if (buzzer_on) {
            ui_beep_ms = 0;
        } else {
            board_beep_stat_inc(2);
        }
        board_buzzer_set_full(buzzer_on);
    } else if (fixed_volume) {
        if (buzzer_on) {
            ui_beep_ms = 0;
        }
        board_buzzer_set_percent(buzzer_on, fixed_volume);
    } else if (ui_beep_ms) {
        if (elapsed_ms >= ui_beep_ms) {
            ui_beep_ms = 0;
            board_buzzer_set(0);
        } else {
            ui_beep_ms = (uint16_t)(ui_beep_ms - elapsed_ms);
            board_buzzer_set(1);
        }
    } else {
        if (!buzzer_on) {
            board_beep_stat_inc(3);
        }
        board_buzzer_set(buzzer_on);
    }
}

int main(void) {
    uint16_t input_settle_ms = STARTUP_INPUT_SETTLE_MS;

    board_init();
    battery_init();
    input_init();
    dmm_init();
    board_buzzer_init();

    lcd_init();
    usb_msc_init();
    ui_init();
    lcd_display_on();
    delay_ms(50);
    board_backlight_set(1);
    load_counter_init();
    ui_beep_ms = STARTUP_BEEP_MS;

    while (1) {
        uint32_t active_start = load_counter_read();
        if (dmm_poll()) {
            dmm_beep_service(0);
            ui_dmm_measurement_updated();
        }
        uint32_t events = input_pressed_events();
        if (events && !input_settle_ms) {
#if HW_TARGET_2C53T
            if ((events & KEY_POWER) && power_hold_confirmed()) {
                shutdown_now(1);
            }
#else
            if (events & KEY_POWER) {
                shutdown_now(1);
            }
#endif
            ui_handle_keys(events);
            if (events & KEY_REPEAT) {
                ui_beep_ms = 0;
            } else if (ui_consume_beep_preview()) {
                ui_beep_ms = SETTINGS_BEEP_PREVIEW_MS;
            } else {
                ui_beep_ms = KEY_BEEP_MS;
            }
            dmm_beep_service(0);
        } else {
            ui_tick(20);
        }
        if (ui_auto_sleep_due()) {
            shutdown_now(0);
        }
        fw_update_service();
        uint32_t active_ticks = load_counter_elapsed(active_start, load_counter_read());

        uint32_t idle_start = load_counter_read();
        for (uint8_t i = 0; i < 20u; ++i) {
            usb_msc_poll();
            cdc_shell_service();
            usb_msc_cdc_pump();
            delay_ms(1);
            dmm_beep_service(1);
        }
        dmm_tick(20);
        cdc_shell_tick(20);
        if (input_settle_ms > 20u) {
            input_settle_ms = (uint16_t)(input_settle_ms - 20u);
        } else {
            input_settle_ms = 0;
        }
        uint32_t idle_ticks = load_counter_elapsed(idle_start, load_counter_read());
        ui_set_load_sample(active_ticks, idle_ticks);
    }
}
