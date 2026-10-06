#include "board.h"

#include "app_config.h"
#include "hw.h"

#include <stdint.h>

#ifndef HW_TARGET_2C53T
#define HW_TARGET_2C53T 0
#endif

#if !HW_TARGET_2C53T
typedef struct {
    uint32_t base;
    uint16_t mask;
    uint32_t key;
    uint8_t active_high;
} key_pin_t;

static const key_pin_t key_pins[] = {
    {GPIOB_BASE, 1u << 6, KEY_MOVE, 0},
    {GPIOB_BASE, 1u << 7, KEY_F2, 0},
    {GPIOC_BASE, 1u << 14, KEY_F3, 0},
    {GPIOE_BASE, 1u << 1, KEY_F4, 1},
    {GPIOB_BASE, 1u << 9, KEY_AUTO, 0},
    {GPIOE_BASE, 1u << 4, KEY_MENU, 0},
    {GPIOC_BASE, 1u << 15, KEY_LEFT, 0},
    {GPIOE_BASE, 1u << 5, KEY_RIGHT, 0},
    {GPIOE_BASE, 1u << 6, KEY_UP, 0},
    {GPIOA_BASE, 1u << 3, KEY_DOWN, 0},
    {GPIOC_BASE, 1u << 13, KEY_OK, 0},
    {GPIOB_BASE, 1u << 8, KEY_CH1, 0},
    {GPIOE_BASE, 1u << 2, KEY_CH2, 0},
    {GPIOE_BASE, 1u << 3, KEY_SAVE, 0},
    {GPIOD_BASE, 1u << 3, KEY_POWER, 0},
};
#endif /* !HW_TARGET_2C53T */

static uint16_t g_battery_mv;
static uint8_t g_battery_percent;
static uint8_t g_battery_charging;
static uint8_t g_battery_filter_valid;
static uint8_t g_backlight_on;
static uint8_t g_backlight_percent = 100;
static volatile uint8_t g_buzzer_on;
static uint8_t g_buzzer_volume = 50;
#if HW_TARGET_2C53T
/* 2C53T tone, taken from stock: TMR11 (0x40015400) runs DIV=2399, PR=24 —
 * 2 kHz when APB2 is 120 MHz — and a beep is a nonzero C1DT, stock writing
 * 10 of 25 from the EXTI3 ISR (0x08009C5A). The port never reprograms the
 * PLL, so the pitch follows whatever the factory bootloader left; the shell
 * can sweep the divider to find the piezo's resonance. */
enum {
    BUZZER53_TONE_PR = 24u,
    BUZZER53_TONE_DIV_DEFAULT = 2399u,
};
static uint16_t g_buzzer_tone_div = BUZZER53_TONE_DIV_DEFAULT;
/* Shortest tone the continuity beep is allowed to make. A tap on the probes
 * can be one PC3 pulse wide, and the keep-alive hold alone (30 ms) turns that
 * into a click rather than a beep, so the first pulse after silence latches
 * this length. The unit is one pass of the beep service, measured at 1305/s
 * (0.77 ms), and the length is linear in it: swept with the software trigger
 * 2026-09-07, hold 140/320/500/700/1000 gave 141/321/504/705/1006 ticks =
 * 109/247/388/543/775 ms. 320 is the default because 140 (~110 ms) was hard
 * to catch by ear while working the probes. `beep hold <n>` retunes it. */
static uint16_t g_buzzer_min_beep_ms = 320u;
static volatile uint32_t g_dmm_beep_rise_count;
static volatile uint32_t g_dmm_beep_fall_count;
/* Where a tap turns into a tone, counted so the chain can be read instead of
 * guessed at: how many edges arrived with the ISR disarmed, how many tones
 * the ISR itself started, how many times the tone went on and off by any
 * path, and how long the last tone actually lasted in SysTick cycles (the
 * only honest time unit here; wraps above 2^24 cycles). */
static volatile uint32_t g_beep_noarm_count;
static volatile uint32_t g_beep_irq_start_count;
static volatile uint32_t g_beep_on_count;
static volatile uint32_t g_beep_off_count;
static volatile uint32_t g_beep_last_on_cycles;
static volatile uint32_t g_beep_seen_taken;
/* Hunt for a line that reacts to a probe touch faster than the meter SoC's
 * own continuity detector does. Measured 2026-09-07: five brief taps moved
 * PC3 by exactly zero edges, so PC3 only pulses once the SoC has decided the
 * short is real. PC0 is labelled "probe continuity" in the stock reverse
 * engineering (low confidence: same vintage as the EXTI3-is-PA3 claim that
 * turned out wrong) and PC7 "probe detect". Sampled from the beep service,
 * which runs about every 1.5 ms, so a 20 ms tap cannot hide between polls.
 * Read only -- neither pin is reconfigured, the scope path owns PC0. */
static volatile uint32_t g_pc0_edges;
static volatile uint32_t g_pc7_edges;
static uint8_t g_pc0_last = 0xFFu;
static uint8_t g_pc7_last = 0xFFu;
/* Whole-GPIO transition sweep: if any pin on this board reacts to a probe
 * touch sooner than the SoC's continuity detector does, it shows up here.
 * Sampled in the same service, counted per pin, and zeroed on demand so a
 * quiet baseline can be subtracted from a tap run -- the keypad scan, the LCD
 * bus and the SPI lines all move on their own. */
static volatile uint16_t g_pin_edges[5][16];
static uint16_t g_pin_last[5];
static uint8_t g_pin_primed;
static volatile uint32_t g_pin_samples;
/* Who ends a tone, and how fast the beep service actually runs. Slots:
 * 0 = service calls, 1 = min-length latches, 2 = silenced by the diode
 * branch (hold ran out), 3 = silenced by another branch (mode gate flapped),
 * 4 = silenced by the key-beep path. The tone measured 106 ms with the hold
 * set to 320 units, so the unit is not what the startup-beep calibration
 * suggested and one of these paths is cutting it short. */
static volatile uint32_t g_beep_stat[5];
/* Tone length in service ticks. SysTick is a 24-bit down-counter, so the
 * cycle figure next to it wraps every 2^24 cycles (~175 ms at 96 MHz) and is
 * useless for anything longer -- that wrap is what made the tone look
 * non-monotonic in the hold sweep. Ticks do not wrap. */
static volatile uint32_t g_beep_on_ticks;
static volatile uint32_t g_beep_last_on_ticks;
static volatile uint8_t g_buzzer_driving;
/* PC3 measured 2026-09-07: silent (idle HIGH) until the probes are shorted in
 * continuity, then a pulse train -- 50 000 edges in a few seconds. So it is a
 * keep-alive request, not a one-shot, which is exactly what stock's ISR
 * expects: every pulse re-arms the tone and a timeout kills it. The earlier
 * self-masking flood guard is gone with the uncertainty that motivated it; it
 * would silence the buzzer mid-beep. */
#endif
static volatile uint8_t g_dmm_beep_irq_armed;
static volatile uint8_t g_dmm_beep_irq_full;
static volatile uint8_t g_dmm_beep_edge_seen;
static volatile uint8_t g_power_off_requested;

void delay_ms(uint32_t ms) {
    while (ms--) {
        for (volatile uint32_t i = 0; i < 12000u; ++i) {
            __asm__ volatile("nop");
        }
    }
}

static void gpio_config_nibble(uint32_t base, uint8_t pin, uint8_t cfg) {
    volatile uint32_t *reg = pin < 8 ? &GPIO_CRL(base) : &GPIO_CRH(base);
    uint8_t shift = (uint8_t)((pin & 7u) * 4u);
    uint32_t value = *reg;
    value &= ~(0xFu << shift);
    value |= ((uint32_t)cfg & 0xFu) << shift;
    *reg = value;
}

void gpio_config_mask(uint32_t base, uint16_t mask, uint8_t cfg) {
    for (uint8_t pin = 0; pin < 16; ++pin) {
        if (mask & (1u << pin)) {
            gpio_config_nibble(base, pin, cfg);
        }
    }
}

void board_init(void) {
    REG32(SCB_VTOR) = APP_BASE_ADDR;

    RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6);
    RCC_AHBENR |= (1u << 8);

#if HW_TARGET_2C53T
    gpio_config_mask(GPIOC_BASE, 1u << 9, 0x1); // PC9 power hold, must go HIGH first
    gpio_set(GPIOC_BASE, 1u << 9);

    gpio_config_mask(GPIOB_BASE, 1u << 8, 0x1); // PB8 LCD backlight, off until display ready
    gpio_clear(GPIOB_BASE, 1u << 8);

    gpio_config_mask(GPIOD_BASE, 1u << 6, 0x1); // PD6 LCD reset (same as 2C23T)
    gpio_set(GPIOD_BASE, 1u << 6);
#else
    gpio_config_mask(GPIOB_BASE, 1u << 2, 0x1); // PB2 power hold
    gpio_set(GPIOB_BASE, 1u << 2);

    gpio_config_mask(GPIOB_BASE, 1u << 0, 0x8); // PB0 charger status, active low
    gpio_set(GPIOB_BASE, 1u << 0);

    gpio_config_mask(GPIOA_BASE, 1u << 0, 0x1); // PA0 LCD backlight
    gpio_clear(GPIOA_BASE, 1u << 0);

    gpio_config_mask(GPIOA_BASE, 1u << 1, 0x1); // PA1 buzzer, quiet until PWM init
    gpio_clear(GPIOA_BASE, 1u << 1);

    gpio_config_mask(GPIOD_BASE, 1u << 3, 0x8); // PD3 power key, input pull-up
    gpio_set(GPIOD_BASE, 1u << 3);

    gpio_config_mask(GPIOD_BASE, 1u << 6, 0x1); // PD6 LCD reset
    gpio_set(GPIOD_BASE, 1u << 6);
#endif
}

void load_counter_init(void) {
    SYSTICK_CTRL = 0;
    SYSTICK_LOAD = 0x00FFFFFFu;
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = 5u; // processor clock, no interrupt
}

uint32_t load_counter_read(void) {
    return SYSTICK_VAL & 0x00FFFFFFu;
}

uint32_t load_counter_elapsed(uint32_t start, uint32_t end) {
    return (start - end) & 0x00FFFFFFu;
}

void power_key_exti_init(void) {
    g_power_off_requested = 0;
#if HW_TARGET_2C53T
    /* 2C53T POWER (PC8) is polled via the key matrix scan; no EXTI needed. */
    __asm__ volatile("cpsie i");
#else
    AFIO_EXTICR1 = (AFIO_EXTICR1 & ~(0xFu << 12)) | (0x3u << 12); // EXTI3 = port D
    EXTI_PR = 1u << 3;
    EXTI_RTSR &= ~(1u << 3);
    EXTI_FTSR |= 1u << 3;
    EXTI_IMR |= 1u << 3;
    REG32(NVIC_ISER0) = 1u << 9; // EXTI3 IRQ
    __asm__ volatile("cpsie i");
#endif
}

void power_key_irq_handler(void) {
    EXTI_PR = 1u << 3;
    EXTI_IMR &= ~(1u << 3);
    g_power_off_requested = 1;
}

uint8_t board_power_off_requested(void) {
    return g_power_off_requested;
}

void board_power_off(void) {
#if HW_TARGET_2C53T
    gpio_clear(GPIOC_BASE, 1u << 9);
    /* On battery the rail is gone within milliseconds of releasing the hold.
     * Still running half a second later means USB is powering the board, and
     * the old bare spin here froze the unit until the cable came out: the
     * screen stayed lit and the power key, polled by the main loop, was dead.
     * Instead go dark and wait for the key -- let go of the press that got us
     * here, then a fresh press restarts the firmware. Pulling the cable still
     * powers the unit off, since the hold stays released. */
    delay_ms(500);
    board_backlight_set(0);
    while (!gpio_read(GPIOC_BASE, 1u << 8)) {
    }
    delay_ms(50);
    for (;;) {
        if (!gpio_read(GPIOC_BASE, 1u << 8)) {
            delay_ms(30);
            if (!gpio_read(GPIOC_BASE, 1u << 8)) {
                break;
            }
        }
    }
    REG32(0xE000ED0Cu) = 0x05FA0004u; /* AIRCR SYSRESETREQ */
#else
    gpio_clear(GPIOB_BASE, 1u << 2);
#endif
    while (1) {
    }
}

static uint16_t percent_to_timer_compare(uint8_t percent, uint16_t max_compare) {
    if (percent > 100u) {
        percent = 100u;
    }
    return (uint16_t)((uint32_t)max_compare * percent / 100u);
}

static void tmr5_update_counter(void) {
    if (g_backlight_on || g_buzzer_on) {
        TMR_CTRL1(TMR5_BASE) |= 1u;
    } else {
        TMR_CTRL1(TMR5_BASE) &= ~1u;
    }
}

#if HW_TARGET_2C53T
/* PB8 is TMR10_CH1, the other half of the discovery that named the piezo:
 * stock enables IOPBEN+TMR10EN and IOPBEN+TMR11EN back to back, and its
 * TMR10 runs DIV=119, PR=99 -- 10 kHz at a 120 MHz APB2 -- with C1DT
 * (0x40015034) as the brightness. Until now this board drove PB8 as a plain
 * output and the brightness setting was cosmetic.
 *
 * Full brightness deliberately stays a plain GPIO high, exactly the path this
 * board has always used: at the default level the screen does not depend on
 * the timer at all, so a mistake here cannot leave a dark display on a device
 * that only comes back through MENU+Power. The UI's levels are
 * 20/40/60/80/100, so only the four lower ones engage the PWM. */
enum {
    BACKLIGHT53_PR = 99u,
    BACKLIGHT53_DIV = 119u,
};

static void backlight53_pwm_off(void) {
    TMR_CCEN(TMR10_BASE) &= ~1u;
    TMR_CTRL1(TMR10_BASE) &= ~1u;
}

static void backlight53_apply(void) {
    if (!g_backlight_on) {
        gpio_config_mask(GPIOB_BASE, 1u << 8, 0x1u);
        gpio_clear(GPIOB_BASE, 1u << 8);
        backlight53_pwm_off();
        return;
    }
    if (g_backlight_percent >= 100u) {
        gpio_config_mask(GPIOB_BASE, 1u << 8, 0x1u);
        gpio_set(GPIOB_BASE, 1u << 8);
        backlight53_pwm_off();
        return;
    }
    TMR_C1DT(TMR10_BASE) =
        percent_to_timer_compare(g_backlight_percent, BACKLIGHT53_PR);
    TMR_CCEN(TMR10_BASE) |= 1u;  // CH1 enable
    TMR_CTRL1(TMR10_BASE) |= 1u; // counter on
    gpio_config_mask(GPIOB_BASE, 1u << 8, 0xBu); // TMR10 CH1, AF push-pull
}

void board_backlight_set(uint8_t on) {
    g_backlight_on = on ? 1u : 0u;
    backlight53_apply();
}

void board_backlight_set_level(uint8_t percent) {
    if (percent > 100u) {
        percent = 100u;
    }
    g_backlight_percent = percent;
    backlight53_apply();
}

uint8_t board_backlight_level(void) {
    return g_backlight_percent;
}
#else
void board_backlight_set(uint8_t on) {
    g_backlight_on = on ? 1u : 0u;
    if (on) {
        gpio_config_mask(GPIOA_BASE, 1u << 0, 0xBu); // TMR5 CH1, AF push-pull
        TMR_C1DT(TMR5_BASE) = percent_to_timer_compare(g_backlight_percent, 999u);
        TMR_CCEN(TMR5_BASE) |= 1u; // CH1 enable
    } else {
        TMR_CCEN(TMR5_BASE) &= ~1u;
        gpio_config_mask(GPIOA_BASE, 1u << 0, 0x1);
        gpio_clear(GPIOA_BASE, 1u << 0);
    }
    tmr5_update_counter();
}

void board_backlight_set_level(uint8_t percent) {
    if (percent > 100u) {
        percent = 100u;
    }
    g_backlight_percent = percent;
    TMR_C1DT(TMR5_BASE) = percent_to_timer_compare(g_backlight_percent, 999u);
}
#endif

void board_buzzer_init(void) {
    RCC_APB1ENR |= 1u << 3; // TMR5

#if !HW_TARGET_2C53T
    /* PA0/PA1 are backlight/buzzer only on the 2C23T; on the 2C53T their role
     * is unknown, so they are deliberately left untouched (no buzzer output). */
    gpio_config_mask(GPIOA_BASE, 1u << 0, 0x1); // PA0 backlight stays off until display is ready
    gpio_clear(GPIOA_BASE, 1u << 0);
    gpio_config_mask(GPIOA_BASE, 1u << 1, 0xBu); // TMR5 CH2, AF push-pull
#endif
#if HW_TARGET_2C53T
    /* Piezo: PB9 = TMR11_CH1. Between beeps the pin stays a plain output
     * because the meter frontend pose still lists it (dmm53_apply_mux), so a
     * beep borrows it as AF push-pull and hands it back. */
    RCC_APB2ENR |= 1u << 20; // TMR10, backlight PWM on PB8
    TMR_CTRL1(TMR10_BASE) = 0;
    TMR_CCEN(TMR10_BASE) = 0;
    TMR_PSC(TMR10_BASE) = BACKLIGHT53_DIV;
    TMR_PR(TMR10_BASE) = BACKLIGHT53_PR;
    TMR_RPR(TMR10_BASE) = 0;
    TMR_C1DT(TMR10_BASE) = BACKLIGHT53_PR; // full until a level says otherwise
    TMR_CCM1(TMR10_BASE) = (6u << 4) | (1u << 3); // CH1 PWM mode A, preload
    TMR_BRK(TMR10_BASE) |= 1u << 15;              // MOE, as in stock's init
    TMR_EG(TMR10_BASE) = 1u;
    TMR_CTRL1(TMR10_BASE) = 1u << 7;              // ARPE, counter off

    RCC_APB2ENR |= 1u << 21; // TMR11, stock enables it right after IOPBEN
    TMR_CTRL1(TMR11_BASE) = 0;
    TMR_CCEN(TMR11_BASE) = 0;
    TMR_PSC(TMR11_BASE) = g_buzzer_tone_div;
    TMR_PR(TMR11_BASE) = BUZZER53_TONE_PR;
    TMR_RPR(TMR11_BASE) = 0;
    TMR_C1DT(TMR11_BASE) = 0;
    TMR_CCM1(TMR11_BASE) = (6u << 4) | (1u << 3); // CH1 PWM mode A, preload
    TMR_BRK(TMR11_BASE) |= 1u << 15;              // MOE, as in stock's init
    TMR_EG(TMR11_BASE) = 1u;
    TMR_CTRL1(TMR11_BASE) = 1u << 7;              // ARPE, counter off

    /* Beep request: PC3, input pull-up, EXTI3 -> IRQ9. Stock arms the rising
     * edge only (IOMUX_EXTIC1 field 3 = port C at 0x0802C764, RTSR bit 3 set
     * at 0x0802C7CC, FTSR bit 3 cleared, NVIC IRQ9 enabled). We arm both
     * edges so one bench run says which one the SoC actually produces. */
    gpio_config_mask(GPIOC_BASE, 1u << 3, 0x8u);
    gpio_set(GPIOC_BASE, 1u << 3);
    AFIO_EXTICR1 = (AFIO_EXTICR1 & ~(0xFu << 12)) | (0x2u << 12); // EXTI3 = port C
    EXTI_PR = 1u << 3;
    EXTI_RTSR |= 1u << 3;
    EXTI_FTSR |= 1u << 3;
    EXTI_IMR |= 1u << 3;
    REG32(NVIC_ISER0) = 1u << 9; // EXTI3 IRQ
#elif !HW_TARGET_HW40
    gpio_config_mask(GPIOC_BASE, 1u << 7, 0x8u); // DMM beep request, active low
    gpio_set(GPIOC_BASE, 1u << 7);
    AFIO_EXTICR2 = (AFIO_EXTICR2 & ~(0xFu << 12)) | (0x2u << 12); // EXTI7 = port C
    EXTI_PR = 1u << 7;
    EXTI_FTSR |= 1u << 7;
    EXTI_RTSR &= ~(1u << 7);
    EXTI_IMR |= 1u << 7;
    REG32(NVIC_ISER0) = 1u << 23; // EXTI9_5 IRQ
#else
    EXTI_IMR &= ~(1u << 7);
    EXTI_PR = 1u << 7;
#endif

    TMR_CTRL1(TMR5_BASE) = 0;
    TMR_CCEN(TMR5_BASE) = 0;
    TMR_PSC(TMR5_BASE) = 71u;   // 1 MHz timer tick when APB1 timer clock is 72 MHz
    TMR_PR(TMR5_BASE) = 999u;   // about 1 kHz
    TMR_C1DT(TMR5_BASE) = percent_to_timer_compare(g_backlight_percent, 999u);
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(g_buzzer_volume, 500u);
    TMR_CCM1(TMR5_BASE) = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TMR_EG(TMR5_BASE) = 1u;
    TMR_CTRL1(TMR5_BASE) = 1u << 7; // auto-reload preload, counter off until needed
    g_backlight_on = 0;
    g_buzzer_on = 0;
}

#if HW_TARGET_2C53T
/* Drive the piezo. Claiming the pin last and releasing it first keeps PB9 out
 * of the timer's hands for every instant it belongs to the frontend, and ODR
 * is never touched here: the level dmm53_apply_mux last wrote sits there
 * untouched while the pin is in AF mode, so the pose comes back by itself --
 * including a DMM53_AUX_AFE_HIGH build, where it is HIGH. */
static void buzzer53_drive(uint8_t on, uint8_t percent) {
    static uint32_t on_start;

    if (on && !g_buzzer_driving) {
        g_buzzer_driving = 1;
        on_start = load_counter_read();
        g_beep_on_ticks = 0;
        g_beep_on_count++;
    } else if (!on && g_buzzer_driving) {
        g_buzzer_driving = 0;
        g_beep_last_on_cycles = load_counter_elapsed(on_start, load_counter_read());
        g_beep_last_on_ticks = g_beep_on_ticks;
        g_beep_off_count++;
    }
    if (on) {
        TMR_C1DT(TMR11_BASE) = percent_to_timer_compare(percent, BUZZER53_TONE_PR);
        TMR_CCEN(TMR11_BASE) |= 1u;  // CH1 enable
        TMR_CTRL1(TMR11_BASE) |= 1u; // counter on
        gpio_config_mask(GPIOB_BASE, 1u << 9, 0xBu); // TMR11 CH1, AF push-pull
    } else {
        gpio_config_mask(GPIOB_BASE, 1u << 9, 0x1u); // back to the pose's output
        TMR_CCEN(TMR11_BASE) &= ~1u;
        TMR_CTRL1(TMR11_BASE) &= ~1u;
        TMR_C1DT(TMR11_BASE) = 0;
    }
}

void board_buzzer_set_tone_div(uint16_t div) {
    g_buzzer_tone_div = div;
    TMR_PSC(TMR11_BASE) = div;
    TMR_EG(TMR11_BASE) = 1u;
}

uint16_t board_buzzer_tone_div(void) {
    return g_buzzer_tone_div;
}

void board_buzzer_set_min_beep_ms(uint16_t ms) {
    g_buzzer_min_beep_ms = ms;
}
#endif

static void board_buzzer_set_at(uint8_t on, uint8_t percent) {
    on = on ? 1u : 0u;
    if (percent > 100u) {
        percent = 100u;
    }
    if (!percent) {
        on = 0;
    }
#if HW_TARGET_2C53T
    if (on == g_buzzer_on &&
        (!on || TMR_C1DT(TMR11_BASE) == percent_to_timer_compare(percent, BUZZER53_TONE_PR))) {
        return;
    }
    g_buzzer_on = on;
    buzzer53_drive(on, percent);
#else
    if (on == g_buzzer_on && (!on || TMR_C2DT(TMR5_BASE) == percent_to_timer_compare(percent, 500u))) {
        return;
    }
    g_buzzer_on = on;
    if (on) {
        TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(percent, 500u);
        TMR_CCEN(TMR5_BASE) |= 1u << 4; // CH2 enable
    } else {
        TMR_CCEN(TMR5_BASE) &= ~(1u << 4);
        gpio_config_mask(GPIOA_BASE, 1u << 1, 0x1);
        gpio_clear(GPIOA_BASE, 1u << 1);
        gpio_config_mask(GPIOA_BASE, 1u << 1, 0xBu);
    }
    tmr5_update_counter();
#endif
}

void board_buzzer_set(uint8_t on) {
    board_buzzer_set_at(on, g_buzzer_volume);
}

void board_buzzer_set_full(uint8_t on) {
    board_buzzer_set_at(on, 100);
}

void board_buzzer_set_percent(uint8_t on, uint8_t percent) {
    board_buzzer_set_at(on, percent);
}

void board_buzzer_set_volume(uint8_t percent) {
    if (percent > 100u) {
        percent = 100u;
    }
    g_buzzer_volume = percent;
#if HW_TARGET_2C53T
    if (g_buzzer_on) {
        TMR_C1DT(TMR11_BASE) = percent_to_timer_compare(g_buzzer_volume, BUZZER53_TONE_PR);
    }
#else
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(g_buzzer_volume, 500u);
#endif
    if (!g_buzzer_volume && g_buzzer_on) {
        board_buzzer_set(0);
    }
}

#if HW_TARGET_2C53T || !HW_TARGET_HW40
static void board_buzzer_start_from_irq(void) {
    uint8_t percent = g_dmm_beep_irq_full ? 100u : g_buzzer_volume;
    if (!percent) {
        return;
    }
    if (g_buzzer_on) {
        return;
    }
    g_buzzer_on = 1;
#if HW_TARGET_2C53T
    g_beep_irq_start_count++;
    buzzer53_drive(1, percent);
#else
    TMR_C2DT(TMR5_BASE) = percent_to_timer_compare(percent, 500u);
    TMR_CCEN(TMR5_BASE) |= 1u << 4;
    tmr5_update_counter();
#endif
}
#endif

void board_probe_watch_tick(void) {
#if HW_TARGET_2C53T
    uint8_t pc0 = gpio_read(GPIOC_BASE, 1u << 0) ? 1u : 0u;
    uint8_t pc7 = gpio_read(GPIOC_BASE, 1u << 7) ? 1u : 0u;

    if (g_pc0_last != 0xFFu && pc0 != g_pc0_last) {
        g_pc0_edges++;
    }
    if (g_pc7_last != 0xFFu && pc7 != g_pc7_last) {
        g_pc7_edges++;
    }
    if (g_buzzer_driving) {
        g_beep_on_ticks++;
    }
    g_pc0_last = pc0;
    g_pc7_last = pc7;

    {
        uint32_t port;
        for (port = 0; port < 5u; ++port) {
            uint16_t idr = (uint16_t)GPIO_IDR(GPIOA_BASE + port * 0x400u);
            if (g_pin_primed) {
                uint16_t diff = (uint16_t)(idr ^ g_pin_last[port]);
                uint8_t bit;
                for (bit = 0; bit < 16u && diff; ++bit) {
                    if (diff & (1u << bit)) {
                        uint16_t v = g_pin_edges[port][bit];
                        if (v != 0xFFFFu) {
                            g_pin_edges[port][bit] = (uint16_t)(v + 1u);
                        }
                        diff = (uint16_t)(diff & ~(1u << bit));
                    }
                }
            }
            g_pin_last[port] = idr;
        }
        g_pin_primed = 1u;
        g_pin_samples++;
    }
#endif
}

#if HW_TARGET_2C53T
void board_beep_stat_inc(uint8_t slot) {
    if (slot < 5u) {
        g_beep_stat[slot]++;
    }
}

uint32_t board_beep_stat(uint8_t slot) {
    return slot < 5u ? g_beep_stat[slot] : 0u;
}

uint32_t board_beep_last_on_ticks(void) {
    return g_beep_last_on_ticks;
}

void board_pin_sweep_clear(void) {
    uint8_t i;
    for (i = 0; i < 5u; ++i) {
        g_beep_stat[i] = 0;
    }

    uint32_t port;
    uint8_t bit;

    for (port = 0; port < 5u; ++port) {
        for (bit = 0; bit < 16u; ++bit) {
            g_pin_edges[port][bit] = 0;
        }
    }
    g_pin_samples = 0;
    g_pin_primed = 0;
    g_pc0_edges = 0;
    g_pc7_edges = 0;
}

uint16_t board_pin_sweep_read(uint8_t port, uint8_t bit) {
    if (port > 4u || bit > 15u) {
        return 0;
    }
    return g_pin_edges[port][bit];
}

uint32_t board_pin_sweep_samples(void) {
    return g_pin_samples;
}

void board_probe_watch_read(uint8_t *pc0, uint32_t *pc0_edges,
                            uint8_t *pc7, uint32_t *pc7_edges) {
    *pc0 = gpio_read(GPIOC_BASE, 1u << 0) ? 1u : 0u;
    *pc7 = gpio_read(GPIOC_BASE, 1u << 7) ? 1u : 0u;
    *pc0_edges = g_pc0_edges;
    *pc7_edges = g_pc7_edges;
}
#else
/* The bench counters above live on the 2C53T hunt for the beep chain; the
 * 2C23T shares the policy code that pokes one of them, so it gets a stub
 * rather than the counters. */
void board_beep_stat_inc(uint8_t slot) {
    (void)slot;
}
#endif

uint16_t board_buzzer_min_beep_ms(void) {
#if HW_TARGET_2C53T
    return g_buzzer_min_beep_ms;
#else
    return 0;
#endif
}

uint8_t board_dmm_beep_active(void) {
#if HW_TARGET_2C53T
    /* No level to read on this board: PC3 idles HIGH and asks for a tone by
     * pulsing, so the request lives in the edges (board_dmm_beep_edge_seen)
     * and a level test here would beep forever. The raw level is still
     * available to the bench through board_dmm_beep_probe. */
    return 0;
#elif HW_TARGET_HW40
    return 0;
#else
    return gpio_read(GPIOC_BASE, 1u << 7) ? 0u : 1u;
#endif
}

uint8_t board_dmm_beep_edge_seen(void) {
    uint8_t seen;
    __asm__ volatile("cpsid i" ::: "memory");
    seen = g_dmm_beep_edge_seen;
    g_dmm_beep_edge_seen = 0;
    __asm__ volatile("cpsie i" ::: "memory");
#if HW_TARGET_2C53T
    if (seen) {
        g_beep_seen_taken++;
    }
#endif
    return seen;
}

void board_dmm_beep_irq_arm(uint8_t enabled) {
    g_dmm_beep_irq_armed = enabled ? 1u : 0u;
    if (!enabled) {
        g_dmm_beep_irq_full = 0;
        g_dmm_beep_edge_seen = 0;
    }
}

void board_dmm_beep_irq_force_full(uint8_t enabled) {
    g_dmm_beep_irq_full = enabled ? 1u : 0u;
}

#if HW_TARGET_2C53T
/* EXTI3 is PC3 on the 2C53T, not the power key: stock writes port C into
 * IOMUX_EXTIC1 field 3 and hangs its buzzer ISR (0x08009C11) on it. */
void board_dmm_beep_irq_handler(void) {
    if (!(EXTI_PR & (1u << 3))) {
        return;
    }
    EXTI_PR = 1u << 3;
    if (gpio_read(GPIOC_BASE, 1u << 3)) {
        g_dmm_beep_rise_count++;
        g_dmm_beep_edge_seen = 1;
        if (g_dmm_beep_irq_armed) {
            board_buzzer_start_from_irq();
        } else {
            g_beep_noarm_count++;
        }
    } else {
        g_dmm_beep_fall_count++;
    }
}

void board_dmm_beep_probe(uint32_t *rise, uint32_t *fall, uint8_t *level) {
    if (rise) {
        *rise = g_dmm_beep_rise_count;
    }
    if (fall) {
        *fall = g_dmm_beep_fall_count;
    }
    if (level) {
        *level = gpio_read(GPIOC_BASE, 1u << 3) ? 1u : 0u;
    }
}

/* Count PC3's transitions by polling, with the SysTick load counter measuring
 * the window: the firmware's own "ms" is a calibrated nop loop, so cycles are
 * the only honest unit here. Rate = edges/2 per cycle times the CPU clock. */
void board_dmm_beep_chain_probe(uint32_t *noarm, uint32_t *irq_start,
                                uint32_t *on, uint32_t *off,
                                uint32_t *last_cycles, uint32_t *seen_taken,
                                uint8_t *armed) {
    *noarm = g_beep_noarm_count;
    *irq_start = g_beep_irq_start_count;
    *on = g_beep_on_count;
    *off = g_beep_off_count;
    *last_cycles = g_beep_last_on_cycles;
    *seen_taken = g_beep_seen_taken;
    *armed = g_dmm_beep_irq_armed;
}

void board_dmm_beep_pulse_probe(uint32_t *edges, uint32_t *high,
                                uint32_t *samples, uint32_t *cycles) {
    uint32_t e = 0;
    uint32_t h = 0;
    uint32_t n = 0;
    uint32_t start;
    uint8_t last = gpio_read(GPIOC_BASE, 1u << 3) ? 1u : 0u;

    start = load_counter_read();
    for (volatile uint32_t i = 0; i < 30000u; ++i) {
        uint8_t now = gpio_read(GPIOC_BASE, 1u << 3) ? 1u : 0u;
        if (now != last) {
            ++e;
            last = now;
        }
        if (now) {
            ++h;
        }
        ++n;
    }
    *cycles = load_counter_elapsed(start, load_counter_read());
    *edges = e;
    *high = h;
    *samples = n;
}
#endif

void EXTI9_5_IRQHandler(void) {
#if !HW_TARGET_HW40
    if (EXTI_PR & (1u << 7)) {
        EXTI_PR = 1u << 7;
        if (!gpio_read(GPIOC_BASE, 1u << 7)) {
            g_dmm_beep_edge_seen = 1;
            if (g_dmm_beep_irq_armed) {
                board_buzzer_start_from_irq();
            }
        }
    }
#else
    if (EXTI_PR & (1u << 7)) {
        EXTI_PR = 1u << 7;
    }
#endif
}

static uint8_t battery_percent_from_mv(uint16_t mv) {
    if (mv >= 4100u) {
        return 100;
    }
    if (mv >= 4000u) {
        return (uint8_t)(95u + (uint32_t)(mv - 4000u) * 5u / 100u);
    }
    if (mv >= 3900u) {
        return (uint8_t)(90u + (uint32_t)(mv - 3900u) * 5u / 100u);
    }
    if (mv >= 3800u) {
        return (uint8_t)(85u + (uint32_t)(mv - 3800u) * 5u / 100u);
    }
    if (mv >= 3700u) {
        return (uint8_t)(75u + (uint32_t)(mv - 3700u) * 10u / 100u);
    }
    if (mv >= 3600u) {
        return (uint8_t)(50u + (uint32_t)(mv - 3600u) * 25u / 100u);
    }
    if (mv >= 3500u) {
        return (uint8_t)(35u + (uint32_t)(mv - 3500u) * 15u / 100u);
    }
    if (mv >= 3400u) {
        return (uint8_t)(15u + (uint32_t)(mv - 3400u) * 20u / 100u);
    }
    if (mv >= 3300u) {
        return (uint8_t)(5u + (uint32_t)(mv - 3300u) * 10u / 100u);
    }
    if (mv >= 3200u) {
        return (uint8_t)(1u + (uint32_t)(mv - 3200u) * 4u / 100u);
    }
    return 0;
}

static uint16_t battery_adc_read_raw(void) {
    ADC_SR(ADC1_BASE) = 0;
    ADC_CR2(ADC1_BASE) |= 1u << 22; // SWSTART
    for (uint32_t timeout = 0; timeout < 100000u; ++timeout) {
        if (ADC_SR(ADC1_BASE) & (1u << 1)) {
            return (uint16_t)(ADC_DR(ADC1_BASE) & 0x0FFFu);
        }
    }
    return 0;
}

void battery_init(void) {
#if HW_TARGET_2C53T
    gpio_config_mask(GPIOB_BASE, 1u << 1, 0x0); // PB1 ADC input (ADC1 ch9 on 2C53T)
#else
    gpio_config_mask(GPIOA_BASE, 1u << 2, 0x0); // PA2 ADC input
#endif

    RCC_APB2ENR |= 1u << 9; // ADC1 clock
    RCC_CFGR = (RCC_CFGR & ~(3u << 14)) | (2u << 14); // ADC clock = PCLK2 / 6

    ADC_CR1(ADC1_BASE) = 0;
    ADC_CR2(ADC1_BASE) = 0;
#if HW_TARGET_2C53T
    ADC_SMPR2(ADC1_BASE) = (ADC_SMPR2(ADC1_BASE) & ~(7u << 27)) | (7u << 27);
    ADC_SQR1(ADC1_BASE) = 0;
    ADC_SQR3(ADC1_BASE) = 9u;
#else
    ADC_SMPR2(ADC1_BASE) = (ADC_SMPR2(ADC1_BASE) & ~(7u << 6)) | (7u << 6);
    ADC_SQR1(ADC1_BASE) = 0;
    ADC_SQR3(ADC1_BASE) = 2u;
#endif

    ADC_CR2(ADC1_BASE) |= 1u; // ADON
    delay_ms(2);
    ADC_CR2(ADC1_BASE) |= 1u << 3; // reset calibration
    for (uint32_t timeout = 0; timeout < 100000u && (ADC_CR2(ADC1_BASE) & (1u << 3)); ++timeout) {
    }
    ADC_CR2(ADC1_BASE) |= 1u << 2; // calibration
    for (uint32_t timeout = 0; timeout < 100000u && (ADC_CR2(ADC1_BASE) & (1u << 2)); ++timeout) {
    }
    ADC_CR2(ADC1_BASE) |= (7u << 17) | (1u << 20) | 1u; // software trigger
    battery_update();
}

void battery_update(void) {
    uint32_t sum = 0;
    uint16_t min_raw = 0x0FFFu;
    uint16_t max_raw = 0;
    uint16_t raw;
    uint16_t raw_mv;

    for (uint8_t i = 0; i < 18; ++i) {
        raw = battery_adc_read_raw();
        sum += raw;
        if (raw < min_raw) {
            min_raw = raw;
        }
        if (raw > max_raw) {
            max_raw = raw;
        }
    }
    sum -= min_raw;
    sum -= max_raw;
    raw_mv = (uint16_t)((sum * 6600u + (4095u * 8u)) / (4095u * 16u));

    if (!g_battery_filter_valid) {
        g_battery_mv = raw_mv;
        g_battery_filter_valid = 1;
    } else {
        g_battery_mv = (uint16_t)(((uint32_t)g_battery_mv * 7u + raw_mv + 4u) / 8u);
    }
    g_battery_percent = battery_percent_from_mv(g_battery_mv);
    battery_update_charging_status();
}

void battery_update_charging_status(void) {
#if HW_TARGET_2C53T
    /* No dedicated charger-status GPIO known on the 2C53T; use the same
     * >4.3V heuristic as the OpenScope 2C53T firmware. */
    g_battery_charging = g_battery_mv > 4300u ? 1u : 0u;
#else
    g_battery_charging = gpio_read(GPIOB_BASE, 1u << 0) ? 0u : 1u;
#endif
}

uint16_t battery_millivolts(void) {
    return g_battery_mv;
}

uint8_t battery_percent(void) {
    return g_battery_percent;
}

uint8_t battery_is_charging(void) {
    return g_battery_charging;
}

#if HW_TARGET_2C53T
/*
 * 2C53T keypad: bidirectional 4x3 matrix + 3 passive pins.
 * Rows PA7/PB0/PC5/PE2, columns PA8/PC10/PE3; passive PC8 (POWER,
 * active LOW), PB7 (PRM, active HIGH), PC13 (UP, active LOW).
 * Scan algorithm ported from OpenScope-2C53T button_scan.c
 * (hardware-confirmed 15/15, extracted from stock firmware).
 *
 * Physical -> logical mapping: like-named keys map 1:1; the three
 * 2C53T-only buttons map to the 2C23T softkeys: SELECT->F2,
 * TRIGGER->F3, PRM->F4 (MOVE is F1 on both).
 */

static void matrix_pin_in_pullup(uint32_t base, uint16_t mask) {
    gpio_config_mask(base, mask, 0x8);
    gpio_set(base, mask);
}

static void matrix_pin_out_low(uint32_t base, uint16_t mask) {
    gpio_config_mask(base, mask, 0x1);
    gpio_clear(base, mask);
}

static void matrix_settle(void) {
    for (volatile uint32_t i = 0; i < 8u; ++i) {
        __asm__ volatile("nop");
    }
}

static void matrix_park(void) {
    matrix_pin_in_pullup(GPIOA_BASE, (1u << 7) | (1u << 8));
    matrix_pin_in_pullup(GPIOB_BASE, 1u << 0);
    matrix_pin_in_pullup(GPIOC_BASE, (1u << 5) | (1u << 10));
    matrix_pin_in_pullup(GPIOE_BASE, (1u << 2) | (1u << 3));
}

void input_init(void) {
    gpio_config_mask(GPIOC_BASE, 1u << 8, 0x8); // PC8 POWER, active low
    gpio_set(GPIOC_BASE, 1u << 8);
    gpio_config_mask(GPIOB_BASE, 1u << 7, 0x8); // PB7 PRM, active HIGH -> pull-down
    gpio_clear(GPIOB_BASE, 1u << 7);
    gpio_config_mask(GPIOC_BASE, 1u << 13, 0x8); // PC13 UP, active low
    gpio_set(GPIOC_BASE, 1u << 13);
    matrix_park();
}

uint32_t input_read_keys(void) {
    uint32_t keys = 0;
    uint8_t row = 0;

    /* Passive pins */
    if (!gpio_read(GPIOC_BASE, 1u << 8)) {
        keys |= KEY_POWER;
    }
    if (gpio_read(GPIOB_BASE, 1u << 7)) {
        keys |= KEY_F4; /* PRM */
    }
    if (!gpio_read(GPIOC_BASE, 1u << 13)) {
        keys |= KEY_UP;
    }

    /* Phase 1: rows input pull-up, columns output LOW */
    matrix_pin_in_pullup(GPIOA_BASE, 1u << 7);
    matrix_pin_in_pullup(GPIOB_BASE, 1u << 0);
    matrix_pin_in_pullup(GPIOC_BASE, 1u << 5);
    matrix_pin_in_pullup(GPIOE_BASE, 1u << 2);
    matrix_pin_out_low(GPIOA_BASE, 1u << 8);
    matrix_pin_out_low(GPIOC_BASE, 1u << 10);
    matrix_pin_out_low(GPIOE_BASE, 1u << 3);
    matrix_settle();

    if (!gpio_read(GPIOA_BASE, 1u << 7)) {
        row |= 1u;
    }
    if (!gpio_read(GPIOC_BASE, 1u << 5)) {
        row |= 2u;
    }
    if (!gpio_read(GPIOB_BASE, 1u << 0)) {
        row |= 4u;
    }
    if (!gpio_read(GPIOE_BASE, 1u << 2)) {
        row |= 8u;
    }

    /* Exactly one active row required, otherwise skip the matrix */
    if (row == 1u || row == 2u || row == 4u || row == 8u) {
        /* Phase 2: swap — rows output LOW, columns input pull-up */
        matrix_pin_in_pullup(GPIOA_BASE, 1u << 8);
        matrix_pin_in_pullup(GPIOC_BASE, 1u << 10);
        matrix_pin_in_pullup(GPIOE_BASE, 1u << 3);
        matrix_pin_out_low(GPIOA_BASE, 1u << 7);
        matrix_pin_out_low(GPIOB_BASE, 1u << 0);
        matrix_pin_out_low(GPIOC_BASE, 1u << 5);
        matrix_pin_out_low(GPIOE_BASE, 1u << 2);
        matrix_settle();

        if (!gpio_read(GPIOE_BASE, 1u << 3)) { /* col PE3 */
            keys |= (row == 1u) ? KEY_CH2
                  : (row == 2u) ? KEY_DOWN
                  : (row == 4u) ? KEY_SAVE
                                : KEY_MENU;
        }
        if (!gpio_read(GPIOA_BASE, 1u << 8)) { /* col PA8 */
            keys |= (row == 1u) ? KEY_F3 /* TRIGGER */
                  : (row == 2u) ? KEY_F2 /* SELECT */
                  : (row == 4u) ? KEY_MOVE
                                : KEY_RIGHT;
        }
        if (!gpio_read(GPIOC_BASE, 1u << 10)) { /* col PC10 */
            keys |= (row == 1u) ? KEY_LEFT
                  : (row == 2u) ? KEY_CH1
                  : (row == 4u) ? KEY_OK
                                : KEY_AUTO;
        }
    }

    matrix_park();
    return keys;
}
#else
void input_init(void) {
    for (uint32_t i = 0; i < sizeof(key_pins) / sizeof(key_pins[0]); ++i) {
        gpio_config_mask(key_pins[i].base, key_pins[i].mask, 0x8);
        if (key_pins[i].active_high) {
            gpio_clear(key_pins[i].base, key_pins[i].mask);
        } else {
            gpio_set(key_pins[i].base, key_pins[i].mask);
        }
    }
}

uint32_t input_read_keys(void) {
    uint32_t keys = 0;
    for (uint32_t i = 0; i < sizeof(key_pins) / sizeof(key_pins[0]); ++i) {
        uint8_t high = gpio_read(key_pins[i].base, key_pins[i].mask) ? 1u : 0u;
        uint8_t pressed = key_pins[i].active_high ? high : (uint8_t)!high;
        if (pressed) {
            keys |= key_pins[i].key;
        }
    }
    return keys;
}
#endif /* HW_TARGET_2C53T */

static uint32_t input_debounced_keys(void) {
    enum {
        DEBOUNCE_STABLE_POLLS = 2,
    };
    static uint32_t stable_keys;
    static uint32_t candidate_keys;
    static uint8_t stable_count;
    uint32_t raw = input_read_keys();

    if (raw == candidate_keys) {
        if (stable_count < DEBOUNCE_STABLE_POLLS) {
            ++stable_count;
        }
    } else {
        candidate_keys = raw;
        stable_count = 0;
    }

    if (stable_count >= DEBOUNCE_STABLE_POLLS) {
        stable_keys = candidate_keys;
    }
    return stable_keys;
}

static uint32_t input_repeat_event(uint32_t now,
                                   uint32_t last,
                                   uint32_t key,
                                   uint16_t *hold_ms,
                                   uint16_t *repeat_ms) {
    enum {
        INPUT_POLL_MS = 20,
        REPEAT_START_MS = 360,
        REPEAT_FAST_MS = 900,
        REPEAT_SLOW_INTERVAL_MS = 120,
        REPEAT_FAST_INTERVAL_MS = 40,
    };
    uint16_t interval;

    if (!(now & key)) {
        *hold_ms = 0;
        *repeat_ms = 0;
        return 0;
    }
    if (!(last & key)) {
        *hold_ms = 0;
        *repeat_ms = 0;
        return 0;
    }

    if (*hold_ms < 2000u) {
        *hold_ms = (uint16_t)(*hold_ms + INPUT_POLL_MS);
    }
    if (*hold_ms < REPEAT_START_MS) {
        return 0;
    }

    interval = *hold_ms >= REPEAT_FAST_MS ? REPEAT_FAST_INTERVAL_MS : REPEAT_SLOW_INTERVAL_MS;
    *repeat_ms = (uint16_t)(*repeat_ms + INPUT_POLL_MS);
    if (*repeat_ms >= interval) {
        *repeat_ms = 0;
        return key | KEY_REPEAT;
    }
    return 0;
}

static uint32_t input_short_long_event(uint32_t now,
                                       uint32_t last,
                                       uint32_t key,
                                       uint32_t long_key,
                                       uint16_t *hold_ms,
                                       uint8_t *long_sent) {
    enum {
        INPUT_POLL_MS = 20,
        LONG_PRESS_MS = 700,
    };

    if (now & key) {
        if (!(last & key)) {
            *hold_ms = 0;
            *long_sent = 0;
            return 0;
        }
        if (!*long_sent) {
            if (*hold_ms < LONG_PRESS_MS) {
                *hold_ms = (uint16_t)(*hold_ms + INPUT_POLL_MS);
            }
            if (*hold_ms >= LONG_PRESS_MS) {
                *long_sent = 1;
                return long_key;
            }
        }
        return 0;
    }

    if (last & key) {
        uint32_t event = *long_sent ? 0u : key;
        *hold_ms = 0;
        *long_sent = 0;
        return event;
    }

    *hold_ms = 0;
    *long_sent = 0;
    return 0;
}

uint32_t input_pressed_events(void) {
    enum {
        SHORT_LONG_KEYS = KEY_MOVE | KEY_F2 | KEY_F3 | KEY_F4 | KEY_AUTO | KEY_SAVE | KEY_CH1,
        STARTUP_GUARD_POLLS = 10,
    };
    static uint32_t last_keys;
    static uint32_t startup_block_keys;
    static uint8_t startup_guard_polls = STARTUP_GUARD_POLLS;
    static uint8_t initialized;
    static uint16_t move_hold_ms;
    static uint16_t f2_hold_ms;
    static uint16_t f3_hold_ms;
    static uint16_t f4_hold_ms;
    static uint16_t auto_hold_ms;
    static uint16_t save_hold_ms;
    static uint16_t left_hold_ms;
    static uint16_t right_hold_ms;
    static uint16_t up_hold_ms;
    static uint16_t down_hold_ms;
    static uint16_t left_repeat_ms;
    static uint16_t right_repeat_ms;
    static uint16_t up_repeat_ms;
    static uint16_t down_repeat_ms;
    static uint8_t move_long_sent;
    static uint8_t f2_long_sent;
    static uint8_t f3_long_sent;
    static uint8_t f4_long_sent;
    static uint8_t auto_long_sent;
    static uint8_t save_long_sent;
    static uint16_t ch1_hold_ms;
    static uint8_t ch1_long_sent;
    uint32_t now = input_debounced_keys();
    uint32_t events;

    if (!initialized) {
        initialized = 1;
        startup_block_keys = 0;
        last_keys = now;
        return 0;
    }

    if (startup_guard_polls) {
        --startup_guard_polls;
        startup_block_keys |= now & SHORT_LONG_KEYS;
        last_keys = now;
        return 0;
    }

    startup_block_keys &= now;
    now &= ~startup_block_keys;
    last_keys &= ~startup_block_keys;
    events = now & ~last_keys;

    // Keys with long actions emit their short action on release, so a long press
    // cannot also run the short action first.
    events &= ~KEY_MOVE;
    events &= ~KEY_F2;
    events &= ~KEY_F3;
    events &= ~KEY_F4;
    events &= ~KEY_AUTO;
    events &= ~KEY_SAVE;
    events &= ~KEY_CH1;

    events |= input_short_long_event(now, last_keys, KEY_MOVE, KEY_MOVE_LONG, &move_hold_ms, &move_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_F2, KEY_F2_LONG, &f2_hold_ms, &f2_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_F3, KEY_F3_LONG, &f3_hold_ms, &f3_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_F4, KEY_F4_LONG, &f4_hold_ms, &f4_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_AUTO, KEY_AUTO_LONG, &auto_hold_ms, &auto_long_sent);
    events |= input_short_long_event(now, last_keys, KEY_SAVE, KEY_SAVE_LONG, &save_hold_ms, &save_long_sent);

    events |= input_repeat_event(now, last_keys, KEY_LEFT, &left_hold_ms, &left_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_RIGHT, &right_hold_ms, &right_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_UP, &up_hold_ms, &up_repeat_ms);
    events |= input_repeat_event(now, last_keys, KEY_DOWN, &down_hold_ms, &down_repeat_ms);

    events |= input_short_long_event(now, last_keys, KEY_CH1, KEY_CH1_LONG, &ch1_hold_ms, &ch1_long_sent);

    last_keys = now;
    return events;
}
