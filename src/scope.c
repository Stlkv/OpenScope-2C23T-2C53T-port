#include "scope.h"

#include "board.h"
#include "fpga.h"
#include "hw.h"

#include <stdint.h>

#ifndef SCOPE_HW_CAPTURE
#define SCOPE_HW_CAPTURE 0
#endif
#ifndef SCOPE_ANALOG_CONFIG
#define SCOPE_ANALOG_CONFIG 0
#endif
#ifndef SCOPE_ATTENUATOR_CONFIG
#define SCOPE_ATTENUATOR_CONFIG 0
#endif

/* Fastest timebase step the MCU-paced sampler is allowed to own, as an index
 * into the UI's timebase table (18 = 50 ms/div = 2 ms per point). A build
 * parameter because it tracks what the hardware will actually sustain, and
 * that number moved twice in one bench session — reverting an over-ambitious
 * floor is then one flag, not a code change. 17 was tried and reverted: see
 * the note on SCOPE_HW_SLOW_TIMEBASE_START below. */
#ifndef SCOPE53_ROLL_FLOOR
#define SCOPE53_ROLL_FLOOR 18
#endif

enum {
    SCOPE_TIMING_SETTLE_MS = 2,
    SCOPE_ANALOG_RANGE_SETTLE_MS = 2,
#if HW_TARGET_2C53T
    /* The 2C53T has no working FPGA timing register (fpga_write_timing is a
     * stub), so every "fast" timebase renders the same free-running 33 us
     * window, and the slow end belongs to the MCU-paced sampler. Where the
     * two meet is set by the ENGINE, and that was measured the hard way.
     * Neither the CPU nor the bus is the limit any more: with DMA the pacer
     * tick costs ~10 us and a 1023-byte window moves in 0.34 ms at 24 MHz.
     * What does not keep up is the FPGA refilling the window it just handed
     * over. At index 18 (a drain every 2 ms) the points carry signal —
     * p=00-94; at index 17 (every 1.2 ms) they freeze to p=92-93 with over=0,
     * i.e. we are simply re-reading a window the engine has not refreshed.
     * So the refill takes somewhere between 1.2 and 2 ms, and ~500 points/s
     * is the hardware ceiling of this timebase. Reading faster buys nothing;
     * only a different acquisition scheme (equivalent-time sampling of the
     * fast window) can show a waveform above ~25 Hz. */
    SCOPE_FAST_ALIGN_TIMEBASE_MAX = 5,
    SCOPE_HW_SLOW_TIMEBASE_START = SCOPE53_ROLL_FLOOR,
#elif HW_TARGET_HW40
    SCOPE_FAST_ALIGN_TIMEBASE_MAX = 5,
    SCOPE_HW_SLOW_TIMEBASE_START = 21,
#else
    SCOPE_FAST_ALIGN_TIMEBASE_MAX = 3,
    SCOPE_HW_SLOW_TIMEBASE_START = 18,
#endif
    SCOPE_SLOW_POINT_COUNT = 300,
    SCOPE_SLOW_OVERRUN_SKIP = 4,
#if HW_TARGET_2C53T
    /* 100 kHz tick: 10 us of interval resolution, and 1 s/div still fits the
     * 16-bit period register (40 ms per point = 4000 ticks). */
    SCOPE_SLOW_TIMER_TICK_HZ = 100000u,
#else
    SCOPE_SLOW_TIMER_TICK_HZ = 10000u,
#endif
};

/* Timer clock feeding TMR1's prescaler. The port never reprograms the PLL, so
 * this is whatever the factory bootloader left behind — MEASURED on the bench
 * 2026-08-15, not assumed: with psc=719/pr=19 (a nominal 200 us at the old
 * 72 MHz guess) the sampler produced 17008 points/s = 58.8 us apiece over an
 * 87 s window, a factor of 3.40 fast. 240/72 = 3.33, and 240 MHz is this
 * part's ceiling, so the bootloader leaves the timer clock at 240 MHz.
 * Method, if it ever needs redoing: two DBG dumps a known wall-clock apart,
 * divide the point counter (32-bit for exactly this reason). */
#ifndef SCOPE_SLOW_TIMER_CLK_HZ
#define SCOPE_SLOW_TIMER_CLK_HZ 240000000u
#endif

enum {
    SCOPE_STATUS_IDLE,
    SCOPE_STATUS_NO_FPGA,
    SCOPE_STATUS_WAIT,
    SCOPE_STATUS_READ_ERROR,
    SCOPE_STATUS_FRAME,
};

static uint16_t scope_frame_count;
static uint16_t scope_wait_count;
static uint16_t scope_error_count;
static uint8_t scope_last_status;
static volatile uint8_t scope_slow_irq_enabled;
static volatile uint8_t scope_slow_samples[SCOPE_SLOW_POINT_COUNT * 2u];
static volatile uint16_t scope_slow_write_index;
static volatile uint16_t scope_slow_count;
static volatile uint16_t scope_slow_seq;
static uint16_t scope_slow_psc;
static uint16_t scope_slow_pr;
static uint16_t scope_slow_cost;         /* longest point read, timer ticks */
static uint16_t scope_slow_over;         /* point reads that outlasted their interval */
static volatile uint8_t scope_slow_skip; /* intervals to sit out after an overrun */

#if SCOPE_HW_CAPTURE && SCOPE_ANALOG_CONFIG
static uint8_t scope_analog_ready;
static uint16_t last_scope_dac[2] = {0xFFFFu, 0xFFFFu};
#if SCOPE_ATTENUATOR_CONFIG
static uint8_t last_scope_vdiv[2] = {0xFFu, 0xFFu};
#endif

#if SCOPE_ATTENUATOR_CONFIG
static void gpio_write_pin(uint32_t base, uint32_t mask, uint8_t value) {
    if (value) {
        gpio_set(base, mask);
    } else {
        gpio_clear(base, mask);
    }
}
#endif

static uint16_t scope_clamp_dac(uint16_t value) {
    return value > 4095u ? 4095u : value;
}

/* CH1's offset is DAC1 on both boards. CH2's is NOT DAC2 on the 2C53T: its
 * comparator reference is a TMR13 CH1 PWM on PA6 behind an RC filter (stock
 * V1.2.0, upstream's decode of gpio_mux_porta_portb; C1DT @ 0x40001C34).
 * Writing DHR12RD here moved DAC2/PA5, which is in nothing's path on this
 * board, so the CH2 offset control was inert while the real reference sat at
 * whatever the boot arm left it — the knob turned and nothing followed it.
 * Route CH2 to the timer on 2C53T builds; the 2C23T path is unchanged. */
static void scope_dac_set_offsets(uint16_t ch1_dac, uint16_t ch2_dac) {
    enum {
        DAC_DHR12RD_ADDR = 0x40007420u,
        DAC_DHR12R1_ADDR = 0x40007408u,
    };

    ch1_dac = scope_clamp_dac(ch1_dac);
    ch2_dac = scope_clamp_dac(ch2_dac);
    if (ch1_dac == last_scope_dac[0] && ch2_dac == last_scope_dac[1]) {
        return;
    }
#if HW_TARGET_2C53T
    REG32(DAC_DHR12R1_ADDR) = (uint32_t)ch1_dac;
    fpga53_ch2_ref_set(ch2_dac);
#else
    REG32(DAC_DHR12RD_ADDR) = (uint32_t)ch1_dac | ((uint32_t)ch2_dac << 16);
#endif
    last_scope_dac[0] = ch1_dac;
    last_scope_dac[1] = ch2_dac;
}

static void scope_dac_init(uint16_t ch1_dac, uint16_t ch2_dac) {
    enum {
        DAC_CR_ADDR = 0x40007400u,
        DAC_EN1 = 1u << 0,
        DAC_BOFF1 = 1u << 1,
        DAC_EN2 = 1u << 16,
        DAC_BOFF2 = 1u << 17,
    };

    RCC_APB1ENR |= 1u << 29; // DAC
    gpio_config_mask(GPIOA_BASE, (1u << 4) | (1u << 5), 0x0);
    REG32(DAC_CR_ADDR) = (REG32(DAC_CR_ADDR) & ~(DAC_BOFF1 | DAC_BOFF2)) | DAC_EN1 | DAC_EN2;
    scope_dac_set_offsets(ch1_dac, ch2_dac);
}

#if SCOPE_ATTENUATOR_CONFIG
static void scope_set_channel_range(uint8_t channel, uint16_t range_code) {
    uint8_t a = 0;
    uint8_t b = 0;
    uint8_t c = 0;
    uint8_t d = 0;

    switch (range_code) {
    case 0:
        b = 1;
        c = 1;
        break;
    case 2000:
        a = 1;
        break;
    case 1000:
        a = 1;
        d = 1;
        break;
    case 500:
        a = 1;
        c = 1;
        break;
    case 400:
        a = 1;
        c = 1;
        d = 1;
        break;
    case 200:
        a = 1;
        b = 1;
        break;
    case 100:
        a = 1;
        b = 1;
        d = 1;
        break;
    case 20:
        d = 1;
        break;
    case 10:
        c = 1;
        break;
    case 8:
        c = 1;
        d = 1;
        break;
    case 4:
        b = 1;
        break;
    case 2:
        b = 1;
        d = 1;
        break;
    case 40:
    default:
        break;
    }

    if (channel == 1u) {
        gpio_write_pin(GPIOA_BASE, 1u << 6, a);
        gpio_write_pin(GPIOD_BASE, 1u << 2, b);
        gpio_write_pin(GPIOC_BASE, 1u << 12, c);
        gpio_write_pin(GPIOC_BASE, 1u << 11, d);
    } else {
        gpio_write_pin(GPIOA_BASE, 1u << 7, a);
        gpio_write_pin(GPIOD_BASE, 1u << 12, b);
        gpio_write_pin(GPIOD_BASE, 1u << 13, c);
        gpio_write_pin(GPIOB_BASE, 1u << 1, d);
    }
}
#endif

#if SCOPE_ATTENUATOR_CONFIG
static uint16_t scope_range_mv(uint8_t vdiv) {
    static const uint16_t ranges[] = {1000, 400, 200, 100, 40, 20, 10, 4, 2};

    if (vdiv >= (uint8_t)(sizeof(ranges) / sizeof(ranges[0]))) {
        vdiv = 0;
    }
    return ranges[vdiv];
}
#endif

static void scope_analog_begin(void) {
#if SCOPE_ATTENUATOR_CONFIG
    gpio_config_mask(GPIOA_BASE, (1u << 6) | (1u << 7) | (1u << 10), 0x1);
    gpio_config_mask(GPIOB_BASE, 1u << 1, 0x1);
    gpio_config_mask(GPIOC_BASE, (1u << 11) | (1u << 12), 0x1);
    gpio_config_mask(GPIOD_BASE, (1u << 2) | (1u << 12) | (1u << 13), 0x1);
    gpio_config_mask(GPIOE_BASE, 1u << 0, 0x1);
#else
    gpio_config_mask(GPIOA_BASE, 1u << 10, 0x1);
    gpio_config_mask(GPIOE_BASE, 1u << 0, 0x1);
#endif

    if (scope_analog_ready) {
        return;
    }

    gpio_set(GPIOA_BASE, 1u << 10); // CH1 DC coupling
    gpio_set(GPIOE_BASE, 1u << 0);  // CH2 DC coupling
    /* 2048 centers CH1 because DAC1 is a real DAC; CH2's centering code is a
     * measured property of the TMR13 PWM + RC and lives with the timer, so ask
     * for it rather than assuming the two are the same number. */
#if HW_TARGET_2C53T
    scope_dac_init(2048u, fpga53_ch2_ref_get());
#else
    scope_dac_init(2048u, 2048u);
#endif
    scope_analog_ready = 1;
}
#endif

/* Timebase step -> nanoseconds per TENTH of a horizontal division: index 18
 * holds 5000000 and the UI labels that step 50MS. Everything reading this
 * table has to multiply by ten to get a division, which is exactly the trap
 * the roll interval fell into first time round.
 * One copy: the app sits a few hundred bytes under the 224 KB self-update
 * ceiling, and three private copies of this cost more than they read. */
static const uint32_t timebase_unit_ns[] = {
    5u, 10u, 20u, 50u, 100u, 200u, 500u,
    1000u, 2000u, 5000u, 10000u, 20000u, 50000u,
    100000u, 200000u, 500000u, 1000000u, 2000000u, 5000000u,
    10000000u, 20000000u, 50000000u, 100000000u, 200000000u,
    500000000u, 1000000000u,
};

static uint32_t scope_span_for_timebase(uint8_t timebase) {
    uint32_t ns;
    uint32_t span;

    if (timebase >= (uint8_t)(sizeof(timebase_unit_ns) / sizeof(timebase_unit_ns[0]))) {
        timebase = 4;
    }
    if (timebase >= SCOPE_HW_SLOW_TIMEBASE_START) {
        return 0x80000u;
    }
    ns = timebase_unit_ns[timebase];
    span = 52428800u / ns;
    if (span > 0x100000u) {
        span = 0x100000u;
    }
    return span;
}

#if SCOPE_HW_CAPTURE && HW_TARGET_2C53T
/* Interval between roll points, in timer ticks. The ms-granular sibling below
 * bottoms out at 1 ms per point — 3.3 ms/div — which is far slower than the
 * bus can sample and would leave the whole interesting range (mains hum,
 * audio, anything in the hundreds of Hz) unreachable. Ticks give 10 us. */
static uint16_t scope_slow_interval_ticks_for_timebase(uint8_t timebase) {
    /* 12 divisions across the screen, SCOPE_SLOW_POINT_COUNT points in it —
     * so one point is a 25th of a division. Careful with the table's unit:
     * timebase_unit_ns is a TENTH of a division (index 18 holds 5000000 and
     * the UI labels it 50MS), so the division is worth ten of those. Divide
     * before multiplying — ten seconds per division in nanoseconds would not
     * fit in 32 bits. */
    enum { POINTS_PER_DIV = SCOPE_SLOW_POINT_COUNT / 12u };
    const uint32_t tick_ns = 1000000000u / SCOPE_SLOW_TIMER_TICK_HZ;
    uint32_t unit_ns;
    uint32_t point_ns;
    uint32_t ticks;

    if (timebase >= (uint8_t)(sizeof(timebase_unit_ns) / sizeof(timebase_unit_ns[0]))) {
        timebase = 19u;
    }
    unit_ns = timebase_unit_ns[timebase];
    point_ns = (unit_ns / POINTS_PER_DIV) * 10u +
               ((unit_ns % POINTS_PER_DIV) * 10u) / POINTS_PER_DIV;
    ticks = (point_ns + tick_ns / 2u) / tick_ns;
    if (!ticks) {
        ticks = 1u;
    }
    if (ticks > 0xFFFFu) {
        ticks = 0xFFFFu;
    }
    return (uint16_t)ticks;
}
#endif /* SCOPE_HW_CAPTURE && HW_TARGET_2C53T */

#if SCOPE_HW_CAPTURE && !HW_TARGET_2C53T
static uint16_t scope_slow_interval_ms_for_timebase(uint8_t timebase) {
    uint32_t div_ms;
    uint32_t interval;

    if (timebase >= (uint8_t)(sizeof(timebase_unit_ns) / sizeof(timebase_unit_ns[0]))) {
        timebase = 19u;
    }
    div_ms = (timebase_unit_ns[timebase] + 50000u) / 100000u;
    if (!div_ms) {
        div_ms = 1u;
    }
    interval = (div_ms * 12u + SCOPE_SLOW_POINT_COUNT / 2u) / SCOPE_SLOW_POINT_COUNT;
    if (!interval) {
        interval = 1u;
    }
    if (interval > 60000u) {
        interval = 60000u;
    }
    return (uint16_t)interval;
}
#endif /* SCOPE_HW_CAPTURE && !HW_TARGET_2C53T */

uint8_t scope_hw_enabled(void) {
    return SCOPE_HW_CAPTURE ? 1u : 0u;
}

uint8_t scope_hw_ready(void) {
#if SCOPE_HW_CAPTURE
    return fpga_ready();
#else
    return 1u;
#endif
}

void scope_hw_configure_channels(uint8_t timebase,
                                 uint8_t ch1_vdiv,
                                 uint8_t ch2_vdiv,
                                 uint8_t ch1_dc,
                                 uint8_t ch2_dc,
                                 uint16_t ch1_dac,
                                 uint16_t ch2_dac) {
#if !SCOPE_HW_CAPTURE
    (void)timebase;
    (void)ch1_vdiv;
    (void)ch2_vdiv;
    (void)ch1_dc;
    (void)ch2_dc;
    (void)ch1_dac;
    (void)ch2_dac;
#endif
#if SCOPE_HW_CAPTURE
#if HW_TARGET_2C53T
    fpga53_note_configure();
#endif
    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return;
    }
#if SCOPE_ANALOG_CONFIG
    scope_analog_begin();
    if (ch1_dc) {
        gpio_set(GPIOA_BASE, 1u << 10);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 10);
    }
    if (ch2_dc) {
        gpio_set(GPIOE_BASE, 1u << 0);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 0);
    }

#if SCOPE_ANALOG_CONFIG && SCOPE_ATTENUATOR_CONFIG
    uint8_t range_changed = 0;
    if (ch1_vdiv != last_scope_vdiv[0]) {
        scope_set_channel_range(1, scope_range_mv(ch1_vdiv));
        last_scope_vdiv[0] = ch1_vdiv;
        range_changed = 1;
    }
    if (ch2_vdiv != last_scope_vdiv[1]) {
        scope_set_channel_range(2, scope_range_mv(ch2_vdiv));
        last_scope_vdiv[1] = ch2_vdiv;
        range_changed = 1;
    }
    if (range_changed) {
        delay_ms(SCOPE_ANALOG_RANGE_SETTLE_MS);
    }
#else
    (void)ch1_vdiv;
    (void)ch2_vdiv;
#endif
    scope_dac_set_offsets(ch1_dac, ch2_dac);
#else
    (void)ch1_dc;
    (void)ch2_dc;
    (void)ch1_dac;
    (void)ch2_dac;
    (void)ch1_vdiv;
    (void)ch2_vdiv;
#endif
    fpga_write_scope_timing(scope_span_for_timebase(timebase));
#if HW_TARGET_2C53T
    /* The span above means nothing on this board — fpga_write_scope_timing is
     * a stub. The rate is set by the engine's own index. */
    fpga53_set_timebase(timebase);
    /* Volts/div, for the first time as an analog control rather than a label:
     * each channel's relay bank takes the ladder row its step calls for. The
     * SCOPE_ATTENUATOR_CONFIG path above stays off on this board — it is
     * 2C23T's, and it drives one shared range index at the wrong pins. */
    fpga53_set_channel_range(0u, ch1_vdiv);
    fpga53_set_channel_range(1u, ch2_vdiv);
#endif
    delay_ms(SCOPE_TIMING_SETTLE_MS);

    fpga_capture_latch();
#else
    (void)timebase;
    (void)ch1_vdiv;
    (void)ch2_vdiv;
    (void)ch1_dc;
    (void)ch2_dc;
#endif
}

void scope_hw_configure(uint8_t timebase, uint8_t vdiv) {
    scope_hw_configure_channels(timebase, vdiv, vdiv, 1, 1, 2048u, 2048u);
}

void scope_hw_slow_stop(void) {
#if SCOPE_HW_CAPTURE
    scope_slow_irq_enabled = 0;
    TMR_IDEN(TMR1_BASE) &= ~1u;
    TMR_CTRL1(TMR1_BASE) &= ~1u;
    TMR_STS(TMR1_BASE) = ~1u;
#endif
}

void scope_hw_slow_start(uint8_t timebase) {
#if SCOPE_HW_CAPTURE
#if HW_TARGET_2C53T
    /* Halved on purpose: the DMA sampler needs two ticks per point (one to
     * start a channel's transfer, one to collect it), so the pacer runs at
     * twice the point rate. See the sampler comment in fpga.c. */
    uint32_t ticks = (uint32_t)scope_slow_interval_ticks_for_timebase(timebase) / 2u;
#else
    uint16_t interval_ms = scope_slow_interval_ms_for_timebase(timebase);
    uint32_t ticks = ((uint32_t)interval_ms * SCOPE_SLOW_TIMER_TICK_HZ) / 1000u;
#endif

    if (!ticks) {
        ticks = 1u;
    }
    if (ticks > 0xFFFFu) {
        ticks = 0xFFFFu;
    }

    scope_hw_slow_stop();
    __asm__ volatile("cpsid i" ::: "memory");
    scope_slow_write_index = 0;
    scope_slow_count = 0;
    scope_slow_cost = 0;
    scope_slow_over = 0;
    scope_slow_skip = 0;
    ++scope_slow_seq;
    __asm__ volatile("cpsie i" ::: "memory");

    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return;
    }

    fpga_write_scope_timing(scope_span_for_timebase(timebase));
#if HW_TARGET_2C53T
    /* The span above means nothing on this board — fpga_write_scope_timing is
     * a stub. The rate is set by the engine's own index. */
    fpga53_set_timebase(timebase);
#endif
    delay_ms(SCOPE_TIMING_SETTLE_MS);
    fpga_capture_latch();

#if HW_TARGET_2C53T
    /* Always full: a short read hands back a window the engine has not
     * refreshed (bench 2026-08-15 — the point span collapsed to a single
     * value), so a "cheap" point is not a point at all. The price is 1.6 ms
     * of SPI per point, which is what sets SCOPE_HW_SLOW_TIMEBASE_START. */
    fpga53_slow_point_set_full(1u);
#endif

    RCC_APB2ENR |= 1u << 11; // TMR1
    TMR_CTRL1(TMR1_BASE) = 0;
    TMR_IDEN(TMR1_BASE) = 0;
    scope_slow_psc = (uint16_t)((SCOPE_SLOW_TIMER_CLK_HZ / SCOPE_SLOW_TIMER_TICK_HZ) - 1u);
    scope_slow_pr = (uint16_t)(ticks - 1u);
    TMR_PSC(TMR1_BASE) = scope_slow_psc;
    TMR_PR(TMR1_BASE) = scope_slow_pr;
    TMR_EG(TMR1_BASE) = 1u;
    TMR_STS(TMR1_BASE) = ~1u;
    scope_slow_irq_enabled = 1;
    TMR_IDEN(TMR1_BASE) |= 1u;
    REG32(NVIC_ISER0) = 1u << 25; // TMR1 update IRQ
    TMR_CTRL1(TMR1_BASE) = 1u;
#else
    (void)timebase;
#endif
}

void scope_hw_slow_timer_debug(uint16_t *psc, uint16_t *pr, uint16_t *cost, uint16_t *over) {
    if (psc) {
        *psc = scope_slow_psc;
    }
    if (pr) {
        *pr = scope_slow_pr;
    }
    if (cost) {
        *cost = scope_slow_cost;
    }
    if (over) {
        *over = scope_slow_over;
    }
}

uint8_t scope_hw_slow_snapshot(uint8_t *ch1,
                               uint8_t *ch2,
                               uint16_t max_points,
                               uint16_t *count,
                               uint16_t *seq) {
#if SCOPE_HW_CAPTURE
    uint16_t local_count;
    uint16_t local_write;
    uint16_t local_seq;
    uint16_t start;

    if (!ch1 || !ch2 || !count || !seq || !max_points) {
        return 0;
    }

    __asm__ volatile("cpsid i" ::: "memory");
    local_count = scope_slow_count;
    local_write = scope_slow_write_index;
    local_seq = scope_slow_seq;
    if (local_count > max_points) {
        local_count = max_points;
    }
    start = scope_slow_count >= SCOPE_SLOW_POINT_COUNT ? local_write : 0u;
    for (uint16_t i = 0; i < local_count; ++i) {
        uint16_t src = (uint16_t)(start + i);
        if (src >= SCOPE_SLOW_POINT_COUNT) {
            src = (uint16_t)(src - SCOPE_SLOW_POINT_COUNT);
        }
        ch1[i] = scope_slow_samples[(uint16_t)(src * 2u)];
        ch2[i] = scope_slow_samples[(uint16_t)(src * 2u + 1u)];
    }
    __asm__ volatile("cpsie i" ::: "memory");

    *count = local_count;
    *seq = local_seq;
    return 1;
#else
    (void)ch1;
    (void)ch2;
    (void)max_points;
    (void)count;
    (void)seq;
    return 0;
#endif
}

void scope_hw_slow_irq_handler(void) {
#if SCOPE_HW_CAPTURE
    uint8_t sample[2];
    uint16_t dst;
    uint16_t t0;

    if (!(TMR_STS(TMR1_BASE) & 1u)) {
        return;
    }
    TMR_STS(TMR1_BASE) = ~1u;
    if (!scope_slow_irq_enabled) {
        return;
    }
    /* Overrun brake. A point read that outlasts its interval leaves the update
     * flag already set on the way out, so the handler is re-entered forever and
     * the main loop never runs — the UI simply freezes (bench 2026-08-15, the
     * 50 ms/div step with full-window reads). Skipping a few intervals after an
     * overrun costs sample points and keeps the instrument answering. */
    if (scope_slow_skip) {
        --scope_slow_skip;
        return;
    }
    t0 = (uint16_t)TMR_CVAL(TMR1_BASE);
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return;
    }
    /* No point this tick is routine, not an error: the DMA sampler answers on
     * every second tick, and backs off entirely while the main loop holds the
     * bus. */
    if (!fpga_capture_read_slow_point(sample)) {
        return;
    }
    if (TMR_STS(TMR1_BASE) & 1u) {
        /* The pacer wrapped while we held the bus: the read costs more than the
         * interval, so its true cost is unknown here — only that it is over. */
        ++scope_slow_over;
        scope_slow_skip = SCOPE_SLOW_OVERRUN_SKIP;
    } else {
        /* The counter wraps at pr, and it can wrap between the flag check
         * above and this read — take the modular difference rather than
         * trusting t1 >= t0, or a 6-tick read reports as 65522 (bench
         * 2026-08-15, where exactly that happened). */
        uint16_t t1 = (uint16_t)TMR_CVAL(TMR1_BASE);
        uint16_t span = (uint16_t)(scope_slow_pr + 1u);
        uint16_t cost = (uint16_t)(t1 >= t0 ? (t1 - t0) : (span - t0 + t1));
        if (cost > scope_slow_cost) {
            scope_slow_cost = cost;
        }
    }

    dst = scope_slow_write_index;
    scope_slow_samples[(uint16_t)(dst * 2u)] = sample[0];
    scope_slow_samples[(uint16_t)(dst * 2u + 1u)] = sample[1];
    ++dst;
    if (dst >= SCOPE_SLOW_POINT_COUNT) {
        dst = 0;
    }
    scope_slow_write_index = dst;
    if (scope_slow_count < SCOPE_SLOW_POINT_COUNT) {
        ++scope_slow_count;
    }
    ++scope_slow_seq;
    scope_last_status = SCOPE_STATUS_FRAME;
    ++scope_frame_count;
#endif
}

void TMR1_UP_IRQHandler(void) {
    scope_hw_slow_irq_handler();
}

void scope_hw_set_offsets(uint16_t ch1_dac, uint16_t ch2_dac) {
#if SCOPE_HW_CAPTURE && SCOPE_ANALOG_CONFIG
    scope_analog_begin();
    scope_dac_set_offsets(ch1_dac, ch2_dac);
#else
    (void)ch1_dac;
    (void)ch2_dac;
#endif
}

void scope_hw_arm(void) {
#if SCOPE_HW_CAPTURE
    fpga_init_once();
    if (fpga_ready()) {
        fpga_capture_latch();
    } else {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
    }
#endif
}

uint8_t scope_hw_capture(uint8_t *dst, uint16_t len, uint8_t timebase) {
#if SCOPE_HW_CAPTURE
    uint8_t *read_dst;
    uint16_t read_len;

    if (!dst || len < FPGA_SCOPE_BUFFER_BYTES) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }

    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return 0;
    }
    if (!fpga_capture_ready()) {
        scope_last_status = SCOPE_STATUS_WAIT;
        ++scope_wait_count;
        return 0;
    }

    read_dst = dst;
    read_len = FPGA_SCOPE_BUFFER_BYTES;
    if (timebase <= SCOPE_FAST_ALIGN_TIMEBASE_MAX) {
        // Original firmware reads the fastest ranges into buffer+1.
        // That keeps the FPGA byte stream phase aligned for CH1/CH2 pairs.
        dst[0] = 128u;
        dst[1] = 128u;
        read_dst = dst + 1u;
        read_len = FPGA_SCOPE_BUFFER_BYTES - 1u;
    }

    if (!fpga_capture_read(read_dst, read_len)) {
        fpga_capture_latch();
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }
    if (timebase <= SCOPE_FAST_ALIGN_TIMEBASE_MAX) {
        dst[0] = dst[2];
        dst[1] = dst[3];
    }
    fpga_capture_latch();
    scope_last_status = SCOPE_STATUS_FRAME;
    ++scope_frame_count;
    return 1;
#else
    (void)dst;
    (void)len;
    (void)timebase;
    return 0;
#endif
}

uint8_t scope_hw_capture_point(uint8_t sample[2], uint8_t timebase) {
#if SCOPE_HW_CAPTURE
    if (!sample) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }
    (void)timebase;

    fpga_init_once();
    if (!fpga_ready()) {
        scope_last_status = SCOPE_STATUS_NO_FPGA;
        ++scope_error_count;
        return 0;
    }
    if (!fpga_capture_read_slow_point(sample)) {
        scope_last_status = SCOPE_STATUS_READ_ERROR;
        ++scope_error_count;
        return 0;
    }
    scope_last_status = SCOPE_STATUS_FRAME;
    ++scope_frame_count;
    return 1;
#else
    (void)sample;
    (void)timebase;
    return 0;
#endif
}

uint16_t scope_hw_frame_count(void) {
    return scope_frame_count;
}

uint16_t scope_hw_wait_count(void) {
    return scope_wait_count;
}

uint16_t scope_hw_error_count(void) {
    return scope_error_count;
}

uint8_t scope_hw_last_status(void) {
    return scope_last_status;
}
