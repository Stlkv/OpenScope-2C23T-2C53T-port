#include "fpga.h"

#include "app_config.h"
#include "board.h"
#include "fpga_bitstream.h"
#include "hw.h"

#include <stdint.h>

#ifndef HW_TARGET_2C53T
#define HW_TARGET_2C53T 0
#endif

#if HW_TARGET_2C53T
/*
 * 2C53T warm-handoff transport (READ-ONLY, experimental).
 *
 * The 2C53T FPGA link is hardware SPI3 (PB3 SCK / PB4 MISO / PB5 MOSI,
 * JTAG pins released via AFIO remap), software CS on PB6 (idle HIGH),
 * SPI mode 3, with PC6 = SPI enable (HIGH), PB11 = active mode (HIGH)
 * and PC0 = data-ready from the FPGA.
 *
 * Acquisition protocol (from the OpenScope-2C53T issue-#18 Saleae capture
 * of a stock boot): one CS-LOW window per channel — opcode 0x04 (CH1) /
 * 0x05 (CH2), then 2 more status bytes, then 1023 unsigned samples,
 * 1026 bytes total. Stock applies an ADC offset of -28.
 *
 * This build deliberately performs NO configuration: no bitstream upload,
 * no config commands (all three known config-entry routes are measured
 * dead on this FPGA — it only accepts configuration in ways stock somehow
 * triggers). Instead it relies on a WARM HANDOFF: boot stock, let it
 * configure the FPGA and enter scope mode, then re-flash this firmware
 * via MENU + pinhole-reset (board power never drops, USB attached).
 * If the FPGA's SRAM design and acquisition state survive the MCU swap,
 * the reads below return live waveforms; if not, they return flat 0xFF —
 * either result is a useful experiment.
 */

enum {
    FPGA53_SPI_STS_RXNE = 1u << 0,
    FPGA53_SPI_STS_TXE = 1u << 1,
    FPGA53_SPI_STS_BSY = 1u << 7,
    FPGA53_XFER_TIMEOUT = 100000u,
    FPGA53_ADC_OFFSET = 28u, /* subtracted from raw samples, per stock */
    FPGA53_CH_SAMPLES = 1023u,
    FPGA53_FORCED_READ_POLLS = 200u,
};

#ifndef FPGA53_SWAP_ORDER
#define FPGA53_SWAP_ORDER 0
#endif

#ifndef FPGA53_PRE_CMD
#define FPGA53_PRE_CMD 0
#endif

#ifndef FPGA53_SEND_CFG
#define FPGA53_SEND_CFG 0
#endif

#ifndef FPGA_SPI_BR
#define FPGA_SPI_BR 2u
#endif

static uint8_t fpga_loaded;
static uint8_t fpga53_frame[FPGA_SCOPE_BUFFER_BYTES];
static uint8_t fpga53_ch_buf[FPGA53_CH_SAMPLES];
static uint16_t fpga53_notready_polls;
static uint8_t fpga53_force_read;
static fpga53_diag_t fpga53_diag = { .fe_idx = 0xFFu, .fe_idx_b = 0xFFu };

void fpga53_note_configure(void) {
    ++fpga53_diag.cfg_calls;
}

void fpga53_note_poll(void) {
    ++fpga53_diag.poll_calls;
}

/* Analog-frontend experiment: PRM (F4) in scope mode cycles 16 patterns of
 * {PC12, PE4, PE5, PE6} — input-routing and attenuation controls per the
 * OpenScope-2C53T pinout. After the warm-handoff MCU reset these pins float
 * (stock drove them); CH1 happens to conduct in the floating state, CH2 is
 * open. Pattern bits: idx3=PC12, idx2=PE4, idx1=PE5, idx0=PE6. */
void fpga53_fe_cycle(void) {
    uint8_t idx = fpga53_diag.fe_idx;
    idx = (uint8_t)((idx == 0xFFu) ? 0u : ((idx + 1u) & 0x0Fu));
    fpga53_diag.fe_idx = idx;

    if (idx & 0x08u) {
        gpio_set(GPIOC_BASE, 1u << 12);
    } else {
        gpio_clear(GPIOC_BASE, 1u << 12);
    }
    if (idx & 0x04u) {
        gpio_set(GPIOE_BASE, 1u << 4);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 4);
    }
    if (idx & 0x02u) {
        gpio_set(GPIOE_BASE, 1u << 5);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 5);
    }
    if (idx & 0x01u) {
        gpio_set(GPIOE_BASE, 1u << 6);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 6);
    }
    gpio_config_mask(GPIOC_BASE, 1u << 12, 0x1u);
    gpio_config_mask(GPIOE_BASE, (1u << 4) | (1u << 5) | (1u << 6), 0x1u);
}

/* Bank B: gain-select / undocumented frontend pins.
 * Bits: idx3=PA15, idx2=PA10, idx1=PB9, idx0=PA6. */
void fpga53_fe_cycle_b(void) {
    uint8_t idx = fpga53_diag.fe_idx_b;
    idx = (uint8_t)((idx == 0xFFu) ? 0u : ((idx + 1u) & 0x0Fu));
    fpga53_diag.fe_idx_b = idx;

    if (idx & 0x08u) {
        gpio_set(GPIOA_BASE, 1u << 15);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 15);
    }
    if (idx & 0x04u) {
        gpio_set(GPIOA_BASE, 1u << 10);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 10);
    }
    if (idx & 0x02u) {
        gpio_set(GPIOB_BASE, 1u << 9);
    } else {
        gpio_clear(GPIOB_BASE, 1u << 9);
    }
    if (idx & 0x01u) {
        gpio_set(GPIOA_BASE, 1u << 6);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 6);
    }
    gpio_config_mask(GPIOA_BASE, (1u << 15) | (1u << 10) | (1u << 6), 0x1u);
    gpio_config_mask(GPIOB_BASE, 1u << 9, 0x1u);
}

void fpga53_get_diag(fpga53_diag_t *d) {
    if (!d) {
        return;
    }
    fpga53_diag.inited = fpga_loaded;
    fpga53_diag.pc0 = (GPIO_IDR(GPIOC_BASE) & 1u) ? 1u : 0u;
    d->inited = fpga53_diag.inited;
    d->pc0 = fpga53_diag.pc0;
    d->reads = fpga53_diag.reads;
    d->forced = fpga53_diag.forced;
    d->r0 = fpga53_diag.r0;
    d->r1 = fpga53_diag.r1;
    d->r2 = fpga53_diag.r2;
    d->smin = fpga53_diag.smin;
    d->smax = fpga53_diag.smax;
    d->init_calls = fpga53_diag.init_calls;
    d->cfg_calls = fpga53_diag.cfg_calls;
    d->poll_calls = fpga53_diag.poll_calls;
    for (uint8_t i = 0; i < 5u; ++i) {
        d->cst[i] = fpga53_diag.cst[i];
    }
    d->smin2 = fpga53_diag.smin2;
    d->smax2 = fpga53_diag.smax2;
    d->dup = fpga53_diag.dup;
    d->fe_idx = fpga53_diag.fe_idx;
    d->fe_idx_b = fpga53_diag.fe_idx_b;
}

static uint8_t fpga53_xfer(uint8_t tx) {
    uint32_t timeout = FPGA53_XFER_TIMEOUT;
    while (!(SPI_STS(SPI3_BASE) & FPGA53_SPI_STS_TXE)) {
        if (!--timeout) {
            return 0xFFu;
        }
    }
    SPI_DT(SPI3_BASE) = tx;
    timeout = FPGA53_XFER_TIMEOUT;
    while (!(SPI_STS(SPI3_BASE) & FPGA53_SPI_STS_RXNE)) {
        if (!--timeout) {
            return 0xFFu;
        }
    }
    return (uint8_t)SPI_DT(SPI3_BASE);
}

void fpga_init_once(void) {
    ++fpga53_diag.init_calls;
    if (fpga_loaded) {
        return;
    }

    RCC_APB1ENR |= 1u << 15; // SPI3
    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24); // release PB3/PB4/PB5 from JTAG

    /* Pre-load safe output levels BEFORE switching pin modes (no glitches):
     * CS idle HIGH, SPI-enable HIGH, active-mode HIGH — same levels stock
     * holds in scope mode, so the warm handoff does not disturb the FPGA. */
    gpio_set(GPIOB_BASE, (1u << 6) | (1u << 11));
    gpio_set(GPIOC_BASE, 1u << 6);

    gpio_config_mask(GPIOB_BASE, (1u << 3) | (1u << 5), 0xBu); // SCK/MOSI AF push-pull
    gpio_config_mask(GPIOB_BASE, 1u << 4, 0x4u);               // MISO floating input
    gpio_config_mask(GPIOB_BASE, (1u << 6) | (1u << 11), 0x1u); // CS, active-mode
    gpio_config_mask(GPIOC_BASE, 1u << 6, 0x1u);               // SPI enable
    gpio_config_mask(GPIOC_BASE, 1u << 0, 0x4u);               // PC0 data-ready input

    /* SPI mode 3, master, software NSS, 8-bit */
    SPI_CTRL1(SPI3_BASE) = 0;
    SPI_CTRL2(SPI3_BASE) = 0;
    SPI_CTRL1(SPI3_BASE) = (1u << 9) | (1u << 8) | (1u << 2) |
                           (((uint32_t)FPGA_SPI_BR & 7u) << 3) |
                           (1u << 1) | (1u << 0);
    SPI_CTRL1(SPI3_BASE) |= 1u << 6; // SPE

    /* Scope trigger comparator reference: stock programs DAC1 (PA4,
     * DHR12R1 @ 0x40007408). The MCU reset during the warm handoff zeroed
     * it, which would leave the FPGA trigger with a 0V reference. Restore
     * a mid-scale level. */
    RCC_APB1ENR |= 1u << 29; // DAC
    gpio_config_mask(GPIOA_BASE, 1u << 4, 0x0); // PA4 analog
    REG32(0x40007400u) |= 1u;                   // DAC_CR: EN1
    REG32(0x40007408u) = 2048u;                 // DHR12R1 mid-scale

    /* Scope-mode SPI3 config writes + 0x03 status read (stock sends these
     * after configuration; reply 00 01 42 2E 2E). The 2026-08-12 run with
     * these enabled saw a dead bus (all-FF), but that run was confounded by
     * a USB replug power cycle that wiped the FPGA SRAM config — so this
     * path is UNTESTED on a live FPGA, not disproven. Default off: keep the
     * baseline warm handoff strictly read-only; enable with
     * FPGA53_SEND_CFG=1 for an A/B experiment. */
#if FPGA53_SEND_CFG
    {
        static const uint8_t cfg[5][2] = {
            {0x01u, 0x08u}, {0x02u, 0x03u}, {0x06u, 0x00u},
            {0x07u, 0x00u}, {0x08u, 0xADu},
        };
        for (uint8_t i = 0; i < 5u; ++i) {
            gpio_clear(GPIOB_BASE, 1u << 6);
            (void)fpga53_xfer(cfg[i][0]);
            (void)fpga53_xfer(cfg[i][1]);
            gpio_set(GPIOB_BASE, 1u << 6);
            delay_ms(1);
        }
        gpio_clear(GPIOB_BASE, 1u << 6);
        fpga53_diag.cst[0] = fpga53_xfer(0x03u);
        fpga53_diag.cst[1] = fpga53_xfer(0xFFu);
        fpga53_diag.cst[2] = fpga53_xfer(0xFFu);
        fpga53_diag.cst[3] = fpga53_xfer(0xFFu);
        fpga53_diag.cst[4] = fpga53_xfer(0xFFu);
        gpio_set(GPIOB_BASE, 1u << 6);
    }
#endif

    fpga53_notready_polls = 0;
    fpga53_force_read = 0;
    fpga_loaded = 1u;
}

uint8_t fpga_ready(void) {
    return fpga_loaded;
}

/* Read-only build: timing / signal-buffer writes are not implemented for
 * the 2C53T yet. The capture rate stays whatever stock configured before
 * the warm handoff, so the UI timebase is cosmetic for now. */
void fpga_write_timing(uint32_t tuning_word, uint32_t span) {
    (void)tuning_word;
    (void)span;
}

void fpga_write_scope_timing(uint32_t span) {
    (void)span;
}

void fpga_write_signal_buffer(const uint8_t *data, uint16_t len) {
    (void)data;
    (void)len;
}

void fpga_capture_latch(void) {
}

uint8_t fpga_capture_ready(void) {
    if (GPIO_IDR(GPIOC_BASE) & 1u) { // PC0 data-ready
        fpga53_notready_polls = 0;
        return 1u;
    }
    /* Diagnostic fallback: if data-ready never rises, force one read every
     * N polls so the screen shows the bus state (flat 0xFF = FPGA silent)
     * instead of waiting forever. */
    if (++fpga53_notready_polls >= FPGA53_FORCED_READ_POLLS) {
        fpga53_notready_polls = 0;
        fpga53_force_read = 1u;
        ++fpga53_diag.forced;
        return 1u;
    }
    return 0;
}

void fpga_capture_ready_irq_handler(void) {
}

static uint32_t fpga53_win_sum[2];

static void fpga53_read_channel(uint8_t opcode) {
    uint8_t rmin = 0xFFu;
    uint8_t rmax = 0;
    uint32_t sum = 0;

    gpio_clear(GPIOB_BASE, 1u << 6); // CS assert
    uint8_t r0 = fpga53_xfer(opcode);
    uint8_t r1 = fpga53_xfer(0xFFu);
    uint8_t r2 = fpga53_xfer(0xFFu);
    for (uint16_t i = 0; i < FPGA53_CH_SAMPLES; ++i) {
        uint8_t raw = fpga53_xfer(0xFFu);
        if (raw < rmin) {
            rmin = raw;
        }
        if (raw > rmax) {
            rmax = raw;
        }
        int16_t cal = (int16_t)raw - (int16_t)FPGA53_ADC_OFFSET;
        if (cal < 0) {
            cal = 0;
        }
        sum = sum * 31u + raw;
        fpga53_ch_buf[i] = (uint8_t)cal;
    }
    gpio_set(GPIOB_BASE, 1u << 6); // CS deassert

    if (opcode == 0x04u) {
        fpga53_diag.r0 = r0;
        fpga53_diag.r1 = r1;
        fpga53_diag.r2 = r2;
        fpga53_diag.smin = rmin;
        fpga53_diag.smax = rmax;
        fpga53_win_sum[0] = sum;
    } else {
        fpga53_diag.smin2 = rmin;
        fpga53_diag.smax2 = rmax;
        fpga53_win_sum[1] = sum;
        fpga53_diag.dup = (fpga53_win_sum[0] == fpga53_win_sum[1]) ? 1u : 0u;
    }
}

uint8_t fpga_capture_read(uint8_t *dst, uint16_t len) {
    if (!dst || !fpga_loaded) {
        return 0;
    }
    fpga53_force_read = 0;
    ++fpga53_diag.reads;

#if FPGA53_PRE_CMD
    /* Stock sends a one-byte pre-acquisition command 0x80|voltage_range in
     * its own CS window before reading (per the stock-firmware RE). Try a
     * mid-range value. */
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)fpga53_xfer(0x83u);
    gpio_set(GPIOB_BASE, 1u << 6);
#endif

    /* 1023 samples per channel, UI expects FPGA_SAMPLE_COUNT (2048)
     * interleaved pairs — stretch 2x (nearest neighbour). */
#if FPGA53_SWAP_ORDER
    fpga53_read_channel(0x05u);
#else
    fpga53_read_channel(0x04u);
#endif
    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        uint16_t src = (uint16_t)(i >> 1);
        if (src >= FPGA53_CH_SAMPLES) {
            src = FPGA53_CH_SAMPLES - 1u;
        }
        fpga53_frame[(uint16_t)(i * 2u)] = fpga53_ch_buf[src];
    }
#if FPGA53_SWAP_ORDER
    fpga53_read_channel(0x04u);
#else
    fpga53_read_channel(0x05u);
#endif
    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        uint16_t src = (uint16_t)(i >> 1);
        if (src >= FPGA53_CH_SAMPLES) {
            src = FPGA53_CH_SAMPLES - 1u;
        }
        fpga53_frame[(uint16_t)(i * 2u + 1u)] = fpga53_ch_buf[src];
    }

    if (len > FPGA_SCOPE_BUFFER_BYTES) {
        len = FPGA_SCOPE_BUFFER_BYTES;
    }
    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = fpga53_frame[(uint16_t)(FPGA_SCOPE_BUFFER_BYTES - len + i)];
    }
    return 1u;
}

uint8_t fpga_capture_read_slow_point(uint8_t sample[2]) {
    (void)sample;
    return 0;
}

#else /* !HW_TARGET_2C53T */

enum {
    FPGA_LATCH_SETTLE_MS = 1,
    SPI_CAPTURE_TIMEOUT = 60000u,

    SPI_STS_RXNE = 1u << 0,
    SPI_STS_TXE = 1u << 1,
    SPI_STS_BSY = 1u << 7,
};

#ifndef FPGA53_SWAP_ORDER
#define FPGA53_SWAP_ORDER 0
#endif

#ifndef FPGA53_PRE_CMD
#define FPGA53_PRE_CMD 0
#endif

#ifndef FPGA53_SEND_CFG
#define FPGA53_SEND_CFG 0
#endif

#ifndef FPGA_SPI_BR
#define FPGA_SPI_BR 2u
#endif

static uint8_t fpga_ready_flag;
static uint8_t fpga_loaded;
static uint32_t fpga_last_tuning_word;
static uint32_t fpga_last_span;

static void fpga_start_clock_output(void) {
#if HW_TARGET_HW40
    // Stock 2.1.0 leaves the bootloader-provided CLKOUT/MCO selection intact
    // and only programs this AT32-specific CFGR2 field during board init.
    RCC_CFGR2 = (RCC_CFGR2 & ~(7u << 16)) | (2u << 16);
#else
    RCC_CFGR = (RCC_CFGR & ~(7u << 24)) | (4u << 24);
    RCC_CFGR2 = (RCC_CFGR2 & ~((1u << 16) | (15u << 28))) | (11u << 28);
#endif
}

#if HW_TARGET_HW40

enum {
    FPGA_HW4_BUS_MASK = 0x00FFu,
    FPGA_HW4_CLK_PIN = 1u << 3,   // PB3
    FPGA_HW4_MISO_PIN = 1u << 4,  // PB4, also capture-ready in runtime mode
    FPGA_HW4_MOSI_PIN = 1u << 5,  // PB5
    FPGA_HW4_SELECT_PIN = 1u << 15, // PA15
    FPGA_HW4_RESET_PIN = 1u << 8, // PC8
    FPGA_HW4_AUX_PIN = 1u << 10,  // PC10, initialized by stock 2.1.0 board setup
    FPGA_HW4_READY_POLL_CYCLES = 4000u,
};

static volatile uint8_t fpga_hw4_capture_ready_seen;

static void fpga_hw4_short_delay(void) {
    for (volatile uint32_t i = 0; i < 80u; ++i) {
        __asm__ volatile("nop");
    }
}

static void fpga_hw4_bus_input(void) {
    gpio_config_mask(GPIOC_BASE, FPGA_HW4_BUS_MASK, 0x4u);
}

static void fpga_hw4_bus_output(void) {
    gpio_config_mask(GPIOC_BASE, FPGA_HW4_BUS_MASK, 0x1u);
}

static void fpga_hw4_bus_write(uint8_t value) {
    GPIO_ODR(GPIOC_BASE) = (GPIO_ODR(GPIOC_BASE) & ~FPGA_HW4_BUS_MASK) | value;
}

static uint8_t fpga_hw4_bitbang_transfer_byte(uint8_t value) {
    uint8_t result = 0;

    for (uint8_t i = 0; i < 8u; ++i) {
        gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN);
        if (value & 0x80u) {
            gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
        } else {
            gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
        }
        gpio_set(GPIOB_BASE, FPGA_HW4_CLK_PIN);
        result = (uint8_t)(result << 1);
        value = (uint8_t)(value << 1);
        if (gpio_read(GPIOB_BASE, FPGA_HW4_MISO_PIN)) {
            result |= 1u;
        }
    }
    return result;
}

static uint32_t fpga_hw4_read_register32(uint32_t addr) {
    uint8_t bytes[4];

    (void)fpga_hw4_bitbang_transfer_byte(0);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(addr >> 24));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(addr >> 16));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(addr >> 8));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)addr);
    for (uint8_t i = 0; i < 4u; ++i) {
        bytes[i] = fpga_hw4_bitbang_transfer_byte(0);
    }
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);

    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

static void fpga_hw4_write_command_word(uint16_t command) {
    (void)fpga_hw4_bitbang_transfer_byte(0);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)(command >> 8));
    (void)fpga_hw4_bitbang_transfer_byte((uint8_t)command);
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
}

static void fpga_hw4_parallel_write_byte(uint8_t value) {
    fpga_hw4_bus_write(value);
    gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
    gpio_set(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
}

static uint8_t fpga_hw4_parallel_read_byte(void) {
    gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
    gpio_set(GPIOB_BASE, FPGA_HW4_CLK_PIN);
    fpga_hw4_short_delay();
    return (uint8_t)(GPIO_IDR(GPIOC_BASE) & FPGA_HW4_BUS_MASK);
}

static void fpga_hw4_post_config_flush(void) {
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    fpga_hw4_bus_input();
    for (uint8_t i = 0; i < 16u; ++i) {
        (void)fpga_hw4_parallel_read_byte();
    }
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

static void fpga_hw4_scope_read_state(void) {
    fpga_hw4_bus_input();
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

static void fpga_hw4_write_neutral_signal_buffer(void) {
    gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    fpga_hw4_bus_output();
    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        fpga_hw4_parallel_write_byte(0);
    }
    fpga_hw4_bus_input();
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

static void fpga_hw4_write_timing_frame(uint32_t tuning_word, uint32_t span) {
    gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    fpga_hw4_bus_output();

    fpga_hw4_parallel_write_byte((uint8_t)(tuning_word >> 24));
    fpga_hw4_parallel_write_byte((uint8_t)(tuning_word >> 16));
    fpga_hw4_parallel_write_byte((uint8_t)(tuning_word >> 8));
    fpga_hw4_parallel_write_byte((uint8_t)tuning_word);
    fpga_hw4_parallel_write_byte((uint8_t)(span >> 24));
    fpga_hw4_parallel_write_byte((uint8_t)(span >> 16));
    fpga_hw4_parallel_write_byte((uint8_t)(span >> 8));
    fpga_hw4_parallel_write_byte((uint8_t)span);

    for (uint8_t i = 0; i < 8u; ++i) {
        fpga_hw4_parallel_write_byte(i);
    }

    fpga_hw4_bus_input();
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

static void fpga_hw4_begin(void) {
    RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4);
    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24); // release PB3/PB4/PB5 from JTAG

    gpio_config_mask(GPIOB_BASE, FPGA_HW4_CLK_PIN | FPGA_HW4_MOSI_PIN, 0x1u);
    gpio_config_mask(GPIOB_BASE, FPGA_HW4_MISO_PIN, 0x4u);
    gpio_config_mask(GPIOA_BASE, FPGA_HW4_SELECT_PIN, 0x1u);
    gpio_config_mask(GPIOA_BASE, 1u << 8, 0x9u);
    gpio_config_mask(GPIOC_BASE, FPGA_HW4_RESET_PIN | FPGA_HW4_AUX_PIN, 0x1u);
    fpga_hw4_bus_input();

    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_CLK_PIN | FPGA_HW4_MOSI_PIN);
    gpio_clear(GPIOC_BASE, FPGA_HW4_AUX_PIN);
    fpga_start_clock_output();
}

void fpga_init_once(void) {
    uint32_t status;

    if (fpga_loaded) {
        return;
    }

    fpga_hw4_begin();
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOC_BASE, FPGA_HW4_RESET_PIN);
    delay_ms(10);
    gpio_set(GPIOC_BASE, FPGA_HW4_RESET_PIN);
    delay_ms(1);

    (void)fpga_hw4_read_register32(0x11000000u);
    (void)fpga_hw4_read_register32(0x13000000u);
    (void)fpga_hw4_read_register32(0x41000000u);
    fpga_hw4_write_command_word(0x1200u);
    fpga_hw4_write_command_word(0x1500u);
    (void)fpga_hw4_bitbang_transfer_byte(0);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    (void)fpga_hw4_bitbang_transfer_byte(0x3Bu);
    for (uint32_t i = 0; i < fpga_bitstream_len; ++i) {
        (void)fpga_hw4_bitbang_transfer_byte(fpga_bitstream[i]);
    }
    gpio_set(GPIOA_BASE, FPGA_HW4_SELECT_PIN);

    status = fpga_hw4_read_register32(0x41000000u);
    fpga_hw4_write_command_word(0x3A00u);
    delay_ms(100);
    fpga_hw4_write_timing_frame(0, FPGA_SAMPLE_COUNT);
    fpga_hw4_write_neutral_signal_buffer();
    fpga_hw4_post_config_flush();
    fpga_hw4_capture_ready_seen = 0;
    fpga_ready_flag = status != 0xFFFFFFFFu ? 1u : 0u;
    fpga_loaded = 1u;
}

uint8_t fpga_ready(void) {
    return fpga_ready_flag;
}

void fpga_write_timing(uint32_t tuning_word, uint32_t span) {
    fpga_last_tuning_word = tuning_word;
    fpga_last_span = span;
    fpga_hw4_write_timing_frame(tuning_word, span);
}

void fpga_write_scope_timing(uint32_t span) {
    fpga_last_span = span;
    fpga_hw4_write_timing_frame(fpga_last_tuning_word, span);
}

void fpga_write_signal_buffer(const uint8_t *data, uint16_t len) {
    gpio_set(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    fpga_hw4_bus_output();
    for (uint16_t i = 0; i < len; ++i) {
        fpga_hw4_parallel_write_byte(data[i]);
    }
    fpga_hw4_bus_input();
    gpio_clear(GPIOA_BASE, FPGA_HW4_SELECT_PIN);
    gpio_clear(GPIOB_BASE, FPGA_HW4_MOSI_PIN);
}

void fpga_capture_latch(void) {
    fpga_hw4_capture_ready_seen = 0;
    fpga_hw4_write_timing_frame(fpga_last_tuning_word, fpga_last_span);
    fpga_hw4_scope_read_state();
}

uint8_t fpga_capture_ready(void) {
    for (uint32_t i = 0; i < FPGA_HW4_READY_POLL_CYCLES; ++i) {
        if (gpio_read(GPIOB_BASE, FPGA_HW4_MISO_PIN)) {
            fpga_hw4_capture_ready_seen = 0;
            return 1u;
        }
    }
    return 0;
}

void fpga_capture_ready_irq_handler(void) {
    if (EXTI_PR & FPGA_HW4_MISO_PIN) {
        EXTI_PR = FPGA_HW4_MISO_PIN;
        fpga_hw4_capture_ready_seen = 1;
    }
}

uint8_t fpga_capture_read(uint8_t *dst, uint16_t len) {
    fpga_hw4_scope_read_state();
    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = fpga_hw4_parallel_read_byte();
    }
    return 1u;
}

uint8_t fpga_capture_read_slow_point(uint8_t sample[2]) {
    if (!sample) {
        return 0;
    }

    fpga_hw4_scope_read_state();
    sample[0] = fpga_hw4_parallel_read_byte();
    sample[1] = fpga_hw4_parallel_read_byte();
    fpga_capture_latch();
    return 1u;
}

#else

void fpga_capture_ready_irq_handler(void) {
}

static void spi3_write_raw(uint8_t value) {
    while (!(SPI_STS(SPI3_BASE) & SPI_STS_TXE)) {
    }
    SPI_DT(SPI3_BASE) = value;
    while (SPI_STS(SPI3_BASE) & SPI_STS_BSY) {
    }
    (void)SPI_DT(SPI3_BASE);
}

static uint8_t spi3_transfer_raw_timeout(uint8_t value, uint8_t *out) {
    uint32_t timeout = SPI_CAPTURE_TIMEOUT;

    while (!(SPI_STS(SPI3_BASE) & SPI_STS_TXE)) {
        if (!--timeout) {
            return 0;
        }
    }
    SPI_DT(SPI3_BASE) = value;

    timeout = SPI_CAPTURE_TIMEOUT;
    while (SPI_STS(SPI3_BASE) & SPI_STS_BSY) {
        if (!--timeout) {
            return 0;
        }
    }

    timeout = SPI_CAPTURE_TIMEOUT;
    while (!(SPI_STS(SPI3_BASE) & SPI_STS_RXNE)) {
        if (!--timeout) {
            return 0;
        }
    }
    *out = (uint8_t)SPI_DT(SPI3_BASE);
    return 1;
}

static void spi3_write_framed(uint8_t value) {
    gpio_clear(GPIOA_BASE, 1u << 15);
    spi3_write_raw(value);
    gpio_set(GPIOA_BASE, 1u << 15);
}

static void fpga_spi_begin(void) {
    RCC_APB1ENR |= 1u << 15; // SPI3

    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24); // release PB3/PB4/PB5 from JTAG

    gpio_config_mask(GPIOC_BASE, (1u << 0) | (1u << 1) | (1u << 2) | (1u << 9) | (1u << 10), 0x1);
    gpio_config_mask(GPIOA_BASE, 1u << 15, 0x1);
    gpio_config_mask(GPIOB_BASE, (1u << 3) | (1u << 5), 0x9u);
    gpio_config_mask(GPIOB_BASE, 1u << 4, 0x4);
    gpio_config_mask(GPIOA_BASE, 1u << 8, 0x9u);
    gpio_config_mask(GPIOC_BASE, 1u << 3, 0x4);
    gpio_config_mask(GPIOC_BASE, 1u << 8, 0x8);
    gpio_set(GPIOC_BASE, 1u << 8);

    gpio_set(GPIOA_BASE, 1u << 15);
    gpio_clear(GPIOC_BASE, (1u << 0) | (1u << 1) | (1u << 2));
    fpga_start_clock_output();

    SPI_CTRL1(SPI3_BASE) = 0;
    SPI_CTRL2(SPI3_BASE) = 0;
    SPI_CTRL1(SPI3_BASE) = (1u << 9) | (1u << 8) | (1u << 2) |
                           (((uint32_t)FPGA_SPI_BR & 7u) << 3) |
                           (1u << 1) | (1u << 0);
    SPI_CTRL1(SPI3_BASE) |= 1u << 6;
}

static void fpga_write_strobe(void) {
    gpio_set(GPIOC_BASE, 1u << 2);
    spi3_write_framed(0);
    gpio_clear(GPIOC_BASE, 1u << 2);
}

void fpga_init_once(void) {
    if (fpga_loaded) {
        return;
    }

    fpga_spi_begin();

    gpio_clear(GPIOA_BASE, 1u << 15);
    gpio_clear(GPIOC_BASE, 1u << 9);
    delay_ms(10);
    gpio_set(GPIOC_BASE, 1u << 9);
    delay_ms(1);

    for (uint32_t i = 0; i < fpga_bitstream_len; ++i) {
        spi3_write_raw(fpga_bitstream[i]);
    }
    for (uint16_t i = 0; i < 200u; ++i) {
        spi3_write_raw(0);
    }
    fpga_ready_flag = gpio_read(GPIOC_BASE, 1u << 8) ? 1u : 0u;
    gpio_set(GPIOA_BASE, 1u << 15);

    gpio_clear(GPIOC_BASE, 1u << 2);
    gpio_set(GPIOC_BASE, 1u << 10);
    delay_ms(1);
    gpio_clear(GPIOC_BASE, 1u << 0);
    gpio_clear(GPIOC_BASE, 1u << 10);
    delay_ms(1);
    gpio_set(GPIOC_BASE, 1u << 10);
    delay_ms(1);

    if (gpio_read(GPIOC_BASE, 1u << 8)) {
        fpga_ready_flag = 1u;
    }
    fpga_loaded = 1u;
}

uint8_t fpga_ready(void) {
    return fpga_ready_flag;
}

void fpga_write_timing(uint32_t tuning_word, uint32_t span) {
    fpga_last_tuning_word = tuning_word;
    fpga_last_span = span;
    fpga_write_strobe();
    gpio_set(GPIOC_BASE, 1u << 0);
    gpio_clear(GPIOC_BASE, 1u << 1);

    spi3_write_framed((uint8_t)(tuning_word >> 24));
    spi3_write_framed((uint8_t)(tuning_word >> 16));
    spi3_write_framed((uint8_t)(tuning_word >> 8));
    spi3_write_framed((uint8_t)tuning_word);
    spi3_write_framed((uint8_t)(span >> 24));
    spi3_write_framed((uint8_t)(span >> 16));
    spi3_write_framed((uint8_t)(span >> 8));
    spi3_write_framed((uint8_t)span);

    gpio_clear(GPIOC_BASE, 1u << 0);
}

void fpga_write_scope_timing(uint32_t span) {
    fpga_write_timing(fpga_last_tuning_word, span);
}

void fpga_write_signal_buffer(const uint8_t *data, uint16_t len) {
    fpga_write_strobe();
    gpio_set(GPIOC_BASE, (1u << 0) | (1u << 1));
    for (uint16_t i = 0; i < len; ++i) {
        spi3_write_framed(data[i]);
    }
    gpio_clear(GPIOC_BASE, 1u << 0);
}

void fpga_capture_latch(void) {
    gpio_set(GPIOC_BASE, 1u << 0);
    gpio_set(GPIOC_BASE, 1u << 2);
    delay_ms(FPGA_LATCH_SETTLE_MS);
    gpio_clear(GPIOC_BASE, 1u << 2);
    gpio_clear(GPIOC_BASE, 1u << 0);
}

uint8_t fpga_capture_ready(void) {
    return gpio_read(GPIOC_BASE, 1u << 3) ? 1u : 0u;
}

uint8_t fpga_capture_read(uint8_t *dst, uint16_t len) {
    gpio_clear(GPIOC_BASE, 1u << 0);
    fpga_write_strobe();
    (void)SPI_DT(SPI3_BASE);
    gpio_set(GPIOC_BASE, 1u << 1);

    for (uint16_t i = 0; i < len; ++i) {
        gpio_clear(GPIOA_BASE, 1u << 15);
        if (!spi3_transfer_raw_timeout(0, &dst[i])) {
            gpio_set(GPIOA_BASE, 1u << 15);
            gpio_clear(GPIOC_BASE, 1u << 1);
            return 0;
        }
        gpio_set(GPIOA_BASE, 1u << 15);
    }
    gpio_clear(GPIOC_BASE, 1u << 1);
    return 1;
}

uint8_t fpga_capture_read_slow_point(uint8_t sample[2]) {
    if (!sample) {
        return 0;
    }

    gpio_clear(GPIOC_BASE, 1u << 0);
    fpga_write_strobe();
    (void)SPI_DT(SPI3_BASE);
    gpio_set(GPIOC_BASE, 1u << 1);

    for (uint8_t i = 0; i < 2u; ++i) {
        gpio_clear(GPIOA_BASE, 1u << 15);
        if (!spi3_transfer_raw_timeout(0, &sample[i])) {
            gpio_set(GPIOA_BASE, 1u << 15);
            gpio_clear(GPIOC_BASE, 1u << 1);
            return 0;
        }
        gpio_set(GPIOA_BASE, 1u << 15);
    }

    fpga_capture_latch();
    return 1;
}

#endif

#endif /* !HW_TARGET_2C53T */
