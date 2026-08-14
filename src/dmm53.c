/* 2C53T multimeter backend.
 *
 * The 2C53T's DMM is a separate SoC (SD7501 per the issue-#18 PCB trace)
 * behind a pi122U31 digital isolator on USART2 (MCU PA2=TX, PA3=RX, 9600
 * 8N1) — not the FPGA, and not the 2C23T's USART3 meter. Protocol per the
 * upstream RE (usart2_isr_state_machine.md) and their working driver:
 *   TX: 10-byte frames [0][0][cmd_hi][cmd_lo][0..0][(hi+lo)&0xFF]
 *   RX: 12-byte data frames (hdr 5A A5, BCD digits + status in [6])
 *       10-byte echo frames (hdr AA 55, [3]=echoed cmd)
 * Data frames only flow while TX commands keep arriving — poll (0x00,0x09)
 * at ~4 Hz. The NV FPGA image services this path, so the meter works with
 * no FPGA configuration at all.
 *
 * Stage 1 scope (EXPERIMENT-LOG / METER-PLAN): transport + wake preamble +
 * poll + raw on-screen telemetry through the standard dmm.h API. Value
 * decode (BCD + band overrides) is stage 3.
 */

#include "dmm.h"

#include "board.h"
#include "fw_update.h"
#include "hw.h"
#include "meter_data.h"
#include "usb_msc.h"
#include "w25q.h"

#include <stdint.h>

#if HW_TARGET_2C53T

#ifndef USART2_BASE
#define USART2_BASE 0x40004400u
#endif

enum {
    USART_STS_FE = 1u << 1,
    USART_STS_NE = 1u << 2,
    USART_STS_ORE = 1u << 3,
    USART_STS_RXNE = 1u << 5,
    USART_STS_TXE = 1u << 7,
    USART_STS_TC = 1u << 6,

    USART_CTRL1_RE = 1u << 2,
    USART_CTRL1_TE = 1u << 3,
    USART_CTRL1_RXNEIE = 1u << 5,
    USART_CTRL1_UE = 1u << 13,

    DMM53_TX_LEN = 10,
    DMM53_DATA_LEN = 12,
    DMM53_ECHO_LEN = 10,

    DMM53_POLL_MS = 250,      /* ~4 Hz, matches upstream meter_poll */
    DMM53_BAUD_RETRY_MS = 700, /* per-candidate window (wake takes ~60ms) */
    DMM53_WAKE_STEP_MS = 10,
};

/* PCLK1 candidates: the port never reprograms the PLL, so the effective
 * APB1 clock depends on what the factory bootloader left behind. Cycle
 * until frames sync, same strategy as the 2C23T dmm.c auto-baud. */
static const uint32_t dmm53_pclk[] = {
    36000000u, 60000000u, 72000000u, 96000000u, 108000000u, 120000000u,
};
enum { DMM53_PCLK_COUNT = sizeof(dmm53_pclk) / sizeof(dmm53_pclk[0]) };

#ifndef DMM_UART_BAUD
#define DMM_UART_BAUD 9600u
#endif

static uint8_t dmm53_started;
static uint8_t dmm53_powered;
static uint8_t dmm53_mode;
static uint8_t dmm53_baud_index;
static uint8_t dmm53_baud_locked;
static uint16_t dmm53_retry_ms;
static uint16_t dmm53_poll_timer_ms;
static uint8_t dmm53_wake_step;
static uint16_t dmm53_wake_timer_ms;

/* RX ISR state */
static volatile uint8_t rx_buf[DMM53_DATA_LEN];
static volatile uint8_t rx_index;
static volatile uint8_t rx_data_frame[DMM53_DATA_LEN];
static volatile uint8_t rx_data_ready;
static volatile uint8_t rx_echo_cmd;
static volatile uint16_t rx_byte_count;
static volatile uint16_t rx_data_count;
static volatile uint16_t rx_echo_count;
static volatile uint16_t rx_err_count;

static uint8_t dmm53_frame[DMM53_DATA_LEN]; /* last data frame, main-loop copy */
static uint8_t dmm53_frame_valid;

static char dmm53_value[28];
static char dmm53_unit[8];
static char dmm53_status[40];

static const char hex_digits[] = "0123456789ABCDEF";

static uint8_t u16_to_dec(char *out, uint16_t v) {
    char tmp[5];
    uint8_t n = 0;
    uint8_t i;
    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && n < 5u);
    for (i = 0; i < n; ++i) {
        out[i] = tmp[n - 1u - i];
    }
    return n;
}

static uint32_t dmm53_brr(void) {
    uint32_t pclk = dmm53_pclk[dmm53_baud_index];
    return (pclk + DMM_UART_BAUD / 2u) / DMM_UART_BAUD;
}

static void dmm53_uart_apply(void) {
    USART_CTRL1(USART2_BASE) = 0;
    USART_CTRL2(USART2_BASE) = 0;
    USART_CTRL3(USART2_BASE) = 0;
    USART_BAUDR(USART2_BASE) = dmm53_brr();
    (void)USART_STS(USART2_BASE);
    (void)USART_DT(USART2_BASE);
    rx_index = 0;
    USART_CTRL1(USART2_BASE) =
        USART_CTRL1_UE | USART_CTRL1_RXNEIE | USART_CTRL1_TE | USART_CTRL1_RE;
}

static uint8_t uart_wait_txe(void) {
    uint32_t guard = 200000u;
    while (!(USART_STS(USART2_BASE) & USART_STS_TXE)) {
        if (!--guard) {
            return 0;
        }
    }
    return 1u;
}

static uint16_t dmm53_tx_count;

/* Blocking 10-byte command send (~10.4ms at 9600) — called from tick, not ISR. */
static void dmm53_send_cmd(uint8_t cmd_hi, uint8_t cmd_lo) {
    uint8_t frame[DMM53_TX_LEN] = {0};
    frame[2] = cmd_hi;
    frame[3] = cmd_lo;
    frame[9] = (uint8_t)(cmd_hi + cmd_lo);
    ++dmm53_tx_count;
    for (uint8_t i = 0; i < DMM53_TX_LEN; ++i) {
        if (!uart_wait_txe()) {
            return;
        }
        USART_DT(USART2_BASE) = frame[i];
    }
}

static uint8_t dmm53_probe_cmd(void) {
    /* PC7 = probe-detect strap (stock reads it): HIGH -> 0x07, else 0x0A. */
    return (GPIO_IDR(GPIOC_BASE) & (1u << 7)) ? 0x07u : 0x0Au;
}

static void dmm53_send_mode_sequence(void);

/* Meter analog posture, mirrored from upstream fpga_set_meter_frontend_baseline:
 * PB11/PC6 kept HIGH, PC11 = meter MUX on, PC12 = probe routed to meter,
 * PE4 H / PE5 L / PE6 H relays, PB9/PA6/PA15/PA10 H + PB10 L gain keys. */
static void dmm53_frontend_baseline(void) {
    gpio_set(GPIOB_BASE, (1u << 11) | (1u << 9));
    gpio_set(GPIOC_BASE, (1u << 6) | (1u << 11) | (1u << 12));
    gpio_set(GPIOE_BASE, (1u << 4) | (1u << 6));
    gpio_clear(GPIOE_BASE, 1u << 5);
    gpio_set(GPIOA_BASE, (1u << 6) | (1u << 15) | (1u << 10));
    gpio_clear(GPIOB_BASE, 1u << 10);

    gpio_config_mask(GPIOB_BASE, (1u << 11) | (1u << 9) | (1u << 10), 0x1u);
    gpio_config_mask(GPIOC_BASE, (1u << 6) | (1u << 11) | (1u << 12), 0x1u);
    gpio_config_mask(GPIOE_BASE, (1u << 4) | (1u << 5) | (1u << 6), 0x1u);
    gpio_config_mask(GPIOA_BASE, (1u << 6) | (1u << 15) | (1u << 10), 0x1u);
}

/* Wake preamble, mirrored from upstream fpga_send_meter_wake_preamble:
 * (0x05,0x08) -> (0x05,0x09 START) -> (0x05,probe) -> (0x05,0x14 VAR),
 * 10-20ms apart. Driven as steps from dmm_tick. */
static void dmm53_wake_send_step(uint8_t step) {
    switch (step) {
    case 0:
        dmm53_send_cmd(0x05u, 0x08u);
        break;
    case 1:
        dmm53_send_cmd(0x05u, 0x09u);
        break;
    case 2:
        dmm53_send_cmd(0x05u, dmm53_probe_cmd());
        break;
    case 3:
        dmm53_send_cmd(0x05u, 0x14u);
        break;
    case 4:
        /* Wake done — switch the SoC into the selected meter mode. */
        dmm53_send_mode_sequence();
        break;
    default:
        break;
    }
}

static uint8_t dmm53_submode(void);

/* Mode-specific command sequence, mirrored from upstream
 * fpga_send_meter_mode_sequence (RE: mode init dispatcher FUN_0800b908).
 * Blocking sends (~10ms each) provide the stock-like pacing. */
static void dmm53_send_mode_sequence(void) { /* fwd-declared above */
    uint8_t submode = dmm53_submode();
    dmm53_send_cmd(0x00u, 0x00u); /* RESET */
    switch (submode) {
    case 5: /* Frequency */
        dmm53_send_cmd(0x00u, 0x1Fu);
        dmm53_send_cmd(0x00u, 0x09u);
        dmm53_send_cmd(0x00u, 0x20u);
        dmm53_send_cmd(0x00u, 0x21u);
        break;
    case 6: /* Resistance */
        dmm53_send_cmd(0x00u, 0x12u);
        dmm53_send_cmd(0x00u, 0x13u);
        dmm53_send_cmd(0x00u, 0x14u);
        dmm53_send_cmd(0x00u, 0x09u);
        dmm53_send_cmd(0x00u, dmm53_probe_cmd());
        break;
    case 7: /* Continuity */
    case 8: /* Diode */
        dmm53_send_cmd(0x00u, 0x2Cu);
        break;
    case 9: /* Capacitance */
        dmm53_send_cmd(0x00u, 0x08u);
        dmm53_send_cmd(0x00u, 0x09u);
        dmm53_send_cmd(0x00u, dmm53_probe_cmd());
        dmm53_send_cmd(0x00u, 0x16u);
        dmm53_send_cmd(0x00u, 0x17u);
        dmm53_send_cmd(0x00u, 0x18u);
        dmm53_send_cmd(0x00u, 0x19u);
        break;
    default: /* DCV/ACV/DCA/ACA: basic meter */
        dmm53_send_cmd(0x00u, 0x09u);
        dmm53_send_cmd(0x00u, dmm53_probe_cmd());
        dmm53_send_cmd(0x00u, 0x1Au);
        dmm53_send_cmd(0x00u, 0x1Bu);
        dmm53_send_cmd(0x00u, 0x1Cu);
        dmm53_send_cmd(0x00u, 0x1Du);
        dmm53_send_cmd(0x00u, 0x1Eu);
        break;
    }
}

static void dmm53_restart_wake(void) {
    dmm53_wake_step = 0;
    dmm53_wake_timer_ms = 0;
    dmm53_poll_timer_ms = 0;
    /* Per-mode f6 capture: each mode's band bytes shouldn't be diluted
     * by the previous mode's rotation. */
    meter_f6_history_count = 0;
}

static void dmm53_format_status(void) {
    /* "B<idx> R<bytes> D<data> E<echo> X<err>" */
    char *p = dmm53_status;
    *p++ = 'B';
    *p++ = (char)('0' + dmm53_baud_index);
    *p++ = dmm53_baud_locked ? '!' : '?';
    *p++ = ' ';
    *p++ = 'R';
    p += u16_to_dec(p, rx_byte_count);
    *p++ = ' ';
    *p++ = 'D';
    p += u16_to_dec(p, rx_data_count);
    *p++ = ' ';
    *p++ = 'E';
    p += u16_to_dec(p, rx_echo_count);
    *p++ = ' ';
    *p++ = 'X';
    p += u16_to_dec(p, rx_err_count);
    *p = '\0';
}

static void dmm53_format_value(void) {
    /* Raw telemetry: data frame bytes [2..8] as hex, [6] (band/status)
     * separated: "xxxxxxxx.bb.xxxx" style kept simple: 7 bytes hex. */
    char *p = dmm53_value;
    for (uint8_t i = 2; i < 9u; ++i) {
        uint8_t b = dmm53_frame[i];
        *p++ = hex_digits[b >> 4];
        *p++ = hex_digits[b & 0xFu];
        if (i == 5u || i == 6u) {
            *p++ = '.';
        }
    }
    *p = '\0';
}

void dmm_init(void) {
    /* Boot-time setup only: UART + NVIC. The analog posture (relays, meter
     * MUX) is applied on meter-mode entry (dmm_reenter) so a boot into
     * scope mode leaves the warm-handoff relay state untouched. */

    /* Free PA15 (JTDI) for the gain key: SWJ_CFG=010 keeps SWD only.
     * Same write fpga.c does; idempotent. */
    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24);

    /* PC7 probe strap: floating input. */
    gpio_config_mask(GPIOC_BASE, 1u << 7, 0x4u);

    /* USART2 on PA2 (AF push-pull) / PA3 (floating input). */
    RCC_APB1ENR |= 1u << 17; // USART2
    gpio_config_mask(GPIOA_BASE, 1u << 2, 0xBu);
    gpio_config_mask(GPIOA_BASE, 1u << 3, 0x4u);

    dmm53_baud_index = 0;
    dmm53_baud_locked = 0;
    dmm53_retry_ms = 0;
    dmm53_uart_apply();
    REG32(NVIC_ISER1) = 1u << 6; // IRQ38, USART2 global

    dmm53_value[0] = '\0';
    dmm53_unit[0] = 'R';
    dmm53_unit[1] = 'A';
    dmm53_unit[2] = 'W';
    dmm53_unit[3] = '\0';
    dmm53_format_status();

    meter_data_init();
    dmm53_started = 1u;
    dmm53_powered = 0; /* silent until meter mode is entered */
}

void dmm_pause(void) {
    dmm53_powered = 0;
    /* Meter MUX off; the other relays keep their last state (stage 1 —
     * scope re-entry re-applies its own settings). */
    gpio_clear(GPIOC_BASE, 1u << 11);
}

void dmm_hw4_set_mode_gate(uint8_t active) {
    (void)active;
}

void dmm_reenter(uint8_t mode_index) {
    dmm53_mode = mode_index;
    dmm53_powered = 1u;
    dmm53_frontend_baseline();
    dmm53_restart_wake();
}

void dmm_set_mode(uint8_t mode_index) {
    /* A mode set that isn't a mode *change* must not tear the meter down.
     * The UI calls this on every AUTO press and on every apply-selected-mode,
     * including when the selection is already active; each call used to
     * restart the wake preamble and replay the mode sequence, so the SoC never
     * got the ~2.5 s it needs to settle and the reading kept jumping — which
     * is what our auto-range was chasing. Upstream hit the same thing from the
     * other side and stopped reconfiguring the DMM mode on poll (komzpa,
     * 801e2dd in PR #13).
     *
     * Only skip once the meter is powered and past the wake preamble: an
     * unpowered or still-waking meter has nothing settled to protect, and
     * dmm_reenter() stays the unconditional entry point for meter-mode entry. */
    if (dmm53_powered && dmm53_wake_step >= 5u && mode_index == dmm53_mode) {
        return;
    }

    dmm53_mode = mode_index;
    dmm53_restart_wake();
}

/* Port UI meter modes (ui.c dmm_mode_names, 0-12) → upstream decode
 * submodes (meter_data.c, 0-9). AUTO/LIVE/TEMP fall back to DCV. */
static uint8_t dmm53_submode(void) {
    static const uint8_t map[13] = {
        0u, /* AUTO DETECT  -> DCV */
        0u, /* DC VOLTAGE   -> DCV */
        1u, /* AC VOLTAGE   -> ACV */
        6u, /* RESISTANCE   -> Ohm */
        8u, /* DIODE        -> Diode */
        9u, /* CAPACITANCE  -> Cap */
        0u, /* LIVE WIRE    -> DCV */
        7u, /* CONTINUITY   -> Cont */
        0u, /* TEMPERATURE  -> DCV (no upstream submode) */
        5u, /* AC HIGH CURR -> ACA(A) */
        3u, /* DC HIGH CURR -> DCA(A) */
        4u, /* AC LOW CURR  -> ACA(mA) */
        2u, /* DC LOW CURR  -> DCA(mA) */
    };
    return dmm53_mode < 13u ? map[dmm53_mode] : 0u;
}

uint8_t dmm_poll(void) {
    if (!dmm53_powered || !rx_data_ready) {
        return 0;
    }
    __asm__ volatile("cpsid i" ::: "memory");
    for (uint8_t i = 0; i < DMM53_DATA_LEN; ++i) {
        dmm53_frame[i] = rx_data_frame[i];
    }
    rx_data_ready = 0;
    __asm__ volatile("cpsie i" ::: "memory");
    dmm53_frame_valid = 1u;
    dmm53_baud_locked = 1u; /* real framed traffic = correct baud */
    meter_data_process_frame(dmm53_frame, dmm53_submode());
    dmm53_format_value();
    dmm53_format_status();
    return 1u;
}

void dmm_tick(uint32_t elapsed_ms) {
    if (!dmm53_started || !dmm53_powered) {
        return;
    }

    /* Wake preamble steps (10/10/10/20ms apart) + mode sequence. */
    if (dmm53_wake_step < 5u) {
        dmm53_wake_timer_ms = (uint16_t)(dmm53_wake_timer_ms + elapsed_ms);
        uint16_t need = (dmm53_wake_step == 0u)
                            ? 0u
                            : (dmm53_wake_step >= 3u ? 20u : DMM53_WAKE_STEP_MS);
        if (dmm53_wake_timer_ms >= need) {
            dmm53_wake_send_step(dmm53_wake_step);
            ++dmm53_wake_step;
            dmm53_wake_timer_ms = 0;
        }
        dmm53_format_status();
        return;
    }

    /* Steady state: poll (0x00, 0x09) at ~4 Hz. */
    dmm53_poll_timer_ms = (uint16_t)(dmm53_poll_timer_ms + elapsed_ms);
    if (dmm53_poll_timer_ms >= DMM53_POLL_MS) {
        dmm53_poll_timer_ms = 0;
        dmm53_send_cmd(0x00u, 0x09u);
    }

    /* Auto-baud: cycle PCLK candidates until framed traffic arrives. */
    if (!dmm53_baud_locked) {
        if (rx_data_count || rx_echo_count) {
            dmm53_baud_locked = 1u;
        } else {
            dmm53_retry_ms = (uint16_t)(dmm53_retry_ms + elapsed_ms);
            if (dmm53_retry_ms >= DMM53_BAUD_RETRY_MS) {
                dmm53_retry_ms = 0;
                dmm53_baud_index =
                    (uint8_t)((dmm53_baud_index + 1u) % DMM53_PCLK_COUNT);
                dmm53_uart_apply();
                dmm53_restart_wake();
            }
        }
    }
    dmm53_format_status();
}

uint8_t dmm_has_reading(void) {
    return dmm53_frame_valid && meter_reading.valid;
}

uint8_t dmm_value_is_numeric(void) {
    return meter_reading.valid &&
           meter_reading.result_class == METER_RESULT_NORMAL;
}

int32_t dmm_value_milli_units(void) {
    float v = meter_reading.value * 1000.0f;
    return (int32_t)(v < 0 ? v - 0.5f : v + 0.5f);
}

const char *dmm_value_text(void) {
    if (meter_reading.valid) {
        return meter_reading.display_str;
    }
    return dmm53_frame_valid ? dmm53_value : "----";
}

const char *dmm_unit_text(void) {
    if (meter_reading.valid && meter_reading.unit_suffix) {
        return meter_reading.unit_suffix;
    }
    return dmm53_unit;
}

const char *dmm_status_text(void) {
    return dmm53_status;
}

uint8_t dmm_reading_is_real(void) {
    return meter_reading.valid &&
           meter_reading.result_class != METER_RESULT_NONE &&
           meter_reading.result_class != METER_RESULT_INVALID;
}

uint8_t dmm_live_wire_active(void) {
    return 0;
}

uint8_t dmm_diode_continuity_active(void) {
    return 0;
}

const char *dmm53_debug_line(uint8_t idx) {
    static char line[4][44];
    char *p;

    switch (idx) {
    case 3: /* firmware-update path state (USB self-update debugging) */
    {
        fw_update_status_t st;
        uint16_t mc[6];
        fw_update_status(&st);
        usb_msc_debug_counts(mc);
        p = line[3];
        /* Build tag: bump when proving a USB self-update took effect. */
        *p++ = '#';
        *p++ = '5';
        *p++ = ' ';
        *p++ = 'U';
        *p++ = (char)('0' + (st.state % 10u));
        *p++ = 'E';
        *p++ = (char)('0' + (st.error % 10u));
        *p++ = ' ';
        *p++ = 'B';
        p += u16_to_dec(p, (uint16_t)(st.bytes >> 10));
        *p++ = 'K';
        *p++ = '/';
        p += u16_to_dec(p, (uint16_t)(st.expected_size >> 10));
        *p++ = 'K';
        *p++ = ' ';
        *p++ = 'W';
        p += u16_to_dec(p, mc[0]);
        *p++ = ' ';
        *p++ = 'X';
        p += u16_to_dec(p, mc[1]);
        *p++ = '@';
        *p++ = (char)('0' + (mc[5] % 10u));
        *p++ = ' ';
        *p++ = 'L';
        p += u16_to_dec(p, mc[2]);
        *p++ = ' ';
        *p++ = 'H';
        p += u16_to_dec(p, mc[4]);
        *p = '\0';
        return line[3];
    }
    case 0: /* baud candidate, wake step, counters */
        p = line[0];
        *p++ = 'B';
        *p++ = (char)('0' + dmm53_baud_index);
        *p++ = dmm53_baud_locked ? '!' : '?';
        *p++ = ' ';
        *p++ = 'W';
        *p++ = (char)('0' + dmm53_wake_step);
        *p++ = ' ';
        *p++ = 'T';
        p += u16_to_dec(p, dmm53_tx_count);
        *p++ = ' ';
        *p++ = 'R';
        p += u16_to_dec(p, rx_byte_count);
        *p++ = ' ';
        *p++ = 'D';
        p += u16_to_dec(p, rx_data_count);
        *p++ = ' ';
        *p++ = 'E';
        p += u16_to_dec(p, rx_echo_count);
        *p++ = ' ';
        *p++ = 'X';
        p += u16_to_dec(p, rx_err_count);
        *p = '\0';
        return line[0];
    case 1: /* full last data frame, 12 bytes hex */
        p = line[1];
        *p++ = 'F';
        for (uint8_t i = 0; i < DMM53_DATA_LEN; ++i) {
            uint8_t b = dmm53_frame[i];
            if ((i & 1u) == 0u) {
                *p++ = ' ';
            }
            *p++ = hex_digits[b >> 4];
            *p++ = hex_digits[b & 0xFu];
        }
        *p = '\0';
        return line[1];
    case 2: /* decoder state: submode, raw BCD, dp, class + f6 rotation.
             * Replaces the retired BRR/flash-ID line — band RE needs to
             * see every distinct frame[6] the SoC rotates through, not
             * just the one frame the F-line happens to catch. */
        p = line[2];
        *p++ = 'S';
        *p++ = (char)('0' + dmm53_submode());
        *p++ = ' ';
        *p++ = 'R';
        p += u16_to_dec(p, (uint16_t)meter_reading.raw_bcd);
        *p++ = ' ';
        /* E = stock's frame[2].3 raw +10000 extension. Added to the value in
         * DCV only; on other submodes this tells us whether the bit ever
         * fires there (stock evidence covers DCV alone). */
        *p++ = 'E';
        *p++ = meter_reading.raw_bcd_extended ? '1' : '0';
        *p++ = ' ';
        *p++ = 'P';
        *p++ = (char)('0' + meter_reading.decimal_pos);
        *p++ = ' ';
        *p++ = 'C';
        *p++ = (char)('0' + (meter_reading.result_class % 10u));
        *p++ = ' ';
        *p++ = 'H';
        for (uint8_t i = 0; i < meter_f6_history_count && i < 6u; ++i) {
            uint8_t b = meter_f6_history[i];
            *p++ = ' ';
            *p++ = hex_digits[b >> 4];
            *p++ = hex_digits[b & 0xFu];
        }
        *p = '\0';
        return line[2];
    default:
        return "";
    }
}

void dmm_uart_irq_handler(void) {
    uint32_t status = USART_STS(USART2_BASE);

    if (status & (USART_STS_ORE | USART_STS_NE | USART_STS_FE)) {
        (void)USART_DT(USART2_BASE);
        ++rx_err_count;
        rx_index = 0;
        return;
    }
    if (!(status & USART_STS_RXNE)) {
        return;
    }

    uint8_t byte = (uint8_t)USART_DT(USART2_BASE);
    ++rx_byte_count;

    if (rx_index == 0u) {
        if (byte == 0x5Au || byte == 0xAAu) {
            rx_buf[0] = byte;
            rx_index = 1u;
        }
    } else if (rx_index == 1u) {
        if ((rx_buf[0] == 0x5Au && byte == 0xA5u) ||
            (rx_buf[0] == 0xAAu && byte == 0x55u)) {
            rx_buf[1] = byte;
            rx_index = 2u;
        } else {
            rx_index = (byte == 0x5Au || byte == 0xAAu) ? 1u : 0u;
            if (rx_index) {
                rx_buf[0] = byte;
            }
        }
    } else {
        rx_buf[rx_index++] = byte;
        if (rx_buf[0] == 0x5Au && rx_index >= DMM53_DATA_LEN) {
            for (uint8_t i = 0; i < DMM53_DATA_LEN; ++i) {
                rx_data_frame[i] = rx_buf[i];
            }
            rx_data_ready = 1u;
            ++rx_data_count;
            rx_index = 0;
        } else if (rx_buf[0] == 0xAAu && rx_index >= DMM53_ECHO_LEN) {
            rx_echo_cmd = rx_buf[3];
            ++rx_echo_count;
            rx_index = 0;
        }
    }
}

void USART2_IRQHandler(void) {
    dmm_uart_irq_handler();
}

#endif /* HW_TARGET_2C53T */
