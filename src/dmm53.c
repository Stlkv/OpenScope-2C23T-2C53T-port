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
 *
 * Stage 4 (METER-PLAN, "modes and ranges") lives here too, and it is driven
 * by meter_plan.[ch] — upstream's reverse-engineered selector/mux tables,
 * ported verbatim. Everything below the tables is ours: one transition path
 * that projects the plan onto the analog frontend pins, queues the plan's
 * 0x05xx selector words on USART2, drains the transport across the switch,
 * and drops the frames the SoC produces mid-transition. It replaces the two
 * things the August bench run recorded as open holes: a frontend frozen in the
 * DCV pose no matter which mode the UI showed, and a hand-written per-submode
 * command switch that never fired for AUTO DETECT and mislabelled submode 5 as
 * a frequency mode.
 */

#include "dmm.h"

#include "board.h"
#include "fw_update.h"
#include "hw.h"
#include "meter_data.h"
#include "meter_plan.h"
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

    DMM53_POLL_MS = 250,       /* ~4 Hz, matches upstream meter_poll */
    DMM53_BAUD_RETRY_MS = 1500, /* per-candidate window; one wake transition
                                 * now costs ~250ms of queued words */
    /* Longest queue: frontend + 4 wake words + frontend + 2 bank words +
     * selector + apply + probe + start = 12. */
    DMM53_SEQ_MAX = 14,
    /* Pseudo-opcode in the step queue: apply the analog frontend for the
     * submode in .lo instead of sending a word. */
    DMM53_STEP_FRONTEND = 0xFFu,
};

/* Auxiliary AFE pins PB9/PA6.
 *
 * meter_plan's baseline keeps both LOW: upstream's note is that the recovered
 * stock sites prove output-low setup for these two and nothing mode-specific.
 * Our own stage-1 pose (2026-08-13) drove them HIGH, taken from an earlier
 * reading of fpga_set_meter_frontend_baseline, and that pose is what the one
 * successful resistance decode ran under. The table wins by default because it
 * is the evidence we can cite; the flag exists so the bench can A/B the two
 * poses in a single rebuild instead of a code edit. */
#ifndef DMM53_AUX_AFE_HIGH
#define DMM53_AUX_AFE_HIGH 0
#endif

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

/* Transition state. The queue is stepped from dmm_tick: each 10-byte word
 * already blocks ~10.4ms on the wire at 9600, and the plan asks for 20ms
 * settle gaps on top, so running a whole transition inline would stall the
 * render loop for a quarter of a second. */
typedef struct {
    uint8_t hi; /* DMM53_STEP_FRONTEND = apply frontend for .lo */
    uint8_t lo;
    uint16_t delay_ms; /* quiet time after this step */
} dmm53_step_t;

static dmm53_step_t dmm53_seq[DMM53_SEQ_MAX];
static uint8_t dmm53_seq_len;
static uint8_t dmm53_seq_pos;
static uint16_t dmm53_seq_timer_ms;
static uint16_t dmm53_seq_wait_ms;

static fpga_meter_transition_plan_t dmm53_plan;
static uint8_t dmm53_transition_busy;
static volatile uint8_t dmm53_discard_frames;
static volatile uint32_t dmm53_skip_count;
static uint16_t dmm53_gpio_planned;
static uint16_t dmm53_gpio_live;

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

/* The decoder session. Owns the reading, the band latch and the f6 history;
 * dmm53_begin_transition() starts a fresh one for every mode (re-)entry, so
 * nothing decoded under the previous mode can leak into the next. */
static meter_session_t dmm53_session;

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

static uint8_t dmm53_plan_submode(void);

/* Claim the frontend pins as push-pull outputs. Called on meter-mode entry,
 * not per transition: PA6 arrives from scope mode as the TMR13 PWM output and
 * has to be taken back before any level write means anything, while the rest
 * only need claiming once. Levels are the transition's business below. */
static void dmm53_frontend_configure_pins(void) {
    gpio_config_mask(GPIOB_BASE, (1u << 11) | (1u << 10) | (1u << 9), 0x1u);
    gpio_config_mask(GPIOC_BASE, (1u << 6) | (1u << 11) | (1u << 12), 0x1u);
    gpio_config_mask(GPIOE_BASE, (1u << 4) | (1u << 5) | (1u << 6), 0x1u);
    gpio_config_mask(GPIOA_BASE, (1u << 15) | (1u << 10) | (1u << 6), 0x1u);
}

/* Bit order copied from upstream fpga_meter_mux_gpio_mask_from_state so a mask
 * printed here and a mask printed by their `meter frontend` mean the same. */
static uint16_t dmm53_mux_mask_from_state(const fpga_meter_mux_gpio_state_t *s) {
    uint16_t mask = 0;
    if (s->pc12) mask |= 1u << 0;
    if (s->pe4)  mask |= 1u << 1;
    if (s->pe5)  mask |= 1u << 2;
    if (s->pe6)  mask |= 1u << 3;
    if (s->pa15) mask |= 1u << 4;
    if (s->pa10) mask |= 1u << 5;
    if (s->pb10) mask |= 1u << 6;
    if (s->pb11) mask |= 1u << 7;
    if (s->pb9)  mask |= 1u << 8;
    if (s->pa6)  mask |= 1u << 9;
    return mask;
}

static uint16_t dmm53_mux_mask_live(void) {
    uint16_t mask = 0;
    if (GPIO_IDR(GPIOC_BASE) & (1u << 12)) mask |= 1u << 0;
    if (GPIO_IDR(GPIOE_BASE) & (1u << 4))  mask |= 1u << 1;
    if (GPIO_IDR(GPIOE_BASE) & (1u << 5))  mask |= 1u << 2;
    if (GPIO_IDR(GPIOE_BASE) & (1u << 6))  mask |= 1u << 3;
    if (GPIO_IDR(GPIOA_BASE) & (1u << 15)) mask |= 1u << 4;
    if (GPIO_IDR(GPIOA_BASE) & (1u << 10)) mask |= 1u << 5;
    if (GPIO_IDR(GPIOB_BASE) & (1u << 10)) mask |= 1u << 6;
    if (GPIO_IDR(GPIOB_BASE) & (1u << 11)) mask |= 1u << 7;
    if (GPIO_IDR(GPIOB_BASE) & (1u << 9))  mask |= 1u << 8;
    if (GPIO_IDR(GPIOA_BASE) & (1u << 6))  mask |= 1u << 9;
    return mask;
}

static void dmm53_write_level(uint32_t base, uint32_t mask, uint8_t high) {
    if (high) {
        gpio_set(base, mask);
    } else {
        gpio_clear(base, mask);
    }
}

/* Project one submode's mux arms onto the pins, mirroring upstream
 * fpga_set_meter_frontend_for_submode: PB11/PC6 and the PC11 meter MUX gate go
 * HIGH first, then the plan's projection of the stock ms[0x02]/ms[0x03] writers
 * (PC12/PE4/PE5/PE6 and PA15/PA10/PB10/PB11) lands on top — so a slot that
 * wants PB11 LOW, such as diode, still gets it.
 *
 * An invalid submode still applies the baseline the model hands back and then
 * emits no selector word at all: fail closed, never a fabricated pose. */
static void dmm53_apply_frontend(uint8_t submode) {
    fpga_meter_mux_gpio_state_t mux;

    gpio_set(GPIOB_BASE, 1u << 11);
    gpio_set(GPIOC_BASE, (1u << 6) | (1u << 11));

    (void)fpga_meter_mux_gpio_state_for_submode(submode, &mux);
#if DMM53_AUX_AFE_HIGH
    mux.pb9 = 1u;
    mux.pa6 = 1u;
#endif
    dmm53_gpio_planned = dmm53_mux_mask_from_state(&mux);

    dmm53_write_level(GPIOC_BASE, 1u << 12, mux.pc12);
    dmm53_write_level(GPIOE_BASE, 1u << 4, mux.pe4);
    dmm53_write_level(GPIOE_BASE, 1u << 5, mux.pe5);
    dmm53_write_level(GPIOE_BASE, 1u << 6, mux.pe6);
    dmm53_write_level(GPIOA_BASE, 1u << 15, mux.pa15);
    dmm53_write_level(GPIOA_BASE, 1u << 10, mux.pa10);
    dmm53_write_level(GPIOB_BASE, 1u << 10, mux.pb10);
    dmm53_write_level(GPIOB_BASE, 1u << 11, mux.pb11);
    dmm53_write_level(GPIOB_BASE, 1u << 9, mux.pb9);
    dmm53_write_level(GPIOA_BASE, 1u << 6, mux.pa6);

    dmm53_gpio_live = dmm53_mux_mask_live();
}

/* Transport drain across a mode switch, the small local half of what stock
 * does at 0x0800741A: clear UEN, drop the meter MUX gate, reset the RX frame
 * index and any frame the ISR had ready, then bring USART2 back. The frontend
 * projection re-asserts PC11 on the next step. Stock also suspends its two
 * DVOM tasks — we have no tasks, the ISR plus this reset is the whole thing. */
static void dmm53_reset_transport(void) {
    USART_CTRL1(USART2_BASE) &= ~USART_CTRL1_UE;
    gpio_clear(GPIOC_BASE, 1u << 11);
    __asm__ volatile("cpsid i" ::: "memory");
    rx_index = 0;
    rx_data_ready = 0;
    __asm__ volatile("cpsie i" ::: "memory");
    (void)USART_STS(USART2_BASE);
    (void)USART_DT(USART2_BASE);
    USART_CTRL1(USART2_BASE) |= USART_CTRL1_UE;
}

static void dmm53_seq_push(uint8_t hi, uint8_t lo, uint16_t delay_ms) {
    if (dmm53_seq_len >= DMM53_SEQ_MAX) {
        return;
    }
    dmm53_seq[dmm53_seq_len].hi = hi;
    dmm53_seq[dmm53_seq_len].lo = lo;
    dmm53_seq[dmm53_seq_len].delay_ms = delay_ms;
    ++dmm53_seq_len;
}

static void dmm53_seq_push_word(uint16_t word, uint16_t delay_ms) {
    dmm53_seq_push((uint8_t)(word >> 8), (uint8_t)(word & 0xFFu), delay_ms);
}

/* Build one transition: the analog pose plus the plan's USART2 words, in
 * upstream's order (fpga_apply_meter_transition -> wake preamble ->
 * fpga_send_meter_mode_sequence). Nothing is sent from here; dmm_tick walks
 * the queue so the wire pacing survives without blocking the UI. */
static void dmm53_begin_transition(uint8_t wake_preamble) {
    uint8_t submode = dmm53_plan_submode();
    uint8_t probe = dmm53_probe_cmd();

    dmm53_plan = fpga_meter_transition_plan_for_submode(submode);
    dmm53_seq_len = 0;
    dmm53_seq_pos = 0;
    dmm53_seq_timer_ms = 0;
    dmm53_seq_wait_ms = 0;
    dmm53_poll_timer_ms = 0;
    dmm53_transition_busy = 1u;
    dmm53_discard_frames = 0;

    /* The previous mode's reading is not this mode's reading. A fresh session
     * drops it — plus the band latch and the f6 history — so the panel cannot
     * show a settled number under a freshly switched label, and a re-entry
     * into the same mode cannot inherit the previous session's dp/unit. */
    meter_session_begin(&dmm53_session, dmm53_plan.submode);
    dmm53_frame_valid = 0;

    dmm53_reset_transport();

    if (wake_preamble) {
        /* Stock brings the meter up on the DCV pose before it names a mode. */
        dmm53_seq_push(DMM53_STEP_FRONTEND, 0u, 20u);
        dmm53_seq_push(0x05u, 0x08u, 10u);            /* configure */
        dmm53_seq_push(0x05u, 0x09u, 10u);            /* start */
        dmm53_seq_push(0x05u, probe, 10u);            /* PC7-gated probe tail */
        dmm53_seq_push(0x05u, 0x14u, 20u);            /* variant setup */
    }

    dmm53_seq_push(DMM53_STEP_FRONTEND, submode, dmm53_plan.settle_ms);

    if (dmm53_plan.has_command_bank_prefix) {
        dmm53_seq_push(0x00u, dmm53_plan.command_bank_first,
                       dmm53_plan.settle_ms);
        dmm53_seq_push(0x00u, dmm53_plan.command_bank_second,
                       dmm53_plan.settle_ms);
    }
    if (dmm53_plan.has_config_word) {
        dmm53_seq_push_word(dmm53_plan.config_word, dmm53_plan.settle_ms);
    }
    if (dmm53_plan.selector_word != FPGA_METER_INVALID_SELECTOR_WORD) {
        dmm53_seq_push_word(dmm53_plan.selector_word, dmm53_plan.settle_ms);
    }
    if (dmm53_plan.has_apply_word) {
        dmm53_seq_push_word(dmm53_plan.apply_word, dmm53_plan.settle_ms);
    }
    if (dmm53_plan.has_probe_detect) {
        dmm53_seq_push(0x05u, probe, 10u);
    }
    if (dmm53_plan.start_word) {
        dmm53_seq_push_word(dmm53_plan.start_word, dmm53_plan.settle_ms);
    }
}

/* Walk the queue. One step per tick at most: every word blocks ~10.4ms on the
 * wire already, and the settle gaps are what the plan asks for between them. */
static void dmm53_seq_tick(uint32_t elapsed_ms) {
    const dmm53_step_t *step;

    dmm53_seq_timer_ms = (uint16_t)(dmm53_seq_timer_ms + elapsed_ms);
    if (dmm53_seq_timer_ms < dmm53_seq_wait_ms) {
        return;
    }
    dmm53_seq_timer_ms = 0;
    dmm53_seq_wait_ms = 0;

    if (dmm53_seq_pos >= dmm53_seq_len) {
        /* Last settle has elapsed: hand the SoC's first frames to the discard
         * policy and let the poll cadence take over. */
        dmm53_discard_frames = dmm53_plan.discard_frames;
        dmm53_transition_busy = 0;
        return;
    }

    step = &dmm53_seq[dmm53_seq_pos++];
    if (step->hi == DMM53_STEP_FRONTEND) {
        dmm53_apply_frontend(step->lo);
    } else {
        dmm53_send_cmd(step->hi, step->lo);
    }
    dmm53_seq_wait_ms = step->delay_ms;
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

    meter_session_begin(&dmm53_session, dmm53_plan_submode());
    dmm53_started = 1u;
    dmm53_powered = 0; /* silent until meter mode is entered */
}

void dmm_pause(void) {
    dmm53_powered = 0;
    /* Drop any half-sent transition: whatever mode comes next rebuilds the
     * whole queue, and a stale busy flag would gate the decoder shut. */
    dmm53_seq_len = 0;
    dmm53_seq_pos = 0;
    dmm53_seq_wait_ms = 0;
    dmm53_transition_busy = 0;
    /* Meter MUX off; the other relays keep their last state (scope re-entry
     * re-applies its own posture — fpga53_scope_pose_reapply). */
    gpio_clear(GPIOC_BASE, 1u << 11);
}

void dmm_hw4_set_mode_gate(uint8_t active) {
    (void)active;
}

void dmm_reenter(uint8_t mode_index) {
    dmm53_mode = mode_index;
    dmm53_powered = 1u;
    dmm53_frontend_configure_pins();
    dmm53_begin_transition(1u);
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
     * A transition already in flight for this same mode is left alone: the
     * queue ends in the mode being asked for, so restarting it can only push
     * the settle further out. dmm_reenter() stays the unconditional entry
     * point for meter-mode entry. */
    if (dmm53_powered && mode_index == dmm53_mode) {
        return;
    }

    dmm53_mode = mode_index;
    dmm53_begin_transition(0u);
}

/* Port UI meter modes (ui.c dmm_mode_names, 0-12) → meter_plan local submodes
 * (0-10), which are also meter_data's decode submodes for 0-9 plus temperature
 * at 10.
 *
 * The mapping goes through meter_plan's logical-function enum rather than a
 * hand-written submode table, so an unresolved function (the microamp
 * frontend, which stock V1.2.0 shows no selector for) arrives here as
 * FPGA_METER_INVALID_LOCAL_SUBMODE and takes the fail-closed path instead of
 * being quietly aimed at a neighbouring range.
 *
 * AUTO DETECT and LIVE WIRE have no logical function: the stock selector table
 * has no slot that means "decide for me" and none that means "non-contact live
 * wire". Both keep pointing at DCV, as they did in stage 1 — a placeholder we
 * can name, not evidence. */
static uint8_t dmm53_plan_submode(void) {
    static const uint8_t map[13] = {
        FPGA_METER_FUNCTION_DCV,         /* AUTO DETECT  (placeholder) */
        FPGA_METER_FUNCTION_DCV,         /* DC VOLTAGE */
        FPGA_METER_FUNCTION_ACV,         /* AC VOLTAGE */
        FPGA_METER_FUNCTION_RESISTANCE,  /* RESISTANCE */
        FPGA_METER_FUNCTION_DIODE,       /* DIODE */
        FPGA_METER_FUNCTION_CAPACITANCE, /* CAPACITANCE */
        FPGA_METER_FUNCTION_DCV,         /* LIVE WIRE    (placeholder) */
        FPGA_METER_FUNCTION_CONTINUITY,  /* CONTINUITY */
        FPGA_METER_FUNCTION_TEMPERATURE, /* TEMPERATURE */
        FPGA_METER_FUNCTION_AC_A,        /* AC HIGH CURR */
        FPGA_METER_FUNCTION_DC_A,        /* DC HIGH CURR */
        FPGA_METER_FUNCTION_AC_MA,       /* AC LOW CURR */
        FPGA_METER_FUNCTION_DC_MA,       /* DC LOW CURR */
    };
    uint8_t function =
        dmm53_mode < 13u ? map[dmm53_mode] : (uint8_t)FPGA_METER_FUNCTION_DCV;
    return fpga_meter_submode_for_logical_function(function);
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
    dmm53_baud_locked = 1u; /* real framed traffic = correct baud */

    /* Raw telemetry sees every frame, including the dropped ones — the whole
     * point of the F-line is to show what actually arrived. The decoder does
     * not: frames produced while the selector words and the relays are still
     * moving belong to no mode in particular, and feeding them to the band
     * latch is how a switch to resistance used to land on the old range. */
    dmm53_format_value();
    dmm53_format_status();
    if (!fpga_meter_rx_frame_should_parse(dmm53_transition_busy != 0,
                                          &dmm53_discard_frames,
                                          &dmm53_skip_count)) {
        return 0;
    }

    dmm53_frame_valid = 1u;
    /* The session's submode was fixed by the last transition: the decoder
     * reads the frame under the same submode whose selector words are on the
     * wire, not under whatever the UI mode says right now. */
    meter_session_frame(&dmm53_session, dmm53_frame);
    return 1u;
}

void dmm_tick(uint32_t elapsed_ms) {
    if (!dmm53_started || !dmm53_powered) {
        return;
    }

    /* A transition owns the wire until its queue drains. */
    if (dmm53_transition_busy) {
        dmm53_seq_tick(elapsed_ms);
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
                dmm53_begin_transition(1u);
            }
        }
    }
    dmm53_format_status();
}

uint8_t dmm_has_reading(void) {
    return dmm53_frame_valid && dmm53_session.reading.valid;
}

uint8_t dmm_value_is_numeric(void) {
    return dmm53_session.reading.valid &&
           dmm53_session.reading.result_class == METER_RESULT_NORMAL;
}

int32_t dmm_value_milli_units(void) {
    float v = dmm53_session.reading.value * 1000.0f;
    return (int32_t)(v < 0 ? v - 0.5f : v + 0.5f);
}

const char *dmm_value_text(void) {
    if (dmm53_session.reading.valid) {
        return dmm53_session.reading.display_str;
    }
    return dmm53_frame_valid ? dmm53_value : "----";
}

const char *dmm_unit_text(void) {
    if (dmm53_session.reading.valid && dmm53_session.reading.unit_suffix) {
        return dmm53_session.reading.unit_suffix;
    }
    return dmm53_unit;
}

const char *dmm_status_text(void) {
    return dmm53_status;
}

uint8_t dmm_reading_is_real(void) {
    return dmm53_session.reading.valid &&
           dmm53_session.reading.result_class != METER_RESULT_NONE &&
           dmm53_session.reading.result_class != METER_RESULT_INVALID;
}

uint8_t dmm_live_wire_active(void) {
    return 0;
}

uint8_t dmm_diode_continuity_active(void) {
    return 0;
}

/* Fixed-width hex, most significant nibble first. */
static char *hex_n(char *p, uint16_t v, uint8_t nibbles) {
    while (nibbles--) {
        *p++ = hex_digits[(v >> (nibbles * 4u)) & 0xFu];
    }
    return p;
}

static char *hex_word_or_dashes(char *p, uint8_t present, uint16_t v) {
    if (!present) {
        *p++ = '-';
        *p++ = '-';
        *p++ = '-';
        *p++ = '-';
        return p;
    }
    return hex_n(p, v, 4u);
}

const char *dmm53_debug_line(uint8_t idx) {
    static char line[5][56];
    char *p;

    switch (idx) {
    case 4: /* firmware-update path state (USB self-update debugging).
             * Off the meter overlay since the CDC shell arrived — `version`
             * and `fwstat` answer the same question over the cable, and the
             * meter screen owes its fourth line to the meter. */
    {
        fw_update_status_t st;
        uint16_t mc[6];
        fw_update_status(&st);
        usb_msc_debug_counts(mc);
        p = line[4];
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
        return line[4];
    }
    case 0: /* baud candidate, transition queue position, counters */
        p = line[0];
        *p++ = 'B';
        *p++ = (char)('0' + dmm53_baud_index);
        *p++ = dmm53_baud_locked ? '!' : '?';
        *p++ = ' ';
        *p++ = 'Q';
        p += u16_to_dec(p, dmm53_seq_pos);
        *p++ = '/';
        p += u16_to_dec(p, dmm53_seq_len);
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
        p += u16_to_dec(p, dmm53_session.submode);
        *p++ = ' ';
        *p++ = 'R';
        p += u16_to_dec(p, (uint16_t)dmm53_session.reading.raw_bcd);
        *p++ = ' ';
        /* E = stock's frame[2].3 raw +10000 extension. Added to the value in
         * DCV only; on other submodes this tells us whether the bit ever
         * fires there (stock evidence covers DCV alone). */
        *p++ = 'E';
        *p++ = dmm53_session.reading.raw_bcd_extended ? '1' : '0';
        *p++ = ' ';
        *p++ = 'P';
        *p++ = (char)('0' + dmm53_session.reading.decimal_pos);
        *p++ = ' ';
        *p++ = 'C';
        *p++ = (char)('0' + (dmm53_session.reading.result_class % 10u));
        *p++ = ' ';
        /* V = DCV exponent class from the stock status bits, '-' when the
         * stock DCV path did not run for this frame. A wrong DCV number is
         * only diagnosable next to the class the frame claimed. */
        *p++ = 'V';
        *p++ = (dmm53_session.reading.dcv_stock_class > 4u)
                   ? '-'
                   : (char)('0' + dmm53_session.reading.dcv_stock_class);
        *p++ = ' ';
        *p++ = 'H';
        for (uint8_t i = 0; i < dmm53_session.f6_history_count && i < 6u; ++i) {
            uint8_t b = dmm53_session.f6_history[i];
            *p++ = ' ';
            *p++ = hex_digits[b >> 4];
            *p++ = hex_digits[b & 0xFu];
        }
        *p = '\0';
        return line[2];
    case 3: /* transition plan: what the tables asked for, and what the pins
             * actually read back. G planned/live differing is the one thing
             * that separates "the mode never reached the frontend" from "the
             * frontend moved and the SoC still answers with the old range" —
             * the ambiguity the August session had no way to resolve. */
        p = line[3];
        *p++ = 'P';
        p += u16_to_dec(p, dmm53_mode);
        *p++ = ' ';
        *p++ = 'S';
        p += u16_to_dec(p, dmm53_plan.submode);
        *p++ = '/';
        if (dmm53_plan.stock_mode == FPGA_METER_INVALID_STOCK_MODE) {
            *p++ = '-';
        } else {
            p += u16_to_dec(p, dmm53_plan.stock_mode);
        }
        *p++ = ' ';
        *p++ = 'C';
        p = hex_word_or_dashes(p, dmm53_plan.has_config_word,
                               dmm53_plan.config_word);
        *p++ = ' ';
        *p++ = 'W';
        p = hex_word_or_dashes(
            p, dmm53_plan.selector_word != FPGA_METER_INVALID_SELECTOR_WORD,
            dmm53_plan.selector_word);
        *p++ = ' ';
        *p++ = 'A';
        p = hex_word_or_dashes(p, dmm53_plan.has_apply_word,
                               dmm53_plan.apply_word);
        *p++ = ' ';
        *p++ = 'G';
        p = hex_n(p, dmm53_gpio_planned, 3u);
        *p++ = '/';
        p = hex_n(p, dmm53_gpio_live, 3u);
        *p++ = ' ';
        *p++ = 'X';
        p += u16_to_dec(p, (uint16_t)(dmm53_skip_count > 0xFFFFu
                                          ? 0xFFFFu
                                          : dmm53_skip_count));
        *p++ = '.';
        p += u16_to_dec(p, dmm53_discard_frames);
        *p = '\0';
        return line[3];
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
