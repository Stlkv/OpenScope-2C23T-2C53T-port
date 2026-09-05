#include "settings.h"
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

#ifndef FPGA53_V04_CONFIG
#define FPGA53_V04_CONFIG 0
#endif

/* Skip configuration when the FPGA already carries this build's bitstream
 * (warm boot). Set to 0 for an A/B run against the old unconditional
 * behaviour. */
#ifndef FPGA53_WARM_SKIP
#define FPGA53_WARM_SKIP 1
#endif

/* STATUS(0x41) after a configuration that took (DONE_FINAL set) — the value
 * the overlay calls CFG:OK. Bit 31 is masked off before comparing: the 0x80
 * first-window marker can land on it. */
#define FPGA53_STATUS_CFG_OK 0x0003F460u

/* Delay before the post-config SPI3 writes. Stock waits ~600ms between the
 * 0x3A config close and the five config writes (issue-#18 capture); a fresh
 * Gowin design needs PLL lock + internal reset release before its control
 * registers accept writes. */
#ifndef FPGA53_SEND_CFG_DELAY_MS
#define FPGA53_SEND_CFG_DELAY_MS 0
#endif

/* How many times to re-issue the five config writes + status read while the
 * status reply differs from stock's armed reply (00 01 42 2E 2E). */
#ifndef FPGA53_SEND_CFG_RETRIES
#define FPGA53_SEND_CFG_RETRIES 1
#endif

#ifndef FPGA53_SWEEP_PRECMD
#define FPGA53_SWEEP_PRECMD 0
#endif

#ifndef FPGA53_SWEEP_CFG02
#define FPGA53_SWEEP_CFG02 0
#endif

/* Arm-bit hunt: sweep register 0x01's value with PC0-pulse autodetect. */
#ifndef FPGA53_SWEEP_CFG01
#define FPGA53_SWEEP_CFG01 0
#endif

/* Hold the stock-driven-but-unmapped pins HIGH: PD3 (static HIGH from SPI3
 * bring-up), PD2 (asserted on scope-mode entry — top run-line candidate),
 * PC4 (mode-flag-2 level). Post-config run relevance was untestable until a
 * live self-configured FPGA existed (unmapped_mcu_fpga_pin_candidates.md). */
#ifndef FPGA53_RUN_PINS
#define FPGA53_RUN_PINS 0
#endif

/* CH2 trigger reference via TMR13 CH1 PWM on PA6 (upstream static-analysis
 * lead, issue #18 2026-08-12): mid-scale duty at boot. */
#ifndef FPGA53_TMR13_REF
#define FPGA53_TMR13_REF 0
#endif

#ifndef FPGA53_SWAP_ORDER
#define FPGA53_SWAP_ORDER 0
#endif

/* Read each window twice, keep the second — stock's anti-tearing scheme. */
#ifndef FPGA53_PINGPONG
#define FPGA53_PINGPONG 1
#endif

#ifndef FPGA53_PRE_CMD
#define FPGA53_PRE_CMD 0
#endif

#ifndef FPGA53_SEND_CFG
#define FPGA53_SEND_CFG 0
#endif

/* Bench experiment for issue #18 (2026-08-16): hold PB11 LOW through config
 * AND the arm writes, instead of the HIGH level stock holds there.
 *
 * The two projects measured this pin from opposite ends and got opposite
 * answers. Here, an armed engine keeps delivering data with PB11 LOW — rows 2
 * and 7 of the relay ladder depend on it, five dumps on 2026-08-16. Upstream's
 * guest-coldtrace will not ARM unless the pin is HIGH, and the reconstructed
 * netlist constraints call the same pad run_enable. Neither result contradicts
 * the other while our pose runs AFTER the arm writes: the pin can be needed
 * during config and free afterwards.
 *
 * This build is the discriminator. If the part still reaches DONE_FINAL and
 * frames still carry data with the pin LOW from the first bring-up write, PB11
 * is a relay on this hardware and upstream's arm failure is something else. If
 * config or the arm dies, the pin belongs to the run path and our rows 2 and 7
 * are only safe because they come late — which the volts/div ladder would then
 * have to work around. The flag's default is in fpga.h, where the dump can see
 * it too. */

/* PC0 data-ready polarity. Upstream's working cold rig (guest-coldtrace,
 * issue #18 2026-08-13) treats PC0 as ACTIVE LOW with an input pull-up
 * (undriven = HIGH = not ready); our warm-handoff builds historically used
 * active HIGH. 1 = ready when LOW + pull-up (cold-config rig), 0 = legacy
 * ready-when-HIGH + floating (bench-proven warm handoff). */
#ifndef FPGA53_PC0_READY_LOW
#define FPGA53_PC0_READY_LOW 0
#endif

/* Drive the analog-frontend relay/gain bank to upstream's scope posture at
 * boot (their fpga_set_scope_frontend_range case 7: PC12 HIGH = DC coupling,
 * bench-measured 2026-08-12). On a cold boot nothing pre-arms the bank: the
 * coils float, CH1 conducts only by accident and CH2 is OPEN — upstream's
 * live-both-channels coldtrace always drives this bank (incl. PB10, absent
 * from all our earlier 16-pattern sweeps). Also swaps PA6 out of the manual
 * Y-bank cycle for PB10 (PA6 carries the TMR13 PWM reference now). */
#ifndef FPGA53_FE_SCOPE_POSE
#define FPGA53_FE_SCOPE_POSE 0
#endif

/* Drive the PD12/PD13 coupling relays (HIGH = DC) instead of leaving them
 * wherever the boot put them. Part of the scope pose, so it rides the same
 * flag by default. */
#ifndef FPGA53_FE_COUPLING
#define FPGA53_FE_COUPLING FPGA53_FE_SCOPE_POSE
#endif

/* Stock-style paced readout: never gate on PC0 — stock reads the 0x04/0x05
 * window pair every ~29 ms unconditionally, and each read hands back the
 * latest (re-armed) capture. Bench 2026-08-13: PC0-gating breaks one way or
 * the other depending on engine state — free-run pulses LOW (no signal),
 * triggered captures leave it un-asserted (signal present) → frames only via
 * rare forced reads. With pacing, reads happen at the UI loop rate and PC0
 * stays a diagnostic. Overrides FPGA53_PC0_READY_LOW gating. */
#ifndef FPGA53_READ_PACED
#define FPGA53_READ_PACED 0
#endif

#ifndef FPGA53_CFG02_VAL
#define FPGA53_CFG02_VAL 0x03u
#endif

#ifndef FPGA53_DUAL_READ
#define FPGA53_DUAL_READ 0
#endif

/* MCU-side decimation ("slow point"): the capture engine free-runs at its own
 * ~31 MSa/s and every read hands back the freshest 1023-sample window, so a
 * single window is one instantaneous level for anything slower than ~30 kHz.
 * Sampling that level on a timer gives a slow timebase whose rate the MCU
 * owns outright — no FPGA timing register involved (they are still a stub,
 * see fpga_write_timing). Averaging a few samples trades nothing away: the
 * whole window spans 33 us. */
#ifndef FPGA53_SLOW_POINT_SAMPLES
#define FPGA53_SLOW_POINT_SAMPLES 8u
#endif

/* Whether a slow point drains the whole 1023-sample window before releasing
 * CS. ANSWERED ON THE BENCH 2026-08-15, and the answer is yes, always: the
 * engine refreshes its window only once one has been fully clocked out. With
 * short reads the point span over a 256-point block was p=92-92 — one frozen
 * value, no matter that the sampler was running at 17 kHz; with full reads it
 * was p=00-94, the real signal. Short reads are kept only as a build knob for
 * re-testing that claim, never as the operating mode. Cost of a full point:
 * 531 timer ticks = 1.6 ms for both channels at ~12 MHz SPI. */
#ifndef FPGA53_SLOW_POINT_FULL
#define FPGA53_SLOW_POINT_FULL 1
#endif

/* Timing-register falsification sweep (upstream Step 0: it is NOT proven that
 * 0x0F/0x10/0x11 change the sample rate). Walks each register through a value
 * ladder, dwelling a few frames per value, and records the window's shape
 * metric for every step into a table the DBG dump prints. One bench run with
 * a periodic input answers it: if any register divides the rate, its rows show
 * the edge count rising and the period shrinking. */
/* Sweep stock's one-byte "fast timebase config" instead of the register
 * ladder. Implies the sweep machinery below. */
#ifndef FPGA53_SWEEP_TIMING_DWELL
#define FPGA53_SWEEP_TIMING_DWELL 12u
#endif

/* Minimum peak-to-peak spread (raw counts) before the window metric is
 * believed — below this the trace is baseline noise and any "period" would be
 * noise crossings. */
#ifndef FPGA53_METRIC_MIN_SPREAD
#define FPGA53_METRIC_MIN_SPREAD 8u
#endif

#ifndef FPGA_SPI_BR
#define FPGA_SPI_BR 2u
#endif

static uint8_t fpga_loaded;
static uint8_t fpga53_frame[FPGA_SCOPE_BUFFER_BYTES];
static uint8_t fpga53_ch_buf[FPGA53_CH_SAMPLES];
static uint16_t fpga53_notready_polls;
static uint8_t fpga53_force_read;
static uint8_t fpga53_cfg01_started;
/* SPI3 is shared between the main-loop window read and the timer-paced slow
 * point sampler, which runs from the TMR1 IRQ. The sampler backs off whenever
 * the main loop owns the bus rather than interleaving bytes into someone
 * else's CS window. */
static volatile uint8_t fpga53_bus_busy;
/* Defined with the DMA sampler further down; the window read needs these to
 * move a window without a byte loop, and to reclaim the bus from an in-flight
 * roll transfer. */
static void fpga53_dma_cancel(void);
static void fpga53_dma_arm(uint16_t count);
static uint8_t fpga53_dma_complete(void);
static void fpga53_dma_stop(void);
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
#if FPGA53_FE_SCOPE_POSE
        gpio_set(GPIOB_BASE, 1u << 10);   /* PA6 = TMR13 PWM now; sweep PB10 */
#else
        gpio_set(GPIOA_BASE, 1u << 6);
#endif
    } else {
#if FPGA53_FE_SCOPE_POSE
        gpio_clear(GPIOB_BASE, 1u << 10);
#else
        gpio_clear(GPIOA_BASE, 1u << 6);
#endif
    }
#if FPGA53_FE_SCOPE_POSE
    gpio_config_mask(GPIOA_BASE, (1u << 15) | (1u << 10), 0x1u);
    gpio_config_mask(GPIOB_BASE, (1u << 9) | (1u << 10), 0x1u);
#else
    gpio_config_mask(GPIOA_BASE, (1u << 15) | (1u << 10) | (1u << 6), 0x1u);
    gpio_config_mask(GPIOB_BASE, 1u << 9, 0x1u);
#endif
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
    d->sweep_val = fpga53_diag.sweep_val;
    d->sweep_hit = fpga53_diag.sweep_hit;
    d->v04_id = fpga53_diag.v04_id;
    d->v04_stb = fpga53_diag.v04_stb;
    d->v04_sta = fpga53_diag.v04_sta;
    d->warm = fpga53_diag.warm;
    d->bs_ok = fpga53_diag.bs_ok;
    d->pose_calls = fpga53_diag.pose_calls;
    d->win_edges = fpga53_diag.win_edges;
    d->win_period = fpga53_diag.win_period;
    d->win_edges2 = fpga53_diag.win_edges2;
    d->win_period2 = fpga53_diag.win_period2;
    d->wmin = fpga53_diag.wmin;
    d->wmax = fpga53_diag.wmax;
    d->wmin2 = fpga53_diag.wmin2;
    d->wmax2 = fpga53_diag.wmax2;
    d->crh_boot = fpga53_diag.crh_boot;
    d->fe_ch2 = fpga53_diag.fe_ch2;
    d->slow_points = fpga53_diag.slow_points;
    d->slow_busy = fpga53_diag.slow_busy;
    d->slow_min = fpga53_diag.slow_min;
    d->slow_max = fpga53_diag.slow_max;
    d->slow_full = fpga53_diag.slow_full;
    d->dma_fail = fpga53_diag.dma_fail;
    d->tsweep_reg = fpga53_diag.tsweep_reg;
    d->tsweep_val = fpga53_diag.tsweep_val;
    d->tsweep_row = fpga53_diag.tsweep_row;
    d->tsweep_done = fpga53_diag.tsweep_done;
}


#if FPGA53_V04_CONFIG
/*
 * Config-entry attempt using the 2C23T V0.4 loader sequence (documented by
 * maksidze in OpenScope-2C53T issue #18): bit-banged SSPI, framing exactly
 * as the proven-working V0.4 firmware does it —
 *   IDCODE(0x11) -> USERCODE(0x13) -> STATUS(0x41) -> INIT_ADDR(0x12 00)
 *   -> CONFIG_ENABLE(0x15 00) -> flush -> CS-LOW 0x3B + full bitstream
 *   -> STATUS -> CONFIG_DISABLE(0x3A 00)
 * with the 2C53T payload (byte-exact vs the stock Saleae capture) and the
 * 2C53T pins: PB3=CLK, PB4=MISO, PB5=MOSI, CS=PB6. No reset pulse (the
 * 2C53T stock boot capture shows none; the V0.4 reset pin maps to the
 * 2C53T POWER button, so it must not be driven).
 *
 * DO NOT convert this to hardware SPI to make boot faster. It is not a
 * placeholder for a "proper" driver — the GPIO drive is the load-bearing part.
 * Upstream spent 2026-08-26/27 narrowing exactly this (EXP-31..37 on
 * DavidClawson/OpenScope-2C53T `main`): a logic-analyzer view proved their
 * hardware-SPI config frames are byte-perfect and the 0x15 frames from AF and
 * from bit-bang are DIGITALLY IDENTICAL; the write rate was excluded in both
 * directions at a matched 470 kHz; what is left as the differentiator is
 * GPIO-driven pins versus alternate-function-driven pins, with the mechanism
 * below the analyzer's 24 MHz resolution and the line parked there. Their
 * hardware-SPI path still walls; this bit-bang path configures the part. Until
 * somebody explains the mechanism, treat "slow and gapped through GPIO" as a
 * requirement — and for the same reason the settle delay and the unhurried
 * bitstream upload are not optimization targets either.
 */
/*
 * Where the payload comes from.
 *
 * EXTERN (default): the bitstream lives in its own flash region, written once
 * by dropping the store file on the USB volume — see fpga_bitstream_store.h.
 * That keeps 113 KB of constant data out of a 224 KB app slot, which is the
 * difference between "12 bytes free" and "half the slot free".
 *
 * Embedded (FPGA53_BITSTREAM_EXTERN=0): the old arrangement, kept as the
 * provisioning and recovery build — it is the image that can configure an
 * FPGA on a unit whose store has never been written.
 */
#ifndef FPGA53_BITSTREAM_EXTERN
#define FPGA53_BITSTREAM_EXTERN 1
#endif

#if FPGA53_BITSTREAM_EXTERN
#include "fpga_bitstream_store.h"
#else
#include "fpga_bitstream_2c53t.h"
#endif

/* 1 = the last upload reached DONE_FINAL, i.e. the part now carries our
 * design and the warm token may be written. */
static uint8_t fpga53_cfg_ok;

#if FPGA53_BITSTREAM_EXTERN
static const uint8_t *fpga53_bs_data(void) {
    return (const uint8_t *)FPGA_BS_DATA_BASE;
}

static uint32_t fpga53_bs_length(void) {
    const fpga_bs_header_t *h = (const fpga_bs_header_t *)FPGA_BS_STORE_BASE;

    if (h->magic != FPGA_BS_MAGIC ||
        h->check != (uint32_t)~(h->magic + h->length + h->fingerprint) ||
        h->length < 1024u ||
        h->length > FPGA_BS_STORE_MAX - FPGA_BS_HDR_BYTES) {
        return 0;
    }
    return h->length;
}

static uint32_t fpga53_bs_stored_fingerprint(void) {
    return ((const fpga_bs_header_t *)FPGA_BS_STORE_BASE)->fingerprint;
}
#else
static const uint8_t *fpga53_bs_data(void) {
    return fpga_h2_cal_table;
}

static uint32_t fpga53_bs_length(void) {
    return FPGA_H2_CAL_TABLE_SIZE;
}
#endif

static uint8_t v04_xfer(uint8_t value) {
    uint8_t result = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        gpio_clear(GPIOB_BASE, 1u << 3);
        if (value & 0x80u) {
            gpio_set(GPIOB_BASE, 1u << 5);
        } else {
            gpio_clear(GPIOB_BASE, 1u << 5);
        }
        gpio_set(GPIOB_BASE, 1u << 3);
        result = (uint8_t)(result << 1);
        value = (uint8_t)(value << 1);
        if (GPIO_IDR(GPIOB_BASE) & (1u << 4)) {
            result |= 1u;
        }
    }
    return result;
}

static uint32_t v04_read_reg32(uint32_t addr) {
    uint8_t b[4];
    (void)v04_xfer(0);
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)v04_xfer((uint8_t)(addr >> 24));
    (void)v04_xfer((uint8_t)(addr >> 16));
    (void)v04_xfer((uint8_t)(addr >> 8));
    (void)v04_xfer((uint8_t)addr);
    for (uint8_t i = 0; i < 4u; ++i) {
        b[i] = v04_xfer(0);
    }
    gpio_set(GPIOB_BASE, 1u << 6);
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

static void v04_cmd16(uint16_t cmd) {
    (void)v04_xfer(0);
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)v04_xfer((uint8_t)(cmd >> 8));
    (void)v04_xfer((uint8_t)cmd);
    gpio_set(GPIOB_BASE, 1u << 6);
}

static void fpga53_v04_configure(void) {
    /* Bit-bang pin setup: CLK/MOSI/CS outputs, MISO floating input.
     * CS idle HIGH, CLK idle HIGH (mode-3 idle), PC6/PB11 already HIGH. */
    gpio_set(GPIOB_BASE, (1u << 3) | (1u << 6));
    gpio_clear(GPIOB_BASE, 1u << 5);
    gpio_config_mask(GPIOB_BASE, (1u << 3) | (1u << 5) | (1u << 6), 0x1u);
    gpio_config_mask(GPIOB_BASE, 1u << 4, 0x4u);

    fpga53_diag.v04_id = v04_read_reg32(0x11000000u);
    (void)v04_read_reg32(0x13000000u);
    fpga53_diag.v04_stb = v04_read_reg32(0x41000000u);

    v04_cmd16(0x1200u); /* INIT_ADDR */
    v04_cmd16(0x1500u); /* CONFIG_ENABLE */

    (void)v04_xfer(0);
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)v04_xfer(0x3Bu);
    {
        const uint8_t *bs = fpga53_bs_data();
        uint32_t len = fpga53_bs_length();
        for (uint32_t i = 0; i < len; ++i) {
            (void)v04_xfer(bs[i]);
        }
    }
    gpio_set(GPIOB_BASE, 1u << 6);

    fpga53_diag.v04_sta = v04_read_reg32(0x41000000u);
    fpga53_cfg_ok =
        ((fpga53_diag.v04_sta & 0x7FFFFFFFu) == FPGA53_STATUS_CFG_OK) ? 1u : 0u;
    v04_cmd16(0x3A00u); /* CONFIG_DISABLE */

    /* Stock fidelity (issue-#18 Saleae decode): after the 3A close, stock
     * sends exactly one single-byte 0x00 flush frame in its own CS window,
     * then leaves the bus idle for ~607ms before the config writes. */
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)v04_xfer(0);
    gpio_set(GPIOB_BASE, 1u << 6);
    delay_ms(100);
}

/* ── Warm boot: keep a configuration we already own ───────────────────────
 *
 * The self-update path never resets the FPGA: fw_update.c ends with a direct
 * `bx` into the freshly written image, and MENU+pinhole is only an MCU reset
 * — in both cases the FPGA rail stays up and the design we uploaded is still
 * running. Re-running the V0.4 sequence there cannot help (a configured part
 * answers the config port with zeros) and can hurt: per upstream Exps L/M,
 * touching the config port of a configured part desynchronises acquisition.
 * That is the most likely reason a warm reboot has been showing V/B/A zeros
 * and no trace, while the same binary works from a cold start.
 *
 * So: after a successful configuration, leave a token in RAM saying "this
 * part carries THIS bitstream"; on the next boot, if the token is intact,
 * skip config entirely and go straight to the arm writes. The token lives at
 * the fixed address ORIGIN(RAM) (section .warm_token, see linker.ld) because
 * the image that writes it and the image that reads it are different builds.
 *
 * Two independent invalidators, so a genuine power cycle can never be
 * mistaken for a warm boot: SRAM loses the token when the rail drops, and the
 * reset-cause register reports POR (we clear the flags on every boot, so the
 * flag can only come from the power-on that just happened). The fingerprint
 * covers the third case — a build whose bitstream differs from the one the
 * FPGA is actually running.
 *
 * The case neither invalidator covers: a brownout deep enough to wipe the
 * FPGA's SRAM config but not the MCU's. Then the token lies, and the symptom
 * — WARM=1 with a dead engine — looks exactly like a firmware regression.
 * Power-cycle first, bisect the code second; that is also why the bench rule
 * is to keep the battery in place. */
#define RCC_CSR_REG   REG32(RCC_BASE + 0x24u)
#define RCC_CSR_RMVF  (1u << 24) /* write 1: clear the reset-cause flags */
#define RCC_CSR_PORRSTF (1u << 27)

#define WARM_TOKEN_MAGIC 0x53433533u /* "SC53" */

typedef struct {
    uint32_t magic;
    uint32_t fingerprint;
    uint32_t check; /* ~(magic + fingerprint): catches a half-written token */
} warm_token_t;

__attribute__((section(".warm_token"), used))
static volatile warm_token_t warm_token;

/* Rolling sum over the payload. Doubles as the store's integrity check (the
 * header carries the value the host computed with the same algorithm) and as
 * the warm token's identity of "which bitstream is the FPGA running". */
static uint32_t fpga53_bitstream_fingerprint(void) {
    const uint8_t *p = fpga53_bs_data();
    uint32_t len = fpga53_bs_length();
    uint32_t f = len;

    for (uint32_t i = 0; i < len; ++i) {
        f = ((f << 1) | (f >> 31)) + p[i];
    }
    return f;
}

static void fpga53_warm_token_write(uint8_t valid, uint32_t fingerprint) {
    warm_token.magic = valid ? WARM_TOKEN_MAGIC : 0u;
    warm_token.fingerprint = valid ? fingerprint : 0u;
    warm_token.check = valid ? ~(WARM_TOKEN_MAGIC + fingerprint) : 0u;
}

/* 1 = the FPGA is already running this build's bitstream; do not configure. */
static uint8_t fpga53_warm_boot(uint32_t fingerprint) {
    uint32_t csr = RCC_CSR_REG;
    RCC_CSR_REG = csr | RCC_CSR_RMVF; /* arm the flag for the next boot */
    if (csr & RCC_CSR_PORRSTF) {
        return 0u;
    }
    if (warm_token.magic != WARM_TOKEN_MAGIC) {
        return 0u;
    }
    if (warm_token.check != (uint32_t)~(WARM_TOKEN_MAGIC + warm_token.fingerprint)) {
        return 0u;
    }
    return (warm_token.fingerprint == fingerprint) ? 1u : 0u;
}
#endif /* FPGA53_V04_CONFIG */

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

/* CH2 trigger comparator reference: per upstream's static analysis of stock
 * V1.2.0, CH2's reference is NOT the DAC but a TMR13 CH1 PWM (C1DT @
 * 0x40001C34), per-range duty via the same cal formula as DAC1. TMR13_CH1 =
 * PA6 (default mapping) — the "undocumented frontend" pin. Mid-scale 50% duty;
 * an RC filter on the board turns it into a DC reference. Nobody programs
 * TMR13 after an MCU reset, so CH2's comparator reference is dead without it —
 * and the meter reclaims PA6 as a plain GPIO gain key, so scope-mode entry has
 * to take it back (see fpga53_scope_pose_reapply). */
/* The code TMR13 is armed with at boot. NOT mid-scale: DAC1 centers CH1 at 2048
 * because it is a true 12-bit DAC, but TMR13 drives an RC filter, so its
 * centering code is a measured quantity. Our previous value was 2048 — the DAC
 * assumption — which put CH2 near the bottom of the window.
 *
 * MEASURED on bench unit #2 (2026-09-05, `ch2ref <code>` then the CH2 envelope
 * out of `dbg`, no signal on the probe, three dumps per point, envelope
 * midpoint):
 *
 *   code 2048 -> 70.5    code 2501 -> 128.5    code 2544 -> 133.5
 *   code 2480 -> 126.0   code 2520 -> 131.0    code 3072 -> 201.0
 *
 * so 2501 centers CH2 here, repeatable across two runs, bracketed by its
 * neighbours. CH1 read 74-77 at every one of those codes, so the reference is
 * channel-local — that is the control for this measurement.
 *
 * Upstream measured 2544 on THEIR unit #1 (DavidClawson, 0b7f9d09, 2026-08-27:
 * 2048 -> 65, 3072 -> 195). The slopes agree to three digits — 0.1274 ADC per
 * code here against their 0.127 — and the units differ only by a ~6 ADC offset,
 * which is why the codes differ by 43. So the mechanism transfers between units
 * and the constant does not: a unit that is not #2 should re-run the sweep.
 * Since 2026-09-06 the calibrated value lives in the settings page, in
 * scope_bias[1][range] alongside every other per-unit offset: ui_init() presets
 * it here and scope_hw_set_offsets()/scope_hw_configure_channels() keep it
 * following the range. What remains here is the value the timer holds before
 * any of that runs — the arm at fpga_init and the scope pose — so it tracks the
 * settings default rather than being a second, independent number. */
#ifndef FPGA53_TMR13_REF_CODE
#define FPGA53_TMR13_REF_CODE ((uint16_t)SETTINGS_SCOPE_BIAS_CH2_2C53T_DEFAULT)
#endif

static uint16_t fpga53_ch2_ref_code = FPGA53_TMR13_REF_CODE;
static uint8_t fpga53_tmr13_armed;

/* CH1's opposite number. DAC1 is a true 12-bit DAC, so mid-scale is at least a
 * defensible starting point — but "defensible" is not "measured", and on unit
 * #2 mid-scale leaves CH1 at ADC 75, not 128. The centering code is per-unit
 * exactly like CH2's and lives in scope_bias[0][range]; this is the value the
 * DAC holds before the first UI push. */
#ifndef FPGA53_DAC1_REF_CODE
#define FPGA53_DAC1_REF_CODE ((uint16_t)SETTINGS_SCOPE_BIAS_CH1_2C53T_DEFAULT)
#endif
static uint16_t fpga53_ch1_ref_code = FPGA53_DAC1_REF_CODE;
static uint8_t fpga53_dac1_armed;

/* Last value written to scope-engine register 0x08. The arm sequence puts 0xAD
 * there, so that is what the engine is running with until something changes
 * it. */
static uint8_t fpga53_trig_level = 0xADu;

static void fpga53_tmr13_ref_apply(void) {
#if FPGA53_TMR13_REF
    RCC_APB1ENR |= 1u << 7; // TMR13
    gpio_config_mask(GPIOA_BASE, 1u << 6, 0xBu); // PA6 AF push-pull
    REG32(0x40001C28u) = 0u;      // PSC
    REG32(0x40001C2Cu) = 4095u;   // ARR: 12-bit scale like the DAC
    REG32(0x40001C34u) = fpga53_ch2_ref_code;  // CCR1 (C1DT)
    REG32(0x40001C18u) = 0x68u;   // CCMR1: OC1M=PWM1, OC1PE
    REG32(0x40001C20u) = 0x1u;    // CCER: CC1E
    REG32(0x40001C14u) = 0x1u;    // EGR: UG (latch PSC/ARR/CCR)
    REG32(0x40001C00u) = 0x81u;   // CR1: ARPE | CEN
    fpga53_tmr13_armed = 1u;
#endif
}

/* Runtime CH2 reference. Two callers: the scope's vertical-offset path (which
 * on this board must reach TMR13, not DAC2 — see scope_dac_set_offsets) and the
 * shell's `ch2ref`, which exists so the centering code above can be measured
 * without a rebuild per candidate value. Arms TMR13 first if the boot path was
 * compiled out or has not run yet, so a shell probe works on any build. */
void fpga53_ch2_ref_set(uint16_t code) {
#if FPGA53_TMR13_REF
    if (code > 4095u) {
        code = 4095u;
    }
    fpga53_ch2_ref_code = code;
    if (!fpga53_tmr13_armed) {
        fpga53_tmr13_ref_apply();
    } else {
        REG32(0x40001C34u) = code;
    }
#else
    (void)code;
#endif
}

/* Adopt a stored code WITHOUT touching hardware. The boot path calls this from
 * ui_init(), where the settings record has just been read and the device may
 * still be about to come up in meter mode — arming TMR13 there would take PA6
 * away from the meter's gain key before the meter has even started. The value
 * sits in the module until whoever legitimately owns PA6 arms the timer. */
void fpga53_ch2_ref_preset(uint16_t code) {
    if (code > 4095u) {
        return;
    }
    fpga53_ch2_ref_code = code;
}

uint16_t fpga53_ch2_ref_get(void) {
    return fpga53_ch2_ref_code;
}

uint8_t fpga53_ch2_ref_armed(void) {
    return fpga53_tmr13_armed;
}

/* CH1's three, mirroring CH2's. The DAC needs no pin negotiation — PA4 is
 * nobody else's — so `set` can bring it up on any build without the ownership
 * question TMR13 has with the meter's gain key. */
void fpga53_ch1_ref_set(uint16_t code) {
    if (code > 4095u) {
        code = 4095u;
    }
    fpga53_ch1_ref_code = code;
    if (!fpga53_dac1_armed) {
        RCC_APB1ENR |= 1u << 29;
        gpio_config_mask(GPIOA_BASE, 1u << 4, 0x0);
        REG32(0x40007400u) |= 1u;
        fpga53_dac1_armed = 1u;
    }
    REG32(0x40007408u) = code;
}

void fpga53_ch1_ref_preset(uint16_t code) {
    if (code > 4095u) {
        return;
    }
    fpga53_ch1_ref_code = code;
}

uint16_t fpga53_ch1_ref_get(void) {
    return fpga53_ch1_ref_code;
}

uint8_t fpga53_ch1_ref_armed(void) {
    return fpga53_dac1_armed;
}

/* Write one scope-engine register, in the shape the arm sequence uses: SPI3
 * dropped to /256 (~470 kHz), one CS frame per register, the fast divider
 * restored afterwards. The five arm writes went out this way and are the only
 * evidence we have that these registers take at this clock; sending them at /8
 * never armed anything (journal, 2026-08-13). These are SCOPE-ENGINE opcodes,
 * not Gowin config-port opcodes, and writing them to a live design is
 * bench-proven safe.
 *
 * Register 0x08 is believed to be the digital trigger level, offset-binary,
 * with the arm sequence's 0xAD meaning level 0x2D. That reading came from this
 * project and has never been tested by changing it — which is what the shell's
 * `trigreg` exists for. */
uint8_t fpga53_scope_reg_write(uint8_t reg, uint8_t value) {
    uint32_t ctrl1_saved;

    if (!fpga_loaded) {
        return 0u;
    }
    ctrl1_saved = SPI_CTRL1(SPI3_BASE);
    SPI_CTRL1(SPI3_BASE) &= ~(1u << 6);                                /* SPE=0 */
    SPI_CTRL1(SPI3_BASE) = (SPI_CTRL1(SPI3_BASE) & ~(7u << 3)) | (7u << 3); /* /256 */
    SPI_CTRL1(SPI3_BASE) |= 1u << 6;                                   /* SPE=1 */

    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)fpga53_xfer(reg);
    (void)fpga53_xfer(value);
    gpio_set(GPIOB_BASE, 1u << 6);
    delay_ms(2);

    SPI_CTRL1(SPI3_BASE) &= ~(1u << 6);
    SPI_CTRL1(SPI3_BASE) = ctrl1_saved;
    SPI_CTRL1(SPI3_BASE) |= 1u << 6;
    if (reg == 0x08u) {
        fpga53_trig_level = value;
    }
    return 1u;
}

uint8_t fpga53_trig_level_get(void) {
    return fpga53_trig_level;
}

/* Stock's coarse relay/attenuator ladder, ONE table shared by both channels
 * under a pin isomorphism (upstream 4c8ae0b, both range functions traced
 * arm-by-arm from the stock image: gpio_mux_portc_porte @ 0x080088A4 for CH1,
 * gpio_mux_porta_portb @ 0x08008A58 for CH2):
 *
 *   bit0 = input PATH select   PC12 <-> PA15   HIGH = direct, LOW = attenuated
 *   bit1 =                     PE4  <-> PB11
 *   bit2 =                     PE5  <-> PB10
 *   bit3 =                     PE6  <-> PA10
 *
 * Two things this table says that our hand-written pose got wrong. First, the
 * two channels have INDEPENDENT banks — CH2's is not a spectator of CH1's.
 * Second, bit0 is the input path, not the coupling: LOW attenuates about 30x
 * rather than disconnecting. Our pose put CH1 on 0x0D (path direct) and left
 * CH2's pins at 0x02 (path attenuated, and 0x0D is not even a stock code), so
 * the same probe signal reached the two channels through paths differing by
 * that factor — which is what the bench measured on 2026-08-16 as "CH2 flat":
 * 6 counts of envelope against CH1's 189. */
static const uint8_t fpga53_relay_tbl[10] = {
    0x0Bu, 0x0Fu, 0x05u, 0x03u, 0x07u, 0x0Au, 0x0Eu, 0x0Cu, 0x02u, 0x06u,
};

/* The table IS the volts/div ladder. Measured on CH2 at three input amplitudes,
 * each row's gain taken as the SLOPE between two unclipped points so that the
 * noise floor — five counts at the coarse rows, fifteen at the sensitive ones —
 * drops out instead of inflating the small spans:
 *
 *   row        0     1     2     3     4     5     6     7     8     9
 *   mV/count   0.4   0.80  1.98  4.0   7.5   19.4  35.0  ~117  ~233  ~350
 *   1-2-5      0.4   0.8   2     4     8     20    40    80    200   400
 *
 * Row 4 came out 7.4 from the 100/300 mV pair and 7.69 from the 300/1000 pair,
 * independently — that agreement is what says the method is sound. Rows 7-9
 * rest on differences of six, three and two counts and are the loose end; they
 * need volts at the input, not millivolts.
 *
 * At 25 counts per division that ladder is exactly ten 1-2-5 steps, 10 mV/div
 * through 10 V/div, which is stock's own ladder. Our UI carries nine of them
 * from 20 mV up, so a volts/div index becomes a row by adding one, and row 0
 * is the sensitive step we do not offer yet. Nothing is fitted and nothing is
 * left over for software to make up: the relay ladder and the knob are the
 * same ladder.
 *
 * Rows 2 and 7 clear bit1, which drives PB11 LOW — the pin config and arm hold
 * HIGH. Upstream measured that an armed capture survives it and we now depend
 * on that at two volts/div settings; the pose runs after the arm writes, so if
 * a capture ever dies on those steps, this is the first place to look. */
enum { FPGA53_RELAY_COUNTS_PER_DIV = 25 };

/* Row per channel, as the knob last set it. The default sits mid-ladder for
 * the boot frames before the UI has configured anything. */
static uint8_t fpga53_relay_row[2] = {4u, 4u};

static void fpga53_relay_apply_ch1(uint8_t bits) {
    if (bits & 0x01u) {
        gpio_set(GPIOC_BASE, 1u << 12);
    } else {
        gpio_clear(GPIOC_BASE, 1u << 12);
    }
    if (bits & 0x02u) {
        gpio_set(GPIOE_BASE, 1u << 4);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 4);
    }
    if (bits & 0x04u) {
        gpio_set(GPIOE_BASE, 1u << 5);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 5);
    }
    if (bits & 0x08u) {
        gpio_set(GPIOE_BASE, 1u << 6);
    } else {
        gpio_clear(GPIOE_BASE, 1u << 6);
    }
    gpio_config_mask(GPIOC_BASE, 1u << 12, 0x1u);
    gpio_config_mask(GPIOE_BASE, (1u << 4) | (1u << 5) | (1u << 6), 0x1u);
}

static void fpga53_relay_apply_ch2(uint8_t bits) {
    if (bits & 0x01u) {
        gpio_set(GPIOA_BASE, 1u << 15);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 15);
    }
    if (bits & 0x02u) {
        gpio_set(GPIOB_BASE, 1u << 11);
    } else {
        gpio_clear(GPIOB_BASE, 1u << 11);
    }
    if (bits & 0x04u) {
        gpio_set(GPIOB_BASE, 1u << 10);
    } else {
        gpio_clear(GPIOB_BASE, 1u << 10);
    }
    if (bits & 0x08u) {
        gpio_set(GPIOA_BASE, 1u << 10);
    } else {
        gpio_clear(GPIOA_BASE, 1u << 10);
    }
    gpio_config_mask(GPIOA_BASE, (1u << 15) | (1u << 10), 0x1u);
    gpio_config_mask(GPIOB_BASE, (1u << 10) | (1u << 11), 0x1u);
    fpga53_diag.fe_ch2 = bits;
}

/* Relay-ladder sweep: what each row of the stock table is actually worth.
 *
 * The table is stock's own, but nothing in it says which row corresponds to
 * which volts/div — upstream measured the rows are not 1:1 with the ten vdiv
 * settings and that the fine gain lives in the digital layer. Our UI has nine
 * steps against the table's ten rows, so binding the knob to a row needs the
 * ladder measured rather than assumed: hold each row for a dwell, record the
 * envelope of CH2's trimmed window over the second half of it (the first half
 * is relay settling), and print the ten spans. With one fixed input on the
 * probe those spans ARE the attenuation ladder.
 *
 * CH2 only, CH1 left where it is, so the run has one variable. */
#ifndef FPGA53_RELAY_SWEEP
#define FPGA53_RELAY_SWEEP 0
#endif
#ifndef FPGA53_RELAY_DWELL
#define FPGA53_RELAY_DWELL 24
#endif

#if FPGA53_RELAY_SWEEP
/* Two sets: the pass being measured now, and the last one that completed.
 * A single pass per boot cannot be checked against anything — the first run of
 * this sweep recorded 20 counts for row 0 while the very same dump, on the
 * very same row, showed 63. Reporting only completed passes, and numbering
 * them, is what turns that into a reading someone can reproduce or refute. */
static uint8_t fpga53_rsw_min[10];
static uint8_t fpga53_rsw_max[10];
static uint8_t fpga53_rsw_live_min[10];
static uint8_t fpga53_rsw_live_max[10];
static uint8_t fpga53_rsw_row;
static uint8_t fpga53_rsw_dwell;
static uint8_t fpga53_rsw_pass;

void fpga53_relay_sweep_get(uint8_t row, uint8_t *code, uint8_t *mn, uint8_t *mx,
                            uint8_t *pass) {
    if (row > 9u) {
        row = 9u;
    }
    if (code) {
        *code = fpga53_relay_tbl[row];
    }
    if (mn) {
        *mn = fpga53_rsw_min[row];
    }
    if (mx) {
        *mx = fpga53_rsw_max[row];
    }
    if (pass) {
        *pass = fpga53_rsw_pass;
    }
}
#endif

void fpga53_set_channel_range(uint8_t ch, uint8_t vdiv_idx) {
    /* Our nine volts/div steps start at 20 mV, the ladder's row 1. */
    uint8_t row = (uint8_t)(vdiv_idx + 1u);

    if (ch > 1u) {
        return;
    }
    if (row > 9u) {
        row = 9u;
    }
    if (fpga53_relay_row[ch] == row) {
        return; /* relays: write only on a change, they are mechanical */
    }
    fpga53_relay_row[ch] = row;
    if (ch) {
        fpga53_relay_apply_ch2(fpga53_relay_tbl[row]);
    } else {
        fpga53_relay_apply_ch1(fpga53_relay_tbl[row]);
    }
}

uint16_t fpga53_range_uv_per_count(uint8_t vdiv_idx, uint32_t vdiv_mv) {
    (void)vdiv_idx;
    /* One division is 25 counts by construction of the ladder above, so the
     * volts/div setting alone fixes what a count is worth. Microvolts because
     * the sensitive steps are fractions of a millivolt per count. */
    return (uint16_t)((vdiv_mv * 1000u) / FPGA53_RELAY_COUNTS_PER_DIV);
}

/* Input coupling: PD12 (CH1) and PD13 (CH2), HIGH = DC.
 *
 * Our port has never written these pins — only PD2/PD3 — and the reset default
 * leaves PD12 in EXMC alternate function (address line A17), where ODR is
 * ignored and the pin sits LOW. Upstream found this on their unit (GPIOD CRH
 * read 0xBB4BBBBB) and measured what it costs: a permanently AC-coupled input
 * with a ~9 Hz high-pass, which they had spent a morning attributing to the
 * analog front end. Whether OUR unit boots the same way is what crh_boot in
 * the dump answers — it is sampled before this function runs. */
static void fpga53_fe_coupling_apply(void) {
#if FPGA53_FE_COUPLING
    gpio_set(GPIOD_BASE, (1u << 12) | (1u << 13));     /* both DC */
    gpio_config_mask(GPIOD_BASE, (1u << 12) | (1u << 13), 0x1u);
#endif
}

/* Analog frontend for scope mode: both relay banks from the stock table, the
 * coupling relays, and stock's scope-mode selector/mux posture (upstream Exp R
 * decode of the 4-way PC2:PC1 selector — stock takes the ==3 arm: PC2 HIGH,
 * PC1 LOW — and PC11 = meter MUX enable, HIGH only in meter mode). Cold boots
 * own the relay bank: floating coils leave the input path wherever it was. */
static void fpga53_fe_scope_pose_apply(void) {
#if FPGA53_FE_SCOPE_POSE
    fpga53_relay_apply_ch1(fpga53_relay_tbl[fpga53_relay_row[0]]);
    fpga53_relay_apply_ch2(fpga53_relay_tbl[fpga53_relay_row[1]]);
    fpga53_fe_coupling_apply();
    gpio_set(GPIOC_BASE, 1u << 2);                     /* PC2 HIGH */
    gpio_clear(GPIOC_BASE, (1u << 1) | (1u << 11));    /* PC1 LOW, PC11 LOW */
    gpio_config_mask(GPIOC_BASE, (1u << 1) | (1u << 2) | (1u << 11), 0x1u);
#endif
}

/* Re-apply everything the analog frontend needs for scope mode.
 *
 * Both blocks above used to live inline in the FPGA config path, i.e. they ran
 * once at boot. The meter applies its own posture on entry
 * (dmm53_frontend_baseline) and dmm_pause() clears only PC11, so a meter
 * round-trip left the relay bank (PE4/PE5) and the gain keys (PA15/PA10/PB9)
 * in meter positions, and PA6 a plain GPIO instead of the TMR13 PWM. Measured
 * on the bench 2026-08-14 with a DBGREQ dump taken after a meter visit:
 * PE4=1/PE5=0 where scope wants 0/1, PA15=1 and PA10=1 where scope wants 0/0,
 * while PC1/PC2/PC11/PC12 were already correct because dmm_pause() happens to
 * fix PC11.
 *
 * Called on scope-mode entry so the posture is the same no matter which mode
 * ran before. Both halves are idempotent absolute writes. */
void fpga53_scope_pose_reapply(void) {
    ++fpga53_diag.pose_calls;
    fpga53_tmr13_ref_apply();
    fpga53_fe_scope_pose_apply();
}

void fpga_init_once(void) {
    ++fpga53_diag.init_calls;
    if (fpga_loaded) {
        return;
    }
    /* Before any of our writes: how the boot left GPIOD's high half. The
     * coupling pins live there, and an alternate-function nibble means our
     * later BSRR writes to them are ignored. */
    fpga53_diag.crh_boot = GPIO_CRH(GPIOD_BASE);
    /* Configuration owns the bus outright — the slow-point sampler must not
     * clock a byte into the middle of the bitstream upload. */
    fpga53_bus_busy = 1u;

    RCC_APB1ENR |= 1u << 15; // SPI3
    AFIO_MAPR = (AFIO_MAPR & ~(7u << 24)) | (2u << 24); // release PB3/PB4/PB5 from JTAG

    /* Pre-load safe output levels BEFORE switching pin modes (no glitches):
     * CS idle HIGH, SPI-enable HIGH, active-mode HIGH — same levels stock
     * holds in scope mode, so the warm handoff does not disturb the FPGA. */
#if FPGA53_PB11_LOW_AT_CONFIG
    /* The one deviation from stock's levels, and the point of this build: PB11
     * goes out LOW here and stays LOW until the UI's first relay write. */
    gpio_set(GPIOB_BASE, 1u << 6);
    gpio_clear(GPIOB_BASE, 1u << 11);
#else
    gpio_set(GPIOB_BASE, (1u << 6) | (1u << 11));
#endif
    gpio_set(GPIOC_BASE, 1u << 6);
    gpio_config_mask(GPIOB_BASE, (1u << 6) | (1u << 11), 0x1u);
    gpio_config_mask(GPIOC_BASE, 1u << 6, 0x1u);

#if FPGA53_RUN_PINS
    /* PD3: stock holds it HIGH from SPI3 bring-up onward. */
    gpio_set(GPIOD_BASE, 1u << 3);
    gpio_config_mask(GPIOD_BASE, 1u << 3, 0x1u);
#endif

#if FPGA53_V04_CONFIG
    /* Attempt full FPGA configuration (V0.4 sequence) before bringing up
     * the SPI3 transport. Works from a cold boot if it works at all — and on
     * a warm boot it must not run at all, see the warm-token block above. */
    delay_ms(2); // PB11/PC6 settle (stock raises PB11 ~1ms before traffic)
    {
        uint32_t fingerprint = fpga53_bitstream_fingerprint();
#if FPGA53_BITSTREAM_EXTERN
        /* An unwritten or half-written store must never be clocked into the
         * FPGA — a garbage upload takes the config port down until the next
         * power cycle, and the part would lose a design it may already be
         * running happily. The header's fingerprint is what makes "the file
         * arrived whole" checkable at all. */
        fpga53_diag.bs_ok = (fpga53_bs_length() &&
                             fingerprint == fpga53_bs_stored_fingerprint()) ? 1u : 0u;
#else
        fpga53_diag.bs_ok = 1u;
#endif
#if FPGA53_WARM_SEED
        /* Bench seed build: flashed onto a device whose FPGA is known (from
         * its own telemetry) to be carrying this bitstream right now, so the
         * first boot may assume what it cannot yet prove — there is no token
         * until some build writes one. Still goes through fpga53_warm_boot()
         * for its side effect (clearing the reset-cause flags), then leaves a
         * real token so every later boot decides on evidence. */
        (void)fpga53_warm_boot(fingerprint);
        fpga53_diag.warm = 1u;
        fpga53_warm_token_write(1u, fingerprint);
#elif FPGA53_WARM_SKIP
        fpga53_diag.warm = fpga53_warm_boot(fingerprint);
#endif
        if (!fpga53_diag.warm && fpga53_diag.bs_ok) {
            fpga53_v04_configure();
            fpga53_warm_token_write(fpga53_cfg_ok, fingerprint);
        }
    }
#endif

#if FPGA53_RUN_PINS
    /* PD2 (scope-mode-entry level, run-line suspect) + PC4 (mode-flag-2
     * level): stock asserts both only after the upload — mirror that. */
    gpio_set(GPIOD_BASE, 1u << 2);
    gpio_config_mask(GPIOD_BASE, 1u << 2, 0x1u);
    gpio_set(GPIOC_BASE, 1u << 4);
    gpio_config_mask(GPIOC_BASE, 1u << 4, 0x1u);
#endif

    gpio_config_mask(GPIOB_BASE, (1u << 3) | (1u << 5), 0xBu); // SCK/MOSI AF push-pull
    gpio_config_mask(GPIOB_BASE, 1u << 4, 0x4u);               // MISO floating input
    gpio_config_mask(GPIOB_BASE, (1u << 6) | (1u << 11), 0x1u); // CS, active-mode
    gpio_config_mask(GPIOC_BASE, 1u << 6, 0x1u);               // SPI enable
#if FPGA53_PC0_READY_LOW
    /* PC0 input with PULL-UP (upstream cold rig): undriven reads HIGH = "not
     * ready", so a floating line cannot fake readiness; a configured FPGA
     * actively driving it LOW wins over the weak pull. */
    gpio_set(GPIOC_BASE, 1u << 0);                             // ODR=1 selects pull-UP
    gpio_config_mask(GPIOC_BASE, 1u << 0, 0x8u);               // PC0 input pull-up/down
#else
    gpio_config_mask(GPIOC_BASE, 1u << 0, 0x4u);               // PC0 data-ready input
#endif

    /* SPI mode 3, master, software NSS, 8-bit */
    SPI_CTRL1(SPI3_BASE) = 0;
    SPI_CTRL2(SPI3_BASE) = 0;
    SPI_CTRL1(SPI3_BASE) = (1u << 9) | (1u << 8) | (1u << 2) |
                           (((uint32_t)FPGA_SPI_BR & 7u) << 3) |
                           (1u << 1) | (1u << 0);
    SPI_CTRL1(SPI3_BASE) |= 1u << 6; // SPE

    /* CH1's vertical-offset reference: DAC1 (PA4, DHR12R1 @ 0x40007408). An
     * MCU reset zeroes it, which leaves the channel without a reference and
     * the frames empty — restoring it is what produced this port's first live
     * trace (journal, 2026-08-12). That entry calls DAC1 "the trigger level";
     * it is the OFFSET injector. Measured here 2026-09-06 the same way CH2's
     * TMR13 code was: the code moves the CH1 baseline linearly and does not
     * gate frames. The digital trigger level is a different thing, SPI3
     * register 0x08. */
    RCC_APB1ENR |= 1u << 29; // DAC
    gpio_config_mask(GPIOA_BASE, 1u << 4, 0x0); // PA4 analog
    REG32(0x40007400u) |= 1u;                   // DAC_CR: EN1
    REG32(0x40007408u) = fpga53_ch1_ref_code;   // DHR12R1
    fpga53_dac1_armed = 1u;

    fpga53_tmr13_ref_apply();

    /* Scope-mode SPI3 config writes + 0x03 status read (stock sends these
     * after configuration; reply 00 01 42 2E 2E). The 2026-08-12 run with
     * these enabled saw a dead bus (all-FF), but that run was confounded by
     * a USB replug power cycle that wiped the FPGA SRAM config — so this
     * path is UNTESTED on a live FPGA, not disproven. Default off: keep the
     * baseline warm handoff strictly read-only; enable with
     * FPGA53_SEND_CFG=1 for an A/B experiment. */
#if FPGA53_SEND_CFG
    {
#if FPGA53_SEND_CFG_DELAY_MS
        delay_ms(FPGA53_SEND_CFG_DELAY_MS);
#endif
        /* Upstream arm mechanics (issue #18 / guest-coldtrace, 2026-08-13):
         * the five writes take only when clocked SLOW — their working rig
         * switches SPI3 to /256 (~470 kHz) for the writes AND the 0x03 read,
         * with 2 ms CS-framed gaps, then restores the fast divider. All our
         * earlier attempts clocked them at /8 = 12 MHz and never armed. */
        uint32_t ctrl1_saved = SPI_CTRL1(SPI3_BASE);
        SPI_CTRL1(SPI3_BASE) &= ~(1u << 6); /* SPE=0 */
        SPI_CTRL1(SPI3_BASE) =
            (SPI_CTRL1(SPI3_BASE) & ~(7u << 3)) | (7u << 3); /* BR=/256 */
        SPI_CTRL1(SPI3_BASE) |= 1u << 6; /* SPE=1 */
        static const uint8_t cfg[5][2] = {
            {0x01u, 0x08u}, {0x02u, FPGA53_CFG02_VAL}, {0x06u, 0x00u},
            {0x07u, 0x00u}, {0x08u, 0xADu},
        };
        /* Stock's reply to the 0x03 status read once capture is armed. */
        static const uint8_t cst_ok[5] = {0x00u, 0x01u, 0x42u, 0x2Eu, 0x2Eu};
        for (uint8_t attempt = 0; attempt < FPGA53_SEND_CFG_RETRIES; ++attempt) {
            if (attempt) {
                delay_ms(200);
            }
            for (uint8_t i = 0; i < 5u; ++i) {
                gpio_clear(GPIOB_BASE, 1u << 6);
                (void)fpga53_xfer(cfg[i][0]);
                (void)fpga53_xfer(cfg[i][1]);
                gpio_set(GPIOB_BASE, 1u << 6);
                delay_ms(2);
            }
            gpio_clear(GPIOB_BASE, 1u << 6);
            fpga53_diag.cst[0] = fpga53_xfer(0x03u);
            fpga53_diag.cst[1] = fpga53_xfer(0xFFu);
            fpga53_diag.cst[2] = fpga53_xfer(0xFFu);
            fpga53_diag.cst[3] = fpga53_xfer(0xFFu);
            fpga53_diag.cst[4] = fpga53_xfer(0xFFu);
            gpio_set(GPIOB_BASE, 1u << 6);
            uint8_t match = 1u;
            for (uint8_t i = 0; i < 5u; ++i) {
                if (fpga53_diag.cst[i] != cst_ok[i]) {
                    match = 0;
                    break;
                }
            }
            if (match) {
                break;
            }
        }
        SPI_CTRL1(SPI3_BASE) &= ~(1u << 6); /* SPE=0 */
        SPI_CTRL1(SPI3_BASE) = ctrl1_saved & ~(1u << 6);
        SPI_CTRL1(SPI3_BASE) = ctrl1_saved; /* restore fast BR + SPE */
    }
#endif

    /* GPIOB the instant the arm sequence ends and before any pose or relay
     * write can move PB11 again. Without this the dump can only show the pin
     * where the volts/div row left it, which says nothing about the level the
     * config and the arm actually ran under. */
    fpga53_diag.idrb_arm = GPIO_IDR(GPIOB_BASE);

#if FPGA53_OP0A_PROBE
    /* Stock's 16-bit read: op 0x09 carries the high byte, then CS drops and a
     * second frame with opcode 0x0A carries the low one. Frame shape is stock's
     * own — opcode, one dummy, then the byte that matters — and both passes run
     * here, with the engine armed and before the pose, so nothing of ours has
     * touched the frontend between the config and the number. */
    for (uint8_t pass = 0; pass < 8u; ++pass) {
        /* Passes 4-7 repeat the whole thing at /256. Two cold boots on /8 gave
         * bytes that were reproducible but shaped nothing like upstream's, and
         * a one-bit step between passes is what this port's known readback skew
         * looks like — so the clock is the variable to move before the number
         * means anything. */
        if (pass == 4u) {
            SPI_CTRL1(SPI3_BASE) &= ~(1u << 6); /* SPE=0 */
            SPI_CTRL1(SPI3_BASE) =
                (SPI_CTRL1(SPI3_BASE) & ~(7u << 3)) | (7u << 3); /* BR=/256 */
            SPI_CTRL1(SPI3_BASE) |= 1u << 6; /* SPE=1 */
        }
        gpio_clear(GPIOB_BASE, 1u << 6);
        fpga53_diag.op09_bytes[pass][0] = fpga53_xfer(0x09u);
        fpga53_diag.op09_bytes[pass][1] = fpga53_xfer(0xFFu);
        fpga53_diag.op09_bytes[pass][2] = fpga53_xfer(0xFFu);
        gpio_set(GPIOB_BASE, 1u << 6);
        delay_ms(1);
        gpio_clear(GPIOB_BASE, 1u << 6);
        fpga53_diag.op0a_bytes[pass][0] = fpga53_xfer(0x0Au);
        fpga53_diag.op0a_bytes[pass][1] = fpga53_xfer(0xFFu);
        fpga53_diag.op0a_bytes[pass][2] = fpga53_xfer(0xFFu);
        gpio_set(GPIOB_BASE, 1u << 6);
        delay_ms(1);
    }
    /* Back to the working read clock, whatever the probe did. */
    SPI_CTRL1(SPI3_BASE) &= ~(1u << 6);
    SPI_CTRL1(SPI3_BASE) = (SPI_CTRL1(SPI3_BASE) & ~(7u << 3)) |
                           (((uint32_t)FPGA_SPI_BR & 7u) << 3);
    SPI_CTRL1(SPI3_BASE) |= 1u << 6;
#endif

    fpga53_fe_scope_pose_apply();

#ifdef FPGA53_TBIDX_SET
    /* Stock's one-byte "fast timebase config": the timebase index in its own
     * CS window. Bench 2026-08-15 found values 0x01-0x03 (and 0x13) change the
     * capture in a way nothing else has: with them the window catches signal
     * edges every dwell (max spread 0xB1), where the untouched engine never
     * caught one in twelve frames (spread 0x04). Whether that is the sample
     * rate dropping or a trigger aligning the capture is what a known input
     * frequency has to settle — the window's edge count says which. */
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)fpga53_xfer((uint8_t)FPGA53_TBIDX_SET);
    gpio_set(GPIOB_BASE, 1u << 6);
    delay_ms(2);
#endif

    fpga53_notready_polls = 0;
    fpga53_force_read = 0;
    fpga_loaded = 1u;
    fpga53_bus_busy = 0;
}

uint8_t fpga_ready(void) {
    return fpga_loaded;
}

#if HW_TARGET_2C53T
/* UI timebase step -> engine index, and what one frame-buffer entry is then
 * worth. Entry, not window sample: the renderer stretches the window 2x, so an
 * entry is half a sample interval.
 *
 * The mapping picks, for each step, the fastest index whose window still spans
 * the requested screen time — 831 trimmed samples, so 66 us at index 07 up to
 * 3.4 ms at index 0C. Steps below 5 us/div ask for less time than even the
 * fastest window holds and steps above 200 us/div for more than the slowest
 * does; both clamp, and there the trace spans what the engine gives rather
 * than what the label says.
 *
 * 500 us/div through 5 ms/div used to be uncovered — too slow for index 0C's
 * window, too fast for the roll sampler, which needs seconds. Indices 0x0D
 * through 0x10 fill it, measured against a 2 kHz input: 0.125, 0.05, 0.025 and
 * 0.0124 MSa/s. Against 50 kHz those same indices had returned two crossings
 * or fewer and were written off as possibly dead. That was aliasing, not a
 * dead capture: 40 us per sample against a 20 us period is far below Nyquist.
 *
 * 10 and 20 ms/div are covered by 0x11 and 0x12, measured against 20 Hz at
 * 5000 and 2490 Sa/s. The whole ladder, 0x07 to 0x13: 12.5 MSa/s, 5, 2.5,
 * 1.25, 0.5, 0.25, 0.125, 0.05, 0.025, 0.0124, then 5000, 2490 and 1194 Sa/s
 * — ten thousandfold, and 1-2-5 throughout. */
static const uint8_t fpga53_tb_index[] = {
    /* 50NS 100NS 200NS 500NS 1US 2US */
    0x07u, 0x07u, 0x07u, 0x07u, 0x07u, 0x07u,
    /* 5US  10US  20US  50US  100US 200US */
    0x07u, 0x08u, 0x09u, 0x0Au, 0x0Bu, 0x0Cu,
    /* 500US 1MS  2MS   5MS */
    0x0Du, 0x0Eu, 0x0Fu, 0x10u,
    /* 10MS 20MS */
    0x11u, 0x12u,
    /* 50MS onward belongs to the roll sampler; these entries only keep the
     * table as long as the UI's step list. Index 13's window is 696 ms and
     * would cover 50 ms/div, but it refreshes 1.4 times a second where roll
     * scrolls continuously — which is better there is a UI question, not a
     * measurement, so the floor stays put. */
    0x13u, 0x13u, 0x13u, 0x13u, 0x13u, 0x13u, 0x13u, 0x13u,
};

/* Nanoseconds per frame-buffer entry, indexed the same way — half a sample
 * interval, because of the 2x stretch. Index 0x0C read 0.0494 of the base rate
 * against a 50 kHz input, where it gets five samples per period, and an exact
 * 0.2500 MSa/s against a 2 kHz one, where it gets 125. The second measurement
 * is the one worth keeping. */
static const uint32_t fpga53_tb_entry_ns[] = {
    40u, 40u, 40u, 40u, 40u, 40u,
    40u, 100u, 200u, 400u, 1000u, 2000u,
    4000u, 10000u, 20000u, 40404u,
    /* 10MS 20MS. Index 0x11 is 105000 and not the 100000 a 1-2-5 ladder would
     * want: measured against 50 Hz with eight intervals in the window it comes
     * out at 210 us per sample, and the ratio to index 0x12 — which needs no
     * assumption about the generator — is 1.905 where a clean halving would be
     * 2.008. The 20 Hz sweep had said 200 us, but that reading had three
     * intervals to work with. Index 0x12 anchors the pair: 400.0 us from 50 Hz
     * over sixteen intervals, against 401.6 us from the independent 2 kHz run,
     * which is also what says the generator is honest. */
    105000u, 200803u,
    /* 50MS onward: roll territory, index 13's interval. */
    418848u, 418848u, 418848u, 418848u, 418848u, 418848u, 418848u, 418848u,
};

enum { FPGA53_TB_STEPS = sizeof(fpga53_tb_index) / sizeof(fpga53_tb_index[0]) };

static uint8_t fpga53_tb_now = 0xFFu;
static uint32_t fpga53_tb_entry_now = 100u;
/* What the setter has actually done. Two builds in a row disagreed with the
 * bench about where the engine was, and there was no way to tell a setter that
 * never ran from one whose command did not take. */
static uint16_t fpga53_tb_calls;   /* entries to fpga53_set_timebase */
static uint16_t fpga53_tb_skips;   /* entries that returned before writing */
static uint16_t fpga53_tb_sent;    /* `01 <idx>` commands written */
static uint8_t fpga53_tb_want = 0xFFu; /* index last asked for */

void fpga53_tb_debug(uint8_t *now, uint8_t *want, uint32_t *entry_ns,
                     uint16_t *calls, uint16_t *skips, uint16_t *sent) {
    if (now) { *now = fpga53_tb_now; }
    if (want) { *want = fpga53_tb_want; }
    if (entry_ns) { *entry_ns = fpga53_tb_entry_now; }
    if (calls) { *calls = fpga53_tb_calls; }
    if (skips) { *skips = fpga53_tb_skips; }
    if (sent) { *sent = fpga53_tb_sent; }
}

uint32_t fpga53_frame_entry_ns(void) {
    return fpga53_tb_entry_now;
}

void fpga53_set_timebase(uint8_t ui_timebase) {
    uint8_t idx;

    ++fpga53_tb_calls;
    if (!fpga_loaded) {
        ++fpga53_tb_skips;
        return;
    }
#if FPGA53_SWEEP_TB01
    /* The sweep owns `01 <idx>` in a sweep build. Leaving both writers on the
     * command means the UI silently overwrites whichever step the sweep is
     * dwelling on, and the table comes out measuring the knob. */
    (void)ui_timebase;
    return;
#else
    if (ui_timebase >= (uint8_t)FPGA53_TB_STEPS) {
        ui_timebase = (uint8_t)(FPGA53_TB_STEPS - 1u);
    }
    idx = fpga53_tb_index[ui_timebase];
    fpga53_tb_entry_now = fpga53_tb_entry_ns[ui_timebase];
    fpga53_tb_want = idx;
    if (idx == fpga53_tb_now) {
        ++fpga53_tb_skips;
        return;
    }

    /* One write, then let the engine fill a whole window at the new rate.
     *
     * A walk with a per-step dwell, and a descending homing pass to give the
     * walk a known start, lived here for an hour. Both were built on index
     * 0x11 reading 250 samples per period when reached one way and 205 the
     * other — which was not the rate changing but the estimate collapsing:
     * against a 20 Hz input that index fits three intervals in a window, and
     * one spurious crossing among four moves the answer by a fifth. Measured
     * again at 50 Hz, where the same window holds eight intervals, every path
     * gives the same 4.79 kSa/s. There is no path dependence, so there is no
     * reason to walk.
     *
     * Same bus etiquette as a capture read: a roll transfer may be in flight,
     * and clocking a command into someone else's CS frame is how a bus gets
     * wedged. */
    fpga53_bus_busy = 1u;
    fpga53_dma_cancel();
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)fpga53_xfer(0x01u);
    (void)fpga53_xfer(idx);
    gpio_set(GPIOB_BASE, 1u << 6);
    fpga53_tb_now = idx;
    ++fpga53_tb_sent;
    fpga53_bus_busy = 0;

    /* Let the engine fill one whole window at the new rate before anyone reads
     * it, or the first frame after a step is half old rate and half new. At
     * index 0C a window is 4.1 ms, which is why this is computed and not a
     * constant. */
    delay_ms((uint32_t)((fpga53_tb_entry_now * 2u * FPGA53_CH_SAMPLES) / 1000000u) + 2u);
#endif
}
#endif

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
#if FPGA53_READ_PACED
    /* Stock cadence: read unconditionally at the caller's loop rate. */
    fpga53_notready_polls = 0;
    return 1u;
#endif
#if FPGA53_PC0_READY_LOW
    if (!(GPIO_IDR(GPIOC_BASE) & 1u)) { // PC0 data-ready, active LOW (upstream cold rig)
#else
    if (GPIO_IDR(GPIOC_BASE) & 1u) { // PC0 data-ready
#endif
#if FPGA53_SWEEP_CFG01
        /* PC0 rose after a 01-register sweep write: freeze the sweep so the
         * winning value stays in the A field (shown as A<val>!). */
        if (fpga53_cfg01_started && !fpga53_diag.sweep_hit) {
            fpga53_diag.sweep_hit = 1u;
        }
#endif
        fpga53_notready_polls = 0;
        return 1u;
    }
#if FPGA53_SWEEP_CFG01
    /* Arm-bit hunt: while PC0 is dead, walk register 0x01 through all 256
     * values (stock sends 01 08). Two polls of dwell per value; skip until
     * the boot one-shot buffer has been consumed by the first read, so the
     * leftover ready level cannot fake a hit. */
    if (!fpga53_diag.sweep_hit && fpga53_diag.reads >= 1u) {
        static uint8_t cfg01_dwell;
        if (++cfg01_dwell >= 2u) {
            cfg01_dwell = 0;
            fpga53_diag.sweep_val = fpga53_cfg01_started
                                        ? (uint8_t)(fpga53_diag.sweep_val + 1u)
                                        : 0u;
            fpga53_cfg01_started = 1u;
            gpio_clear(GPIOB_BASE, 1u << 6);
            (void)fpga53_xfer(0x01u);
            (void)fpga53_xfer(fpga53_diag.sweep_val);
            gpio_set(GPIOB_BASE, 1u << 6);
        }
    }
#endif
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

/* Shape of the CH1 window, in the window's own sample units: how many times
 * the trace crosses its mid level going up, and the mean distance between
 * those crossings. Sample-rate experiments need a number, not an eyeball —
 * halve the rate and the period doubles, whatever the volts are doing.
 * Hysteresis at +-1/8 of the span keeps noise from manufacturing edges. */
/* Honours the same head/tail trim as the renderer and the seam analyser.
 * It did not until 2026-08-16, and the cost was visible: a glitch in the
 * contaminated head adds spurious crossings, which inflates the edge count and
 * deflates the mean period, so the same 50 kHz input reported T16=1478 in one
 * dump and 1600 in the next. A metric that disagrees with itself between dumps
 * cannot be the thing a sweep is judged by. */
/* Runs per channel, on whichever window is currently in fpga53_ch_buf: the two
 * reads share the buffer, so CH2's numbers have to be taken between its read
 * and the next CH1 read rather than at the end of the frame.
 *
 * It also carries the glitch count that used to live in the seam build alone.
 * That count is what decided FPGA53_HEAD_SKIP, and it only ever looked at CH1;
 * keeping it here, for both channels and in every scope build, is what makes
 * "did CH2 survive the read rewrite" answerable from one dump — with CH1 in
 * the same dump as the baseline, which is the only baseline worth trusting. */
static uint16_t fpga53_gl_max[2];
static uint16_t fpga53_gl_hit[2];
static uint16_t fpga53_gl_frames[2];
static uint32_t fpga53_gl_last[2];

/* Envelope of the trimmed window across frames, per channel, cleared every
 * time a dump reads it. One window is 166 us, so on a slow input it holds a
 * flat level and its own min-max says nothing about whether the channel is
 * alive — but the level wanders between frames, and the envelope catches that
 * wandering. It is what makes a 20 Hz input usable as a liveness test: the
 * channel with the probe on it swings, the other one sits still. */
static uint8_t fpga53_env_min[2] = {0xFFu, 0xFFu};
static uint8_t fpga53_env_max[2];

/* A crossing pair closer than this is the acquisition, not the input: at
 * 50 kHz and 5 MSa/s the grid runs 48-52 samples, and a real edge jitters by
 * one or two. Same constant the 2026-08-16 seam hunt used. */
enum { FPGA53_GLITCH_GAP = 30u };

static void fpga53_window_metrics(uint8_t ch) {
    uint8_t mn = 0xFFu;
    uint8_t mx = 0;
    uint8_t hi;
    uint8_t lo;
    uint8_t above;
    uint16_t edges = 0;
    uint16_t first = 0;
    uint16_t last = 0;
    uint16_t prev = 0;   /* previous crossing of EITHER polarity, 0 = none yet */
    uint8_t hit = 0;
    /* The engine hands the same window back four or five times before it
     * refills, so counting every read would inflate the denominator with
     * duplicates — and unequally between channels, since the two are read at
     * different points of the refill. The rolling sum of the whole window is
     * the freshness test the seam analyser already used. */
    uint8_t fresh = fpga53_win_sum[ch] != fpga53_gl_last[ch] ? 1u : 0u;

    fpga53_gl_last[ch] = fpga53_win_sum[ch];
    if (fresh) {
        ++fpga53_gl_frames[ch];
    }

    for (uint16_t i = FPGA53_HEAD_SKIP; i < FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP;
         ++i) {
        uint8_t v = fpga53_ch_buf[i];
        if (v < mn) {
            mn = v;
        }
        if (v > mx) {
            mx = v;
        }
    }
    if (ch) {
        fpga53_diag.wmin2 = mn;
        fpga53_diag.wmax2 = mx;
    } else {
        fpga53_diag.wmin = mn;
        fpga53_diag.wmax = mx;
    }
    if (fresh) {
        if (mn < fpga53_env_min[ch]) {
            fpga53_env_min[ch] = mn;
        }
        if (mx > fpga53_env_max[ch]) {
            fpga53_env_max[ch] = mx;
        }
    }
    if ((uint8_t)(mx - mn) < FPGA53_METRIC_MIN_SPREAD) {
        if (ch) {
            fpga53_diag.win_edges2 = 0;
            fpga53_diag.win_period2 = 0;
        } else {
            fpga53_diag.win_edges = 0;
            fpga53_diag.win_period = 0;
        }
        return;
    }

    hi = (uint8_t)(mn + (((uint16_t)(mx - mn) * 5u) / 8u));
    lo = (uint8_t)(mn + (((uint16_t)(mx - mn) * 3u) / 8u));
    above = fpga53_ch_buf[FPGA53_HEAD_SKIP] >= hi ? 1u : 0u;
    for (uint16_t i = FPGA53_HEAD_SKIP + 1u;
         i < FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP; ++i) {
        uint8_t v = fpga53_ch_buf[i];
        uint8_t crossed = 0;

        if (!above && v >= hi) {
            above = 1u;
            if (!edges) {
                first = i;
            } else {
                last = i;
            }
            ++edges;
            crossed = 1u;
        } else if (above && v <= lo) {
            above = 0;
            crossed = 1u;
        }
        /* Both polarities, unlike the period metric above: a glitch inside a
         * plateau shows as a falling crossing followed by a rising one a few
         * samples later, and counting rising edges alone goes blind to it. */
        if (crossed) {
            if (prev && (uint16_t)(i - prev) < FPGA53_GLITCH_GAP) {
                hit = 1u;
                if (fresh && prev > fpga53_gl_max[ch]) {
                    fpga53_gl_max[ch] = prev;
                }
            }
            prev = i;
        }
    }

    if (fresh && hit) {
        ++fpga53_gl_hit[ch];
    }

    if (ch) {
        fpga53_diag.win_edges2 = edges;
        fpga53_diag.win_period2 =
            (edges >= 2u)
                ? (uint16_t)(((uint32_t)(last - first) * 16u) / (edges - 1u))
                : 0u;
    } else {
        fpga53_diag.win_edges = edges;
        fpga53_diag.win_period =
            (edges >= 2u)
                ? (uint16_t)(((uint32_t)(last - first) * 16u) / (edges - 1u))
                : 0u;
    }
}

void fpga53_window_envelope(uint8_t ch, uint8_t *emin, uint8_t *emax) {
    if (ch > 1u) {
        ch = 1u;
    }
    if (emin) {
        *emin = fpga53_env_min[ch];
    }
    if (emax) {
        *emax = fpga53_env_max[ch];
    }
    /* Read and clear: an envelope that never resets saturates on the first
     * connect-disconnect and then reports the same span forever, which reads
     * exactly like a live channel. Each dump gets the span since the previous
     * one, so the bench gesture is "attach the probe, take a dump". */
    fpga53_env_min[ch] = 0xFFu;
    fpga53_env_max[ch] = 0;
}

void fpga53_glitch_stats(uint8_t ch,
                         uint16_t *gmax,
                         uint16_t *hit,
                         uint16_t *frames) {
    if (ch > 1u) {
        ch = 1u;
    }
    if (gmax) {
        *gmax = fpga53_gl_max[ch];
    }
    if (hit) {
        *hit = fpga53_gl_hit[ch];
    }
    if (frames) {
        *frames = fpga53_gl_frames[ch];
    }
}

#if FPGA53_SEAM_LOG
/* Per-frame seam record: the full list of crossings of the same hysteresis
 * band the window metric uses. On a clean capture every gap is the half-period
 * and the list is flat; a rotation of the engine's ring buffer puts exactly one
 * wrong gap in the list, at the wrap.
 *
 * BOTH polarities are recorded, unlike the window metric, which counts rising
 * crossings only. Simulated against a rotated ring (50 kHz, 5 MSa/s): rising
 * edges alone go blind whenever the wrap lands inside a plateau — which is
 * exactly the reported symptom, a plateau stretched half again — while both
 * polarities locate the wrap to +-13 samples over the whole window.
 *
 * Only frames the engine actually refreshed are kept. The bench showed each
 * window being handed back four or five times before a refill, so a ring of
 * raw reads spends most of itself on duplicates (2026-08-16).
 *
 * Runs off the bus, after the DMA read has released CS, so its cost races
 * nothing. */
enum { FPGA53_SEAM_ROWS = 12, FPGA53_SEAM_MAX_EDGES = FPGA53_SEAM_GAPS + 1u };

static fpga53_seam_row_t fpga53_seam_ring[FPGA53_SEAM_ROWS];
static uint8_t fpga53_seam_next;   /* next ring slot to write */
static uint8_t fpga53_seam_filled;
static uint32_t fpga53_seam_last_sum;
static uint8_t fpga53_seam_strip_buf[FPGA53_SEAM_STRIP];
static uint8_t fpga53_seam_head_buf[FPGA53_SEAM_HEAD];
/* Same window head, but kept only for frames the analyser found a glitch in.
 * The plain snapshot follows every read, and reads outnumber fresh frames four
 * to one, so it almost always shows a frame with nothing wrong with it. */
static uint8_t fpga53_seam_bad_buf[FPGA53_SEAM_HEAD];
static uint16_t fpga53_seam_bad_first;
static uint8_t fpga53_seam_bad_gap[4];
/* Where the glitches actually reach, counted over every fresh frame since
 * boot rather than eyeballed off the twelve rows a dump happens to hold. The
 * skip that hides them has to be chosen from this, not from a sample of it. */
static uint16_t fpga53_seam_gmax;    /* furthest sample a glitch started at */
static uint16_t fpga53_seam_gframes; /* fresh frames carrying a glitch */
static uint16_t fpga53_seam_frames;  /* fresh frames analysed */
static uint8_t fpga53_seam_pre[2];   /* r1, r2 of the last CH1 read */
/* Is r2 a sample? Counted over every fresh frame, not over the dozen a dump
 * holds. r1nz: frames whose r1 was anything but 00. jump: frames where r2 sits
 * further from the first window sample than a real edge can travel in one
 * sample step — the capture's own fall is six samples wide and its rise ten,
 * so 0x30 is far outside anything the input can do. */
static uint16_t fpga53_seam_r1nz;
static uint16_t fpga53_seam_jump;
static uint8_t fpga53_seam_band_v[4]; /* vmin, vmax, hi, lo of the last window */

/* Raw window, two views: every 15th sample for shape, and the first samples
 * one by one — the defect lives in the first period and a decimated view of it
 * is a view of nothing. */
static void fpga53_seam_raw_snapshot(void) {
    for (uint8_t i = 0; i < FPGA53_SEAM_STRIP; ++i) {
        fpga53_seam_strip_buf[i] =
            fpga53_ch_buf[(uint16_t)i * (FPGA53_CH_SAMPLES / FPGA53_SEAM_STRIP)];
    }
    for (uint8_t i = 0; i < FPGA53_SEAM_HEAD; ++i) {
        fpga53_seam_head_buf[i] = fpga53_ch_buf[i];
    }
}


static void fpga53_seam_note(void) {
    uint16_t edge[FPGA53_SEAM_MAX_EDGES];
    /* Filled in place, not built on the stack and copied: a struct assignment
     * of this size makes the compiler reach for __aeabi_memcpy, which a
     * freestanding build has no one to link against. */
    fpga53_seam_row_t *row = &fpga53_seam_ring[fpga53_seam_next];
    uint8_t vmin = 0xFFu;
    uint8_t vmax = 0;
    uint8_t n = 0;

    if (fpga53_win_sum[0] == fpga53_seam_last_sum) {
        return; /* same window handed back again: nothing new to record */
    }
    fpga53_seam_last_sum = fpga53_win_sum[0];

    row->first = 0;
    row->r2 = fpga53_diag.r2;
    row->count = 0;
    row->pre[0] = fpga53_seam_pre[0];
    row->pre[1] = fpga53_seam_pre[1];
    for (uint8_t i = 0; i < 4u; ++i) {
        row->pre[2u + i] = fpga53_seam_head_buf[i];
    }
    for (uint8_t i = 0; i < FPGA53_SEAM_GAPS; ++i) {
        row->gap[i] = 0;
    }

    for (uint16_t i = FPGA53_HEAD_SKIP; i < FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP;
         ++i) {
        uint8_t v = fpga53_ch_buf[i];
        if (v < vmin) {
            vmin = v;
        }
        if (v > vmax) {
            vmax = v;
        }
    }
    fpga53_seam_band_v[0] = vmin;
    fpga53_seam_band_v[1] = vmax;
    fpga53_seam_band_v[2] = 0;
    fpga53_seam_band_v[3] = 0;

    if ((uint8_t)(vmax - vmin) >= FPGA53_METRIC_MIN_SPREAD) {
        uint8_t hi = (uint8_t)(vmin + (((uint16_t)(vmax - vmin) * 5u) / 8u));
        uint8_t lo = (uint8_t)(vmin + (((uint16_t)(vmax - vmin) * 3u) / 8u));
        uint8_t above = fpga53_ch_buf[FPGA53_HEAD_SKIP] >= hi ? 1u : 0u;

        fpga53_seam_band_v[2] = hi;
        fpga53_seam_band_v[3] = lo;
        /* Starts where the render starts. The gap list is then a direct check
         * on what reaches the screen, not on a buffer nobody draws. */
        for (uint16_t i = FPGA53_HEAD_SKIP + 1u;
             i < FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP &&
             n < FPGA53_SEAM_MAX_EDGES;
             ++i) {
            uint8_t v = fpga53_ch_buf[i];
            if (!above && v >= hi) {
                above = 1u;
                edge[n++] = i;
            } else if (above && v <= lo) {
                above = 0;
                edge[n++] = i;
            }
        }
    }

    if (n) {
        row->first = edge[0];
        row->count = (uint8_t)(n - 1u);
        for (uint8_t i = 0; i < row->count; ++i) {
            uint16_t g = (uint16_t)(edge[i + 1u] - edge[i]);
            row->gap[i] = g > 255u ? 255u : (uint8_t)g;
        }
    }

    ++fpga53_seam_frames;
    if (row->pre[0]) {
        ++fpga53_seam_r1nz;
    }
    {
        int16_t d = (int16_t)row->pre[1] - (int16_t)row->pre[2];

        if (d < 0) {
            d = (int16_t)-d;
        }
        if (d > 0x30) {
            ++fpga53_seam_jump;
        }
    }
    {
        uint16_t at = row->first;
        uint8_t hit = 0;

        for (uint8_t i = 0; i < row->count; ++i) {
            if (row->gap[i] < 30u) {
                hit = 1u;
                if (at > fpga53_seam_gmax) {
                    fpga53_seam_gmax = at;
                }
            }
            at = (uint16_t)(at + row->gap[i]);
        }
        fpga53_seam_gframes = (uint16_t)(fpga53_seam_gframes + hit);
    }

    /* A glitch shows up as crossings far closer together than a half-period;
     * 30 samples sits well below the 48-52 the grid runs at and well above the
     * one-or-two-sample jitter of a real edge. */
    for (uint8_t i = 1; i < row->count; ++i) {
        /* From gap 1 on, not gap 0. The start-of-window transient always
         * produces a short gap 0 when it happens to cross the band, and that
         * case is already understood (raw head: one sample off the plateau,
         * then a ~10-sample exponential recovery). What is still unexplained
         * are the frames whose short gap sits at sample 40-70, well past any
         * settling — so those are the ones worth keeping a head for. */
        if (row->gap[i] < 30u) {
            for (uint8_t j = 0; j < FPGA53_SEAM_HEAD; ++j) {
                fpga53_seam_bad_buf[j] = fpga53_seam_head_buf[j];
            }
            fpga53_seam_bad_first = row->first;
            for (uint8_t j = 0; j < 4u; ++j) {
                fpga53_seam_bad_gap[j] = j < row->count ? row->gap[j] : 0;
            }
            break;
        }
    }

    fpga53_seam_next = (uint8_t)((fpga53_seam_next + 1u) % FPGA53_SEAM_ROWS);
    if (fpga53_seam_filled < FPGA53_SEAM_ROWS) {
        ++fpga53_seam_filled;
    }
}

const fpga53_seam_row_t *fpga53_seam_row(uint8_t i) {
    uint8_t oldest;

    if (i >= fpga53_seam_filled) {
        return 0;
    }
    oldest = (uint8_t)((fpga53_seam_next + FPGA53_SEAM_ROWS - fpga53_seam_filled) %
                       FPGA53_SEAM_ROWS);
    return &fpga53_seam_ring[(uint8_t)((oldest + i) % FPGA53_SEAM_ROWS)];
}

uint8_t fpga53_seam_rows(void) {
    return fpga53_seam_filled;
}

const uint8_t *fpga53_seam_strip(void) {
    return fpga53_seam_strip_buf;
}

const uint8_t *fpga53_seam_head(void) {
    return fpga53_seam_head_buf;
}

void fpga53_seam_pre_stats(uint16_t *r1nz, uint16_t *jump) {
    if (r1nz) {
        *r1nz = fpga53_seam_r1nz;
    }
    if (jump) {
        *jump = fpga53_seam_jump;
    }
}

void fpga53_seam_stats(uint16_t *gmax, uint16_t *gframes, uint16_t *frames) {
    if (gmax) {
        *gmax = fpga53_seam_gmax;
    }
    if (gframes) {
        *gframes = fpga53_seam_gframes;
    }
    if (frames) {
        *frames = fpga53_seam_frames;
    }
}

const uint8_t *fpga53_seam_bad_head(uint16_t *first, const uint8_t **gaps) {
    if (first) {
        *first = fpga53_seam_bad_first;
    }
    if (gaps) {
        *gaps = fpga53_seam_bad_gap;
    }
    return fpga53_seam_bad_buf;
}

void fpga53_seam_band(uint8_t *vmin, uint8_t *vmax, uint8_t *hi, uint8_t *lo) {
    if (vmin) {
        *vmin = fpga53_seam_band_v[0];
    }
    if (vmax) {
        *vmax = fpga53_seam_band_v[1];
    }
    if (hi) {
        *hi = fpga53_seam_band_v[2];
    }
    if (lo) {
        *lo = fpga53_seam_band_v[3];
    }
}
#endif

/* What the sweep walks.
 *
 * FPGA53_SWEEP_TBIDX: stock's "fast timebase config" as described in its own
 * decompilation — one byte in its own CS window carrying the timebase index,
 * range 0x00-0x13. This is the cheap candidate for the sample-rate divider we
 * have been unable to find anywhere else (the SPI3 register story below did
 * not survive: the stock image contains no 0x26/0x27/0x28 commands and no
 * evidence for 0x0F/0x10/0x11, see mydevice/STOCK-TIMEBASE-ANALYSIS.md).
 *
 * Otherwise: the old two-byte register ladder, kept because it costs nothing
 * and the machinery is shared. */
#if FPGA53_SWEEP_TB01
/* The command stock actually sends: 0x01 then the timebase index. */
static const uint8_t fpga53_tsweep_regs[] = { 0x01u };
static const uint8_t fpga53_tsweep_vals[] = {
    0x00u, 0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u, 0x07u, 0x08u, 0x09u,
    0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, 0x0Fu, 0x10u, 0x11u, 0x12u, 0x13u,
};
#elif FPGA53_SWEEP_TBIDX
static const uint8_t fpga53_tsweep_regs[] = { 0x00u }; /* unused: writes are one byte */
static const uint8_t fpga53_tsweep_vals[] = {
    0x00u, 0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u, 0x07u, 0x08u, 0x09u,
    0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, 0x0Fu, 0x10u, 0x11u, 0x12u, 0x13u,
};
#else
/* 0x06 and 0x07 — the two registers our own arm sequence sets to ZERO and
 * never thinks about again (`06 00, 07 00`, from the stock capture). The
 * upstream decompilation names USART commands 0x26 "timebase: prescaler" and
 * 0x27 "timebase: period"; the offset of 0x20 between those and these looks
 * like the same registers addressed from the other bus. If that reading is
 * right, the sample-rate divider has been sitting in our initialisation the
 * whole time, zeroed on every boot.
 *
 * (0x0F/0x10/0x11 stood here until 2026-08-15. Nothing in the stock image
 * supports them — see mydevice/STOCK-TIMEBASE-ANALYSIS.md — and the one-byte
 * timebase-index command measured as no-op, so the ladder moved here.) */
static const uint8_t fpga53_tsweep_regs[] = { 0x06u, 0x07u };
static const uint8_t fpga53_tsweep_vals[] = {
    0x00u, 0x01u, 0x02u, 0x04u, 0x08u, 0x10u, 0x20u, 0x40u, 0x80u, 0xFFu,
};
#endif

enum {
    FPGA53_TSWEEP_REGS = (uint8_t)(sizeof(fpga53_tsweep_regs)),
    FPGA53_TSWEEP_VALS = (uint8_t)(sizeof(fpga53_tsweep_vals)),
    FPGA53_TSWEEP_ROWS = FPGA53_TSWEEP_REGS * FPGA53_TSWEEP_VALS,
};

#if FPGA53_SWEEP_TIMING
static fpga53_tsweep_row_t fpga53_tsweep_rows[FPGA53_TSWEEP_ROWS];
static fpga53_tsweep_row_t fpga53_tsweep_base;
static uint8_t fpga53_tsweep_started;
static uint8_t fpga53_tsweep_dwell;

static void fpga53_tsweep_write(uint8_t row) {
    uint8_t reg = fpga53_tsweep_regs[row / FPGA53_TSWEEP_VALS];
    uint8_t val = fpga53_tsweep_vals[row % FPGA53_TSWEEP_VALS];

    fpga53_diag.tsweep_reg = reg;
    fpga53_diag.tsweep_val = val;
    gpio_clear(GPIOB_BASE, 1u << 6);
#if !FPGA53_SWEEP_TBIDX
    (void)fpga53_xfer(reg);
#endif
    (void)fpga53_xfer(val);
    gpio_set(GPIOB_BASE, 1u << 6);
}

/* Accumulate over the whole dwell rather than snapshotting the last frame.
 * A 33 us window on a 220 Hz square almost always sits on a flat part and
 * only occasionally catches an edge, so a single frame says little; the
 * MAXIMUM spread across a dwell rises as soon as the window starts spanning
 * more of the signal, which is the first sign of the rate being divided —
 * visible long before whole periods fit and edges can be counted. */
static void fpga53_tsweep_accumulate(fpga53_tsweep_row_t *row) {
    uint8_t spread = (uint8_t)(fpga53_diag.smax - fpga53_diag.smin);
    uint8_t edges = fpga53_diag.win_edges > 255u ? 255u : (uint8_t)fpga53_diag.win_edges;

    if (spread > row->spread) {
        row->spread = spread;
    }
    /* Last frame of the dwell, not the one with the most edges.
     *
     * Max-edges was right for the question it was written for — does anything
     * at all change here. It is wrong for measuring a rate: the winning frame
     * is whichever caught the most crossings, which is the frame that
     * straddled the switch between two indices, or one that caught a slow edge
     * wobbling through the hysteresis band. The 20 Hz run showed exactly that
     * — index 0E, independently measured at 50 kSa/s, came back with 17 edges
     * in a window that can only hold a third of a period.
     *
     * The caller only calls this past the dwell's halfway point, so the engine
     * has been at this index for a while by the time anything is recorded. */
    row->edges = edges;
    row->period = fpga53_diag.win_period;
}
#endif

/* Advance the timing sweep by one frame. Row N holds the window metric
 * measured while row N's (register, value) was in force; the baseline row is
 * taken before the first write, so a dead engine after some value is visible
 * as rows going flat and staying flat. */
static void fpga53_tsweep_step(void) {
#if FPGA53_SWEEP_TIMING
    if (fpga53_diag.tsweep_done) {
        return;
    }
    if (!fpga53_tsweep_started) {
        if (fpga53_tsweep_dwell >= (uint8_t)(FPGA53_SWEEP_TIMING_DWELL / 2u)) {
            fpga53_tsweep_accumulate(&fpga53_tsweep_base);
        }
        if (++fpga53_tsweep_dwell < (uint8_t)FPGA53_SWEEP_TIMING_DWELL) {
            return; /* baseline gets a full dwell of its own, same as a row */
        }
        fpga53_tsweep_started = 1u;
        fpga53_tsweep_dwell = 0;
        fpga53_tsweep_write(0);
        return;
    }
    /* First half of the dwell is settling: the engine may still be filling a
     * window at the previous rate, and at the slow indices one window is half
     * a second. */
    if (fpga53_tsweep_dwell >= (uint8_t)(FPGA53_SWEEP_TIMING_DWELL / 2u)) {
        fpga53_tsweep_accumulate(&fpga53_tsweep_rows[fpga53_diag.tsweep_row]);
    }
    if (++fpga53_tsweep_dwell < (uint8_t)FPGA53_SWEEP_TIMING_DWELL) {
        return;
    }
    fpga53_tsweep_dwell = 0;
    ++fpga53_diag.tsweep_row;
    if (fpga53_diag.tsweep_row >= (uint8_t)FPGA53_TSWEEP_ROWS) {
        fpga53_diag.tsweep_done = 1u;
        return;
    }
    fpga53_tsweep_write(fpga53_diag.tsweep_row);
#endif
}

const fpga53_tsweep_row_t *fpga53_tsweep_table(uint8_t *rows) {
#if FPGA53_SWEEP_TIMING
    if (rows) {
        *rows = (uint8_t)FPGA53_TSWEEP_ROWS;
    }
    return fpga53_tsweep_rows;
#else
    if (rows) {
        *rows = 0;
    }
    return 0;
#endif
}

const fpga53_tsweep_row_t *fpga53_tsweep_baseline(void) {
#if FPGA53_SWEEP_TIMING
    return &fpga53_tsweep_base;
#else
    return 0;
#endif
}

void fpga53_tsweep_axes(const uint8_t **regs,
                        uint8_t *reg_count,
                        const uint8_t **vals,
                        uint8_t *val_count) {
    if (regs) {
        *regs = fpga53_tsweep_regs;
    }
    if (reg_count) {
        *reg_count = (uint8_t)FPGA53_TSWEEP_REGS;
    }
    if (vals) {
        *vals = fpga53_tsweep_vals;
    }
    if (val_count) {
        *val_count = (uint8_t)FPGA53_TSWEEP_VALS;
    }
}

/*
 * Window read, DMA'd.
 *
 * The byte-polled version could not keep up with the engine. It moved 1023
 * bytes in 341 us at 24 MHz while the FPGA fills its window in 205 us, so the
 * writer overtook the reader inside every single read and the frame came back
 * with a seam — on a square wave, a plateau stretched half again with a sharp
 * dip where the phase jumps. Neither PC0 gating nor stock's ping-pong helped,
 * because every read tore the same way (bench 2026-08-15).
 *
 * Stock clocks SPI3 at 60 MHz and drains a window in 137 us, comfortably
 * ahead of the fill — that is why its trace is clean. We cannot reach that by
 * polling (a byte at 48 MHz is 16 core cycles), so the transfer goes to DMA
 * and the sample post-processing happens afterwards, off the bus, where its
 * cost no longer races anything.
 */
/* Decimated raw window per channel, kept for the dump.
 *
 * The bench on 2026-08-16 hit a case the summary numbers cannot settle: with
 * the probe on CH2 the metric reported eight crossings spread across the
 * window, while the screen showed one or two periods at the left edge and a
 * flat top for the rest. One of the two is wrong about what the window holds,
 * and neither min/max nor an edge count can say which — a decimated strip of
 * the samples themselves can. Raw, before the offset subtraction clamps the
 * low end to zero, and stepped across the trimmed window the renderer draws. */
enum { FPGA53_STRIP_LEN = 64 };
static uint8_t fpga53_strip[2][FPGA53_STRIP_LEN];

const uint8_t *fpga53_window_strip(uint8_t ch, uint8_t *len, uint8_t *step) {
    uint16_t s = (uint16_t)((FPGA53_CH_SAMPLES - FPGA53_HEAD_SKIP -
                             FPGA53_TAIL_SKIP) / FPGA53_STRIP_LEN);

    if (ch > 1u) {
        ch = 1u;
    }
    if (len) {
        *len = FPGA53_STRIP_LEN;
    }
    if (step) {
        *step = s > 255u ? 255u : (uint8_t)s;
    }
    return fpga53_strip[ch];
}

static void fpga53_strip_take(uint8_t ch) {
    uint16_t step = (uint16_t)((FPGA53_CH_SAMPLES - FPGA53_HEAD_SKIP -
                                FPGA53_TAIL_SKIP) / FPGA53_STRIP_LEN);

    if (!step) {
        step = 1u;
    }
    for (uint8_t i = 0; i < FPGA53_STRIP_LEN; ++i) {
        fpga53_strip[ch][i] =
            fpga53_ch_buf[(uint16_t)(FPGA53_HEAD_SKIP + (uint16_t)i * step)];
    }
}

static void fpga53_read_channel(uint8_t opcode) {
    uint8_t rmin = 0xFFu;
    uint8_t rmax = 0;
    uint32_t sum = 0;
    uint8_t r0, r1, r2;
    uint32_t guard = FPGA53_XFER_TIMEOUT;

    gpio_clear(GPIOB_BASE, 1u << 6); // CS assert
    r0 = fpga53_xfer(opcode);
    r1 = fpga53_xfer(0xFFu);
    r2 = fpga53_xfer(0xFFu);
    fpga53_dma_arm(FPGA53_CH_SAMPLES);
    while (!fpga53_dma_complete() && --guard) {
    }
    fpga53_dma_stop();
    gpio_set(GPIOB_BASE, 1u << 6); // CS deassert

#if FPGA53_SEAM_LOG
    /* Snapshot before the offset subtraction below. That subtraction clamps
     * everything at or under the offset to zero, which makes "the signal is at
     * its low level" and "these bytes are zeros" indistinguishable — and the
     * head of the window is exactly where that distinction is the question. */
    if (opcode == 0x04u) {
        fpga53_seam_raw_snapshot();
    }
#endif

    fpga53_strip_take(opcode == 0x04u ? 0u : 1u);

    for (uint16_t i = 0; i < FPGA53_CH_SAMPLES; ++i) {
        uint8_t raw = fpga53_ch_buf[i];
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

    if (opcode == 0x04u) {
        fpga53_diag.r0 = r0;
        fpga53_diag.r1 = r1;
        fpga53_diag.r2 = r2;
#if FPGA53_SEAM_LOG
        fpga53_seam_pre[0] = r1;
        fpga53_seam_pre[1] = r2;
#endif
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
    fpga53_bus_busy = 1u;
    /* A roll transfer may still be in flight if the UI just left a slow
     * timebase: take the bus back rather than clocking a window read into
     * someone else's CS frame. */
    fpga53_dma_cancel();
    fpga53_force_read = 0;
    ++fpga53_diag.reads;

#if FPGA53_DUAL_READ
    /* Stock RE dual-mode read (acquisition FSM case 4): one CS window,
     * 0xFF command byte, then a CH1 block followed by a CH2 block. Uses
     * 1024 samples per channel here. Diag: first window stats -> CH1 block,
     * second stats -> CH2 block. */
    {
        uint8_t v;
        uint8_t mn = 0xFFu, mx = 0;
        gpio_clear(GPIOB_BASE, 1u << 6);
        fpga53_diag.r0 = fpga53_xfer(0xFFu);
        for (uint16_t i = 0; i < 1024u; ++i) {
            v = fpga53_xfer(0xFFu);
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            int16_t c = (int16_t)v - (int16_t)FPGA53_ADC_OFFSET;
            if (c < 0) c = 0;
            fpga53_frame[(uint16_t)(((i * 2u) % FPGA_SAMPLE_COUNT) * 2u)] = (uint8_t)c;
            fpga53_frame[(uint16_t)((((i * 2u) + 1u) % FPGA_SAMPLE_COUNT) * 2u)] = (uint8_t)c;
        }
        fpga53_diag.smin = mn;
        fpga53_diag.smax = mx;
        mn = 0xFFu;
        mx = 0;
        for (uint16_t i = 0; i < 1024u; ++i) {
            v = fpga53_xfer(0xFFu);
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            int16_t c = (int16_t)v - (int16_t)FPGA53_ADC_OFFSET;
            if (c < 0) c = 0;
            fpga53_frame[(uint16_t)((((i * 2u) % FPGA_SAMPLE_COUNT) * 2u) + 1u)] = (uint8_t)c;
            fpga53_frame[(uint16_t)(((((i * 2u) + 1u) % FPGA_SAMPLE_COUNT) * 2u) + 1u)] = (uint8_t)c;
        }
        fpga53_diag.smin2 = mn;
        fpga53_diag.smax2 = mx;
        gpio_set(GPIOB_BASE, 1u << 6);
    }
    if (len > FPGA_SCOPE_BUFFER_BYTES) {
        len = FPGA_SCOPE_BUFFER_BYTES;
    }
    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = fpga53_frame[(uint16_t)(FPGA_SCOPE_BUFFER_BYTES - len + i)];
    }
    fpga53_bus_busy = 0;
    return 1u;
#endif

#if FPGA53_PRE_CMD
    /* Stock sends a one-byte pre-acquisition command 0x80|voltage_range in
     * its own CS window before reading (per the stock-firmware RE). Try a
     * mid-range value. */
    gpio_clear(GPIOB_BASE, 1u << 6);
    (void)fpga53_xfer(0x83u);
    gpio_set(GPIOB_BASE, 1u << 6);
#endif

#if FPGA53_SWEEP_CFG02
    /* Auto-sweep the value of config register 0x02 (stock sends 02 03 after
     * FPGA configuration — channel-mask suspect). Writes "02 <val>" once per
     * dwell step; freezes when the CH2 window spread crosses the threshold. */
    {
        enum { SWEEP02_DWELL_FRAMES = 6, SWEEP02_SPREAD_THRESHOLD = 12 };
        static uint8_t sweep02_frame;
        static uint8_t sweep02_started;

        if (!fpga53_diag.sweep_hit) {
            if ((uint8_t)(fpga53_diag.smax2 - fpga53_diag.smin2) >= SWEEP02_SPREAD_THRESHOLD) {
                fpga53_diag.sweep_hit = 1u;
            } else if (!sweep02_started || ++sweep02_frame >= SWEEP02_DWELL_FRAMES) {
                sweep02_frame = 0;
                fpga53_diag.sweep_val = sweep02_started
                                            ? (uint8_t)(fpga53_diag.sweep_val + 1u)
                                            : 0u;
                sweep02_started = 1u;
                gpio_clear(GPIOB_BASE, 1u << 6);
                (void)fpga53_xfer(0x02u);
                (void)fpga53_xfer(fpga53_diag.sweep_val);
                gpio_set(GPIOB_BASE, 1u << 6);
            }
        }
    }
#endif

#if FPGA53_SWEEP_PRECMD
    /* Auto-sweep the pre-command byte 0x80..0xFF, dwelling several frames
     * on each value. If the CH2 window's spread exceeds the threshold, the
     * sweep freezes so the winning value stays on screen (sweep_hit=1). */
    {
        enum { SWEEP_DWELL_FRAMES = 6, SWEEP_SPREAD_THRESHOLD = 12 };
        static uint8_t sweep_frame;

        if (!fpga53_diag.sweep_hit) {
            if (fpga53_diag.sweep_val < 0x80u) {
                fpga53_diag.sweep_val = 0x80u;
            }
            if ((uint8_t)(fpga53_diag.smax2 - fpga53_diag.smin2) >= SWEEP_SPREAD_THRESHOLD) {
                fpga53_diag.sweep_hit = 1u;
            } else if (++sweep_frame >= SWEEP_DWELL_FRAMES) {
                sweep_frame = 0;
                fpga53_diag.sweep_val = (uint8_t)(fpga53_diag.sweep_val == 0xFFu
                                                      ? 0x80u
                                                      : fpga53_diag.sweep_val + 1u);
            }
        }
        gpio_clear(GPIOB_BASE, 1u << 6);
        (void)fpga53_xfer(fpga53_diag.sweep_val);
        gpio_set(GPIOB_BASE, 1u << 6);
    }
#endif

    /* 1023 samples per channel, UI expects FPGA_SAMPLE_COUNT (2048)
     * interleaved pairs — stretch 2x (nearest neighbour). */
    /* Ping-pong: read each window twice and keep the second.
     *
     * A single read lands in the middle of the engine refilling its buffer, so
     * the frame carries a seam — on a square wave it shows up as a plateau
     * stretched by half again with a sharp dip where the phase jumps (bench
     * 2026-08-15). Gating on PC0 does not fix it and costs frame rate, which
     * matches what the bench found back on 08-13.
     *
     * Stock solves this by queueing TWO reads back to back and its own
     * decompilation names the scheme "ping-pong, prevents display tearing".
     * The mechanism fits what we measured: the first read drains the window
     * and re-arms the engine, the refill takes 205 us, and the second read
     * starts ~350 us later — after the buffer is whole again. */
#if FPGA53_PINGPONG
#if FPGA53_SWAP_ORDER
    fpga53_read_channel(0x05u);
#else
    fpga53_read_channel(0x04u);
#endif
#endif
#if FPGA53_SWAP_ORDER
    fpga53_read_channel(0x05u);
#else
    fpga53_read_channel(0x04u);
#endif
    /* Indexed by the trace the samples land in, not by the opcode: under
     * FPGA53_SWAP_ORDER the two reads trade places, and what a dump needs to
     * name is the channel the user sees on screen. */
    fpga53_window_metrics(0u);
#if FPGA53_SEAM_LOG
    fpga53_seam_note();
#endif
    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        uint16_t src = (uint16_t)((i >> 1) + FPGA53_HEAD_SKIP);
        if (src >= FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP) {
            src = FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP - 1u;
        }
        fpga53_frame[(uint16_t)(i * 2u)] = fpga53_ch_buf[src];
    }
#if FPGA53_PINGPONG
#if FPGA53_SWAP_ORDER
    fpga53_read_channel(0x04u);
#else
    fpga53_read_channel(0x05u);
#endif
#endif
#if FPGA53_SWAP_ORDER
    fpga53_read_channel(0x04u);
#else
    fpga53_read_channel(0x05u);
#endif
    fpga53_window_metrics(1u);
    for (uint16_t i = 0; i < FPGA_SAMPLE_COUNT; ++i) {
        uint16_t src = (uint16_t)((i >> 1) + FPGA53_HEAD_SKIP);
        if (src >= FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP) {
            src = FPGA53_CH_SAMPLES - FPGA53_TAIL_SKIP - 1u;
        }
        fpga53_frame[(uint16_t)(i * 2u + 1u)] = fpga53_ch_buf[src];
    }

#if FPGA53_RELAY_SWEEP
    /* One row per dwell, measured on the second half of it. Runs once and
     * parks the bank back on row 0, so the relays stop clicking and the device
     * is left in the pose the rest of the session expects. */
    {
        if (!fpga53_rsw_dwell) {
            fpga53_relay_apply_ch2(fpga53_relay_tbl[fpga53_rsw_row]);
            fpga53_rsw_live_min[fpga53_rsw_row] = 0xFFu;
            fpga53_rsw_live_max[fpga53_rsw_row] = 0;
        } else if (fpga53_rsw_dwell >= (uint8_t)(FPGA53_RELAY_DWELL / 2u)) {
            if (fpga53_diag.wmin2 < fpga53_rsw_live_min[fpga53_rsw_row]) {
                fpga53_rsw_live_min[fpga53_rsw_row] = fpga53_diag.wmin2;
            }
            if (fpga53_diag.wmax2 > fpga53_rsw_live_max[fpga53_rsw_row]) {
                fpga53_rsw_live_max[fpga53_rsw_row] = fpga53_diag.wmax2;
            }
        }
        if (++fpga53_rsw_dwell >= (uint8_t)FPGA53_RELAY_DWELL) {
            fpga53_rsw_dwell = 0;
            if (++fpga53_rsw_row >= 10u) {
                fpga53_rsw_row = 0;
                /* Publish the finished pass whole: a dump that catches the
                 * sweep mid-lap would otherwise mix rows from two passes and
                 * read as a ladder that changed shape. */
                for (uint8_t i = 0; i < 10u; ++i) {
                    fpga53_rsw_min[i] = fpga53_rsw_live_min[i];
                    fpga53_rsw_max[i] = fpga53_rsw_live_max[i];
                }
                ++fpga53_rsw_pass;
            }
        }
    }
#endif

    fpga53_tsweep_step();

    if (len > FPGA_SCOPE_BUFFER_BYTES) {
        len = FPGA_SCOPE_BUFFER_BYTES;
    }
    for (uint16_t i = 0; i < len; ++i) {
        dst[i] = fpga53_frame[(uint16_t)(FPGA_SCOPE_BUFFER_BYTES - len + i)];
    }
    fpga53_bus_busy = 0;
    return 1u;
}

/* One point of the MCU-paced slow timebase: the mean of the first few samples
 * of the current window, per channel. Called from the TMR1 IRQ, so it stays
 * short and gives the bus up rather than waiting for it. */
static uint8_t fpga53_slow_full = FPGA53_SLOW_POINT_FULL;

void fpga53_slow_point_set_full(uint8_t full) {
    fpga53_slow_full = full ? 1u : 0u;
    fpga53_diag.slow_full = fpga53_slow_full;
}

static uint8_t fpga53_read_point_channel(uint8_t opcode) {
    uint16_t take = (uint16_t)FPGA53_SLOW_POINT_SAMPLES;
    uint32_t sum = 0;
    int16_t cal;

    if (take > FPGA53_CH_SAMPLES) {
        take = FPGA53_CH_SAMPLES;
    }

    gpio_clear(GPIOB_BASE, 1u << 6); // CS assert
    (void)fpga53_xfer(opcode);
    (void)fpga53_xfer(0xFFu);
    (void)fpga53_xfer(0xFFu);
    for (uint16_t i = 0; i < take; ++i) {
        sum += fpga53_xfer(0xFFu);
    }
    if (fpga53_slow_full) {
        /* Drain the rest of the window so the engine sees exactly the byte
         * count of a normal read — the candidate condition for it to refresh
         * the window at all. */
        for (uint16_t i = take; i < FPGA53_CH_SAMPLES; ++i) {
            (void)fpga53_xfer(0xFFu);
        }
    }
    gpio_set(GPIOB_BASE, 1u << 6); // CS deassert

    cal = (int16_t)(sum / take) - (int16_t)FPGA53_ADC_OFFSET;
    if (cal < 0) {
        cal = 0;
    }
    return (uint8_t)cal;
}

/* Span of the CH1 points over a block: if the sampler is handing back a frozen
 * window, this collapses to a couple of counts no matter what the input does,
 * which separates "no signal in the points" from "wrong timebase" without
 * needing a photograph of the screen. */
static void fpga53_slow_track(uint8_t v) {
    static uint8_t mn = 0xFFu;
    static uint8_t mx;
    static uint16_t n;

    if (v < mn) {
        mn = v;
    }
    if (v > mx) {
        mx = v;
    }
    if (++n >= 256u) {
        fpga53_diag.slow_min = mn;
        fpga53_diag.slow_max = mx;
        mn = 0xFFu;
        mx = 0;
        n = 0;
    }
}

/*
 * DMA sampler.
 *
 * A polled point costs 0.97 ms of CPU at 24 MHz (measured: cost=97 ticks of
 * 10 us), and that is CPU the UI and the USB volume do not get — at 40% duty
 * the host could not even mount the volume. The transfer itself is only going
 * to get so short; what has to go is the CPU sitting in a byte loop waiting
 * for it.
 *
 * So each window goes out over DMA2 (channel 1 receives into the sample
 * buffer, channel 2 feeds 0xFF out — the same controller stock drives for
 * SPI3), and the point is assembled across two pacer ticks: one tick starts a
 * channel's transfer and returns in microseconds, the next collects it and
 * starts the other channel. The pacer therefore runs at twice the point rate
 * (see scope_hw_slow_start), and no DMA interrupt vector is needed — which
 * matters, because the vector numbering on this AT32 clone is not something
 * we have documentation for.
 *
 * If a transfer never completes (wrong channel mapping being the obvious
 * risk), the watchdog below gives up after a few ticks and falls back to
 * polled reads permanently, so a bad guess degrades to yesterday's behaviour
 * instead of a dead sampler. fpga53_diag.dma_fail says which mode is live.
 */
/* How many samples of the window a point clocks out. MEASURED, do not shorten:
 * the engine refreshes its window only when the WHOLE frame has been drained.
 * A 512-sample drain (half the window) was tried on the bench 2026-08-15 and
 * the point span collapsed to p=92-93 — one frozen value — against p=00-94
 * with the full 1023. So it is the frame that re-arms the engine, not a byte
 * count, and this transfer cannot be made cheaper by asking for less of it. */
#ifndef FPGA53_DRAIN_SAMPLES
#define FPGA53_DRAIN_SAMPLES FPGA53_CH_SAMPLES
#endif

enum {
    FPGA53_DMA_RX_CH = 1u, /* SPI3_RX */
    FPGA53_DMA_TX_CH = 2u, /* SPI3_TX */
    FPGA53_DMA_TCIF_RX = 1u << 1, /* TCIF1 */
    FPGA53_DMA_CCR_EN = 1u << 0,
    FPGA53_DMA_CCR_DIR_M2P = 1u << 4,
    FPGA53_DMA_CCR_MINC = 1u << 7,
    FPGA53_DMA_CCR_PL_HIGH = 2u << 12,
    FPGA53_SPI_RXDMAEN = 1u << 0,
    FPGA53_SPI_TXDMAEN = 1u << 1,
    FPGA53_DMA_WATCHDOG_TICKS = 8u,

    FPGA53_PT_IDLE = 0u,
    FPGA53_PT_CH_A = 1u,
    FPGA53_PT_CH_B = 2u,
};

static uint8_t fpga53_dma_tx_pattern = 0xFFu;
static uint8_t fpga53_pt_state;
static uint8_t fpga53_pt_wait;
static uint8_t fpga53_pt_first;
static uint8_t fpga53_dma_polled; /* 1 = watchdog tripped, stay on polled reads */

static void fpga53_dma_stop(void) {
    DMA2_CCR(FPGA53_DMA_RX_CH) = 0;
    DMA2_CCR(FPGA53_DMA_TX_CH) = 0;
    SPI_CTRL2(SPI3_BASE) &= ~(FPGA53_SPI_RXDMAEN | FPGA53_SPI_TXDMAEN);
    DMA2_IFCR = 0x0FFu; /* channels 1-2 flags */
}

/* Program both channels for <count> bytes into fpga53_ch_buf and let them go.
 * CS and the opcode are the caller's business — the window read frames its own
 * transfer, and the roll sampler frames a different one. */
static void fpga53_dma_arm(uint16_t count) {
    RCC_AHBENR |= 1u << 1; // DMA2

    fpga53_dma_stop();
    DMA2_CPAR(FPGA53_DMA_RX_CH) = (uint32_t)(uintptr_t)&SPI_DT(SPI3_BASE);
    DMA2_CMAR(FPGA53_DMA_RX_CH) = (uint32_t)(uintptr_t)fpga53_ch_buf;
    DMA2_CNDTR(FPGA53_DMA_RX_CH) = count;
    DMA2_CPAR(FPGA53_DMA_TX_CH) = (uint32_t)(uintptr_t)&SPI_DT(SPI3_BASE);
    DMA2_CMAR(FPGA53_DMA_TX_CH) = (uint32_t)(uintptr_t)&fpga53_dma_tx_pattern;
    DMA2_CNDTR(FPGA53_DMA_TX_CH) = count;
    /* Receive side first: the byte that arrives is the answer to the byte the
     * transmit side is about to clock out, so it must already be armed. */
    DMA2_CCR(FPGA53_DMA_RX_CH) = FPGA53_DMA_CCR_MINC | FPGA53_DMA_CCR_PL_HIGH |
                                 FPGA53_DMA_CCR_EN;
    DMA2_CCR(FPGA53_DMA_TX_CH) = FPGA53_DMA_CCR_DIR_M2P | FPGA53_DMA_CCR_EN;
    SPI_CTRL2(SPI3_BASE) |= FPGA53_SPI_RXDMAEN | FPGA53_SPI_TXDMAEN;
}

static void fpga53_dma_start(uint8_t opcode) {
    gpio_clear(GPIOB_BASE, 1u << 6); // CS assert, held for the whole window
    (void)fpga53_xfer(opcode);
    (void)fpga53_xfer(0xFFu);
    (void)fpga53_xfer(0xFFu);
    fpga53_dma_arm(FPGA53_DRAIN_SAMPLES);
}

static uint8_t fpga53_dma_complete(void) {
    return (DMA2_ISR & FPGA53_DMA_TCIF_RX) ? 1u : 0u;
}

/* Close out a finished transfer and return the point value: the mean of the
 * first samples, calibrated like the fast path. */
static uint8_t fpga53_dma_collect(void) {
    uint16_t take = (uint16_t)FPGA53_SLOW_POINT_SAMPLES;
    uint32_t sum = 0;
    int16_t cal;

    fpga53_dma_stop();
    gpio_set(GPIOB_BASE, 1u << 6); // CS deassert

    if (take > FPGA53_CH_SAMPLES) {
        take = FPGA53_CH_SAMPLES;
    }
    for (uint16_t i = 0; i < take; ++i) {
        sum += fpga53_ch_buf[i];
    }
    cal = (int16_t)(sum / take) - (int16_t)FPGA53_ADC_OFFSET;
    if (cal < 0) {
        cal = 0;
    }
    return (uint8_t)cal;
}

/* Drop an in-flight transfer and release the bus, leaving the sampler ready to
 * start a fresh point. Safe to call when nothing is in flight. */
static void fpga53_dma_cancel(void) {
    if (fpga53_pt_state == FPGA53_PT_IDLE) {
        return;
    }
    fpga53_dma_stop();
    gpio_set(GPIOB_BASE, 1u << 6);
    fpga53_pt_state = FPGA53_PT_IDLE;
}

static void fpga53_dma_give_up(void) {
    fpga53_dma_stop();
    gpio_set(GPIOB_BASE, 1u << 6);
    fpga53_pt_state = FPGA53_PT_IDLE;
    fpga53_dma_polled = 1u;
    fpga53_diag.dma_fail = 1u;
}

/* Returns 1 when sample[] holds a finished point. A 0 means "not this tick" —
 * with the DMA sampler that is the normal case on half the ticks, not an
 * error. */
uint8_t fpga_capture_read_slow_point(uint8_t sample[2]) {
    if (!sample || !fpga_loaded) {
        return 0;
    }
    if (fpga53_bus_busy) {
        ++fpga53_diag.slow_busy;
        return 0;
    }

    if (fpga53_dma_polled) {
        fpga53_bus_busy = 1u;
#if FPGA53_SWAP_ORDER
        sample[0] = fpga53_read_point_channel(0x05u);
        sample[1] = fpga53_read_point_channel(0x04u);
#else
        sample[0] = fpga53_read_point_channel(0x04u);
        sample[1] = fpga53_read_point_channel(0x05u);
#endif
        fpga53_slow_track(sample[0]);
        ++fpga53_diag.slow_points;
        fpga53_bus_busy = 0;
        return 1u;
    }

#if FPGA53_SWAP_ORDER
    const uint8_t op_a = 0x05u, op_b = 0x04u;
#else
    const uint8_t op_a = 0x04u, op_b = 0x05u;
#endif

    /* NB: an in-flight transfer is NOT signalled through fpga53_bus_busy. That
     * flag means "the main loop owns the bus", and holding it across ticks
     * made every following tick bail out at the check above — the state
     * machine advanced once and stopped (bench 2026-08-15: ROLL n=1). The
     * transfer's own state lives in fpga53_pt_state, and fpga_capture_read
     * cancels it if the main loop ever needs the bus mid-flight. */
    switch (fpga53_pt_state) {
    case FPGA53_PT_IDLE:
        fpga53_pt_wait = 0;
        fpga53_dma_start(op_a);
        fpga53_pt_state = FPGA53_PT_CH_A;
        return 0;

    case FPGA53_PT_CH_A:
        if (!fpga53_dma_complete()) {
            if (++fpga53_pt_wait >= FPGA53_DMA_WATCHDOG_TICKS) {
                fpga53_dma_give_up();
            }
            return 0;
        }
        fpga53_pt_first = fpga53_dma_collect();
        fpga53_pt_wait = 0;
        fpga53_dma_start(op_b);
        fpga53_pt_state = FPGA53_PT_CH_B;
        return 0;

    default:
        if (!fpga53_dma_complete()) {
            if (++fpga53_pt_wait >= FPGA53_DMA_WATCHDOG_TICKS) {
                fpga53_dma_give_up();
            }
            return 0;
        }
        sample[0] = fpga53_pt_first;
        sample[1] = fpga53_dma_collect();
        fpga53_pt_state = FPGA53_PT_IDLE;
        fpga53_slow_track(sample[0]);
        ++fpga53_diag.slow_points;
        return 1u;
    }
}

#else /* !HW_TARGET_2C53T */

enum {
    FPGA_LATCH_SETTLE_MS = 1,
    SPI_CAPTURE_TIMEOUT = 60000u,

    SPI_STS_RXNE = 1u << 0,
    SPI_STS_TXE = 1u << 1,
    SPI_STS_BSY = 1u << 7,
};

#ifndef FPGA53_V04_CONFIG
#define FPGA53_V04_CONFIG 0
#endif

#ifndef FPGA53_SWEEP_PRECMD
#define FPGA53_SWEEP_PRECMD 0
#endif

#ifndef FPGA53_SWEEP_CFG02
#define FPGA53_SWEEP_CFG02 0
#endif

/* Arm-bit hunt: sweep register 0x01's value with PC0-pulse autodetect. */
#ifndef FPGA53_SWEEP_CFG01
#define FPGA53_SWEEP_CFG01 0
#endif

/* Hold the stock-driven-but-unmapped pins HIGH: PD3 (static HIGH from SPI3
 * bring-up), PD2 (asserted on scope-mode entry — top run-line candidate),
 * PC4 (mode-flag-2 level). Post-config run relevance was untestable until a
 * live self-configured FPGA existed (unmapped_mcu_fpga_pin_candidates.md). */
#ifndef FPGA53_RUN_PINS
#define FPGA53_RUN_PINS 0
#endif

/* CH2 trigger reference via TMR13 CH1 PWM on PA6 (upstream static-analysis
 * lead, issue #18 2026-08-12): mid-scale duty at boot. */
#ifndef FPGA53_TMR13_REF
#define FPGA53_TMR13_REF 0
#endif

#ifndef FPGA53_SWAP_ORDER
#define FPGA53_SWAP_ORDER 0
#endif

#ifndef FPGA53_PRE_CMD
#define FPGA53_PRE_CMD 0
#endif

#ifndef FPGA53_SEND_CFG
#define FPGA53_SEND_CFG 0
#endif

#ifndef FPGA53_CFG02_VAL
#define FPGA53_CFG02_VAL 0x03u
#endif

#ifndef FPGA53_DUAL_READ
#define FPGA53_DUAL_READ 0
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
