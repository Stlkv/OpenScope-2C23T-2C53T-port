/*
 * OpenScope 2C53T - Meter Data Parser
 *
 * Parses FPGA USART2 RX frames to extract multimeter readings.
 * The FPGA sends 12-byte data frames (0x5A 0xA5 header + 10 data bytes)
 * containing BCD-encoded measurement digits and status flags.
 *
 * Based on RE analysis:
 *   - reverse_engineering/analysis_v120/FPGA_TASK_ANALYSIS.md
 *   - Functions: meter_data_processor (0x08036AC0) and
 *                meter_mode_handler   (0x080371B0)
 */

#ifndef METER_DATA_H
#define METER_DATA_H

#include <stdint.h>
#include <stdbool.h>

#include "meter_plan.h"

/* Decode submodes, and the width of every per-submode table in meter_data.c.
 * 0..9 are the eight stock meter families as the decoder sees them (DCV, ACV,
 * DC mA, DC A, AC mA, AC A, Ohm, continuity, diode, capacitance); 10 is the
 * local temperature split on the extended slot. This IS meter_plan's local
 * submode numbering — one definition, so the two cannot drift. */
#define METER_SUBMODE_COUNT FPGA_METER_LOCAL_SUBMODE_COUNT

/* ═══════════════════════════════════════════════════════════════════
 * Result Classification
 * ═══════════════════════════════════════════════════════════════════ */

typedef enum {
    METER_RESULT_NONE      = 0,  /* No data yet */
    METER_RESULT_NORMAL    = 1,  /* Valid reading */
    METER_RESULT_UNDERRANGE= 2,  /* Value too small for current range */
    METER_RESULT_OVERRANGE = 3,  /* Exceeds display, not OL */
    METER_RESULT_INVALID   = 4,  /* Unrecognized data */
    METER_RESULT_OVERLOAD  = 5,  /* "OL" — input overloaded */
    METER_RESULT_BLANK     = 6,  /* No measurement (blank display) */
    METER_RESULT_CONTINUITY= 7,  /* Continuity detected */
} meter_result_class_t;

/* ═══════════════════════════════════════════════════════════════════
 * Parsed Meter Reading
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    /* Parsed measurement value */
    float    value;              /* Scaled measurement value */
    int      raw_bcd;            /* Raw BCD integer (0-9999, or 10000-19999
                                  * when the stock +10000 extension fires —
                                  * see raw_bcd_extended) */
    bool     raw_bcd_extended;   /* frame[2].3 seen: stock adds 10000 to the
                                  * four-digit raw in the DCV value path */
    uint8_t  digits[4];          /* Individual BCD digits */
    uint8_t  decimal_pos;        /* Decimal point position (0=none, 1-3) */
    bool     negative;           /* Negative polarity */

    /* Display string (pre-formatted for UI) */
    char     display_str[16];    /* e.g., "-13.82", "OL", "---" */

    /* Unit suffix string for the current range.
     * Points to a static string (never NULL). Example values:
     *   DCV:         "V", "mV"
     *   ACV:         "V", "mV"
     *   DCA/ACA:     "A", "mA", "uA"
     *   Resistance:  "Ohm", "kOhm", "MOhm"
     *   Frequency:   "Hz", "kHz", "MHz"
     *   Capacitance: "nF", "uF"
     *   Diode:       "V"
     */
    const char *unit_suffix;

    /* Unit variant (0..2) within the current submode — mirrors the
     * stock meter_mode_handler's DAT_2000102e. Used to select between
     * e.g. mA / A / uA in the same DCA mode. */
    uint8_t  unit_variant;

    /* Bar graph fraction (0.0 - 1.0) */
    float    bar_fraction;

    /* Classification */
    meter_result_class_t result_class;

    /* Status flags from RX frame byte [7] */
    bool     is_ac;              /* AC mode active */
    bool     is_auto_range;      /* Auto-range enabled */
    bool     is_hold;            /* Hold mode active */

    /* DCV exponent class taken from the stock status bits
     * (frame[8].7 / frame[3].4 / frame[4].4 / frame[5].4, in that priority):
     * 0..4, or 0xFF when the stock DCV path did not run for this frame.
     * Reported on the meter debug line — a wrong DCV reading is only
     * diagnosable if we can see which class the frame claimed. */
    uint8_t  dcv_stock_class;

    /* Meter mode handler state (from frame[6]/[7] status bits) */
    uint8_t  probe_type;         /* 0, 1, or 2 — from frame[7] bit pattern */
    uint8_t  range_indicator;    /* from frame[6] bits 4-5: range band */
    uint8_t  range_cmd;          /* Parameter for auto-range FPGA commands */

    /* Continuity buzzer state */
    bool     continuity_beep;    /* Should buzzer sound */

    /* Validity */
    bool     valid;              /* At least one successful parse */
    uint32_t update_count;       /* Incremented on each new reading */

    /* Debug: raw frame bytes and pre-lookup nibble pairs */
    uint8_t  dbg_frame[12];     /* Last raw USART RX frame */
    uint8_t  dbg_nibbles[4];    /* Pre-lookup nibble pair values */
    uint8_t  dbg_raw_digits[4]; /* Post-lookup digit codes (before masking) */

} meter_reading_t;

/* Debug: distinct values of frame[6] seen within the current session.
 * Up to 8 unique byte values stored; new values push out the oldest.
 * Used by the meter UI debug strip to visualize how many different
 * frame types the FPGA is sending per measurement. */
#define METER_F6_HISTORY_LEN 8

/* ═══════════════════════════════════════════════════════════════════
 * Meter session
 *
 * One session = the decoder's life between mode transitions. It owns
 * everything the decoder accumulates: the reading, the sticky band
 * latch (dp/unit decided from a recognized frame[6] band and reused
 * for unrecognized frames of the same session), and the f6 history.
 * meter_session_begin() is the only reset — the submode is fixed for
 * the whole session, so a re-entry into the same mode starts clean
 * instead of inheriting the previous session's latch (which is what
 * the old function-local statics did).
 *
 * The owner is dmm53.c (one static session on the firmware side);
 * host tests hold their own on the stack.
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t         submode;         /* fixed at begin(); meter_plan local
                                      * submode numbering */
    meter_reading_t reading;

    /* Sticky band latch (resistance/continuity frame[6] bands). */
    bool            band_has_latch;
    uint8_t         band_latch_dp;
    const char     *band_latch_unit;

    /* Distinct frame[6] values seen this session. */
    uint8_t         f6_history[METER_F6_HISTORY_LEN];
    uint8_t         f6_history_count;
} meter_session_t;

/* ═══════════════════════════════════════════════════════════════════
 * API
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * Start a decoder session for one meter submode. Clears the reading
 * (display "---", unit ""), the band latch and the f6 history. Call on
 * every mode transition (and re-entry), before the first frame.
 *
 * submode: meter sub-mode (0..METER_SUBMODE_COUNT-1) for decimal point
 *          placement; out-of-range values fall back to dp=1, no unit
 */
void meter_session_begin(meter_session_t *s, uint8_t submode);

/*
 * Process a complete FPGA USART2 RX data frame (12 bytes) under the
 * session's submode. Extracts BCD digits, detects special codes (OL,
 * continuity, blank), assembles the measurement value, and updates
 * s->reading.
 *
 * Call this from the USART RX task when a valid data frame arrives.
 *
 * frame: pointer to 12-byte RX frame (0x5A 0xA5 + 10 data bytes)
 */
void meter_session_frame(meter_session_t *s, const volatile uint8_t *frame);

#endif /* METER_DATA_H */
