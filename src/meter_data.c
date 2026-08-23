/*
 * OpenScope 2C53T - Meter Data Parser
 *
 * Implements BCD digit extraction from FPGA USART2 RX frames,
 * following the stock firmware's meter_data_processor (0x08036AC0)
 * and meter_mode_handler (0x080371B0).
 *
 * USART2 RX data frame format (12 bytes):
 *   [0] = 0x5A  (header byte 1)
 *   [1] = 0xA5  (header byte 2)
 *   [2]-[6] = packed BCD nibble pairs (measurement digits)
 *   [7] = status flags (AC, auto-range, overload, polarity)
 *   [8]-[9] = additional status
 *   [10]-[11] = extra data (range info)
 *
 * BCD extraction: digits are encoded as cross-byte nibble pairs:
 *   digit0 = lookup((rx[2] & 0xF0) | (rx[3] & 0x0F))
 *   digit1 = lookup((rx[3] & 0xF0) | (rx[4] & 0x0F))
 *   digit2 = lookup((rx[4] & 0xF0) | (rx[5] & 0x0F))
 *   digit3 = lookup((rx[5] & 0xF0) | (rx[6] & 0x0F))
 */

#include "meter_data.h"

/* Ported into the freestanding 23t build (no libc): local shims replace
 * <string.h>. Source: upstream 53t firmware drivers/meter_data.c (GPL),
 * taken verbatim apart from these shims and the HW_TARGET_2C53T guard. */
#if HW_TARGET_2C53T

#ifndef NULL
#define NULL ((void *)0)
#endif

static void *md_memset(void *dst, int c, unsigned long n) {
    unsigned char *d = (unsigned char *)dst;
    while (n--) {
        *d++ = (unsigned char)c;
    }
    return dst;
}

static char *md_strcpy(char *dst, const char *src) {
    char *d = dst;
    while ((*d++ = *src++) != '\0') {
    }
    return dst;
}

static void *md_memmove(void *dst, const void *src, unsigned long n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        while (n--) {
            *d++ = *s++;
        }
    } else if (d > s) {
        d += n;
        s += n;
        while (n--) {
            *--d = *--s;
        }
    }
    return dst;
}

#define memset md_memset
#define strcpy md_strcpy
#define memmove md_memmove

/* No file-scope decoder state: everything the decoder accumulates lives in
 * the caller's meter_session_t (see meter_data.h), so the owner of the mode
 * transition — dmm53.c on the firmware side, the harness in host tests —
 * also owns the reset. */

/* ═══════════════════════════════════════════════════════════════════
 * BCD Nibble Lookup — extracted from stock firmware FUN_08033EF8
 *
 * The FPGA contains a meter IC core (likely FS9922 or similar Chinese
 * multimeter ASIC) that outputs 7-segment LCD drive signals rather
 * than clean BCD. The cross-byte nibble pairs encode which LCD
 * segments are lit, using this scrambled bit mapping:
 *
 *   input bit 0 → segment d (bottom)
 *   input bit 1 → segment c (lower right)
 *   input bit 2 → segment g (middle bar)
 *   input bit 3 → segment b (upper right)
 *   input bit 4 → (unused, masked off by AND 0xEF)
 *   input bit 5 → segment e (lower left)
 *   input bit 6 → segment f (upper left)
 *   input bit 7 → segment a (top)
 *
 * The stock firmware reverses this encoding back to digit values
 * using a function with TBB/TBH jump tables. This lookup table is
 * the equivalent, extracted by simulating FUN_08033EF8 for all 256
 * input values against the V1.2.0 firmware binary.
 *
 * Output codes: 0-9 = BCD digits, 0x0A = OL_hi, 0x0B = OL_lo,
 *   0x0C/0x0D/0x0E/0x0F = special, 0x10 = blank, 0x11 = partial,
 *   0x12 = continuity, 0x13 = mode change, 0x14 = special,
 *   0xFF = invalid/unmapped.
 *
 * Since bit 4 is masked, rows 0x1n and 0x0n are identical, etc.
 * ═══════════════════════════════════════════════════════════════════ */

static const uint8_t bcd_lookup[256] = {
    0x10, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* 0x00-0x0F */
    0x10, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* 0x10-0x1F */
    0xFF, 0xFF, 0xFF, 0x0B, 0x14, 0xFF, 0xFF, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* 0x20-0x2F */
    0xFF, 0xFF, 0xFF, 0x0B, 0x14, 0xFF, 0xFF, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* 0x30-0x3F */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0xFF,  /* 0x40-0x4F */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0xFF,  /* 0x50-0x5F */
    0xFF, 0x0E, 0xFF, 0xFF, 0xFF, 0x0C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* 0x60-0x6F */
    0xFF, 0x0E, 0xFF, 0xFF, 0xFF, 0x0C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* 0x70-0x7F */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x03,  /* 0x80-0x8F */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x03,  /* 0x90-0x9F */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFF, 0xFF,  /* 0xA0-0xAF */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFF, 0xFF,  /* 0xB0-0xBF */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09,  /* 0xC0-0xCF */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09,  /* 0xD0-0xDF */
    0xFF, 0x11, 0xFF, 0xFF, 0xFF, 0x13, 0xFF, 0x06, 0xFF, 0xFF, 0xFF, 0x00, 0x12, 0xFF, 0x0A, 0x08,  /* 0xE0-0xEF */
    0xFF, 0x11, 0xFF, 0xFF, 0xFF, 0x13, 0xFF, 0x06, 0xFF, 0xFF, 0xFF, 0x00, 0x12, 0xFF, 0x0A, 0x08   /* 0xF0-0xFF */
};

static uint8_t bcd_nibble_lookup(uint8_t combined)
{
    return bcd_lookup[combined];
}

/* ═══════════════════════════════════════════════════════════════════
 * Decimal point placement per sub-mode
 *
 * Based on the result formatting switch in meter_data_processor:
 *   Sub-mode 0 (DC V):   X.XXX  → decimal at position 1 (after 1st digit)
 *   Sub-mode 1 (AC V):   XX.XX  → decimal at position 2
 *   Sub-mode 2 (DC mA):  XX.XX  → decimal at position 2
 *   Sub-mode 3 (DC A):   X.XXX  → decimal at position 1
 *   Sub-mode 4 (AC mA):  XX.XX  → decimal at position 2
 *   Sub-mode 5 (AC A):   X.XXX  → decimal at position 1
 *   Sub-mode 6 (Ohm):    X.XXX  → decimal at position 1
 *   Sub-mode 7 (Cont):   XXX.X  → decimal at position 3
 *   Sub-mode 8 (Diode):  X.XXX  → decimal at position 1
 *   Sub-mode 9 (Cap):    XX.XX  → decimal at position 2
 *
 * The actual decimal position depends on the auto-range state,
 * but these are the defaults for the most common range.
 * ═══════════════════════════════════════════════════════════════════ */

/* Default decimal position per submode (index 0 = no decimal, 1-3 = after nth digit).
 * Empirically tuned from hardware readings:
 *   DCV (0): 1-10V range, raw 9899 → 9.899 V, decimal after digit 1
 *   ACV (1): similar
 *   Resistance (6): 20k range, raw 9899 → 98.99 kΩ, decimal after digit 2
 *   Continuity (7): 200Ω range, raw 16 → 1.6 Ω, decimal after digit 3
 *   Diode (8): 2V range, raw 623 → 0.623 V, decimal after digit 1
 *   Capacitance (9): 200nF range, raw 1034 → 103.4 nF, decimal after digit 3
 */
static const uint8_t default_decimal_pos[METER_SUBMODE_COUNT] = {
    1,  /* 0: DCV       — 9.899 V */
    2,  /* 1: ACV       — 98.99 V */
    2,  /* 2: DCA (mA)  — 98.99 mA */
    1,  /* 3: DCA (A)   — 9.899 A */
    2,  /* 4: ACA (mA)  — 98.99 mA */
    1,  /* 5: ACA (A)   — 9.899 A */
    2,  /* 6: Resistance— 98.99 kΩ */
    3,  /* 7: Continuity— 198.9 Ω */
    1,  /* 8: Diode     — 0.623 V */
    3,  /* 9: Capacitance— 198.9 nF */
    1,  /* 10: Temperature — no bench reading yet; keeps the old out-of-range
         * fallback so the row costs nothing it hasn't earned */
};

/* Full-scale values per sub-mode (for bar graph calculation) */
static const float bar_full_scale[METER_SUBMODE_COUNT] = {
    20.0f,    /* DC V: 20V */
    200.0f,   /* AC V: 200V */
    200.0f,   /* DC mA: 200mA */
    10.0f,    /* DC A: 10A */
    200.0f,   /* AC mA: 200mA */
    10.0f,    /* AC A: 10A */
    20.0f,    /* Ohm: 20kOhm */
    200.0f,   /* Cont: 200 Ohm */
    2.0f,     /* Diode: 2V */
    200.0f,   /* Cap: 200nF */
    1000.0f,  /* Temp: same fallback the >=10 branch used */
};

/* ═══════════════════════════════════════════════════════════════════
 * Low-Ω band factory calibration
 *
 * Bench truth on unit #1 (2026-04-04):
 *   147 Ω ref → raw_bcd ≈ 4830/4831 → true_ohms = raw_bcd × 0.0304
 *
 * The FPGA meter IC's low-Ω range outputs raw BCD counts that need a
 * fixed linear correction. The correction factor is independent of
 * whichever dp/unit interpretation the frame's f6 byte happens to
 * suggest — for a 147 Ω input this unit reports raw 4830/4831 and
 * we apply the same factor regardless.
 *
 * Detection: resistance/continuity submode (6 or 7) with frame[6]
 * upper nibble == 0. Bench captures (2026-04-04) show the FPGA
 * rotates through 0x07, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F in this band.
 * Upper nibble 4 (0x40, 0x4B, 0x4D) is the kΩ band and is already
 * accurate without correction.
 *
 * The 0.0304 factor is per-device (factory calibrated). When the
 * SPI flash driver lands it should come from "3:/System file/..."
 * — see flash_fs.c:220 and the open question in
 * analysis_v120/fpga_h2_spi3_bulk.md about whether the stock boot's
 * 115,638-byte SPI3 bulk cal upload is what supplies these values.
 *
 * Bench data:
 *   147 Ω ref  → pre-fix: 48.36 Ω / 4.831 kΩ (flickering)
 *                post-fix: 147 Ω (stable)
 *   3.3 kΩ ref → 3.230 kΩ (unchanged, f6 upper nibble 4)
 *   10 kΩ ref  → 9.840 kΩ (unchanged)
 *   5 V DC ref → 5.008 V  (unchanged, different submode)
 * ═══════════════════════════════════════════════════════════════════ */

#define METER_CAL_LOW_OHM_FACTOR  0.0304f   /* raw_bcd × this = Ω, low band */
#define METER_CAL_KOHM_FACTOR     0.001f    /* raw_bcd × this = kΩ, mid band */

/* ═══════════════════════════════════════════════════════════════════
 * 4-digit float → string formatter (newlib-nano has no %f support)
 *
 * Writes the positive value `v` into `s` as a 4-significant-digit
 * decimal with the decimal point placed for maximum precision.
 * Caller is responsible for the leading sign. Buffer must have room
 * for up to 6 chars + null terminator.
 * ═══════════════════════════════════════════════════════════════════ */

static void format_4digit_unsigned(float v, char *s)
{
    int whole, frac, dp;

    if (v < 10.0f) {
        /* 0.000 - 9.999 → "X.XXX" */
        whole = (int)v;
        frac  = (int)((v - (float)whole) * 1000.0f + 0.5f);
        if (frac >= 1000) { whole++; frac = 0; }
        dp = 3;
    } else if (v < 100.0f) {
        /* 10.00 - 99.99 → "XX.XX" */
        whole = (int)v;
        frac  = (int)((v - (float)whole) * 100.0f + 0.5f);
        if (frac >= 100) { whole++; frac = 0; }
        dp = 2;
    } else if (v < 1000.0f) {
        /* 100.0 - 999.9 → "XXX.X" */
        whole = (int)v;
        frac  = (int)((v - (float)whole) * 10.0f + 0.5f);
        if (frac >= 10) { whole++; frac = 0; }
        dp = 1;
    } else {
        /* 1000 - 9999 → "XXXX" (clamped) */
        whole = (int)(v + 0.5f);
        if (whole > 9999) whole = 9999;
        frac  = 0;
        dp    = 0;
    }

    int pos = 0;
    if (whole >= 1000) s[pos++] = (char)('0' + (whole / 1000) % 10);
    if (whole >= 100)  s[pos++] = (char)('0' + (whole / 100)  % 10);
    if (whole >= 10)   s[pos++] = (char)('0' + (whole / 10)   % 10);
    s[pos++] = (char)('0' + whole % 10);

    if (dp > 0) {
        s[pos++] = '.';
        if (dp == 3) {
            s[pos++] = (char)('0' + (frac / 100) % 10);
            s[pos++] = (char)('0' + (frac / 10)  % 10);
            s[pos++] = (char)('0' + (frac % 10));
        } else if (dp == 2) {
            s[pos++] = (char)('0' + (frac / 10) % 10);
            s[pos++] = (char)('0' + (frac % 10));
        } else {
            s[pos++] = (char)('0' + (frac % 10));
        }
    }
    s[pos] = '\0';
}

/* ═══════════════════════════════════════════════════════════════════
 * Unit suffix table
 *
 * Indexed by [submode][unit_variant]. Variant 0 is the only variant
 * the stock firmware actually uses for submodes 1-7 — see
 * reverse_engineering/analysis_v120/meter_fsm_deep_dive.md Q2: the
 * variant state (stock `DAT_2000102e`) is only written inside the
 * DCV FSM case (case 0 of meter_mode_handler at 0x080371B0), and it
 * persists from there into whatever submode runs next. Other modes
 * read it as a side-effect but never drive it.
 *
 * Variants 1/2 for submodes 1-7 are placeholder strings that will
 * never be selected at runtime today. They're left here as
 * documentation targets for when we wire up per-submode range
 * feedback in a future phase.
 *
 * Strings are ASCII-only so the font renderer doesn't need Greek
 * mu/ohm glyphs.
 * ═══════════════════════════════════════════════════════════════════ */

/* Rows 2/4/5/10 follow upstream meter_data.c as it stands after their August
 * rework: submode 5 is the local AC-A slot, not a frequency mode — the "Hz"
 * row this table used to carry came from an early guess and it is what made
 * AC HIGH CURR read out in hertz. The uA variants are gone with it: stock
 * V1.2.0 exposes no microamp selector (see meter_plan.c), so a uA suffix
 * could only ever mislabel a mA reading. */
static const char * const unit_suffix_table[METER_SUBMODE_COUNT][3] = {
    /*                v0       v1       v2    */
    /* 0 DCV      */ { "V",    "mV",    "mV"   },
    /* 1 ACV      */ { "V",    "mV",    "mV"   },
    /* 2 DCA(mA)  */ { "mA",   "mA",    "mA"   },
    /* 3 DCA(A)   */ { "A",    "A",     "A"    },
    /* 4 ACA(mA)  */ { "mA",   "mA",    "mA"   },
    /* 5 ACA(A)   */ { "A",    "A",     "A"    },
    /* 6 Ohm      */ { "Ohm",  "kOhm",  "MOhm" },
    /* 7 Cont     */ { "Ohm",  "Ohm",   "Ohm"  },
    /* 8 Diode    */ { "V",    "V",     "V"    },
    /* 9 Cap      */ { "nF",   "uF",    "uF"   },
    /* 10 Temp    */ { "C",    "C",     "F"    },
};

/* ═══════════════════════════════════════════════════════════════════
 * DCV decimal exponent from stock status bits
 *
 * Ported from upstream meter_data.c (GPL) as merged on 2026-08-13 —
 * PR #13 (Komzpa) plus that day's hardening pass; the functions there are
 * voltage_range_hint_from_stock_frame, dcv_stock_class_from_frame,
 * apply_stock_dcv_voltage_range_hint, format_stock_decimal_value and
 * apply_stock_dcv_decimal_exponent. Table and bit priority are theirs
 * verbatim; the wiring below is adapted to this decoder's field names.
 *
 * WHY THIS REPLACES OUR frame[6] GUESS. Our DCV decimal point came from
 * default_decimal_pos[0] = 1, confirmed only in the 1-10 V band, plus an
 * empirical "frame[6] == 0x0F -> dp 1" case that said the same thing. That
 * carried the bench failure recorded on 2026-04-04 (unit #1): at 11 V in,
 * frame[6] rotates through {0x0A, 0x0F, 0x0B, 0x07} and raw 997 with a fixed
 * dp of 1 renders 0.997 V. The exponent was never in frame[6]. Stock reads it
 * out of four status bits, in priority order, and divides the extended raw
 * value by 10^class:
 *
 *   frame[8].7 -> class 4,  frame[3].4 -> class 3,
 *   frame[4].4 -> class 2,  frame[5].4 -> class 1,  otherwise class 0.
 *
 * Source for the priority: their analysis_v120/meter_math_pipeline_annotated.c
 * (the annotated RX pass); the later DAT_2000102f/DAT_20001030 formatter is in
 * the raw full_decompile.c. Bytes 10..11 are NOT part of this decision — in
 * live voltage frames they are only an auxiliary/line-frequency hint.
 *
 * PREDICTIONS, fixed before the bench sees this (each one falsifies the port):
 *   1. The band that already worked must be unchanged: 7 V in -> raw 7005 ->
 *      class 3 (divisor 1000) -> 7.005 V, dp 1. Same number as today, by
 *      construction — so a regression there means frame[3].4 is NOT set on
 *      this unit's working frames, and the whole mechanism is wrong for us.
 *      The failure is loud, not silent: class 0 would render raw 7005 as
 *      "7005".
 *   2. The band that failed must move: 11 V in -> raw 997 -> class 2
 *      (divisor 100) -> 9.97 V, i.e. the saturated top of the 10 V band that
 *      the 2026-04-04 note itself called the plausible right answer. If 11 V
 *      still reads 0.997 V, the class came out 3 and the exponent bits are not
 *      what drives this unit.
 *   3. Above 9999 counts the +10000 extension and the class must compose:
 *      raw 19999 at class 4 renders 1.9999.
 *
 * METER_DCV_STOCK_EXPONENT=0 falls back to the old empirical path in one
 * rebuild, for A/B on the bench rather than a code edit.
 * ═══════════════════════════════════════════════════════════════════ */

#ifndef METER_DCV_STOCK_EXPONENT
#define METER_DCV_STOCK_EXPONENT 1
#endif

#if METER_DCV_STOCK_EXPONENT

typedef struct {
    uint8_t class_id;
    uint8_t display_decimal_pos;
    float divisor;
} dcv_stock_class_t;

static const dcv_stock_class_t dcv_stock_classes[] = {
    { 4, 1, 10000.0f },
    { 3, 1, 1000.0f  },
    { 2, 2, 100.0f   },
    { 1, 3, 10.0f    },
    { 0, 0, 1.0f     },
};

static uint8_t voltage_range_hint_from_stock_frame(const volatile uint8_t *frame)
{
    if (frame[8] & 0x80U) return 4;
    if (frame[3] & 0x10U) return 3;
    if (frame[4] & 0x10U) return 2;
    if (frame[5] & 0x10U) return 1;
    return 0;
}

static const dcv_stock_class_t *
dcv_stock_class_from_frame(const volatile uint8_t *frame)
{
    uint8_t class_id = voltage_range_hint_from_stock_frame(frame);
    unsigned n = sizeof(dcv_stock_classes) / sizeof(dcv_stock_classes[0]);

    for (unsigned i = 0; i < n; i++) {
        if (dcv_stock_classes[i].class_id == class_id) {
            return &dcv_stock_classes[i];
        }
    }
    return NULL;
}

/* Decimal position and unit, chosen from the frame's status bits alone —
 * before anyone looks at the decoded digits. That ordering is the instrument
 * contract: the meter IC declares its active range in the report, and the
 * renderer converts by the matching exponent. It is not a recognition table
 * for convenient bench values. */
static bool apply_stock_dcv_voltage_range_hint(meter_reading_t *r,
                                               uint8_t submode,
                                               const volatile uint8_t *frame)
{
    const dcv_stock_class_t *stock_class;

    if (submode != 0) return false;
    stock_class = dcv_stock_class_from_frame(frame);
    if (stock_class == NULL) return false;

    r->decimal_pos = stock_class->display_decimal_pos;
    r->unit_variant = 0;
    r->unit_suffix = "V";
    return true;
}

static void format_stock_decimal_value(int raw_value, uint8_t class_id,
                                       bool negative, char *s)
{
    int pos = 0;
    int divisor = 1;
    uint8_t decimals = class_id;

    if (negative) {
        s[pos++] = '-';
    }
    for (uint8_t i = 0; i < decimals; i++) {
        divisor *= 10;
    }

    int whole = raw_value / divisor;
    int frac = raw_value % divisor;

    if (whole >= 10000) s[pos++] = (char)('0' + (whole / 10000) % 10);
    if (whole >= 1000)  s[pos++] = (char)('0' + (whole / 1000) % 10);
    if (whole >= 100)   s[pos++] = (char)('0' + (whole / 100) % 10);
    if (whole >= 10)    s[pos++] = (char)('0' + (whole / 10) % 10);
    s[pos++] = (char)('0' + whole % 10);

    if (decimals > 0) {
        int place = divisor / 10;
        s[pos++] = '.';
        while (place > 0) {
            s[pos++] = (char)('0' + (frac / place) % 10);
            place /= 10;
        }
    }
    s[pos] = '\0';
}

/* Value and display string for DCV. Stock does two separate things in
 * FUN_08036AC0: build the four-digit raw and add 10000 when frame[2].3 is
 * set (that half is already ours, above), then divide the extended raw by
 * 10^class from the bits read here. No stock-only evidence supports a
 * one-point low-voltage coefficient; factory calibration may still live in
 * W25Q/SPI bulk data, but it is not this exponent table and must not be
 * invented here. */
static bool apply_stock_dcv_decimal_exponent(meter_reading_t *r,
                                             uint8_t submode,
                                             const volatile uint8_t *frame)
{
    const dcv_stock_class_t *stock_class;

    if (submode != 0) return false;
    stock_class = dcv_stock_class_from_frame(frame);
    if (stock_class == NULL) return false;

    float v = (float)r->raw_bcd / stock_class->divisor;
    if (r->negative) v = -v;
    r->value = v;
    r->unit_variant = 0;
    r->unit_suffix = "V";
    r->dcv_stock_class = stock_class->class_id;
    format_stock_decimal_value(r->raw_bcd, stock_class->class_id,
                               r->negative, r->display_str);

    float abs_v = v < 0.0f ? -v : v;
    float full_scale = (submode < METER_SUBMODE_COUNT)
                       ? bar_full_scale[submode] : 1000.0f;
    r->bar_fraction = abs_v / full_scale;
    if (r->bar_fraction > 1.0f) r->bar_fraction = 1.0f;
    return true;
}

#endif /* METER_DCV_STOCK_EXPONENT */

/* ═══════════════════════════════════════════════════════════════════
 * Format value into display string
 * ═══════════════════════════════════════════════════════════════════ */

static void format_reading(meter_reading_t *r, uint8_t submode)
{
    char *s = r->display_str;
    int pos = 0;

    if (r->negative) {
        s[pos++] = '-';
    }

    uint8_t dec = r->decimal_pos;

    /* Stock raw extension: raw_bcd carries a fifth digit that digits[] can't
     * hold (it only ever has the four glyphs the meter SoC sends). The
     * extension is always exactly +10000, so the leading digit is a literal
     * '1' in front of the four, and the decimal point keeps its position
     * among them: dp=1 with digits 5,0,0,0 renders 15.000 rather than 5.000. */
    if (r->raw_bcd >= 10000) {
        s[pos++] = '1';
    }

    for (int i = 0; i < 4; i++) {
        if (i == (int)dec && dec > 0 && dec < 4) {
            s[pos++] = '.';
        }
        s[pos++] = '0' + r->digits[i];
    }
    s[pos] = '\0';

    /* Strip leading zeros (but keep at least one digit before decimal) */
    /* e.g., "0.623" stays, but "0047" becomes "47" */
    if (dec == 0) {
        /* No decimal point — strip leading zeros */
        int start = r->negative ? 1 : 0;
        int first_nonzero = start;
        while (first_nonzero < pos - 1 && s[first_nonzero] == '0') {
            first_nonzero++;
        }
        if (first_nonzero > start) {
            memmove(s + start, s + first_nonzero, pos - first_nonzero + 1);
        }
    }

    /* Calculate float value from BCD */
    float divisor = 1.0f;
    for (int i = 0; i < (4 - (int)dec); i++) {
        divisor *= 10.0f;
    }
    r->value = (float)r->raw_bcd / divisor;
    if (r->negative) r->value = -r->value;

    /* Bar graph fraction */
    float abs_val = r->value < 0 ? -r->value : r->value;
    float full_scale = (submode < METER_SUBMODE_COUNT) ? bar_full_scale[submode] : 1000.0f;
    r->bar_fraction = abs_val / full_scale;
    if (r->bar_fraction > 1.0f) r->bar_fraction = 1.0f;
}

/* ═══════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════ */

void meter_session_begin(meter_session_t *s, uint8_t submode)
{
    memset(s, 0, sizeof(*s));
    s->submode = submode;
    strcpy(s->reading.display_str, "---");
    s->reading.unit_suffix = "";  /* Never NULL — UI can render directly. */
    s->reading.unit_variant = 0;
    /* memset already cleared the band latch and the f6 history — the whole
     * point of a session: a re-entry into the same mode starts clean. */
}

void meter_session_frame(meter_session_t *s, const volatile uint8_t *frame)
{
    meter_reading_t *r = &s->reading;
    uint8_t submode = s->submode;

    /* Validate header */
    if (frame[0] != 0x5A || frame[1] != 0xA5) return;

    /* Save raw frame for debug display */
    for (int i = 0; i < 12; i++) r->dbg_frame[i] = frame[i];

    /* Reset before any early return so the field always describes THIS frame:
     * 0xFF = the stock DCV exponent path did not run. */
    r->dcv_stock_class = 0xFFu;

    /* Extract cross-byte nibble pairs */
    uint8_t b2 = frame[2], b3 = frame[3], b4 = frame[4];
    uint8_t b5 = frame[5], b6 = frame[6];

    uint8_t nib0 = (b2 & 0xF0) | (b3 & 0x0F);
    uint8_t nib1 = (b3 & 0xF0) | (b4 & 0x0F);
    uint8_t nib2 = (b4 & 0xF0) | (b5 & 0x0F);
    uint8_t nib3 = (b5 & 0xF0) | (b6 & 0x0F);

    /* Save pre-lookup nibbles for debug */
    r->dbg_nibbles[0] = nib0;
    r->dbg_nibbles[1] = nib1;
    r->dbg_nibbles[2] = nib2;
    r->dbg_nibbles[3] = nib3;

    uint8_t digit0 = bcd_nibble_lookup(nib0);
    uint8_t digit1 = bcd_nibble_lookup(nib1);
    uint8_t digit2 = bcd_nibble_lookup(nib2);
    uint8_t digit3 = bcd_nibble_lookup(nib3);

    /* Save post-lookup digit codes for debug */
    r->dbg_raw_digits[0] = digit0;
    r->dbg_raw_digits[1] = digit1;
    r->dbg_raw_digits[2] = digit2;
    r->dbg_raw_digits[3] = digit3;

    /* Parse status flags from byte [7]
     * Based on meter_mode_handler FSM at 0x080371B0 in stock firmware.
     * The status byte encodes polarity, AC/DC, overload, and range info. */
    uint8_t status = frame[7];
    r->is_ac = (status & (1 << 2)) != 0;
    r->is_auto_range = (status & (1 << 3)) != 0;
    r->negative = (status & (1 << 0)) != 0;

    /* frame[2].3 = stock's raw +10000 extension flag. Recorded for every
     * frame (including OL/blank/special ones) so the debug line always shows
     * the current bit; it is only added to the value in the normal-BCD path
     * below, and only for DCV. */
    r->raw_bcd_extended = (b2 & 0x08) != 0;

    /* Parse flags from byte [6] */
    uint8_t flags = frame[6];
    r->is_hold = (flags & (1 << 6)) != 0;

    /* Record distinct frame[6] values for the session diagnostic.
     * Push-front with dedup: if the current value is already in the
     * history, leave it alone. Otherwise insert at slot 0 and shift
     * older entries down, dropping the oldest. */
    {
        bool seen = false;
        for (int k = 0; k < s->f6_history_count; k++) {
            if (s->f6_history[k] == flags) { seen = true; break; }
        }
        if (!seen) {
            int n = s->f6_history_count;
            if (n < METER_F6_HISTORY_LEN) n++;
            for (int k = n - 1; k > 0; k--) {
                s->f6_history[k] = s->f6_history[k - 1];
            }
            s->f6_history[0] = flags;
            s->f6_history_count = (uint8_t)n;
        }
    }

    /* Range indicators from frame[6] bits 4-5 (stock firmware FSM case 4) */
    r->range_indicator = (flags >> 4) & 0x03;

    /* Determine probe_type from status bits (stock firmware FSM case 0).
     * This drives the calibration coefficient selection in fpga_state_update.
     *   status bit 1 set → probe_type = 1
     *   status bit 0 set (alone) → probe_type = 2
     *   status bit 3 set (auto-range) → probe_type = 2
     *   otherwise → probe_type = 0
     */
    if (status & (1 << 1)) {
        r->probe_type = 1;
    } else if (status & (1 << 0)) {
        r->probe_type = 2;
    } else if (status & (1 << 3)) {
        r->probe_type = 2;
    } else {
        r->probe_type = 0;
    }

    /* Legacy "range command" parameter — kept for API compatibility.
     *
     * Earlier notes claimed stock firmware "sends cmds 0x1B/0x1C/0x1E
     * as FPGA range-select commands" from FUN_080028e0. That was wrong:
     * see reverse_engineering/analysis_v120/meter_fsm_deep_dive.md Q3.
     * Those bytes are display-queue dispatch codes (indices into the
     * display task function table at 0x804be74), not FPGA commands.
     * Stock firmware does NOT implement firmware-driven auto-ranging;
     * the FPGA meter IC auto-ranges autonomously and the MCU just
     * renders whatever frame it receives. */
    r->range_cmd = r->probe_type;

    /* --- Special value detection --- */

    /* Overload: "OL" */
    if (digit0 == 0x0A && digit1 == 0x0B) {
        r->result_class = METER_RESULT_OVERLOAD;
        strcpy(r->display_str, "OL");
        r->value = 0.0f;
        r->bar_fraction = 1.0f;
        r->continuity_beep = false;
        r->valid = true;
        r->update_count++;
        return;
    }

    /* Blank display */
    if (digit0 == 0x10 && digit1 == 0x10) {
        r->result_class = METER_RESULT_BLANK;
        strcpy(r->display_str, "---");
        r->value = 0.0f;
        r->bar_fraction = 0.0f;
        r->continuity_beep = false;
        r->valid = true;
        r->update_count++;
        return;
    }

    /* Partial blank */
    if (digit0 == 0x10 && digit1 == 0x11) {
        r->result_class = METER_RESULT_BLANK;
        strcpy(r->display_str, "---");
        r->value = 0.0f;
        r->bar_fraction = 0.0f;
        r->valid = true;
        r->update_count++;
        return;
    }

    /* Continuity detection */
    if (digit1 == 0x12 && digit2 == 0x0A && digit3 == 5) {
        r->result_class = METER_RESULT_CONTINUITY;
        r->continuity_beep = true;
        /* Still try to show a value if digit0 is valid */
        if (digit0 <= 9) {
            r->digits[0] = digit0;
            r->digits[1] = 0;
            r->digits[2] = 0;
            r->digits[3] = 0;
            r->raw_bcd = digit0;
            r->decimal_pos = 0;
            format_reading(r, submode);
        } else {
            strcpy(r->display_str, "CONT");
            r->value = 0.0f;
        }
        r->valid = true;
        r->update_count++;
        return;
    }

    /* Invalid digits */
    if (digit0 == 0xFF || digit1 == 0xFF || digit2 == 0xFF || digit3 == 0xFF) {
        r->result_class = METER_RESULT_INVALID;
        strcpy(r->display_str, "ERR");
        r->value = 0.0f;
        r->bar_fraction = 0.0f;
        r->valid = true;
        r->update_count++;
        return;
    }

    /* --- Normal BCD value --- */

    /* Mask off special code high bits for assembly */
    uint8_t d0 = (digit0 >= 0x10) ? (digit0 - 0x10) : digit0;
    uint8_t d1 = (digit1 >= 0x10) ? (digit1 - 0x10) : digit1;
    uint8_t d2 = (digit2 >= 0x10) ? (digit2 - 0x10) : digit2;
    uint8_t d3 = (digit3 >= 0x10) ? (digit3 - 0x10) : digit3;

    /* Clamp individual digits to valid BCD range */
    if (d0 > 9) d0 = 0;
    if (d1 > 9) d1 = 0;
    if (d2 > 9) d2 = 0;
    if (d3 > 9) d3 = 0;

    r->digits[0] = d0;
    r->digits[1] = d1;
    r->digits[2] = d2;
    r->digits[3] = d3;
    r->raw_bcd = d0 * 1000 + d1 * 100 + d2 * 10 + d3;

    /* Stock raw extension — the fifth digit the four-glyph display can't
     * carry. FUN_08036AC0 builds the same four-digit raw, then at 0x08036BFC
     * loads 10000.0f and keeps raw + 10000 when frame[2].3 is set:
     *
     *   vldr s2, [pc, #0x88]   ; 0x08036C88 = 10000.0f
     *   vadd.f32 s2, s0, s2
     *   lsls.w r0, r8, #0x1c   ; r8 = frame[2], bit 3 into sign
     *   it pl
     *   vmovpl.f32 s2, s0      ; bit clear -> discard the +10000
     *
     * Neither our decoder nor upstream's had this, so every DCV reading
     * above 9999 counts decoded as a wrapped four-digit value — the range
     * where auto-range keeps hunting. Evidence:
     * meter_stock_multiplier_tables_2026_06_05.md (komzpa, upstream PR #13).
     *
     * DCV only: stock proves the bit in the DCV value/formatter path alone.
     * Applying it elsewhere would jump the value by 10000/divisor while the
     * digits stay put, so other submodes keep the plain four-digit raw and
     * only report the bit on the decoder debug line. */
    if (submode == 0 && r->raw_bcd_extended) {
        r->raw_bcd += 10000;
    }

    /* Decimal position and unit: static default per submode, then
     * overridden by the empirical frame[6] decoder below when we
     * recognize the range bits.
     *
     * The static defaults handle the 5 V DC case and submodes we
     * haven't captured yet (ACV, DCA, diode, cap). The frame[6]
     * decoder handles resistance auto-ranging where the FPGA meter
     * IC reports different byte patterns per sub-range.
     *
     * See reverse_engineering/analysis_v120/meter_frame_capture_log.md
     * for the 7 captured frames this table is derived from. */
    r->decimal_pos = (submode < METER_SUBMODE_COUNT) ? default_decimal_pos[submode] : 1;
    r->unit_variant = 0;
    r->unit_suffix = (submode < METER_SUBMODE_COUNT)
                     ? unit_suffix_table[submode][0]
                     : "";

    /* Empirical frame[6] range decoder with sticky latch.
     *
     * Observed byte values from bench captures on 2026-04-04:
     *   0x07 — low Ω band (short circuit, ~150 Ω, open circuit)
     *   0x4B — kΩ band, sub-range for values < 10 kΩ (dp=1)
     *   0x4D — kΩ band, sub-range for values 10-99 kΩ (dp=2)
     *   0x0F — DCV mode (dp=1, unit "V")
     *
     * At runtime, the FPGA rotates through multiple frame types per
     * measurement and frame[6] only carries the range bits in some
     * of them. Other frame types have frame[6] set to something we
     * don't recognize yet.
     *
     * To avoid the display flickering between "3.280 kOhm" (correct,
     * from a range frame) and "32.80 Ohm" (wrong, from a non-range
     * frame applying the static default), we latch the last known
     * good decode and reuse it when an unknown frame arrives. The
     * latch lives in the session, so meter_session_begin() is its
     * reset — the old function-local statics survived a re-entry into
     * the same mode and carried the previous session's decision into
     * the new one (and their 0xFF "no latch" sentinel collided with
     * FPGA_METER_INVALID_LOCAL_SUBMODE). */

    uint8_t f6 = flags;  /* already holds frame[6] */
    bool decoded = false;
    uint8_t new_dp = 0;
    const char *new_unit = NULL;

    if (submode == 6 || submode == 7) {
        /* Resistance / continuity: upper nibble of frame[6] indicates
         * band. Upper nibble 0 = low-Ω (raw BCD needs × 0.0304 cal).
         * Upper nibble 4 = kΩ (already accurate without cal).
         * Sub-nibbles within 0x4_ select dp position.
         *
         * Observed f6 history in low-Ω mode: 0x07, 0x0A, 0x0B, 0x0D,
         * 0x0E, 0x0F — all treated as the same band. The raw cal
         * override below decouples the displayed value from this dp
         * hint, so dp=2 here is just a reasonable default for the
         * pre-cal path.
         *
         * Note: submode 0 (DCV) also has a case for 0x0F, but it's
         * guarded by submode above so there's no collision. */
        if ((f6 & 0xF0) == 0x00) {
            new_dp = 2; new_unit = "Ohm";  decoded = true;
        } else if (f6 == 0x4B) {
            new_dp = 1; new_unit = "kOhm"; decoded = true;
        } else if (f6 == 0x4D) {
            new_dp = 2; new_unit = "kOhm"; decoded = true;
        }
    }
#if !METER_DCV_STOCK_EXPONENT
    else if (submode == 0 && f6 == 0x0F) {
        /* Retired: frame[6] never carried the DCV exponent. This case said
         * dp 1 — the same thing default_decimal_pos[0] already said — and it
         * is what rendered 11 V as 0.997 V. Kept only under the A/B fallback
         * so the old behaviour is reproducible in one rebuild. */
        new_dp = 1; new_unit = "V"; decoded = true;
    }
#endif

    /* DCV above ~9.999 V: the 2026-04-04 bench limitation, and what replaced
     * the diagnosis. Captures on unit #1 were:
     *   7V  input → f6 stable at 0x07, raw_bcd=7005, dp=1 → 7.005V ✓
     *   11V input → f6 ROTATES through {0x0A, 0x0F, 0x0B, 0x07},
     *               raw_bcd=997, dp=1 → 0.997V ✗
     *
     * The note above these lines used to blame missing factory calibration for
     * the IC's auto-range, and proposed cataloguing f6 → dp by hand. That was
     * looking in the wrong byte: stock never reads the DCV exponent out of
     * frame[6]. It reads four status bits (see the block above
     * format_reading), and the rotating f6 was a symptom, not the range
     * report. WITHDRAWN: "f6 → dp mapping by range" as a DCV plan.
     *
     * The exponent path is implemented now, with its predictions written down
     * where it is defined. Still open, and NOT addressed by it:
     *   1. Whether the IC's auto-range itself misbehaves above 10 V, i.e.
     *      whether raw 997 at 11 V is a correct saturated report of the 10 V
     *      band or a garbage frame. The exponent path renders whatever the
     *      frame claims; it cannot fix a wrong claim.
     *   2. The H2 SPI3 bulk cal replay (analysis_v120/spi3_bulk_cal_resolved.md)
     *      as the wholesale answer to missing factory cal — this is still the
     *      candidate for the low-Ω coefficient, which upstream has since
     *      withdrawn rather than kept at one unit's 0.0304 (see the band
     *      override below: ours still applies it, theirs fails closed).
     */

    if (decoded) {
        /* This frame carried range bits we recognize. Apply and latch. */
        r->decimal_pos = new_dp;
        r->unit_suffix = new_unit;
        s->band_has_latch = true;
        s->band_latch_dp = new_dp;
        s->band_latch_unit = new_unit;
    } else if (s->band_has_latch) {
        /* This frame's range byte is unknown, but we've seen a good
         * range frame earlier in this session — reuse its decision. */
        r->decimal_pos = s->band_latch_dp;
        r->unit_suffix = s->band_latch_unit;
    }
    /* else: no latch yet, keep the static default set above. */

#if METER_DCV_STOCK_EXPONENT
    /* DCV: the status bits decide the decimal position, not frame[6] and not
     * the per-submode default. Applied after the frame[6] block so the stock
     * evidence wins over the empirical latch on submode 0; resistance and
     * continuity are untouched. */
    (void)apply_stock_dcv_voltage_range_hint(r, submode, frame);
#endif

    /* Format the display string and compute float value */
    r->result_class = METER_RESULT_NORMAL;
    r->continuity_beep = false;
    format_reading(r, submode);

#if METER_DCV_STOCK_EXPONENT
    /* ...and the value/string come from the class divisor, which is what makes
     * the over-9999 band composable with the +10000 raw extension. Runs after
     * format_reading for the same reason upstream does it: the generic path
     * fills in everything else the reading carries. */
    (void)apply_stock_dcv_decimal_exponent(r, submode, frame);
#endif

    /* ── Resistance band factory calibration override ──
     *
     * For resistance/continuity submodes, the FPGA meter IC rotates
     * through multiple frame variants per measurement, each claiming
     * a different dp/unit. The "correct" interpretation depends on
     * which specific frame[6] we happen to catch — without override,
     * the display flickers between e.g. "9.821 kOhm" and "98.24 kOhm"
     * for the SAME 10 kΩ resistor.
     *
     * The FIX is to compute resistance from raw_bcd at the band level,
     * ignoring the per-frame dp hint entirely:
     *
     *   Low-Ω band  (frame[6] upper nibble 0): value = raw_bcd × 0.0304 Ω
     *   kΩ    band  (frame[6] upper nibble 4): value = raw_bcd × 0.001  kΩ
     *
     * Bench data (2026-04-04, unit #1):
     *   147 Ω ref  → raw ≈ 4824 → 4824 × 0.0304 = 146.6 Ω  ✓
     *   3.3 kΩ ref → raw ≈ 3230 → 3230 × 0.001  = 3.230 kΩ ✓
     *   10 kΩ ref  → raw ≈ 9821 → 9821 × 0.001  = 9.821 kΩ ✓
     *
     * The 0.0304 factor is per-device (factory calibrated). When the
     * SPI flash driver lands it should come from "3:/System file/..."
     * — see flash_fs.c:220. The 0.001 kΩ factor is a geometric
     * identity (raw counts already expressed in Ω, shifted to kΩ) and
     * should be stable across units.
     *
     * Higher bands (MΩ, autorange to 200 kΩ / 2 MΩ) are not yet
     * characterized — those will need additional upper-nibble cases.
     *
     * We leave raw_bcd, decimal_pos, and digits[] untouched so the
     * debug overlay at meter_ui.c:948 still shows the FPGA's raw
     * pre-cal report. UI code reads `value` and `display_str` for
     * the final numbers.
     */
    if (submode == 6 || submode == 7) {
        float       scale = 0.0f;
        const char *unit  = NULL;

        switch (f6 & 0xF0) {
        case 0x00:  scale = METER_CAL_LOW_OHM_FACTOR; unit = "Ohm";  break;
        case 0x40:  scale = METER_CAL_KOHM_FACTOR;    unit = "kOhm"; break;
        default:    break;  /* Unknown band — leave format_reading's output */
        }

        if (scale != 0.0f) {
            float v = (float)r->raw_bcd * scale;
            if (r->negative) v = -v;
            r->value       = v;
            r->unit_suffix = unit;

            char *s = r->display_str;
            int   pos = 0;
            float av  = v;
            if (av < 0.0f) { s[pos++] = '-'; av = -av; }
            format_4digit_unsigned(av, s + pos);

            /* Recompute bar graph fraction from the corrected value. */
            float abs_v = v < 0.0f ? -v : v;
            float full_scale = (submode < METER_SUBMODE_COUNT) ? bar_full_scale[submode] : 1000.0f;
            r->bar_fraction = abs_v / full_scale;
            if (r->bar_fraction > 1.0f) r->bar_fraction = 1.0f;
        }
    }

    r->valid = true;
    r->update_count++;
}

#endif /* HW_TARGET_2C53T */
