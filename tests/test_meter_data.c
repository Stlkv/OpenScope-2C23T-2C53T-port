/*
 * Host-side tests for the meter frame decoder (meter_data.c).
 *
 * Ported from upstream DavidClawson/OpenScope-2C53T
 * firmware/tests/test_meter_data.c (68 cases). Upstream's meter_data.c has
 * grown far past this port's fork point (736 -> 1624 lines), so the suite is
 * carried WHOLE, with every case whose functionality this port does not have
 * compiled out behind a feature macro and reported as an explicit SKIP on
 * every run. The SKIP list is the executable map of what remains unported.
 *
 * Adaptation rules used for the live cases:
 *   - upstream `bcd_value`      -> this port's `raw_bcd` (+ raw_bcd_extended)
 *   - `aux_freq_hz` asserts     -> guarded by METER_HAS_AC_EVIDENCE_GATE
 *   - frame-family debug asserts-> guarded by METER_HAS_FRAME_FAMILY_GATE
 * Everything else is verbatim.
 *
 * A trailing "port-specific fixtures" block pins behaviour this port has
 * that upstream does not (or has withdrawn): the PLAN.md DCV exponent
 * fixture table, the low-ohm 0.0304 calibration this port still applies
 * (upstream fails closed - a documented divergence), this port's current
 * per-submode formatting defaults, and the band-latch behaviour that the
 * meter-session refactor is about to change.
 *
 * Build and run: make test-meter-data
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "meter_plan.h"
#include "meter_data.h"

/* ═══════════════════════════════════════════════════════════════════
 * Feature map: what this port has NOT taken from upstream meter_data.c.
 * Flipping a macro to 1 revives its cases; until the feature is ported
 * they will fail loudly rather than silently vanish.
 * ═══════════════════════════════════════════════════════════════════ */

/* Frame-family gate: expected/observed_frame_family, reject_reason, and
 * fail-closed rejection of unmarked/wrong-family frames. This port renders
 * whatever frame arrives for the active submode. */
#define METER_HAS_FRAME_FAMILY_GATE 0

/* AC-evidence gate: aux_freq_hz from frame[10..11] and the 45-65 Hz proof
 * requirement for ACV/ACA submodes. */
#define METER_HAS_AC_EVIDENCE_GATE 0

/* meter_data_invalidate() + submode/display_update_count fields on the
 * reading. This port's reset is meter_session_begin(); the upstream
 * invalidate cases also assert stock/family fields it does not have. */
#define METER_HAS_INVALIDATE 0

/* Stock format FSM: stock_mode/stock_variant/stock_format/stock_display_cmd/
 * stock_unit_index/stock_composite_index debug fields, and the stock
 * formatter ranges for ACV (dp 3 at mains), diode and temperature. This
 * port keeps its own bench-derived per-submode defaults instead. */
#define METER_HAS_STOCK_FSM 0

/* Zero-detect short frames (status 0x28 family) rendered as terminal 0. */
#define METER_HAS_ZERO_DETECT 0

/* Terminal/special frames (OL/blank/partial/ERR) clearing the stale payload
 * fields and the stale continuity beep. This port's special branches leave
 * raw_bcd/digits/beep from the previous frame in place - a known gap found
 * while porting this suite. */
#define METER_HAS_SPECIAL_PAYLOAD_CLEAR 0

/* Digit-read hardening: special segment codes 0x0C..0x14 inside a numeric
 * frame reported as ERR instead of clamped to 0. */
#define METER_HAS_DIGIT_READ_HARDENING 0

/* Upstream WITHDREW the one-point low-ohm 0.0304 calibration and fails
 * closed on the low-ohm band. This port still applies its unit #1 factor -
 * the divergence PLAN.md carries as "name it before they find it". The
 * port's own behaviour is pinned in the port-specific block below. */
#define METER_HAS_UNRESOLVED_LOW_OHM_REJECT 0

/* meter_data_snapshot() + meter_data_test_set_hook() + the pthread-based
 * two-writer coherence test. */
#define METER_HAS_SNAPSHOT 0

static int tests_run = 0;
static int tests_passed = 0;
static int tests_skipped = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  %-58s ", #name); \
    if (test_##name()) { tests_passed++; printf("PASS\n"); } \
    else { printf("FAIL\n"); } \
} while (0)

#define SKIP(name, reason) do { \
    tests_run++; \
    tests_skipped++; \
    printf("  %-58s SKIP (%s)\n", #name, reason); \
} while (0)

#define ASSERT(cond) do { \
    if (!(cond)) { printf("[line %d: %s] ", __LINE__, #cond); return 0; } \
} while (0)

#define ASSERT_STR_EQ(actual, expected) do { \
    if (strcmp((actual), (expected)) != 0) { \
        printf("[line %d: expected \"%s\", got \"%s\"] ", \
               __LINE__, (expected), (actual)); \
        return 0; \
    } \
} while (0)

/* The decoder state lives in a meter_session_t (the firmware's single
 * session is owned by dmm53.c; the tests own this one). Two shims keep the
 * upstream case bodies nearly verbatim: `meter_reading` maps onto the
 * session's reading, and process_frame() begins a fresh session whenever a
 * case switches submode — the same reset dmm53_begin_transition() performs
 * on a mode change. */
static meter_session_t session;
#define meter_reading (session.reading)

static void meter_data_init(void)
{
    meter_session_begin(&session, 0);
}

static int close_to(float actual, float expected, float tolerance)
{
    float delta = actual - expected;
    if (delta < 0.0f) delta = -delta;
    return delta <= tolerance;
}

static int expect_normal_reading(const char *display,
                                 const char *unit,
                                 float value,
                                 float tolerance)
{
    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT_STR_EQ(meter_reading.display_str, display);
    ASSERT_STR_EQ(meter_reading.unit_suffix, unit);
    ASSERT(close_to(meter_reading.value, value, tolerance));
    return 1;
}

static void process_frame(const uint8_t frame[12], uint8_t submode)
{
    if (session.submode != submode) {
        meter_session_begin(&session, submode);
    }
    meter_session_frame(&session, (const volatile uint8_t *)frame);
}

static uint8_t segment_nibble_for_code(uint8_t code)
{
    switch (code) {
    case 0: return 0xEB;
    case 1: return 0x0A;
    case 2: return 0xAD;
    case 3: return 0x8F;
    case 4: return 0x4E;
    case 5: return 0xC7;
    case 6: return 0xE7;
    case 7: return 0x8A;
    case 8: return 0xEF;
    case 9: return 0xCF;
    case 0x0A: return 0xEE;  /* OL high / continuity marker */
    case 0x0B: return 0x23;  /* OL low */
    case 0x0C: return 0x65;  /* special */
    case 0x0D: return 0x27;  /* special */
    case 0x0E: return 0x61;  /* special */
    case 0x0F: return 0x04;  /* special */
    case 0x10: return 0x00;  /* blank */
    case 0x11: return 0xE1;  /* partial blank */
    case 0x12: return 0xEC;  /* continuity */
    case 0x13: return 0xE5;  /* mode-change/special */
    case 0x14: return 0x24;  /* special */
    case 0xFF: return 0x01;  /* invalid/unmapped */
    default: return 0x01;
    }
}

static void build_segment_frame(uint8_t frame[12],
                                uint8_t digit0_code,
                                uint8_t digit1_code,
                                uint8_t digit2_code,
                                uint8_t digit3_code,
                                uint8_t frame6_high,
                                uint8_t status,
                                uint8_t meas_flags,
                                uint8_t additional_status,
                                uint16_t extra)
{
    uint8_t n0 = segment_nibble_for_code(digit0_code);
    uint8_t n1 = segment_nibble_for_code(digit1_code);
    uint8_t n2 = segment_nibble_for_code(digit2_code);
    uint8_t n3 = segment_nibble_for_code(digit3_code);

    memset(frame, 0, 12);
    frame[0] = 0x5A;
    frame[1] = 0xA5;
    frame[2] = n0 & 0xF0;
    frame[3] = (n1 & 0xF0) | (n0 & 0x0F);
    frame[4] = (n2 & 0xF0) | (n1 & 0x0F);
    frame[5] = (n3 & 0xF0) | (n2 & 0x0F);
    frame[6] = (frame6_high & 0xF0) | (n3 & 0x0F);
    frame[7] = status;
    frame[8] = meas_flags;
    frame[9] = additional_status;
    frame[10] = (uint8_t)(extra >> 8);
    frame[11] = (uint8_t)extra;
}

/* ═══════════════════════════════════════════════════════════════════
 * Live upstream cases (adapted names only)
 * ═══════════════════════════════════════════════════════════════════ */

static int test_segment_frame_builder_exercises_cross_byte_lookup(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 5, 0, 0, 8, 0x00, 0x00, 0x02, 0x00, 0x014E);
    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 5008);
    ASSERT(meter_reading.dbg_nibbles[0] == 0xC7);
    ASSERT(meter_reading.dbg_nibbles[1] == 0xEB);
    ASSERT(meter_reading.dbg_nibbles[2] == 0xEB);
    ASSERT(meter_reading.dbg_nibbles[3] == 0xEF);
    ASSERT(meter_reading.dbg_raw_digits[0] == 5);
    ASSERT(meter_reading.dbg_raw_digits[1] == 0);
    ASSERT(meter_reading.dbg_raw_digits[2] == 0);
    ASSERT(meter_reading.dbg_raw_digits[3] == 8);
    return 1;
}

static int test_dcv_5v_frame_keeps_verified_decimal_and_unit(void)
{
    static const uint8_t frame[12] = {
        0x5A, 0xA5, 0xC6, 0xF7, 0xEB, 0xEB,
        0x0F, 0x00, 0x02, 0x00, 0x01, 0x4E,
    };

    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT(meter_reading.raw_bcd == 5008);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT_STR_EQ(meter_reading.display_str, "5.008");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 5.008f, 0.001f));
    return 1;
}

static int test_dcv_live_5v_frame_uses_stock_range_hint(void)
{
    static const uint8_t frame[12] = {
        0x5A, 0xA5, 0x46, 0xDE, 0xCF, 0x4F,
        0x0E, 0x00, 0x02, 0x00, 0x01, 0x82,
    };

    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 4994);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT_STR_EQ(meter_reading.display_str, "4.994");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 4.994f, 0.001f));
    return 1;
}

static int test_dcv_live_32v_frame_uses_stock_range_hint(void)
{
    static const uint8_t frame[12] = {
        0x5A, 0xA5, 0x86, 0x0F, 0xDA, 0xEF,
        0x07, 0x00, 0x02, 0x00, 0x03, 0xFF,
    };

    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 3196);
    ASSERT(meter_reading.decimal_pos == 2);
    ASSERT_STR_EQ(meter_reading.display_str, "31.96");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 31.96f, 0.01f));
    return 1;
}

static int test_dcv_1v2949_frame_uses_stock_extended_raw_and_class4(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 2, 9, 4, 9, 0x00, 0x00, 0x82, 0x00, 0);
    frame[2] |= 0x08;
    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT(meter_reading.raw_bcd == 12949);
    ASSERT(meter_reading.raw_bcd_extended);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT_STR_EQ(meter_reading.display_str, "1.2949");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 1.2949f, 0.0001f));
    ASSERT((meter_reading.dbg_frame[2] & 0x08U) != 0);
    ASSERT(meter_reading.dbg_frame[8] == 0x82);
    return 1;
}

static int test_dcv_1v4979_frame_uses_stock_extended_raw_and_class4(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 4, 9, 7, 9, 0x00, 0x00, 0x82, 0x00, 0);
    frame[2] |= 0x08;
    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT(meter_reading.raw_bcd == 14979);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT_STR_EQ(meter_reading.display_str, "1.4979");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 1.4979f, 0.0001f));
    ASSERT((meter_reading.dbg_frame[2] & 0x08U) != 0);
    ASSERT(meter_reading.dbg_frame[8] == 0x82);
    return 1;
}

static int test_dcv_live_1v5_frame_uses_stock_extended_raw_and_class4(void)
{
    static const uint8_t frame[12] = {
        0x5A, 0xA5, 0x4E, 0xCE, 0x8F, 0x8A,
        0x0A, 0x00, 0x82, 0x00, 0x01, 0x7F,
    };

    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT(meter_reading.raw_bcd == 14977);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT_STR_EQ(meter_reading.display_str, "1.4977");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 1.4977f, 0.0001f));
    ASSERT((meter_reading.dbg_frame[2] & 0x08U) != 0);
    ASSERT(meter_reading.dbg_frame[8] == 0x82);
    return 1;
}

static int test_dcv_live_1v5_rotating_frames_keep_stock_class4(void)
{
    static const uint8_t frames[][12] = {
        { 0x5A, 0xA5, 0x4E, 0xCE, 0x8F, 0xEA, 0x0F, 0x00, 0x82, 0x00, 0x01, 0x7F },
        { 0x5A, 0xA5, 0x4E, 0xCE, 0x8F, 0x8A, 0x0A, 0x00, 0x82, 0x00, 0x01, 0x7F },
        { 0x5A, 0xA5, 0x4E, 0xCE, 0x8F, 0xEA, 0x07, 0x00, 0x82, 0x00, 0x01, 0x7F },
    };

    meter_data_init();
    for (unsigned i = 0; i < sizeof(frames) / sizeof(frames[0]); i++) {
        process_frame(frames[i], 0);
        ASSERT(meter_reading.valid);
        ASSERT(meter_reading.decimal_pos == 1);
        ASSERT(strncmp(meter_reading.display_str, "1.497", 5) == 0);
        ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
        ASSERT(meter_reading.value > 1.4975f);
        ASSERT(meter_reading.value < 1.4979f);
        ASSERT(meter_reading.raw_bcd >= 14976);
        ASSERT(meter_reading.raw_bcd <= 14978);
        ASSERT((meter_reading.dbg_frame[2] & 0x08U) != 0);
        ASSERT(meter_reading.dbg_frame[8] == 0x82);
    }
    return 1;
}

static int test_dcv_stock_range_class_priority_table(void)
{
    struct range_case {
        uint8_t frame3_or;
        uint8_t frame4_or;
        uint8_t frame5_or;
        uint8_t frame8_or;
        int bcd;
        const char *display;
        float value;
    };
    static const struct range_case cases[] = {
        { 0x00, 0x00, 0x00, 0x00, 1234, "1234",   1234.0f },
        { 0x00, 0x00, 0x10, 0x00, 1234, "123.4",   123.4f },
        { 0x00, 0x10, 0x10, 0x00, 1234, "12.34",    12.34f },
        { 0x10, 0x10, 0x10, 0x00, 1234, "1.234",     1.234f },
        { 0x10, 0x10, 0x10, 0x80, 1234, "0.1234",    0.1234f },
    };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint8_t frame[12];

        build_segment_frame(frame, 1, 2, 3, 4,
                            0x00, 0x00, 0x02, 0x00, 0);
        frame[3] |= cases[i].frame3_or;
        frame[4] |= cases[i].frame4_or;
        frame[5] |= cases[i].frame5_or;
        frame[8] |= cases[i].frame8_or;

        meter_data_init();
        process_frame(frame, 0);

        ASSERT(meter_reading.valid);
        ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
        ASSERT(meter_reading.raw_bcd == cases[i].bcd);
        ASSERT_STR_EQ(meter_reading.display_str, cases[i].display);
        ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
        ASSERT(close_to(meter_reading.value, cases[i].value, 0.0002f));
    }
    return 1;
}

static int test_dcv_stock_range_class_priority_all_bit_combinations(void)
{
    static const char *display_no_extend[] = {
        "1234", "123.4", "12.34", "1.234", "0.1234"
    };
    static const char *display_extend[] = {
        "11234", "1123.4", "112.34", "11.234", "1.1234"
    };
    static const float expected_no_extend[] = {
        1234.0f, 123.4f, 12.34f, 1.234f, 0.1234f
    };
    static const float expected_extend[] = {
        11234.0f, 1123.4f, 112.34f, 11.234f, 1.1234f
    };

    for (uint8_t bits = 0; bits < 16; bits++) {
        for (uint8_t extend = 0; extend < 2; extend++) {
            uint8_t frame[12];
            uint8_t expected_class =
                (bits & 0x8U) ? 4U :
                (bits & 0x4U) ? 3U :
                (bits & 0x2U) ? 2U :
                (bits & 0x1U) ? 1U : 0U;

            build_segment_frame(frame, 1, 2, 3, 4,
                                0x00, 0x00, 0x02, 0x00, 0);
            if (bits & 0x1U) frame[5] |= 0x10U;
            if (bits & 0x2U) frame[4] |= 0x10U;
            if (bits & 0x4U) frame[3] |= 0x10U;
            if (bits & 0x8U) frame[8] |= 0x80U;
            if (extend) frame[2] |= 0x08U;

            meter_data_init();
            process_frame(frame, 0);

            ASSERT(meter_reading.valid);
            ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
            ASSERT(meter_reading.raw_bcd == (extend ? 11234 : 1234));
            ASSERT(meter_reading.dcv_stock_class == expected_class);
            ASSERT_STR_EQ(meter_reading.display_str,
                          extend ? display_extend[expected_class]
                                 : display_no_extend[expected_class]);
            ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
            ASSERT(close_to(meter_reading.value,
                            extend ? expected_extend[expected_class]
                                   : expected_no_extend[expected_class],
                            0.0003f));
        }
    }
    return 1;
}

static int test_dcv_class4_priority_requires_frame8_bit7(void)
{
    static const uint8_t frame[12] = {
        0x5A, 0xA5, 0x4E, 0xCE, 0x8F, 0x8A,
        0x0A, 0x00, 0x02, 0x00, 0x01, 0x7F,
    };

    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 14977);
    ASSERT(meter_reading.decimal_pos == 0);
    ASSERT_STR_EQ(meter_reading.display_str, "14977");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 14977.0f, 0.001f));
    ASSERT((meter_reading.dbg_frame[2] & 0x08U) != 0);
    ASSERT(meter_reading.dbg_frame[8] == 0x02);
    return 1;
}

static int test_dcv_synthetic_5008_without_class_bits_stays_class0(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 5, 0, 0, 8, 0x00, 0x00, 0x02, 0x00, 0x014E);
    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 5008);
    ASSERT(meter_reading.dbg_frame[6] == 0x0F);
    ASSERT(meter_reading.decimal_pos == 0);
    ASSERT_STR_EQ(meter_reading.display_str, "5008");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 5008.0f, 0.001f));
#if METER_HAS_AC_EVIDENCE_GATE
    ASSERT(close_to(meter_reading.aux_freq_hz, 0.0f, 0.001f));
#endif
    return 1;
}

static int test_dcv_extra_frequency_hint_does_not_set_voltage_range(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 5, 0, 0, 8, 0x00, 0x00, 0x02, 0x00, 0x0031);
    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 5008);
    ASSERT(meter_reading.decimal_pos == 0);
    ASSERT_STR_EQ(meter_reading.display_str, "5008");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 5008.0f, 0.001f));
#if METER_HAS_AC_EVIDENCE_GATE
    ASSERT(close_to(meter_reading.aux_freq_hz, 49.0f, 0.1f));
#endif
    return 1;
}

static int test_dcv_aux_extra_bytes_do_not_change_stock_range_class(void)
{
    static const char *display_no_extend[] = {
        "5008", "500.8", "50.08", "5.008", "0.5008"
    };
    static const char *display_extend[] = {
        "15008", "1500.8", "150.08", "15.008", "1.5008"
    };
    static const float expected_no_extend[] = {
        5008.0f, 500.8f, 50.08f, 5.008f, 0.5008f
    };
    static const float expected_extend[] = {
        15008.0f, 1500.8f, 150.08f, 15.008f, 1.5008f
    };
    static const uint16_t extra_cases[] = {
        0x0000, 0x0031, 0x014E, 0x017F, 0x03FF, 0xFFFF
    };

    for (uint8_t bits = 0; bits < 16; bits++) {
        uint8_t expected_class =
            (bits & 0x8U) ? 4U :
            (bits & 0x4U) ? 3U :
            (bits & 0x2U) ? 2U :
            (bits & 0x1U) ? 1U : 0U;

        for (uint8_t extend = 0; extend < 2; extend++) {
            for (unsigned i = 0; i < sizeof(extra_cases) / sizeof(extra_cases[0]); i++) {
                uint8_t frame[12];

                build_segment_frame(frame, 5, 0, 0, 8,
                                    0x00, 0x00, 0x02, 0x00, extra_cases[i]);
                if (bits & 0x1U) frame[5] |= 0x10U;
                if (bits & 0x2U) frame[4] |= 0x10U;
                if (bits & 0x4U) frame[3] |= 0x10U;
                if (bits & 0x8U) frame[8] |= 0x80U;
                if (extend) frame[2] |= 0x08U;

                meter_data_init();
                process_frame(frame, 0);

                ASSERT(meter_reading.valid);
                ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
                ASSERT(meter_reading.raw_bcd == (extend ? 15008 : 5008));
                ASSERT_STR_EQ(meter_reading.display_str,
                              extend ? display_extend[expected_class]
                                     : display_no_extend[expected_class]);
                ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
                ASSERT(close_to(meter_reading.value,
                                extend ? expected_extend[expected_class]
                                       : expected_no_extend[expected_class],
                                0.0003f));
                ASSERT(meter_reading.dbg_frame[10] == (uint8_t)(extra_cases[i] >> 8));
                ASSERT(meter_reading.dbg_frame[11] == (uint8_t)extra_cases[i]);
            }
        }
    }
    return 1;
}

static int test_dcv_7005_without_class_bits_stays_class0(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 7, 0, 0, 5, 0x00, 0x00, 0x02, 0x00, 0);
    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.raw_bcd == 7005);
    ASSERT(meter_reading.dbg_frame[6] == 0x07);
    ASSERT(meter_reading.decimal_pos == 0);
    ASSERT_STR_EQ(meter_reading.display_str, "7005");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 7005.0f, 0.001f));
    return 1;
}

static int test_dcv_range_frames_are_not_latched_from_acv_mains(void)
{
    static const uint8_t mains_frame[12] = {
        0x5A, 0xA5, 0xA5, 0xAD, 0xED, 0x9F,
        0x0F, 0x00, 0x02, 0x00, 0x00, 0x31,
    };
    static const uint8_t dcv_frame[12] = {
        0x5A, 0xA5, 0xC6, 0xF7, 0xEB, 0xEB,
        0x0F, 0x00, 0x02, 0x00, 0x01, 0x4E,
    };

    meter_data_init();
    process_frame(mains_frame, 0);
    ASSERT(expect_normal_reading("228.3", "V", 228.3f, 0.05f));
#if METER_HAS_AC_EVIDENCE_GATE
    ASSERT(close_to(meter_reading.aux_freq_hz, 49.0f, 0.1f));
#endif

    process_frame(dcv_frame, 0);
    ASSERT(meter_reading.raw_bcd == 5008);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT(expect_normal_reading("5.008", "V", 5.008f, 0.001f));
#if METER_HAS_AC_EVIDENCE_GATE
    ASSERT(close_to(meter_reading.aux_freq_hz, 0.0f, 0.001f));
#endif
    return 1;
}

static int test_frame6_0x40_is_not_a_global_resistance_family_marker(void)
{
    /*
     * Resistance kOhm frames use frame[6] upper nibble 4 in the current RE
     * notes, but that byte is not a standalone cross-mode family marker:
     * current-family fixtures also use 0x4x as a status/hold-bearing frame.
     * Guard this explicitly so future work does not "solve" wrong-family
     * leakage by turning one frame byte into a magnitude/range classifier.
     */
    uint8_t current_frame[12];

    build_segment_frame(current_frame, 1, 8, 6, 3,
                        0x40, 0x00, 0x00, 0x00, 0);

    meter_data_init();
    process_frame(current_frame, 2);

    ASSERT(expect_normal_reading("18.63", "mA", 18.63f, 0.001f));
    ASSERT(meter_reading.is_hold);
#if METER_HAS_FRAME_FAMILY_GATE
    ASSERT(meter_reading.expected_frame_family ==
           (uint8_t)FPGA_METER_FRAME_FAMILY_CURRENT);
    ASSERT(meter_reading.observed_frame_family ==
           (uint8_t)FPGA_METER_FRAME_FAMILY_CURRENT);
    ASSERT(meter_reading.reject_reason == METER_REJECT_NONE);
#endif
    return 1;
}

static int test_voltage_mode_mains_frame_uses_stock_range_hint(void)
{
    static const uint8_t frame[12] = {
        0x5A, 0xA5, 0xA5, 0xAD, 0xED, 0x9F,
        0x0F, 0x00, 0x02, 0x00, 0x00, 0x31,
    };

    meter_data_init();
    process_frame(frame, 0);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT(meter_reading.raw_bcd == 2283);
    ASSERT(meter_reading.decimal_pos == 3);
    ASSERT_STR_EQ(meter_reading.display_str, "228.3");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 228.3f, 0.05f));
#if METER_HAS_AC_EVIDENCE_GATE
    ASSERT(close_to(meter_reading.aux_freq_hz, 49.0f, 0.1f));
#endif
    return 1;
}

static int test_dcv_high_range_frame_stays_voltage_across_current_transition(void)
{
    static const uint8_t dcv_high_frame[12] = {
        0x5A, 0xA5, 0xA5, 0xAD, 0xED, 0x9F,
        0x0F, 0x00, 0x02, 0x00, 0x00, 0x31,
    };
    uint8_t current_frame[12];

    build_segment_frame(current_frame, 2, 2, 6, 1, 0x00,
                        0x00, 0x00, 0x00, 0);

    meter_data_init();
    process_frame(dcv_high_frame, 0);
    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT(meter_reading.raw_bcd == 2283);
    ASSERT(meter_reading.decimal_pos == 3);
    ASSERT_STR_EQ(meter_reading.display_str, "228.3");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
    ASSERT(close_to(meter_reading.value, 228.3f, 0.05f));

    process_frame(current_frame, 2);
    ASSERT(expect_normal_reading("22.61", "mA", 22.61f, 0.001f));
    ASSERT(meter_reading.decimal_pos == 2);
    return 1;
}

static int test_voltage_mode_mains_rotating_frames_stay_high_voltage(void)
{
    static const uint8_t frames[][12] = {
        { 0x5A, 0xA5, 0xA5, 0xAD, 0xED, 0xF7, 0x07, 0x00, 0x02, 0x00, 0x00, 0x32 },
        { 0x5A, 0xA5, 0xA5, 0xAD, 0xED, 0x97, 0x0A, 0x00, 0x02, 0x00, 0x00, 0x32 },
        { 0x5A, 0xA5, 0xA5, 0xAD, 0xED, 0x5F, 0x0E, 0x00, 0x02, 0x00, 0x00, 0x31 },
    };

    meter_data_init();
    for (unsigned i = 0; i < sizeof(frames) / sizeof(frames[0]); i++) {
        process_frame(frames[i], 0);
        ASSERT(meter_reading.valid);
        ASSERT(meter_reading.decimal_pos == 3);
        ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
        ASSERT(meter_reading.value > 200.0f);
        ASSERT(meter_reading.value < 260.0f);
    }
    return 1;
}

static int test_continuity_frame_sets_beep_from_segment_pattern(void)
{
    uint8_t frame[12];

    build_segment_frame(frame, 0, 0x12, 0x0A, 5, 0x00, 0x00, 0x00, 0x00, 0);
    meter_data_init();
    process_frame(frame, 7);

    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_CONTINUITY);
    ASSERT(meter_reading.continuity_beep);
    ASSERT(meter_reading.dbg_raw_digits[0] == 0);
    ASSERT(meter_reading.dbg_raw_digits[1] == 0x12);
    ASSERT(meter_reading.dbg_raw_digits[2] == 0x0A);
    ASSERT(meter_reading.dbg_raw_digits[3] == 5);
    return 1;
}

static int test_current_submodes_do_not_expose_unproven_microamp_unit(void)
{
    uint8_t current_frame[12];
    uint8_t ac_current_frame[12];
    static const uint8_t modes[] = { 2, 3, 4, 5 };

    build_segment_frame(current_frame, 2, 2, 6, 1, 0x00,
                        0x00, 0x00, 0x00, 0);
    build_segment_frame(ac_current_frame, 2, 2, 6, 1, 0x00,
                        0x00, 0x00, 0x00, 0x0031);

    for (unsigned i = 0; i < sizeof(modes); i++) {
        uint8_t mode = modes[i];
        const uint8_t *frame = (mode == 4 || mode == 5) ?
                               ac_current_frame : current_frame;

        meter_data_init();
        process_frame(frame, mode);
        ASSERT(meter_reading.valid);
        ASSERT_STR_EQ(meter_reading.unit_suffix,
                      (mode == 2 || mode == 4) ? "mA" : "A");
        ASSERT(strcmp(meter_reading.unit_suffix, "uA") != 0);
    }
    return 1;
}

/* ═══════════════════════════════════════════════════════════════════
 * Port-specific fixtures: behaviour this port has and upstream does not
 * (or has withdrawn). These pin the CURRENT port, so the meter-session
 * refactor and future feature ports change them consciously, not silently.
 * ═══════════════════════════════════════════════════════════════════ */

/* The PLAN.md DCV exponent fixture table (host-run 7/7 from the 2026-08-23
 * session), now executed against this tree's meter_data.c instead of a
 * scratch compile. Also pins the dcv_stock_class debug field ("V<class>"
 * on the meter debug line). */
static int test_plan_dcv_exponent_fixture_table(void)
{
    struct fixture {
        uint8_t d[4];       /* digit codes */
        bool extend;        /* frame[2].3 */
        uint8_t class_id;   /* forced class */
        int raw;
        const char *display;
        float value;
    };
    static const struct fixture fixtures[] = {
        { {7, 0, 0, 5}, false, 3,  7005, "7.005",  7.0050f }, /* bench 04-04 */
        { {5, 0, 0, 8}, false, 3,  5008, "5.008",  5.0080f }, /* bench 04-04 */
        { {0, 9, 9, 7}, false, 2,   997, "9.97",   9.9700f }, /* 11 V case */
        { {0, 9, 9, 7}, false, 3,   997, "0.997",  0.9970f }, /* same, class 3 */
        { {9, 9, 9, 9}, true,  4, 19999, "1.9999", 1.9999f }, /* +10000 */
        { {9, 9, 9, 9}, false, 0,  9999, "9999",   9999.0f },
        { {9, 9, 9, 9}, false, 1,  9999, "999.9",  999.9f  },
    };

    for (unsigned i = 0; i < sizeof(fixtures) / sizeof(fixtures[0]); i++) {
        const struct fixture *f = &fixtures[i];
        uint8_t frame[12];

        build_segment_frame(frame, f->d[0], f->d[1], f->d[2], f->d[3],
                            0x00, 0x00, 0x02, 0x00, 0);
        if (f->extend) frame[2] |= 0x08U;
        switch (f->class_id) {
        case 4: frame[8] |= 0x80U; break;
        case 3: frame[3] |= 0x10U; break;
        case 2: frame[4] |= 0x10U; break;
        case 1: frame[5] |= 0x10U; break;
        default: break;
        }

        meter_data_init();
        process_frame(frame, 0);

        ASSERT(meter_reading.valid);
        ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
        ASSERT(meter_reading.raw_bcd == f->raw);
        ASSERT(meter_reading.dcv_stock_class == f->class_id);
        ASSERT_STR_EQ(meter_reading.display_str, f->display);
        ASSERT_STR_EQ(meter_reading.unit_suffix, "V");
        ASSERT(close_to(meter_reading.value, f->value, 0.0002f));
    }
    return 1;
}

/* This port's per-submode formatting defaults, from the bench captures the
 * default_decimal_pos table cites. Upstream's stock FSM formats diode as
 * XXX.X and temperature via its own table; this port renders X.XXX for
 * both until the FSM is ported — pinned here so the difference is a
 * conscious one. */
static int test_port_formatting_defaults_per_submode(void)
{
    uint8_t frame[12];

    meter_data_init();
    build_segment_frame(frame, 1, 2, 3, 4, 0x00, 0x00, 0x00, 0x00, 0);
    process_frame(frame, 2);
    ASSERT(meter_reading.decimal_pos == 2);
    ASSERT(expect_normal_reading("12.34", "mA", 12.34f, 0.001f));

    process_frame(frame, 3);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT(expect_normal_reading("1.234", "A", 1.234f, 0.001f));

    process_frame(frame, 4);
    ASSERT(meter_reading.decimal_pos == 2);
    ASSERT(expect_normal_reading("12.34", "mA", 12.34f, 0.001f));

    process_frame(frame, 5);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT(expect_normal_reading("1.234", "A", 1.234f, 0.001f));

    process_frame(frame, 9);
    ASSERT(meter_reading.decimal_pos == 3);
    ASSERT(expect_normal_reading("123.4", "nF", 123.4f, 0.001f));

    process_frame(frame, 10);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT(expect_normal_reading("1.234", "C", 1.234f, 0.001f));

    build_segment_frame(frame, 0, 6, 2, 3, 0x00, 0x00, 0x00, 0x00, 0);
    process_frame(frame, 8);
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT(expect_normal_reading("0.623", "V", 0.623f, 0.001f));
    return 1;
}

/* The low-ohm band calibration this port still applies (unit #1 factor
 * 0.0304) and the kOhm band identity. Upstream withdrew the low-ohm factor
 * and fails closed; PLAN.md carries this divergence explicitly ("on the
 * same resistor our firmware shows ~147 Ohm, theirs shows ---"). */
static int test_port_resistance_band_calibration_override(void)
{
    uint8_t frame[12];

    meter_data_init();
    /* The range lives in frame[7] bits 3:2, not in frame[6] — measured on
     * unit #2, 2026-09-06, where a shorted probe and a 300 kOhm resistor both
     * report frame[6] upper nibble 0. Low-Ohm range: bit 3. */
    build_segment_frame(frame, 4, 8, 2, 4, 0x00, 0x08, 0x00, 0x00, 0);
    process_frame(frame, 6);
    ASSERT(meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NORMAL);
    ASSERT_STR_EQ(meter_reading.unit_suffix, "Ohm");
    /* 4824 * 0.0304 = 146.65 */
    ASSERT(close_to(meter_reading.value, 146.65f, 0.05f));

    /* High range: bit 2, 100 Ohm per count. This is the bench failure of
     * 2026-09-06 — a 300 kOhm resistor read 90.62 Ohm because raw 2981 took
     * the low-Ohm coefficient. Measured on hardware: raw 2981 -> 298.1 kOhm,
     * and 470 kOhm -> raw 4666 -> 466.6 kOhm. */
    build_segment_frame(frame, 2, 9, 8, 1, 0x00, 0x04, 0x00, 0x00, 0);
    process_frame(frame, 6);
    ASSERT(meter_reading.valid);
    ASSERT_STR_EQ(meter_reading.unit_suffix, "kOhm");
    ASSERT(close_to(meter_reading.value, 298.1f, 0.05f));

    build_segment_frame(frame, 3, 3, 0, 0, 0x40, 0x00, 0x00, 0x00, 0);
    process_frame(frame, 6);
    ASSERT(meter_reading.valid);
    ASSERT_STR_EQ(meter_reading.unit_suffix, "kOhm");
    ASSERT(close_to(meter_reading.value, 3.300f, 0.001f));
    ASSERT_STR_EQ(meter_reading.display_str, "3.300");
    return 1;
}

/* Band-latch behaviour under the session API:
 * (a) a recognized resistance band frame latches dp/unit, and a following
 *     unknown-band frame in the SAME session reuses the latch instead of
 *     the static default;
 * (b) meter_session_begin() is the latch's reset — a re-entry into the
 *     same mode starts clean instead of inheriting the previous session's
 *     dp/unit, which is what the old function-local statics did. */
static int test_port_band_latch_reuses_last_band_within_session(void)
{
    uint8_t kohm_frame[12];
    uint8_t unknown_band_frame[12];

    build_segment_frame(kohm_frame, 3, 3, 0, 0, 0x40, 0x00, 0x00, 0x00, 0);
    /* Upper nibble 2 is a band the decoder does not recognize. */
    build_segment_frame(unknown_band_frame, 3, 3, 0, 0, 0x20, 0x00, 0x00, 0x00, 0);

    meter_session_begin(&session, 6);
    meter_session_frame(&session, kohm_frame);
    ASSERT_STR_EQ(meter_reading.unit_suffix, "kOhm");
    ASSERT(close_to(meter_reading.value, 3.300f, 0.001f));

    meter_session_frame(&session, unknown_band_frame);
    /* Latch reused: still kOhm at dp 1, not the mode-6 static default. */
    ASSERT_STR_EQ(meter_reading.unit_suffix, "kOhm");
    ASSERT(meter_reading.decimal_pos == 1);
    ASSERT(close_to(meter_reading.value, 3.3f, 0.001f));
    return 1;
}

static int test_session_begin_resets_band_latch_on_same_mode_reentry(void)
{
    uint8_t kohm_frame[12];
    uint8_t unknown_band_frame[12];

    build_segment_frame(kohm_frame, 3, 3, 0, 0, 0x40, 0x00, 0x00, 0x00, 0);
    /* Unknown range = both frame[7] range bits at once, a pattern the bench
     * has never seen (it used to be frame[6] upper nibble 2, back when the
     * band was read from frame[6]). */
    build_segment_frame(unknown_band_frame, 3, 3, 0, 0, 0x20, 0x0C, 0x00, 0x00, 0);

    meter_session_begin(&session, 6);
    meter_session_frame(&session, kohm_frame);
    ASSERT_STR_EQ(meter_reading.unit_suffix, "kOhm");

    /* Re-enter the SAME mode: dmm_reenter() -> begin_transition() starts a
     * fresh session. The reading is dropped... */
    meter_session_begin(&session, 6);
    ASSERT(!meter_reading.valid);
    ASSERT(meter_reading.result_class == METER_RESULT_NONE);
    ASSERT_STR_EQ(meter_reading.display_str, "---");
    ASSERT_STR_EQ(meter_reading.unit_suffix, "");

    /* ...and so is the latch: an unknown-band frame now falls back to the
     * mode-6 static default (dp 2, Ohm) instead of the previous session's
     * kOhm decision. */
    meter_session_frame(&session, unknown_band_frame);
    ASSERT_STR_EQ(meter_reading.unit_suffix, "Ohm");
    ASSERT(meter_reading.decimal_pos == 2);
    ASSERT_STR_EQ(meter_reading.display_str, "33.00");
    ASSERT(close_to(meter_reading.value, 33.0f, 0.001f));
    return 1;
}

/* ═══════════════════════════════════════════════════════════════════
 * Skipped upstream cases — bodies compiled out until their feature macro
 * flips. Kept in upstream's main() order; see the macro block on top for
 * what each gap is.
 * ═══════════════════════════════════════════════════════════════════ */

#if METER_HAS_FRAME_FAMILY_GATE || METER_HAS_AC_EVIDENCE_GATE || \
    METER_HAS_INVALIDATE || METER_HAS_STOCK_FSM || METER_HAS_ZERO_DETECT || \
    METER_HAS_SPECIAL_PAYLOAD_CLEAR || METER_HAS_DIGIT_READ_HARDENING || \
    METER_HAS_UNRESOLVED_LOW_OHM_REJECT || METER_HAS_SNAPSHOT
/*
 * When the first feature macro flips to 1, port the matching cases from
 * upstream firmware/tests/test_meter_data.c into this section (they are
 * deliberately not carried as dead source: upstream's copy is the
 * authoritative one to diff against, and a stale duplicate here would
 * drift). The SKIP entries in main() name every case and its gap.
 */
#error "Port the revived cases from upstream tests/test_meter_data.c"
#endif

int main(void)
{
    printf("Meter data frame tests (2C53T port)\n");

    TEST(segment_frame_builder_exercises_cross_byte_lookup);
    TEST(dcv_5v_frame_keeps_verified_decimal_and_unit);
    TEST(dcv_live_5v_frame_uses_stock_range_hint);
    TEST(dcv_live_32v_frame_uses_stock_range_hint);
    TEST(dcv_1v2949_frame_uses_stock_extended_raw_and_class4);
    TEST(dcv_1v4979_frame_uses_stock_extended_raw_and_class4);
    TEST(dcv_live_1v5_frame_uses_stock_extended_raw_and_class4);
    TEST(dcv_live_1v5_rotating_frames_keep_stock_class4);
    SKIP(dcv_live_0200_bare_class_bit_fails_closed,
         "frame-family gate not ported: bare class bit renders instead of failing closed");
    TEST(dcv_stock_range_class_priority_table);
    TEST(dcv_stock_range_class_priority_all_bit_combinations);
    TEST(dcv_class4_priority_requires_frame8_bit7);
    TEST(dcv_synthetic_5008_without_class_bits_stays_class0);
    TEST(dcv_extra_frequency_hint_does_not_set_voltage_range);
    TEST(dcv_aux_extra_bytes_do_not_change_stock_range_class);
    TEST(dcv_7005_without_class_bits_stays_class0);
    TEST(dcv_range_frames_are_not_latched_from_acv_mains);
    SKIP(acv_mains_frame_uses_high_voltage_scale_and_frequency,
         "stock FSM ACV range formatting not ported (port renders ACV at its dp-2 default)");
    SKIP(acv_rejects_dc_voltage_without_ac_evidence,
         "AC-evidence gate not ported");
    SKIP(acv_rejects_live_low_dcv_status24_without_frequency_hint,
         "frame-family + AC-evidence gates not ported");
    SKIP(ac_current_rejects_current_frame_without_ac_evidence,
         "AC-evidence gate not ported");
    SKIP(ac_modes_require_frequency_hint_boundaries,
         "AC-evidence gate not ported (no aux_freq_hz, no 45-65 Hz proof)");
    SKIP(stock_formatter_families_have_regression_fixtures,
         "stock FSM formatter not ported (diode/temperature dp differ); port defaults pinned below");
    SKIP(passive_formatter_debug_fields_cover_diode_and_extended_splits,
         "stock FSM debug fields not ported");
    SKIP(resistance_low_ohm_fails_closed_without_factory_cal,
         "divergence: upstream withdrew the 0.0304 low-ohm cal and fails closed; port still applies it (see PLAN.md)");
    SKIP(invalidate_clears_stale_reading_before_mode_transition,
         "meter_data_invalidate not ported (this port resets via meter_session_begin; case also needs stock/family fields)");
    SKIP(invalidate_clears_stale_reading_for_every_submode,
         "meter_data_invalidate not ported");
    SKIP(parser_stock_mode_tracks_transition_plan_for_every_submode,
         "meter_data_invalidate + stock_mode field not ported");
    SKIP(invalid_submode_rejects_without_becoming_dcv,
         "frame-family gate not ported: invalid submode falls back to dp-1 render");
    SKIP(invalid_submode_rejects_every_frame_family_corpus,
         "frame-family gate not ported");
    SKIP(state_machine_property_matrix_covers_all_submodes,
         "frame-family gate + invalidate not ported");
    SKIP(goal_surface_property_enumerates_dmm_state_machine,
         "frame-family gate + invalidate not ported (rx gate half lives in test_meter_plan)");
    SKIP(invalidate_clears_stale_payload_for_every_ordered_mode_transition,
         "meter_data_invalidate not ported");
    SKIP(transport_gate_blocks_source_frames_during_every_transition,
         "meter_data_invalidate not ported (rx gate itself is covered in test_meter_plan)");
    SKIP(transition_phase_marker_frames_follow_destination_state,
         "frame-family gate + invalidate not ported");
    SKIP(marker_visible_family_mismatch_matrix_clears_stale_payload,
         "frame-family gate not ported");
    SKIP(unclassified_normal_frames_follow_active_family_only,
         "frame-family gate not ported");
    SKIP(dcv_rejects_live_unmarked_frame8_00_regression,
         "frame-family gate not ported: unmarked frame[8]=00 DCV frames still render");
    SKIP(dcv_rejects_live_status20_class_bit_wrong_family,
         "frame-family gate not ported");
    SKIP(dcv_rejects_live_status20_marker_frame_04366_regression,
         "frame-family gate not ported");
    SKIP(dcv_rejects_reported_low_input_numeric_shapes_without_marker,
         "frame-family gate not ported");
    SKIP(dcv_zero_detect_short_frame_reports_zero_not_low_input_number,
         "zero-detect short frames not ported");
    SKIP(passive_zero_detect_short_frames_report_short_state,
         "zero-detect short frames not ported");
    SKIP(zero_detect_short_frames_report_terminal_zero_modes,
         "zero-detect short frames not ported");
    SKIP(zero_detect_does_not_clamp_nonzero_range_hunting_frame,
         "frame-family gate not ported");
    SKIP(acv_rejects_unimplemented_plus_10000_extension,
         "frame-family gate not ported: non-DCV +10000 is silently ignored, not rejected");
    SKIP(current_rejects_unimplemented_plus_10000_extension,
         "frame-family gate not ported: non-DCV +10000 is silently ignored, not rejected");
    SKIP(mixed_special_voltage_digits_do_not_become_numeric_dcv,
         "digit-read hardening not ported: special codes clamp to 0 instead of ERR");
    TEST(frame6_0x40_is_not_a_global_resistance_family_marker);
    TEST(voltage_mode_mains_frame_uses_stock_range_hint);
    TEST(dcv_high_range_frame_stays_voltage_across_current_transition);
    TEST(voltage_mode_mains_rotating_frames_stay_high_voltage);
    SKIP(acv_repeated_rotating_range_frames_do_not_drop_to_2v,
         "stock FSM ACV range formatting not ported");
    TEST(continuity_frame_sets_beep_from_segment_pattern);
    SKIP(continuity_marker_rejected_outside_continuity_mode,
         "frame-family gate not ported: continuity pattern accepted in every submode");
    SKIP(non_continuity_terminal_frames_clear_stale_beep,
         "special-frame payload/beep clear not ported: partial-blank/ERR leave a stale beep");
    SKIP(special_frames_clear_stale_aux_frequency,
         "AC-evidence gate not ported (no aux_freq_hz field)");
    SKIP(special_frames_clear_stale_payload_fields,
         "special-frame payload clear not ported: OL/blank/ERR leave stale raw_bcd/digits");
    SKIP(non_voltage_modes_reject_voltage_payloads,
         "frame-family gate not ported");
    SKIP(voltage_payload_clears_stale_reading_in_all_non_voltage_modes,
         "frame-family gate not ported");
    SKIP(special_voltage_family_frames_are_rejected_outside_voltage,
         "frame-family gate not ported");
    SKIP(voltage_payload_clears_stale_current_reading,
         "frame-family gate not ported");
    SKIP(low_dcv_voltage_payload_clears_stale_current_reading,
         "frame-family gate not ported");
    SKIP(stock_fsm_debug_fields_follow_mode_and_frames,
         "stock FSM debug fields not ported");
    SKIP(large_current_submodes_use_active_local_range_state,
         "stock FSM debug fields not ported (dp/display half pinned in port fixtures)");
    TEST(current_submodes_do_not_expose_unproven_microamp_unit);
    SKIP(snapshot_returns_coherent_latest_completed_reading,
         "meter_data_snapshot not ported");
    SKIP(snapshot_rejects_concurrent_two_writer_window,
         "meter_data_snapshot + test hooks not ported");

    printf("\n  --- port-specific fixtures ---\n");
    TEST(plan_dcv_exponent_fixture_table);
    TEST(port_formatting_defaults_per_submode);
    TEST(port_resistance_band_calibration_override);
    TEST(port_band_latch_reuses_last_band_within_session);
    TEST(session_begin_resets_band_latch_on_same_mode_reentry);

    printf("\n%d/%d passed, %d skipped (unported upstream features)\n",
           tests_passed, tests_run - tests_skipped, tests_skipped);
    return (tests_passed == tests_run - tests_skipped) ? 0 : 1;
}
