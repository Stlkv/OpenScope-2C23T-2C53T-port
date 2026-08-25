#include "cdc_shell.h"

#include "dbgdump.h"
#include "dmm.h"
#include "fw_cache.h"
#include "fw_update.h"
#include "usb_cdc.h"

#if !defined(USB_CDC_SHELL) || USB_CDC_SHELL

#ifndef FW_VERSION_TEXT
#define FW_VERSION_TEXT "v2026.07.1"
#endif

#ifndef APP_BASE_ADDR
#define APP_BASE_ADDR 0x08007000u
#endif

enum {
    SH_LINE_MAX = 72,
    SH_DBG_MAX = 2048,   /* same cap as the DBG.TXT path, same text */
    SH_CHUNK = 64,
    MON_MIN_MS = 100u,
    MON_MAX_MS = 60000u,
};

static char sh_line[SH_LINE_MAX];
static uint8_t sh_line_len;
static uint8_t sh_line_lost;      /* the line overran; drop it at the newline */
static uint8_t sh_was_open;

/* The dump is rendered once, then fed to the ring as room appears: a 2 KB
 * blob does not fit the TX ring and must not be able to block the main loop
 * waiting for the host to read. */
static char sh_dbg[SH_DBG_MAX];
static uint16_t sh_dbg_len;
static uint16_t sh_dbg_pos;

static uint32_t sh_uptime_ms;
static uint32_t mon_period_ms;
static uint32_t mon_elapsed_ms;
static uint16_t mon_skipped;      /* a period fell due while a dump was mid-flight */

/* Raw intake (fwload): every byte after the command line is payload. */
static uint32_t raw_remaining;
static uint32_t raw_expect_crc;
static uint32_t raw_drain;        /* payload to swallow after a failed intake */
static uint8_t raw_have_crc;
static uint8_t raw_slot;
static uint8_t raw_active;
static uint8_t raw_staged;   /* the last fwload verified; fwapply may install it */

/* A torn transfer has to hand the shell back by itself, exactly as upstream's
 * fw_loader.c does after FWL_TIMEOUT_POLLS of silence: without this a host that
 * dies mid-stream (or an operator who types a command into an armed intake)
 * leaves the port mute until the next power cycle. The tick is nominally 20 ms
 * and never faster, so this is a floor of 3 s, not a ceiling. */
#define RAW_SILENCE_MS 3000u
static uint16_t raw_silence_ms;

/* A swap reflashes the app slot and resets, so the verdict line has to reach
 * the host first: the request waits for the TX ring to drain, with a deadline
 * in case nobody is reading. */
static uint8_t swap_pending;
static uint8_t swap_pending_slot;
static uint16_t swap_wait_ms;

/* ─── output ─────────────────────────────────────────────────────────── */

static void sh_out(const char *s) {
    (void)usb_cdc_write_str(s);
}

static void sh_u32(uint32_t v) {
    char buf[11];
    uint8_t n = 0;
    if (!v) {
        sh_out("0");
        return;
    }
    while (v && n < sizeof(buf)) {
        buf[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (n) {
        char c = buf[--n];
        (void)usb_cdc_write(&c, 1);
    }
}

static void sh_hex(uint32_t v, uint8_t digits) {
    static const char hexd[] = "0123456789abcdef";
    char buf[8];
    uint8_t i;
    if (digits > 8u) {
        digits = 8u;
    }
    for (i = 0; i < digits; ++i) {
        buf[digits - 1u - i] = hexd[(v >> (4u * i)) & 0xFu];
    }
    (void)usb_cdc_write(buf, digits);
}

/* ─── parsing ────────────────────────────────────────────────────────── */

static const char *skip_spaces(const char *p) {
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    return p;
}

static uint8_t word_matches(const char *line, const char *word, const char **rest) {
    uint16_t i = 0;
    while (word[i]) {
        if (line[i] != word[i]) {
            return 0;
        }
        ++i;
    }
    if (line[i] != '\0' && line[i] != ' ' && line[i] != '\t') {
        return 0;
    }
    *rest = skip_spaces(line + i);
    return 1;
}

static uint8_t parse_u32(const char **p, uint32_t *out) {
    const char *s = skip_spaces(*p);
    uint32_t v = 0;
    uint8_t digits = 0;
    while (*s >= '0' && *s <= '9') {
        v = v * 10u + (uint32_t)(*s - '0');
        ++s;
        if (++digits > 10u) {
            return 0;
        }
    }
    if (!digits) {
        return 0;
    }
    *p = s;
    *out = v;
    return 1;
}

static uint8_t parse_hex32(const char **p, uint32_t *out) {
    const char *s = skip_spaces(*p);
    uint32_t v = 0;
    uint8_t digits = 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        s += 2;
    }
    for (;;) {
        uint8_t d;
        if (*s >= '0' && *s <= '9') {
            d = (uint8_t)(*s - '0');
        } else if (*s >= 'a' && *s <= 'f') {
            d = (uint8_t)(*s - 'a' + 10);
        } else if (*s >= 'A' && *s <= 'F') {
            d = (uint8_t)(*s - 'A' + 10);
        } else {
            break;
        }
        v = (v << 4) | d;
        ++s;
        if (++digits > 8u) {
            return 0;
        }
    }
    if (!digits) {
        return 0;
    }
    *p = s;
    *out = v;
    return 1;
}

static uint8_t parse_slot(const char **p, uint8_t *slot) {
    const char *s = skip_spaces(*p);
    if (*s == 'a' || *s == 'A') {
        *slot = FW_CACHE_SLOT_A;
    } else if (*s == 'b' || *s == 'B') {
        *slot = FW_CACHE_SLOT_B;
    } else {
        return 0;
    }
    *p = skip_spaces(s + 1);
    return 1;
}

/* ─── commands ───────────────────────────────────────────────────────── */

static void dbg_stream_start(void) {
    sh_dbg_len = dbgdump_render(sh_dbg, (uint16_t)sizeof(sh_dbg));
    sh_dbg_pos = 0;
}

static void cmd_help(const char *args) {
    (void)args;
    sh_out("commands:\r\n"
           "  help | ?                     this list\r\n"
           "  version                      firmware id and USB functions\r\n"
           "  status                       shell/link health, one line\r\n"
           "  dbg                          the DBG.TXT telemetry text, once\r\n"
           "  mon [ms|off]                 repeat dbg every ms (default 500)\r\n"
           "  fwstat | fw                  update + A/B cache state\r\n"
           "  meter                        raw DMM frame, counters, decoded reading\r\n"
           "  fwload <size> <crc32> [a|b]  stage an image into cache slot a/b\r\n"
           "    then stream exactly <size> raw bytes (cdc_flash.py does both)\r\n"
           "  fwapply                      install what fwload just staged\r\n"
           "  fwswap a|b                   install a cached image, no transfer\r\n"
           "  uptime                       ms since boot\r\n");
}

static void cmd_version(const char *args) {
    (void)args;
    sh_out("OpenScope 2C53T port " FW_VERSION_TEXT " app=0x");
    sh_hex(APP_BASE_ADDR, 8);
    sh_out(" usb=MSC+CDC\r\n");
}

static void cmd_status(const char *args) {
    fw_update_status_t st;
    (void)args;
    fw_update_status(&st);
    sh_out("up=");
    sh_u32(sh_uptime_ms);
    sh_out("ms mon=");
    sh_u32(mon_period_ms);
    sh_out(" skip=");
    sh_u32(mon_skipped);
    sh_out(" dbg=");
    sh_u32(sh_dbg_pos);
    sh_out("/");
    sh_u32(sh_dbg_len);
    sh_out(" txfree=");
    sh_u32(usb_cdc_tx_free());
    sh_out(" raw=");
    sh_u32(raw_active ? raw_remaining : 0u);
    sh_out(" fwst=");
    sh_u32(st.state);
    sh_out("\r\n");
}

static void cmd_dbg(const char *args) {
    (void)args;
    dbg_stream_start();
}

static void cmd_mon(const char *args) {
    const char *p = args;
    uint32_t ms;

    if (*p == '\0') {
        ms = 500u;
    } else if (word_matches(p, "off", &p)) {
        mon_period_ms = 0;
        mon_elapsed_ms = 0;
        sh_out("mon off\r\n");
        return;
    } else if (!parse_u32(&p, &ms)) {
        sh_out("mon: want ms or off\r\n");
        return;
    }
    if (ms < MON_MIN_MS) {
        ms = MON_MIN_MS;
    }
    if (ms > MON_MAX_MS) {
        ms = MON_MAX_MS;
    }
    mon_period_ms = ms;
    mon_elapsed_ms = 0;
    mon_skipped = 0;
    sh_out("mon every ");
    sh_u32(ms);
    sh_out("ms\r\n");
    dbg_stream_start();
}

/* ─── firmware over the cable ────────────────────────────────────────────
 * The wire protocol here is NOT new: it is the one this project shipped to
 * upstream in DavidClawson/OpenScope-2C53T#29 (fw_loader.c + scripts/
 * cdc_flash.py), down to the tokens that host script greps for — `GO ` to
 * accept the transfer, `fwload:` + `STAGED` for a verified slot, `ERROR` to
 * abort a stream mid-flight, and the word `recovery` in the goodbye. That is
 * the whole point of the standing goal: ONE host tool drives both firmwares.
 *
 * What differs is underneath, and only because the two firmwares differ:
 * theirs stages through fw_loader.c on the vendor USB stack, ours goes
 * straight into the same W25Q A/B cache slots (0xD00000 / 0xE00000) through
 * fw_cache.c — same map, same manifest-written-last rule, same RAM installer
 * that SYSTEM-RESETS instead of jumping (the half-brick of 2026-08-22). */

static void cmd_fwstat(const char *args) {
    fw_update_status_t st;
    uint8_t slot;
    (void)args;

    fw_update_status(&st);
    sh_out("FWU st=");
    sh_u32(st.state);
    sh_out(" err=");
    sh_u32(st.error);
    sh_out(" bytes=");
    sh_u32(st.bytes);
    sh_out("/");
    sh_u32(st.expected_size);
    sh_out("\r\n");

    for (slot = FW_CACHE_SLOT_A; slot <= FW_CACHE_SLOT_B; ++slot) {
        sh_out(slot == FW_CACHE_SLOT_A ? "FWC A size=" : "FWC B size=");
        sh_u32(fw_cache_slot_size(slot));
        sh_out(" crc=");
        sh_hex(fw_cache_slot_crc(slot), 8);
        sh_out("\r\n");
    }
    sh_out("FWC in=0x");
    sh_hex(fw_cache_intake_status(), 2);
    sh_out(" sw=0x");
    sh_hex(fw_cache_swap_status(), 2);
    sh_out("\r\n");
}

/* fwload <size decimal> <crc32 hex> [a|b] — argument order, the default slot
 * b ("the other firmware" by custom) and the mandatory CRC all match upstream.
 * The CRC is mandatory because it is the only end-to-end check there is: the
 * manifest CRC is computed over whatever arrived and would bless a corrupted
 * image just as happily. */
static void cmd_fwload(const char *args) {
    const char *p = args;
    uint32_t size;
    uint32_t crc = 0;
    uint8_t slot = FW_CACHE_SLOT_B;

    if (raw_active || raw_drain) {
        sh_out("fwload: ERROR a transfer is already in flight\r\n");
        return;
    }
    if (!parse_u32(&p, &size) || !parse_hex32(&p, &crc) || crc == 0u) {
        sh_out("usage: fwload <size bytes, decimal> <crc32 hex> [a|b]\r\n"
               "       then stream exactly that many raw bytes\r\n");
        return;
    }
    /* Absent = slot b, as upstream. PRESENT BUT NOT a|b = refuse: upstream's
     * parser (and this one, until 2026-08-24) quietly fell back to the default
     * instead, so `fwload <size> <crc> c` armed slot b and put the shell in
     * raw mode while the operator was still reading the reply. The next line
     * typed then goes into the image, not the parser. Proposed upstream too. */
    p = skip_spaces(p);
    if (*p != '\0' && (!parse_slot(&p, &slot) || *skip_spaces(p) != '\0')) {
        sh_out("fwload: ERROR bad slot, expected a or b\r\n");
        return;
    }

    if (!fw_cache_intake_begin(slot, size)) {
        sh_out("fwload: ERROR refused, in=0x");
        sh_hex(fw_cache_intake_status(), 2);
        sh_out("\r\n");
        return;
    }
    raw_slot = slot;
    raw_remaining = size;
    raw_expect_crc = crc;
    raw_have_crc = 1;
    raw_staged = 0;
    raw_active = 1;
    raw_silence_ms = 0;
    mon_period_ms = 0;      /* a telemetry dump mid-transfer is noise */
    sh_dbg_len = 0;
    sh_dbg_pos = 0;
    sh_out("GO ");
    sh_u32(size);
    sh_out(" bytes to slot ");
    sh_out(slot == FW_CACHE_SLOT_A ? "a" : "b");
    sh_out(", crc ");
    sh_hex(crc, 8);
    sh_out(" expected\r\n");
}

static void raw_finish(void) {
    uint32_t got = fw_cache_intake_crc();

    raw_active = 0;
    if (got != raw_expect_crc) {
        /* Abort BEFORE finish(): no manifest is written, so the slot cannot be
         * installed by mistake. Any older manifest it still carries now fails
         * its own CRC against the overwritten data, which is the same refusal. */
        fw_cache_intake_abort();
        sh_out("fwload: ERROR crc mismatch got=");
        sh_hex(got, 8);
        sh_out(" want=");
        sh_hex(raw_expect_crc, 8);
        sh_out(" (no manifest written)\r\n");
        return;
    }
    if (!fw_cache_intake_finish()) {
        sh_out("fwload: ERROR intake failed in=0x");
        sh_hex(fw_cache_intake_status(), 2);
        sh_out("\r\n");
        return;
    }
    raw_staged = 1;
    sh_out("fwload: STAGED slot=");
    sh_out(raw_slot == FW_CACHE_SLOT_A ? "a" : "b");
    sh_out(" size=");
    sh_u32(fw_cache_slot_size(raw_slot));
    sh_out(" crc=");
    sh_hex(fw_cache_slot_crc(raw_slot), 8);
    sh_out("\r\n");
}

/* Silence on an armed intake: drop it, say so in the tokens the host greps for
 * (`fwload:` for the verdict line, `ERROR` to abort the stream) and let the
 * line editor have the port back. Nothing is installed and no manifest was
 * written, so the slot keeps whatever it held before the transfer began.
 *
 * The dead transfer arms the drain rather than clearing it: a host that only
 * stalled (>3 s mid-image) may resume, and without the drain the rest of the
 * image would be parsed as command lines. The drain then ages against the
 * same silence limit, so this function is also its expiry: a host that is
 * truly gone costs one more timeout, which reports itself and frees the
 * shell. */
static void raw_timeout(void) {
    if (raw_active) {
        raw_active = 0;
        raw_drain = raw_remaining;
        raw_remaining = 0;
        raw_silence_ms = 0;
        fw_cache_intake_abort();
        sh_out("fwload: ERROR rx went silent; run fwload again\r\n");
        return;
    }
    raw_drain = 0;
    sh_out("fwload: stopped waiting for the aborted image; "
           "shell is listening again\r\n");
}

/* Both installers end here. The goodbye carries the word `recovery` because
 * cdc_flash.py waits for that token — and because it is the thing to know. */
static void install_slot(uint8_t slot) {
    if (!fw_cache_slot_size(slot)) {
        sh_out("fwswap: ERROR slot has no valid manifest\r\n");
        return;
    }
    sh_out("verifying slot, then: erase+program+verify from RAM and\r\n"
           "SYSTEM RESET into the image. keep USB attached (it carries\r\n"
           "the rail through the reset). recovery = MENU+Power IAP.\r\n");
    mon_period_ms = 0;
    swap_pending_slot = slot;
    swap_wait_ms = 0;
    swap_pending = 1;
}

static void cmd_fwapply(const char *args) {
    (void)args;
    if (!raw_staged) {
        sh_out("nothing staged — fwload first (or fwswap a|b)\r\n");
        cmd_fwstat("");
        return;
    }
    install_slot(raw_slot);
}

static void cmd_fwswap(const char *args) {
    const char *p = args;
    uint8_t slot;

    if (!parse_slot(&p, &slot)) {
        sh_out("usage: fwswap a|b   (install a cached image, no transfer)\r\n");
        cmd_fwstat("");
        return;
    }
    install_slot(slot);
}

/* Runs from the tick, once the goodbye is out of the ring (or 300 ms have
 * passed, whichever comes first). fw_cache_swap() returns only on refusal. */
static void swap_run(void) {
    swap_pending = 0;
    fw_cache_swap(swap_pending_slot);
    sh_out("fwswap: ERROR refused, sw=0x");
    sh_hex(fw_cache_swap_status(), 2);
    sh_out("\r\n");
}

#if HW_TARGET_2C53T
/* The meter track was parked in August for want of exactly this: the raw frame
 * that is on screen at the moment the reading misbehaves. The overlay lines
 * are the same ones dmm53.c draws (frame bytes, counters, decoder state and
 * the transition plan), so the screen and the cable can never disagree about
 * what arrived or about which selector words produced it. */
static void cmd_meter(const char *args) {
    uint8_t i;
    (void)args;

    for (i = 0; i < 4u; ++i) {
        const char *line = dmm53_debug_line(i);
        if (line) {
            sh_out(line);
            sh_out("\r\n");
        }
    }
    sh_out("read=");
    sh_out(dmm_value_text());
    sh_out(" ");
    sh_out(dmm_unit_text());
    sh_out(" st=");
    sh_out(dmm_status_text());
    sh_out(dmm_reading_is_real() ? " real=1\r\n" : " real=0\r\n");
}
#endif

static void cmd_uptime(const char *args) {
    (void)args;
    sh_u32(sh_uptime_ms);
    sh_out(" ms\r\n");
}

typedef struct {
    const char *name;
    void (*fn)(const char *args);
} sh_cmd_t;

static const sh_cmd_t sh_cmds[] = {
    { "help", cmd_help },
    { "?", cmd_help },
    { "version", cmd_version },
    { "status", cmd_status },
    { "dbg", cmd_dbg },
    { "mon", cmd_mon },
    /* Longest word first: the dispatcher takes the first match, so "fwload"
     * has to be offered before "fw". */
    { "fwload", cmd_fwload },
    { "fwapply", cmd_fwapply },
    { "fwswap", cmd_fwswap },
    { "fwstat", cmd_fwstat },
    { "fw", cmd_fwstat },          /* alias for the impatient */
#if HW_TARGET_2C53T
    { "meter", cmd_meter },
#endif
    { "uptime", cmd_uptime },
    { 0, 0 },
};

static void sh_dispatch(void) {
    const char *line = skip_spaces(sh_line);
    const sh_cmd_t *c;

    if (*line == '\0') {
        return;
    }
    for (c = sh_cmds; c->name; ++c) {
        const char *args;
        if (word_matches(line, c->name, &args)) {
            c->fn(args);
            return;
        }
    }
    sh_out("? ");
    sh_out(sh_line);
    sh_out(" (help)\r\n");
}

/* ─── input ──────────────────────────────────────────────────────────── */

static void sh_line_byte(uint8_t b) {
    if (b == '\r' || b == '\n') {
        sh_out("\r\n");
        if (sh_line_lost) {
            sh_out("line too long\r\n");
        } else {
            sh_line[sh_line_len] = '\0';
            sh_dispatch();
        }
        sh_line_len = 0;
        sh_line_lost = 0;
        return;
    }
    if (b == 0x08u || b == 0x7Fu) {
        if (sh_line_len) {
            --sh_line_len;
            sh_out("\b \b");
        }
        return;
    }
    if (b < 0x20u || b > 0x7Eu) {
        return;
    }
    if (sh_line_len + 1u >= sizeof(sh_line)) {
        sh_line_lost = 1;
        return;
    }
    sh_line[sh_line_len++] = (char)b;
    (void)usb_cdc_write((const char *)&b, 1); /* echo, so a terminal is usable */
}

void cdc_shell_service(void) {
    uint8_t buf[SH_CHUNK];
    uint16_t n;
    uint16_t i;

    if (usb_cdc_is_open() != sh_was_open) {
        sh_was_open = usb_cdc_is_open();
        sh_line_len = 0;
        sh_line_lost = 0;
        sh_dbg_len = 0;
        sh_dbg_pos = 0;
        if (sh_was_open && !raw_active) {
            cmd_version("");
            sh_out("type help\r\n");
        }
    }

    /* Drain the RX ring; in raw mode everything is payload. */
    while ((n = usb_cdc_read(buf, (uint16_t)sizeof(buf))) != 0u) {
        i = 0;
        if (raw_active || raw_drain) {
            raw_silence_ms = 0;   /* the host is still there */
        }
        if (raw_drain) {
            /* The transfer is dead but the host is still sending the image;
             * swallow the rest instead of reading it as commands. */
            uint16_t skip = n;
            if ((uint32_t)skip > raw_drain) {
                skip = (uint16_t)raw_drain;
            }
            raw_drain -= skip;
            i = skip;
        } else if (raw_active) {
            uint16_t take = n;
            if ((uint32_t)take > raw_remaining) {
                take = (uint16_t)raw_remaining;
            }
            if (!fw_cache_intake_data(buf, take)) {
                raw_active = 0;
                raw_drain = raw_remaining - take;
                raw_remaining = 0;
                sh_out("fwload: ERROR write failed in=0x");
                sh_hex(fw_cache_intake_status(), 2);
                sh_out("\r\n");
                continue;
            }
            raw_remaining -= take;
            i = take;
            if (!raw_remaining) {
                raw_finish();
            }
        }
        for (; i < n; ++i) {
            sh_line_byte(buf[i]);
        }
        if (raw_active) {
            break;   /* let the endpoint refill before the next sector write */
        }
    }

    /* Feed the pending dump while there is room, a packet at a time. */
    while (sh_dbg_pos < sh_dbg_len && usb_cdc_tx_free() >= SH_CHUNK) {
        uint16_t left = (uint16_t)(sh_dbg_len - sh_dbg_pos);
        uint16_t take = left < SH_CHUNK ? left : SH_CHUNK;
        uint16_t sent = usb_cdc_write(&sh_dbg[sh_dbg_pos], take);
        if (!sent) {
            break;
        }
        sh_dbg_pos = (uint16_t)(sh_dbg_pos + sent);
    }
}

void cdc_shell_tick(uint16_t ms) {
    sh_uptime_ms += ms;

    if (swap_pending) {
        swap_wait_ms = (uint16_t)(swap_wait_ms + ms);
        if (!usb_cdc_tx_pending() || swap_wait_ms >= 300u) {
            swap_run();
        }
        return;
    }

    if (raw_active || raw_drain) {
        raw_silence_ms = (uint16_t)(raw_silence_ms + ms);
        if (raw_silence_ms >= RAW_SILENCE_MS) {
            raw_timeout();
        }
        return;
    }

    if (!mon_period_ms) {
        return;
    }
    mon_elapsed_ms += ms;
    if (mon_elapsed_ms < mon_period_ms) {
        return;
    }
    mon_elapsed_ms = 0;
    if (sh_dbg_pos < sh_dbg_len) {
        ++mon_skipped;   /* the host is not keeping up; say so in `status` */
        return;
    }
    if (usb_cdc_is_open()) {
        dbg_stream_start();
    }
}

#else  /* USB_CDC_SHELL == 0 — the recovery image has no room for a shell */

void cdc_shell_tick(uint16_t ms) { (void)ms; }
void cdc_shell_service(void) {}

#endif
