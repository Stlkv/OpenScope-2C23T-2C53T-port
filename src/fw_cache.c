/*
 * fw_cache.c — see fw_cache.h. Two halves:
 *
 *   1. Intake: normal-world code that copies a file's bytes into a cache
 *      slot through the w25q driver, 4 KB sector at a time, CRC on the
 *      fly, manifest written last.
 *
 *   2. The installer: a RAM-resident function that reads the slot back
 *      over SPI2 (polled, no interrupts, no driver — the code below IS
 *      the driver, duplicated because flash-resident w25q.c is unreachable
 *      while the app slot is being erased under our feet) and programs the
 *      app slot page by page, routing between the two flash banks by
 *      address (an installed upstream image crosses 0x08080000), verifies
 *      by read-back, then SYSRESETREQ. Descended from fw_update.c's
 *      fw_ram_install and the upstream-branch fw_loader.c installer.
 */

#include "fw_cache.h"

#include "dbgdump.h"
#include "w25q.h"

enum {
    CACHE_SLOT_SPAN   = 0x00100000u,
    /* Inside upstream's W25Q map these two megabytes are the TAIL of its
     * read-only screenshot volume ("uservol", 0x200000..0xF00000) — the
     * one region its firmware never writes. 0xF00000+ is OFF LIMITS: its
     * usercal / settings / modules / scratch regions live there (an
     * earlier revision of this file put slot B at 0xF00000 and clobbered
     * them — caught 2026-08-22). The clean end state is upstream carving
     * an explicit "fwcache" region out of uservol's tail; these addresses
     * are that proposal. */
    CACHE_SLOT_A_BASE = 0x00D00000u,
    CACHE_SLOT_B_BASE = 0x00E00000u,
    CACHE_SECTOR      = 4096u,
    CACHE_DATA_OFF    = CACHE_SECTOR,           /* manifest sector first */
    CACHE_DATA_MAX    = 0x000B9000u,            /* 0x08007000..0x080C0000 */
    CACHE_MIN_IMAGE   = 8192u,

    APP_BASE          = 0x08007000u,
    APP_CEILING       = 0x080C0000u,            /* bitstream store begins */
    PAGE_SIZE         = 2048u,
};

#define CACHE_MAGIC 0x31435746u /* 'FWC1' little-endian */

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc;
    uint32_t reserved;
} cache_manifest_t;

static uint8_t  intake_slot;
static uint32_t intake_expected;
static uint32_t intake_written;
static uint32_t intake_crc;      /* running, pre-xorout */
static uint8_t  intake_status;
static uint8_t  swap_status;

/* One sector accumulates here before each w25q_write_sector. Also reused
 * as the installer's page buffer (the two can never be active at once:
 * both run from the idle scan, and the installer does not return). */
static uint8_t cache_buf[CACHE_SECTOR];
static uint32_t cache_buf_fill;

static uint32_t slot_base(uint8_t slot)
{
    return slot == FW_CACHE_SLOT_A ? CACHE_SLOT_A_BASE : CACHE_SLOT_B_BASE;
}

/* CRC-32 (zlib) with explicit chaining, since W25Q data is not memory-
 * mapped and arrives in chunks. dbgdump_crc32() is the one-shot cousin. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8u; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc;
}

uint8_t fw_cache_intake_begin(uint8_t slot, uint32_t size)
{
    intake_status = 0;
    intake_written = 0;
    cache_buf_fill = 0;
    if (slot > FW_CACHE_SLOT_B || size < CACHE_MIN_IMAGE ||
        size > CACHE_DATA_MAX || (size & 1u)) {
        intake_status = 0xE1;
        return 0;
    }
    intake_slot = slot;
    intake_expected = size;
    intake_crc = 0xFFFFFFFFu;
    return 1;
}

static uint8_t intake_flush_sector(void)
{
    uint32_t addr = slot_base(intake_slot) + CACHE_DATA_OFF +
                    (intake_written - cache_buf_fill);
    uint32_t fill = cache_buf_fill;
    uint8_t verify[64];

    /* Pad the tail of a final partial sector with 0xFF (erased look). */
    for (uint32_t i = fill; i < CACHE_SECTOR; ++i) {
        cache_buf[i] = 0xFFu;
    }
    if (!w25q_write_sector(addr, cache_buf)) {
        intake_status = 0xE2;
        return 0;
    }
    /* Spot-verify head and tail of the sector; the full-slot CRC at swap
     * time is the exhaustive check, this catches a dead write cheaply. */
    if (!w25q_read(addr, verify, sizeof(verify))) {
        intake_status = 0xE3;
        return 0;
    }
    for (uint32_t i = 0; i < sizeof(verify); ++i) {
        if (verify[i] != cache_buf[i]) {
            intake_status = 0xE3;
            return 0;
        }
    }
    cache_buf_fill = 0;
    return 1;
}

uint8_t fw_cache_intake_data(const uint8_t *data, uint16_t len)
{
    if (intake_status) {
        return 0;
    }
    while (len > 0) {
        uint32_t room = CACHE_SECTOR - cache_buf_fill;
        uint32_t take = len < room ? len : room;
        uint32_t remaining = intake_expected - intake_written;
        if (take > remaining) {
            take = remaining;
        }
        for (uint32_t i = 0; i < take; ++i) {
            cache_buf[cache_buf_fill + i] = data[i];
        }
        intake_crc = crc32_update(intake_crc, data, take);
        cache_buf_fill += take;
        intake_written += take;
        data += take;
        len = (uint16_t)(len - take);
        if (cache_buf_fill == CACHE_SECTOR ||
            intake_written == intake_expected) {
            if (!intake_flush_sector()) {
                return 0;
            }
        }
        if (intake_written == intake_expected) {
            break;
        }
    }
    return 1;
}

uint8_t fw_cache_intake_finish(void)
{
    if (intake_status || intake_written != intake_expected) {
        if (!intake_status) {
            intake_status = 0xE1;
        }
        return 0;
    }
    /* Manifest LAST: a torn intake leaves the old manifest (or none), and
     * the slot fails its swap-time CRC rather than installing garbage. */
    cache_manifest_t *m = (cache_manifest_t *)cache_buf;
    for (uint32_t i = 0; i < CACHE_SECTOR; ++i) {
        cache_buf[i] = 0xFFu;
    }
    m->magic = CACHE_MAGIC;
    m->size = intake_expected;
    m->crc = intake_crc ^ 0xFFFFFFFFu;
    m->reserved = 0;
    if (!w25q_write_sector(slot_base(intake_slot), cache_buf)) {
        intake_status = 0xE2;
        return 0;
    }
    intake_status = 1;
    return 1;
}

void fw_cache_intake_abort(void)
{
    if (!intake_status) {
        intake_status = 0xE1;
    }
}

uint8_t fw_cache_intake_status(void) { return intake_status; }
uint8_t fw_cache_swap_status(void) { return swap_status; }

static uint8_t read_manifest(uint8_t slot, cache_manifest_t *m)
{
    if (!w25q_read(slot_base(slot), (uint8_t *)m, sizeof(*m))) {
        return 0;
    }
    return m->magic == CACHE_MAGIC && m->size >= CACHE_MIN_IMAGE &&
           m->size <= CACHE_DATA_MAX && !(m->size & 1u);
}

uint32_t fw_cache_slot_size(uint8_t slot)
{
    cache_manifest_t m;
    return read_manifest(slot, &m) ? m.size : 0u;
}

uint32_t fw_cache_slot_crc(uint8_t slot)
{
    cache_manifest_t m;
    return read_manifest(slot, &m) ? m.crc : 0u;
}

/* ── The installer ─────────────────────────────────────────────────────
 * RAM-resident. Interrupts off for good; SPI2 is reclaimed by waiting out
 * whatever transfer the USB task had in flight, then driven directly.
 * Success ends in SYSRESETREQ; any failure after the first erase parks in
 * a dead loop with the rail held (recovery: MENU+Power IAP, which this
 * code cannot touch — it writes only 0x08007000 upward). */

#define RF __attribute__((section(".data.ramfunc"), noinline, used))

/* SPI2 + GPIOB raw registers (the w25q driver's pins: CS = PB12). */
#define R_SPI2_STS  (*(volatile uint32_t *)0x40003808u)
#define R_SPI2_DT   (*(volatile uint32_t *)0x4000380Cu)
#define R_GPIOB_BSR (*(volatile uint32_t *)0x40010C10u)
#define R_GPIOB_BRR (*(volatile uint32_t *)0x40010C14u)

RF static uint8_t rf_spi2_xfer(uint8_t v)
{
    uint32_t t = 0x000FFFFFu;
    while (!(R_SPI2_STS & 0x2u) && --t) {          /* TXE */
    }
    R_SPI2_DT = v;
    t = 0x000FFFFFu;
    while (!(R_SPI2_STS & 0x1u) && --t) {          /* RXNE */
    }
    return (uint8_t)R_SPI2_DT;
}

RF static void rf_w25q_read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    R_GPIOB_BRR = 1u << 12;                        /* CS low */
    (void)rf_spi2_xfer(0x03u);
    (void)rf_spi2_xfer((uint8_t)(addr >> 16));
    (void)rf_spi2_xfer((uint8_t)(addr >> 8));
    (void)rf_spi2_xfer((uint8_t)addr);
    for (uint32_t i = 0; i < len; ++i) {
        dst[i] = rf_spi2_xfer(0xFFu);
    }
    R_GPIOB_BSR = 1u << 12;                        /* CS high */
}

/* Dual-bank flash register blocks, selected by address at each page:
 * bank 0 (< 0x08080000) at +0x0C/+0x10/+0x14 with KEYR +0x04, bank 1 at
 * +0x4C/+0x50/+0x54 with KEYR2 +0x44. An upstream-sized image crosses the
 * boundary, so one install can need both. */
RF static void fw_cache_ram_install(uint32_t src, uint32_t size)
{
    uint8_t *page = cache_buf;
    uint32_t end = APP_BASE + ((size + (PAGE_SIZE - 1u)) & ~(PAGE_SIZE - 1u));

    __asm__ volatile("cpsid i" ::: "memory");

    /* Power hold (PC9) no matter what. GPIOC clock is already on. */
    *(volatile uint32_t *)0x40011010u = 1u << 9;

    /* Reclaim SPI2: let any in-flight byte finish, then force CS high. */
    {
        uint32_t t = 0x000FFFFFu;
        while ((R_SPI2_STS & 0x80u) && --t) {      /* BSY */
        }
        R_GPIOB_BSR = 1u << 12;
    }

    if (size == 0 || (size & 1u) || end > APP_CEILING) {
        goto dead;
    }

    for (uint32_t addr = APP_BASE; addr < end; addr += PAGE_SIZE) {
        uint8_t bank1 = addr >= 0x08080000u;
        volatile uint32_t *sts =
            (volatile uint32_t *)(bank1 ? 0x4002204Cu : 0x4002200Cu);
        volatile uint32_t *ctrl =
            (volatile uint32_t *)(bank1 ? 0x40022050u : 0x40022010u);
        volatile uint32_t *fadr =
            (volatile uint32_t *)(bank1 ? 0x40022054u : 0x40022014u);
        volatile uint32_t *keyr =
            (volatile uint32_t *)(bank1 ? 0x40022044u : 0x40022004u);
        uint32_t off = addr - APP_BASE;
        uint32_t n = size - off;
        uint32_t t;
        if (n > PAGE_SIZE) {
            n = PAGE_SIZE;
        }

        rf_w25q_read(src + off, page, PAGE_SIZE);
        for (uint32_t i = n; i < PAGE_SIZE; ++i) {
            page[i] = 0xFFu;
        }

        if (*ctrl & (1u << 7)) {
            *keyr = 0x45670123u;
            *keyr = 0xCDEF89ABu;
        }
        if (*ctrl & (1u << 7)) {
            goto dead;
        }

        t = 0x00FFFFFFu;
        while ((*sts & 1u) && --t) {
        }
        if (!t) {
            goto dead;
        }
        *sts = (1u << 5) | (1u << 2) | (1u << 4);
        *ctrl |= 1u << 1;
        *fadr = addr;
        *ctrl |= 1u << 6;
        t = 0x00FFFFFFu;
        while ((*sts & 1u) && --t) {
        }
        *ctrl &= ~(1u << 1);
        if (!t || (*sts & ((1u << 2) | (1u << 4)))) {
            goto dead;
        }

        for (uint32_t i = 0; i < PAGE_SIZE; i += 2u) {
            uint16_t v = (uint16_t)page[i] | ((uint16_t)page[i + 1u] << 8);
            t = 0x00FFFFFFu;
            while ((*sts & 1u) && --t) {
            }
            if (!t) {
                goto dead;
            }
            *sts = (1u << 5) | (1u << 2) | (1u << 4);
            *ctrl |= 1u;
            *(volatile uint16_t *)(addr + i) = v;
            t = 0x00FFFFFFu;
            while ((*sts & 1u) && --t) {
            }
            *ctrl &= ~1u;
            if (!t || (*sts & ((1u << 2) | (1u << 4)))) {
                goto dead;
            }
        }

        for (uint32_t i = 0; i < PAGE_SIZE; ++i) {
            if (*(volatile uint8_t *)(addr + i) != page[i]) {
                goto dead;
            }
        }
    }

    *(volatile uint32_t *)0xE000ED0Cu = 0x05FA0004u;   /* SYSRESETREQ */

dead:
    while (1) {
    }
}

void fw_cache_swap(uint8_t slot)
{
    cache_manifest_t m;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t data;

    swap_status = 0;
    if (slot > FW_CACHE_SLOT_B || !read_manifest(slot, &m)) {
        swap_status = 0xE4;
        return;
    }
    data = slot_base(slot) + CACHE_DATA_OFF;

    /* Full-slot CRC against the manifest, through the normal driver while
     * the world is still running — this is the moment a FAT-cluster
     * clobber or a torn intake is caught. */
    for (uint32_t off = 0; off < m.size; off += CACHE_SECTOR) {
        uint32_t n = m.size - off;
        if (n > CACHE_SECTOR) {
            n = CACHE_SECTOR;
        }
        if (!w25q_read(data + off, cache_buf, (uint16_t)n)) {
            swap_status = 0xE5;
            return;
        }
        crc = crc32_update(crc, cache_buf, n);
    }
    if ((crc ^ 0xFFFFFFFFu) != m.crc) {
        swap_status = 0xE5;
        return;
    }

    /* Vector-table shape gate, from the freshly CRC-passed first bytes. */
    if (!w25q_read(data, cache_buf, 8)) {
        swap_status = 0xE5;
        return;
    }
    {
        uint32_t sp = (uint32_t)cache_buf[0] | ((uint32_t)cache_buf[1] << 8) |
                      ((uint32_t)cache_buf[2] << 16) |
                      ((uint32_t)cache_buf[3] << 24);
        uint32_t pc = (uint32_t)cache_buf[4] | ((uint32_t)cache_buf[5] << 8) |
                      ((uint32_t)cache_buf[6] << 16) |
                      ((uint32_t)cache_buf[7] << 24);
        if ((sp & 0xFFF00000u) != 0x20000000u ||
            pc < APP_BASE || pc >= APP_CEILING || (pc & 1u) == 0) {
            swap_status = 0xE6;
            return;
        }
    }

    fw_cache_ram_install(data, m.size);
    /* not reached */
}
