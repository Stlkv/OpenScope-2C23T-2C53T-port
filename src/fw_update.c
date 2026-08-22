#include "fw_update.h"

#include "app_config.h"
#include "dbgdump.h"   /* dbgdump_crc32 — the shared CRC-32 (cal restore) */
#include "fpga_bitstream_store.h"
#include "hw.h"

#include <stdint.h>

/* Only the image that READS the bitstream store needs to be able to write it.
 * Keeping the writer out of the provisioning image is what lets that image
 * still fit beside the 113 KB payload it carries. */
#ifndef FPGA53_BITSTREAM_EXTERN
#define FPGA53_BITSTREAM_EXTERN 1
#endif
#define FW_STORE_WRITER FPGA53_BITSTREAM_EXTERN

/* On 2C53T a dropped firmware image no longer lands in internal flash at all:
 * usb_msc.c reads it back through the FAT chain into a W25Q cache slot and
 * fw_cache.c installs it from RAM with a system reset. What is left here is
 * the writer for the ONE region this file still owns -- the bitstream store --
 * plus the transfer status the screen, DBG.TXT and the CDC shell read, and the
 * factory-cal restore. See docs/plans/drop-internal-staging-2026-08-22.md
 * in the workspace.
 *
 * The 2C23T targets keep the staging path: that hardware has neither the store
 * nor our cache layout, and we cannot test it. */
#if HW_TARGET_2C53T
#define FW_STAGING 0
#else
#define FW_STAGING 1
#endif

enum {
    FW_APP_BASE = APP_BASE_ADDR,
#if FW_STAGING
    FW_STAGE_BASE = FW_STAGE_BASE_ADDR,
#endif
    FW_RAM_BASE = 0x20000000u,
    FW_RAM_END = 0x20036078u,
    FW_PAGE_SIZE = 2048u,
    FW_MAX_SIZE = APP_FLASH_SIZE_BYTES,
    FW_APP_END = FW_APP_BASE + FW_MAX_SIZE,
    /* Erase-tracking bitmap: one bit per page of the largest destination this
     * writer can still address. With staging gone that is the store, so the
     * bitmap stops growing with the app slot -- which is what made widening
     * the slot expensive before (14 KB of RAM in the halfword-bitmap era). */
#if FW_STAGING
    FW_DEST_MAX_BYTES = FW_MAX_SIZE,
#else
    FW_DEST_MAX_BYTES = FPGA_BS_STORE_MAX,
#endif
    FW_STAGE_PAGE_COUNT = FW_DEST_MAX_BYTES / FW_PAGE_SIZE,
    FW_STAGE_PAGE_BYTES = (FW_STAGE_PAGE_COUNT + 7u) / 8u,

    FLASH_STS_BSY = 1u << 0,
    FLASH_STS_PGERR = 1u << 2,
    FLASH_STS_WRPRTERR = 1u << 4,
    FLASH_STS_EOP = 1u << 5,
    FLASH_CTRL_PG = 1u << 0,
    FLASH_CTRL_PER = 1u << 1,
    FLASH_CTRL_STRT = 1u << 6,
    FLASH_CTRL_LOCK = 1u << 7,

    FW_APPLY_DELAY_LOOPS = 5,
};

static volatile fw_update_status_t fw_status = {
    .state = FW_UPDATE_STATE_IDLE,
};
static volatile uint8_t fw_apply_requested;
static uint8_t fw_apply_delay_loops;
static volatile uint32_t fw_expected_size;
static uint32_t fw_stage_base_lba;
static uint8_t fw_stage_started;
/* 1 = the file being staged is an FPGA bitstream, not a firmware image: it
 * goes straight to its own flash region and there is nothing to install
 * afterwards. Everything else about the transfer — page erase, halfword
 * programming, resume-safe bookkeeping — is identical, so the destination is
 * a variable rather than a second copy of the writer. */
#if FW_STORE_WRITER && FW_STAGING
static uint8_t fw_blob_mode;
#elif FW_STORE_WRITER
/* Without staging the store is the only destination this writer has. */
#define fw_blob_mode 1u
#else
/* The provisioning image carries the payload itself and has no room to spare:
 * with fw_blob_mode a constant, the destination logic and the whole bank-1
 * register path fold away. */
#define fw_blob_mode 0u
#endif
#if !FW_STAGING
/* 1 = this transfer's bytes are handled by fw_cache.c (an image on its way to
 * a W25Q slot); this file only tracks the status the UI and the shell show.
 * The producer is the same sequential FAT reader either way. */
static uint8_t fw_count_only;

void fw_update_set_count_only(uint8_t on) {
    if (fw_status.state == FW_UPDATE_STATE_APPLYING || fw_stage_started) {
        return;
    }
    fw_count_only = on ? 1u : 0u;
}
#endif
static uint8_t fw_stage_pages[FW_STAGE_PAGE_BYTES];
/* Bytes written contiguously from offset 0 of the staged file. */
static uint32_t fw_covered;
#if FW_STAGING
static uint8_t fw_page_buffer[FW_PAGE_SIZE];
#endif

/*
 * The 1 MB AT32F403A splits its flash into two banks with two independent
 * register sets: bank 0 below 0x08080000, bank 1 ("extended flash") from
 * 0x08080000 up. A program or erase aimed at bank 1 through the bank-0
 * registers is simply dropped — no error, no data. The factory IAP
 * bootloader's HAL routes by address for exactly this reason
 * (53t/reverse_engineering/analysis_v120/stock_iap_bootloader.md § extended
 * flash), and the bitstream store lives at 0x08080000, so the writer has to
 * do the same.
 */
#define FLASH_BANK1_BASE 0x08080000u
#define FLASH_STS1       REG32(FLASH_R_BASE + 0x4Cu)
#define FLASH_CTRL1      REG32(FLASH_R_BASE + 0x50u)
#define FLASH_ADDR1      REG32(FLASH_R_BASE + 0x54u)
#define FLASH_KEYR2      REG32(FLASH_R_BASE + 0x44u)

static uint8_t flash_bank1(uint32_t addr) {
#if FW_STORE_WRITER
    return addr >= FLASH_BANK1_BASE ? 1u : 0u;
#else
    (void)addr;
    return 0u; /* nothing this image writes lives above 0x08080000 */
#endif
}

/* Bounded on purpose. A page erase takes tens of milliseconds, so any real
 * operation finishes long before this; what the bound buys is that a wrong
 * guess about the bank-1 registers degrades into "the write did not happen"
 * (caught later by the store's fingerprint) instead of a spin that would need
 * a power cycle to escape — and would re-hang on the next boot, because the
 * file that triggered it is still on the volume. */
static void flash_wait(uint32_t addr) {
    uint32_t guard = 0x00400000u;

    if (flash_bank1(addr)) {
        while ((FLASH_STS1 & FLASH_STS_BSY) && --guard) {
        }
        return;
    }
    while ((FLASH_STS & FLASH_STS_BSY) && --guard) {
    }
}

static void flash_unlock(uint32_t addr) {
    if (flash_bank1(addr)) {
        if (FLASH_CTRL1 & FLASH_CTRL_LOCK) {
            FLASH_KEYR2 = 0x45670123u;
            FLASH_KEYR2 = 0xCDEF89ABu;
        }
        return;
    }
    if (FLASH_CTRL & FLASH_CTRL_LOCK) {
        FLASH_KEYR = 0x45670123u;
        FLASH_KEYR = 0xCDEF89ABu;
    }
}

static void flash_lock(uint32_t addr) {
    if (flash_bank1(addr)) {
        FLASH_CTRL1 |= FLASH_CTRL_LOCK;
        return;
    }
    FLASH_CTRL |= FLASH_CTRL_LOCK;
}

static void flash_clear_status(uint32_t addr) {
    if (flash_bank1(addr)) {
        FLASH_STS1 = FLASH_STS_EOP | FLASH_STS_PGERR | FLASH_STS_WRPRTERR;
        return;
    }
    FLASH_STS = FLASH_STS_EOP | FLASH_STS_PGERR | FLASH_STS_WRPRTERR;
}

static void flash_erase_page(uint32_t addr) {
    flash_wait(addr);
    flash_clear_status(addr);
    if (flash_bank1(addr)) {
        FLASH_CTRL1 |= FLASH_CTRL_PER;
        FLASH_ADDR1 = addr;
        FLASH_CTRL1 |= FLASH_CTRL_STRT;
        flash_wait(addr);
        FLASH_CTRL1 &= ~FLASH_CTRL_PER;
    } else {
        FLASH_CTRL |= FLASH_CTRL_PER;
        FLASH_ADDR = addr;
        FLASH_CTRL |= FLASH_CTRL_STRT;
        flash_wait(addr);
        FLASH_CTRL &= ~FLASH_CTRL_PER;
    }
    flash_clear_status(addr);
}

static void flash_program_halfword(uint32_t addr, uint16_t value) {
    flash_wait(addr);
    flash_clear_status(addr);
    if (flash_bank1(addr)) {
        FLASH_CTRL1 |= FLASH_CTRL_PG;
        REG16(addr) = value;
        flash_wait(addr);
        FLASH_CTRL1 &= ~FLASH_CTRL_PG;
    } else {
        FLASH_CTRL |= FLASH_CTRL_PG;
        REG16(addr) = value;
        flash_wait(addr);
        FLASH_CTRL &= ~FLASH_CTRL_PG;
    }
    flash_clear_status(addr);
}

/* ─── Factory-calibration restore ─────────────────────────────────────
 * Writes a 4096-byte image back over the factory-calibration page. The
 * source file arrives as CALRSTOR.BIN on the USB volume (usb_msc.c); this
 * function is the only thing in the port that programs the page, and the
 * dump path (CALREQ/dbgdump) never writes it.
 *
 * Safety posture, in order of importance:
 *   - The target address is compile-time fixed. Nothing in the file can
 *     steer the write; a wrong file can at worst write wrong CAL bytes,
 *     which the dump artifact recovers from by re-running the restore.
 *   - Exact-size gate: anything but 4096 bytes is refused untouched.
 *   - Identical content is a no-op: the page is not erased just to be
 *     rebuilt into itself, so a redundant restore carries zero risk.
 *   - Programmed content is verified by direct read-back compare; the
 *     verdict is latched and printed by the CALW line in DBG.TXT.
 *
 * CAL_RESTORE_BASE is overridable for the bench proof-run against a free
 * page (0x080FE800 — inside the 0x080A0000+ free region, clear of the
 * settings page at 0x080FF800) so erase+program+verify can be exercised
 * end-to-end without touching the real calibration. */
#ifndef CAL_RESTORE_BASE
#define CAL_RESTORE_BASE 0x08006000u
#endif
enum { CAL_RESTORE_LEN = 4096u };

static uint8_t  cal_restore_last_status; /* FW_CAL_RESTORE_* (fw_update.h) */
static uint8_t  cal_restore_run_count;
static uint32_t cal_restore_file_crc;

uint8_t  fw_cal_restore_status(void) { return cal_restore_last_status; }
uint8_t  fw_cal_restore_runs(void)   { return cal_restore_run_count; }
uint32_t fw_cal_restore_crc(void)    { return cal_restore_file_crc; }

uint8_t fw_cal_restore(const uint8_t *data, uint32_t len) {
    const volatile uint8_t *page = (const volatile uint8_t *)CAL_RESTORE_BASE;
    uint32_t i;
    uint8_t identical = 1;

    ++cal_restore_run_count;
    if (data == 0 || len != CAL_RESTORE_LEN) {
        cal_restore_file_crc = 0;
        cal_restore_last_status = FW_CAL_RESTORE_BADSIZE;
        return cal_restore_last_status;
    }
    cal_restore_file_crc = dbgdump_crc32(data, CAL_RESTORE_LEN);

    for (i = 0; i < CAL_RESTORE_LEN; ++i) {
        if (page[i] != data[i]) {
            identical = 0;
            break;
        }
    }
    if (identical) {
        cal_restore_last_status = FW_CAL_RESTORE_IDENTICAL;
        return cal_restore_last_status;
    }

    flash_unlock(CAL_RESTORE_BASE);
    flash_erase_page(CAL_RESTORE_BASE);
    flash_erase_page(CAL_RESTORE_BASE + FW_PAGE_SIZE);
    for (i = 0; i < CAL_RESTORE_LEN; i += 2u) {
        flash_program_halfword(CAL_RESTORE_BASE + i,
                               (uint16_t)(data[i] | ((uint16_t)data[i + 1u] << 8)));
    }
    flash_lock(CAL_RESTORE_BASE);

    for (i = 0; i < CAL_RESTORE_LEN; ++i) {
        if (page[i] != data[i]) {
            cal_restore_last_status = FW_CAL_RESTORE_MISMATCH;
            return cal_restore_last_status;
        }
    }
    cal_restore_last_status = FW_CAL_RESTORE_WRITTEN;
    return cal_restore_last_status;
}

#if FW_STAGING
static uint32_t fw_dest_base(void) {
    return fw_blob_mode ? FPGA_BS_STORE_BASE : (uint32_t)FW_STAGE_BASE;
}

static uint32_t fw_dest_max(void) {
    return fw_blob_mode ? FPGA_BS_STORE_MAX : (uint32_t)FW_MAX_SIZE;
}
#else
/* Bytes that only pass through for counting (an image bound for the cache)
 * still have to be size-checked against where they are really going, and that
 * is the app slot the installer will program. */
static uint32_t fw_dest_base(void) { return FPGA_BS_STORE_BASE; }
static uint32_t fw_dest_max(void) {
    return fw_count_only ? (uint32_t)FW_MAX_SIZE : FPGA_BS_STORE_MAX;
}
#endif

static uint8_t bit_get(uint8_t *bits, uint32_t bit) {
    return (bits[bit >> 3] & (uint8_t)(1u << (bit & 7u))) ? 1u : 0u;
}

static void bit_set(uint8_t *bits, uint32_t bit) {
    bits[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
}

static void bits_clear(uint8_t *bits, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        bits[i] = 0;
    }
}

static uint32_t min_u32(uint32_t a, uint32_t b) {
    return a < b ? a : b;
}

static uint8_t vector_valid(const uint8_t *data) {
    uint32_t sp = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                  ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    uint32_t pc = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                  ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

    if (sp < FW_RAM_BASE || sp > FW_RAM_END) {
        return 0;
    }
    if ((pc & 1u) == 0) {
        return 0;
    }
    pc &= ~1u;
    return pc >= FW_APP_BASE && pc < FW_APP_END;
}

static void fw_set_error(uint8_t error) {
    fw_status.error = error;
    fw_status.state = FW_UPDATE_STATE_ERROR;
    ++fw_status.sequence;
}

static uint8_t fw_stage_expected_complete(void) {
    if (fw_expected_size < 8192u || fw_expected_size > fw_dest_max()) {
        return 0;
    }
    return fw_covered >= fw_expected_size;
}

static void fw_maybe_ready(void) {
    if (fw_status.state == FW_UPDATE_STATE_STAGING &&
        fw_expected_size >= 8192u &&
        fw_expected_size <= fw_dest_max() &&
        fw_status.bytes >= fw_expected_size &&
        fw_stage_expected_complete()) {
        fw_status.state = FW_UPDATE_STATE_READY;
        ++fw_status.sequence;
    }
}

static void fw_stage_reset(uint32_t base_lba) {
    fw_stage_started = 1;
    fw_stage_base_lba = base_lba;
    fw_status.state = FW_UPDATE_STATE_STAGING;
    fw_status.error = FW_UPDATE_ERR_NONE;
    fw_status.bytes = 0;
    fw_status.expected_size = fw_expected_size;
    fw_status.base_lba = base_lba;
    fw_apply_requested = 0;
    fw_apply_delay_loops = 0;
    bits_clear(fw_stage_pages, sizeof(fw_stage_pages));
    fw_covered = 0;
    ++fw_status.sequence;
}

/*
 * Programming is halfword-granular, and a programmed halfword cannot be
 * changed without erasing its whole page — so the writer has to know what has
 * already landed. It used to answer that from a bitmap with one bit per
 * halfword: 14 KB of RAM for a 224 KB slot, growing with the slot, in a
 * firmware whose free RAM is measured in single kilobytes.
 *
 * It never needed to. The only producer is the FAT reader in usb_msc.c, which
 * walks the staged file sector by sector with offsets strictly increasing from
 * zero, so a single watermark — bytes covered contiguously from the start —
 * answers the same questions in four bytes:
 *
 *   behind the watermark   a re-send; accepted if flash already agrees with
 *                          it, rejected if it does not (the page would have to
 *                          be erased, and the bytes to rebuild it with are no
 *                          longer in RAM)
 *   at the watermark       the normal case: erase on first touch, program
 *   ahead of the watermark a hole, which no legitimate producer can create;
 *                          rejected rather than papered over
 *
 * A rejection ends the transfer the way every other staging error does: the
 * file is deleted from the volume and has to be copied again, a few seconds of
 * a cycle that is already automated. A silent hole would cost an image that
 * passes its completeness check and does not boot.
 */
static uint8_t fw_flash_matches(uint32_t offset, const uint8_t *data, uint16_t len) {
    const uint8_t *flash = (const uint8_t *)(fw_dest_base() + offset);

#if !FW_STAGING
    if (fw_count_only) {
        return 1; /* nothing of ours is in flash to disagree with */
    }
#endif

    for (uint16_t i = 0; i < len; ++i) {
        if (flash[i] != data[i]) {
            return 0;
        }
    }
    return 1;
}

static void fw_stage_program(uint32_t offset, const uint8_t *data, uint16_t len) {
    uint32_t addr = fw_dest_base() + offset;

#if !FW_STAGING
    if (fw_count_only) {
        return;
    }
#endif

    flash_unlock(addr);
    for (uint16_t i = 0; i < len; i = (uint16_t)(i + 2u)) {
        uint32_t page = (offset + i) / FW_PAGE_SIZE;
        uint16_t value = data[i];

        /* A trailing odd byte can only be the last byte of the file; pad it
         * with the erased value so the halfword is programmable. */
        value |= (i + 1u < len) ? ((uint16_t)data[i + 1u] << 8) : 0xFF00u;

        if (!bit_get(fw_stage_pages, page)) {
            flash_erase_page(fw_dest_base() + page * FW_PAGE_SIZE);
            bit_set(fw_stage_pages, page);
        }
        flash_program_halfword(addr + i, value);
    }
    flash_lock(addr);
}

static uint8_t fw_stage_write(uint32_t offset, const uint8_t *data, uint16_t len) {
    if (offset >= fw_dest_max() || (uint32_t)len > fw_dest_max() - offset) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
        return 0;
    }
    if (offset & 1u) {
        /* Halfword alignment is a hardware fact, and no producer emits odd
         * offsets — only a trailing odd length, at end of file. */
        fw_set_error(FW_UPDATE_ERR_ORDER);
        return 0;
    }

    if (offset < fw_covered) {
        uint16_t dup = (uint16_t)min_u32(fw_covered - offset, len);
        if (!fw_flash_matches(offset, data, dup)) {
            fw_set_error(FW_UPDATE_ERR_ORDER);
            return 0;
        }
        offset += dup;
        data += dup;
        len = (uint16_t)(len - dup);
        if (!len) {
            return 1;
        }
    }
    if (offset != fw_covered) {
        fw_set_error(FW_UPDATE_ERR_ORDER);
        return 0;
    }

    fw_stage_program(offset, data, len);
    fw_covered = offset + len;

    if (fw_status.bytes < fw_covered) {
        fw_status.bytes = fw_covered;
        ++fw_status.sequence;
    }
    fw_maybe_ready();
    return 1;
}

void fw_update_usb_data(uint32_t lba, uint16_t sector_offset, const uint8_t *data, uint16_t len) {
    uint32_t offset;

    if (!data || !len || fw_status.state == FW_UPDATE_STATE_ERROR) {
        return;
    }

    if (!fw_stage_started) {
        if (sector_offset != 0 || len < 8u || !vector_valid(data)) {
            return;
        }
        fw_stage_reset(lba);
    }

    if (lba < fw_stage_base_lba) {
        return;
    }
    offset = (lba - fw_stage_base_lba) * 512u + sector_offset;
    if (fw_expected_size) {
        if (offset >= fw_expected_size) {
            return;
        }
        if ((uint32_t)len > fw_expected_size - offset) {
            len = (uint16_t)(fw_expected_size - offset);
        }
    }
    if (!len) {
        return;
    }
    (void)fw_stage_write(offset, data, len);
}

uint8_t fw_update_request_apply(void) {
    if (fw_status.state == FW_UPDATE_STATE_READY &&
        fw_expected_size >= 8192u &&
        fw_expected_size <= fw_dest_max() &&
        fw_status.bytes >= fw_expected_size) {
#if !FW_STAGING
        if (!fw_count_only) {
            /* The store is written as the file is read, so "apply" is only an
             * acknowledgement: report success so the caller deletes the file. */
            fw_status.state = FW_UPDATE_STATE_IDLE;
            ++fw_status.sequence;
            return 1;
        }
        /* An image: the bytes are in a W25Q slot and fw_cache.c installs them.
         * Say FLASHING on the screen and let the caller pull the trigger. */
        fw_status.state = FW_UPDATE_STATE_APPLYING;
        ++fw_status.sequence;
        return 1;
#else
        if (fw_blob_mode) {
            /* Blob transfers land in their final place as they stream, so
             * "apply" is only an acknowledgement: report success so the
             * caller deletes the file, and go idle. */
            fw_status.state = FW_UPDATE_STATE_IDLE;
            ++fw_status.sequence;
            return 1;
        }
        fw_apply_requested = 1;
        fw_apply_delay_loops = FW_APPLY_DELAY_LOOPS;
        fw_status.state = FW_UPDATE_STATE_APPLYING;
        ++fw_status.sequence;
        return 1;
#endif
    }
    return 0;
}

static void fw_update_note_file_size(uint32_t size) {
    if (size >= 8192u && size <= fw_dest_max()) {
        if (size > fw_expected_size) {
            fw_expected_size = size;
            fw_status.expected_size = size;
            if (fw_status.state == FW_UPDATE_STATE_READY && fw_status.bytes < fw_expected_size) {
                fw_status.state = FW_UPDATE_STATE_STAGING;
            }
            ++fw_status.sequence;
        }
        fw_maybe_ready();
    } else if (size) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
    }
}

/* Pick the destination for the next staged file. Must be called after
 * fw_update_clear() (which resets it) and before the first byte arrives —
 * the destination is baked into fw_stage_reset()'s bookkeeping. */
void fw_update_set_blob_mode(uint8_t blob) {
#if FW_STORE_WRITER && FW_STAGING
    if (fw_status.state == FW_UPDATE_STATE_APPLYING || fw_stage_started) {
        return;
    }
    fw_blob_mode = blob ? 1u : 0u;
#else
    (void)blob;
#endif
}

void fw_update_note_file(uint32_t base_lba, uint32_t size) {
    if (size < 8192u || size > fw_dest_max()) {
        if (size) {
            fw_set_error(FW_UPDATE_ERR_RANGE);
        }
        return;
    }
    if (base_lba && (!fw_stage_started || fw_stage_base_lba != base_lba)) {
        if (fw_status.state != FW_UPDATE_STATE_APPLYING) {
            fw_expected_size = size;
            fw_stage_reset(base_lba);
        }
    }
    fw_update_note_file_size(size);
}

void fw_update_clear(void) {
    if (fw_status.state == FW_UPDATE_STATE_APPLYING) {
        return;
    }
    fw_apply_requested = 0;
    fw_apply_delay_loops = 0;
    fw_expected_size = 0;
    fw_stage_started = 0;
    fw_stage_base_lba = 0;
#if FW_STORE_WRITER && FW_STAGING
    fw_blob_mode = 0;
#endif
#if !FW_STAGING
    fw_count_only = 0;
#endif
    fw_status.state = FW_UPDATE_STATE_IDLE;
    fw_status.error = FW_UPDATE_ERR_NONE;
    fw_status.bytes = 0;
    fw_status.expected_size = 0;
    fw_status.base_lba = 0;
    ++fw_status.sequence;
}

#if FW_STAGING
__attribute__((section(".data.ramfunc"), noinline, used))
static void fw_ram_install(uint32_t src, uint32_t dst, uint32_t size) {
    volatile uint32_t *flash_sts = (volatile uint32_t *)0x4002200Cu;
    volatile uint32_t *flash_ctrl = (volatile uint32_t *)0x40022010u;
    volatile uint32_t *flash_addr = (volatile uint32_t *)0x40022014u;
    volatile uint32_t *flash_keyr = (volatile uint32_t *)0x40022004u;
    volatile uint8_t *page = fw_page_buffer;
    uint32_t end;
    uint32_t page_size = 2048u;
    uint32_t timeout;
    uint32_t complete = 0;

    __asm__ volatile("cpsid i" ::: "memory");

    if (dst != FW_APP_BASE || size == 0 || size > FW_MAX_SIZE) {
        goto fail_now;
    }
    end = dst + ((size + 2047u) & ~2047u);
    if (end > FW_APP_END) {
        goto fail_now;
    }

    if (*flash_ctrl & (1u << 7)) {
        *flash_keyr = 0x45670123u;
        *flash_keyr = 0xCDEF89ABu;
    }
    if (*flash_ctrl & (1u << 7)) {
        goto fail_now;
    }

    for (uint32_t addr = dst; addr < end; addr += 2048u) {
        uint32_t page_off = addr - dst;
        uint32_t page_bytes = size - page_off;
        if (page_bytes > page_size) {
            page_bytes = page_size;
        }
        uint32_t write_bytes = (page_bytes + 1u) & ~1u;
        for (uint32_t i = 0; i < page_size; ++i) {
            page[i] = i < page_bytes ? *(volatile uint8_t *)(src + page_off + i) : 0xFFu;
        }

        timeout = 0x00FFFFFFu;
        while ((*flash_sts & 1u) && --timeout) {
        }
        if (!timeout) {
            goto fail_now;
        }
        *flash_sts = (1u << 5) | (1u << 2) | (1u << 4);
        *flash_ctrl |= 1u << 1;
        *flash_addr = addr;
        *flash_ctrl |= 1u << 6;
        timeout = 0x00FFFFFFu;
        while ((*flash_sts & 1u) && --timeout) {
        }
        if (!timeout || (*flash_sts & ((1u << 2) | (1u << 4)))) {
            goto fail_now;
        }
        *flash_ctrl &= ~(1u << 1);

        for (uint32_t off = 0; off < write_bytes; off += 2u) {
            uint16_t value = (uint16_t)page[off] | ((uint16_t)page[off + 1u] << 8);
            timeout = 0x00FFFFFFu;
            while ((*flash_sts & 1u) && --timeout) {
            }
            if (!timeout) {
                goto fail_now;
            }
            *flash_sts = (1u << 5) | (1u << 2) | (1u << 4);
            *flash_ctrl |= 1u;
            *(volatile uint16_t *)(addr + off) = value;
            timeout = 0x00FFFFFFu;
            while ((*flash_sts & 1u) && --timeout) {
            }
            if (!timeout || (*flash_sts & ((1u << 2) | (1u << 4)))) {
                goto fail_now;
            }
            *flash_ctrl &= ~1u;
        }

        for (uint32_t i = 0; i < page_bytes; ++i) {
            if (*(volatile uint8_t *)(addr + i) != page[i]) {
                goto fail_now;
            }
        }
    }

    complete = 1;

fail_now:
    *flash_ctrl &= ~((1u << 0) | (1u << 1));
    *flash_ctrl |= 1u << 7;
    if (complete) {
        *(volatile uint32_t *)0x40021018u |= (1u << 3); // GPIOB clock
        *(volatile uint32_t *)0x40010C00u =
            (*(volatile uint32_t *)0x40010C00u & ~(0xFu << 8)) | (0x1u << 8);
        *(volatile uint32_t *)0x40010C10u = 1u << 2; // keep PB2 power hold high
        *(volatile uint32_t *)0xE000E010u = 0; // SysTick off
        for (uint32_t i = 0; i < 8u; ++i) {
            *(volatile uint32_t *)(0xE000E180u + i * 4u) = 0xFFFFFFFFu;
            *(volatile uint32_t *)(0xE000E280u + i * 4u) = 0xFFFFFFFFu;
        }
        *(volatile uint32_t *)0xE000ED08u = dst;
        __asm__ volatile(
            "dsb\n"
            "isb\n"
            "ldr r0, [%0]\n"
            "ldr r1, [%0, #4]\n"
            "msr msp, r0\n"
            "bx r1\n"
            :
            : "r"(dst)
            : "r0", "r1", "memory");
    }
    while (1) {
    }
}

void fw_update_service(void) {
    uint32_t size;

    if (!fw_apply_requested || fw_status.state != FW_UPDATE_STATE_APPLYING) {
        return;
    }
    if (fw_apply_delay_loops) {
        --fw_apply_delay_loops;
        return;
    }
    fw_apply_requested = 0;
    size = fw_expected_size ? fw_expected_size : fw_status.bytes;
    size = (size + 1u) & ~1u;
    if (size < 8192u || size > FW_MAX_SIZE) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
        return;
    }
    if (!vector_valid((const uint8_t *)FW_STAGE_BASE)) {
        fw_set_error(FW_UPDATE_ERR_VECTOR);
        return;
    }
    fw_status.state = FW_UPDATE_STATE_APPLYING;
    ++fw_status.sequence;
    fw_ram_install(FW_STAGE_BASE, FW_APP_BASE, size);
}
#else
/* No internal installer on 2C53T: fw_cache.c programs the app slot from RAM
 * while reading the W25Q, and ends in SYSRESETREQ rather than a jump. */
void fw_update_service(void) {}
#endif

void fw_update_status(fw_update_status_t *status) {
    if (!status) {
        return;
    }
    __asm__ volatile("cpsid i" ::: "memory");
    *status = fw_status;
    __asm__ volatile("cpsie i" ::: "memory");
}
