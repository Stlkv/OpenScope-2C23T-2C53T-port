#include "fw_update.h"

#include "app_config.h"
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

enum {
    FW_APP_BASE = APP_BASE_ADDR,
    FW_STAGE_BASE = FW_STAGE_BASE_ADDR,
    FW_RAM_BASE = 0x20000000u,
    FW_RAM_END = 0x20036078u,
    FW_PAGE_SIZE = 2048u,
    FW_MAX_SIZE = APP_FLASH_SIZE_BYTES,
    FW_APP_END = FW_APP_BASE + FW_MAX_SIZE,
    FW_STAGE_PAGE_COUNT = FW_MAX_SIZE / FW_PAGE_SIZE,
    FW_STAGE_PAGE_BYTES = (FW_STAGE_PAGE_COUNT + 7u) / 8u,
    FW_STAGE_HALFWORD_COUNT = (FW_MAX_SIZE + 1u) / 2u,
    FW_STAGE_HALFWORD_BYTES = (FW_STAGE_HALFWORD_COUNT + 7u) / 8u,

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
#if FW_STORE_WRITER
static uint8_t fw_blob_mode;
#else
/* The provisioning image carries the payload itself and has no room to spare:
 * with fw_blob_mode a constant, the destination logic and the whole bank-1
 * register path fold away. */
#define fw_blob_mode 0u
#endif
static uint8_t fw_stage_pages[FW_STAGE_PAGE_BYTES];
static uint8_t fw_stage_halfwords[FW_STAGE_HALFWORD_BYTES];
static uint8_t fw_page_buffer[FW_PAGE_SIZE];

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

static uint32_t fw_dest_base(void) {
    return fw_blob_mode ? FPGA_BS_STORE_BASE : (uint32_t)FW_STAGE_BASE;
}

static uint32_t fw_dest_max(void) {
    return fw_blob_mode ? FPGA_BS_STORE_MAX : (uint32_t)FW_MAX_SIZE;
}

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
    uint32_t halfwords;

    if (fw_expected_size < 8192u || fw_expected_size > fw_dest_max()) {
        return 0;
    }
    halfwords = (fw_expected_size + 1u) / 2u;
    for (uint32_t bit = 0; bit < halfwords; ++bit) {
        if (!bit_get(fw_stage_halfwords, bit)) {
            return 0;
        }
    }
    return 1;
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
    bits_clear(fw_stage_halfwords, sizeof(fw_stage_halfwords));
    ++fw_status.sequence;
}

static uint8_t fw_stage_rewrite_page(uint32_t page_offset,
                                     uint16_t page_inner,
                                     const uint8_t *data,
                                     uint16_t len) {
    uint32_t page = page_offset / FW_PAGE_SIZE;
    uint32_t first_halfword = page_offset / 2u;
    uint32_t first_changed = (uint32_t)page_inner & ~1u;
    uint32_t end_changed = ((uint32_t)page_inner + len + 1u) & ~1u;

    for (uint32_t i = 0; i < FW_PAGE_SIZE; i += 2u) {
        uint32_t bit = first_halfword + i / 2u;
        uint16_t value = bit_get(fw_stage_halfwords, bit) ?
                         REG16(fw_dest_base() + page_offset + i) : 0xFFFFu;
        fw_page_buffer[i] = (uint8_t)value;
        fw_page_buffer[i + 1u] = (uint8_t)(value >> 8);
    }

    for (uint16_t i = 0; i < len; ++i) {
        fw_page_buffer[page_inner + i] = data[i];
    }
    for (uint32_t i = first_changed; i < end_changed; i += 2u) {
        bit_set(fw_stage_halfwords, first_halfword + i / 2u);
    }

    flash_erase_page(fw_dest_base() + page_offset);
    bit_set(fw_stage_pages, page);
    for (uint32_t i = 0; i < FW_PAGE_SIZE; i += 2u) {
        uint32_t bit = first_halfword + i / 2u;
        if (bit_get(fw_stage_halfwords, bit)) {
            uint16_t value = (uint16_t)fw_page_buffer[i] |
                             ((uint16_t)fw_page_buffer[i + 1u] << 8);
            flash_program_halfword(fw_dest_base() + page_offset + i, value);
        }
    }
    return 1;
}

static uint8_t fw_stage_write_page(uint32_t offset, const uint8_t *data, uint16_t len) {
    uint32_t page_offset = offset & ~(FW_PAGE_SIZE - 1u);
    uint16_t page_inner = (uint16_t)(offset - page_offset);
    uint32_t addr = fw_dest_base() + offset;
    uint32_t page = offset / FW_PAGE_SIZE;
    uint8_t rewrite = (uint8_t)(offset & 1u);

    if (page_inner + len > FW_PAGE_SIZE) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
        return 0;
    }

    for (uint16_t i = 0; i < len && !rewrite; i = (uint16_t)(i + 2u)) {
        uint32_t byte_off = offset + i;
        uint32_t word_bit = byte_off / 2u;
        uint16_t value = data[i];
        if (i + 1u < len) {
            value |= (uint16_t)data[i + 1u] << 8;
        } else {
            value |= 0xFF00u;
        }
        if (bit_get(fw_stage_halfwords, word_bit) && REG16(addr + i) != value) {
            rewrite = 1;
        }
    }

    flash_unlock(addr);
    if (rewrite) {
        uint8_t ok = fw_stage_rewrite_page(page_offset, page_inner, data, len);
        flash_lock(addr);
        return ok;
    }

    if (!bit_get(fw_stage_pages, page)) {
        flash_erase_page(fw_dest_base() + page * FW_PAGE_SIZE);
        bit_set(fw_stage_pages, page);
    }

    for (uint16_t i = 0; i < len; i = (uint16_t)(i + 2u)) {
        uint32_t byte_off = offset + i;
        uint32_t word_bit = byte_off / 2u;
        uint16_t value = data[i];
        if (i + 1u < len) {
            value |= (uint16_t)data[i + 1u] << 8;
        } else {
            value |= 0xFF00u;
        }
        if (bit_get(fw_stage_halfwords, word_bit)) {
            if (REG16(addr + i) == value) {
                continue;
            }
            flash_lock(addr);
            return fw_stage_write_page(offset, data, len);
        }
        flash_program_halfword(addr + i, value);
        bit_set(fw_stage_halfwords, word_bit);
    }
    flash_lock(addr);
    return 1;
}

static uint8_t fw_stage_write(uint32_t offset, const uint8_t *data, uint16_t len) {
    uint32_t end_offset = offset + len;

    if (offset >= fw_dest_max() || (uint32_t)len > fw_dest_max() - offset) {
        fw_set_error(FW_UPDATE_ERR_RANGE);
        return 0;
    }

    while (len) {
        uint16_t page_inner = (uint16_t)(offset & (FW_PAGE_SIZE - 1u));
        uint16_t chunk = (uint16_t)min_u32(len, FW_PAGE_SIZE - page_inner);
        if (!fw_stage_write_page(offset, data, chunk)) {
            flash_lock(fw_dest_base() + offset);
            return 0;
        }
        offset += chunk;
        data += chunk;
        len = (uint16_t)(len - chunk);
    }

    if (fw_status.bytes < end_offset) {
        fw_status.bytes = end_offset;
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
#if FW_STORE_WRITER
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
#if FW_STORE_WRITER
    fw_blob_mode = 0;
#endif
    fw_status.state = FW_UPDATE_STATE_IDLE;
    fw_status.error = FW_UPDATE_ERR_NONE;
    fw_status.bytes = 0;
    fw_status.expected_size = 0;
    fw_status.base_lba = 0;
    ++fw_status.sequence;
}

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

void fw_update_status(fw_update_status_t *status) {
    if (!status) {
        return;
    }
    __asm__ volatile("cpsid i" ::: "memory");
    *status = fw_status;
    __asm__ volatile("cpsie i" ::: "memory");
}
