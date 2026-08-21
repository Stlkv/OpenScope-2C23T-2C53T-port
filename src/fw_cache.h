/*
 * fw_cache.h — A/B firmware image cache on the W25Q, and the installer
 * that swaps the app slot from it.
 *
 * WHY: the bench runs two firmwares that take turns in the app slot (this
 * port and upstream OpenScope, 595 KB and growing). The internal 1 MB can
 * never stage the big image (28K IAP + 595K slot + 595K stage > 1024K), so
 * both images live permanently in raw W25Q sectors OUTSIDE the FAT volume's
 * normal reach, and switching firmware needs no host at all:
 *
 *   fill a slot:  drop FWCACHEA.BIN / FWCACHEB.BIN on the USB volume
 *                 (any size up to the app-slot ceiling; the idle scan
 *                 copies it into the slot, CRC on the fly, manifest
 *                 written LAST so a torn copy can never look valid)
 *   swap:         drop an empty SWAPA / SWAPB file — the slot is
 *                 re-verified against its manifest, then a RAM-resident
 *                 installer reads the W25Q over SPI while programming the
 *                 app slot, verifies by read-back, and SYSTEM-RESETS into
 *                 the new image. Never a jump: a cross-firmware jump was
 *                 bench-tried (2026-08-22) and half-bricks.
 *
 * Chip layout (W25Q128, 16 MB; volume LBA0 = chip addr 0):
 *
 *   0x00E00000  slot A: manifest sector (4K) + image data   1 MB
 *   0x00F00000  slot B: manifest sector (4K) + image data   1 MB
 *
 * The FAT volume spans the chip, so a sufficiently full volume COULD place
 * file clusters up here; the manifest CRC catches any such clobber at swap
 * time and a re-drop of the cache file heals it. A carved-out region layer
 * is the clean fix and is deliberately out of scope here.
 *
 * Fits the shared flash map in fpga_bitstream_store.h: installed images may
 * run 0x08007000..0x080C0000 (740 KB ceiling, below the bitstream store).
 */

#ifndef FW_CACHE_H
#define FW_CACHE_H

#include <stdint.h>

#define FW_CACHE_SLOT_A     0u
#define FW_CACHE_SLOT_B     1u

/* Intake (usb_msc.c streams the file's bytes through these). */
uint8_t fw_cache_intake_begin(uint8_t slot, uint32_t size);
uint8_t fw_cache_intake_data(const uint8_t *data, uint16_t len);
uint8_t fw_cache_intake_finish(void);
void    fw_cache_intake_abort(void);

/* Swap. Verifies the slot's manifest + CRC + vector table, then installs
 * and SYSTEM-RESETS. Returns only on refusal (status says why). */
void fw_cache_swap(uint8_t slot);

/* One status byte per concern, for the DBG.TXT line (dbgdump.c):
 *   intake: 0 idle, 1 ok, 0xE1 size, 0xE2 w25q write, 0xE3 verify
 *   swap:   0 never, 0xE4 no/bad manifest, 0xE5 crc, 0xE6 vector
 *           (a successful swap is never observed — the device resets) */
uint8_t  fw_cache_intake_status(void);
uint8_t  fw_cache_swap_status(void);
/* Manifest presence probe for the debug line: size in bytes, 0 = invalid. */
uint32_t fw_cache_slot_size(uint8_t slot);
uint32_t fw_cache_slot_crc(uint8_t slot);

#endif /* FW_CACHE_H */
