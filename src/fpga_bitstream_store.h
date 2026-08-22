#pragma once

#include <stdint.h>

/*
 * Where the FPGA bitstream lives when it is not carried inside the app image.
 *
 * The payload is 115,638 bytes — over half of a 224 KB app slot, and it is
 * constant data, not code. Keeping it out of the image is what turns "12 bytes
 * of headroom" into "half the slot free", so the default build reads it from
 * its own region of internal flash instead.
 *
 * Flash map (AT32F403A, 1 MB, 0x08000000-0x080FFFFF) — the SHARED map,
 * agreed geometry for BOTH firmwares that take turns in the app slot
 * (this port and upstream DavidClawson's OpenScope; see
 * docs/plans/flash-uklad-v2-2026-08-22.md in the workspace):
 *
 *   0x08000000  factory IAP bootloader           28 KB   nobody writes this
 *   0x08006000  factory calibration page          4 KB   dump/restore tooling
 *   0x08007000  app slot                     to 740 KB   whichever firmware
 *                                                        is installed; the
 *                                                        upstream image is
 *                                                        595 KB and GROWING
 *                                                        (there is no staging
 *                                                        region any more: a
 *                                                        dropped image is read
 *                                                        back into a W25Q cache
 *                                                        slot and installed
 *                                                        from there, so the app
 *                                                        slot owns everything
 *                                                        up to the store —
 *                                                        docs/plans/drop-
 *                                                        internal-staging-
 *                                                        2026-08-22.md)
 *   0x080C0000  FPGA bitstream store            128 KB   <- this region
 *   0x080E0000  reserve                          126 KB
 *   0x080FF800  settings (ours)                   2 KB
 *
 * MOVED 2026-08-22, from 0x08080000: the upstream image outgrew 512 KB and
 * overwrote the old location twice on this bench. 0x080C0000 clears a 740 KB
 * app-slot ceiling. Migration is one file: the store is provisioned by
 * dropping F2C23T-FPGA-BITSTREAM.BIN on the USB volume, and a boot with an
 * empty/invalid store just reports BS=0 until that is done.
 *
 * Layout inside the store: a 16-byte header followed by the payload. The whole
 * thing is written verbatim from a file the host drops on the USB volume, so
 * the header is part of the file, not something the firmware synthesises. A
 * truncated or corrupted transfer is caught at boot by recomputing the
 * fingerprint over the payload and comparing it with the header's copy —
 * which is why the header may sit ahead of the data it describes.
 */

#define FPGA_BS_STORE_BASE  0x080C0000u
#define FPGA_BS_STORE_MAX   0x00020000u /* 128 KB */
#define FPGA_BS_HDR_BYTES   16u
#define FPGA_BS_DATA_BASE   (FPGA_BS_STORE_BASE + FPGA_BS_HDR_BYTES)

/* 'G','W','B','S' little-endian — Gowin bitstream store. */
#define FPGA_BS_MAGIC       0x53425747u

typedef struct {
    uint32_t magic;
    uint32_t length;      /* payload bytes following the header */
    uint32_t fingerprint; /* rolling sum over the payload, see fpga.c */
    uint32_t check;       /* ~(magic + length + fingerprint) */
} fpga_bs_header_t;
