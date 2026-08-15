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
 * Flash map (AT32F403A, 1 MB, 0x08000000-0x080FFFFF):
 *
 *   0x08000000  factory IAP bootloader           28 KB
 *   0x08007000  app slot                        224 KB   <- the image we build
 *   0x08040000  self-update staging             256 KB   <- room to grow to
 *   0x08080000  FPGA bitstream store            128 KB   <- this region
 *   0x080A0000  free                            384 KB
 *
 * The store is deliberately placed above the staging area's growth room, so
 * enlarging the app slot later (staging can run to 0x08080000) does not have
 * to move the bitstream and re-provision every unit.
 *
 * Layout inside the store: a 16-byte header followed by the payload. The whole
 * thing is written verbatim from a file the host drops on the USB volume, so
 * the header is part of the file, not something the firmware synthesises. A
 * truncated or corrupted transfer is caught at boot by recomputing the
 * fingerprint over the payload and comparing it with the header's copy —
 * which is why the header may sit ahead of the data it describes.
 */

#define FPGA_BS_STORE_BASE  0x08080000u
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
