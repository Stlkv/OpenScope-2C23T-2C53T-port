#pragma once

#include <stdint.h>

enum {
    FW_UPDATE_STATE_IDLE,
    FW_UPDATE_STATE_STAGING,
    FW_UPDATE_STATE_READY,
    FW_UPDATE_STATE_ERROR,
    FW_UPDATE_STATE_APPLYING,
};

enum {
    FW_UPDATE_ERR_NONE,
    FW_UPDATE_ERR_RANGE,
    FW_UPDATE_ERR_VECTOR,
    /* The staged bytes did not arrive in order, or a re-sent chunk disagreed
     * with what is already in flash. Like every staging error this drops the
     * file from the volume; copy it again to retry. */
    FW_UPDATE_ERR_ORDER,
};

typedef struct {
    uint8_t state;
    uint8_t error;
    uint32_t sequence;
    uint32_t bytes;
    uint32_t expected_size;
    uint32_t base_lba;
} fw_update_status_t;

/* Factory-calibration restore (CALRSTOR.BIN -> the cal page, usb_msc.c).
 * Status codes, readable via the getters below and printed as the CALW line
 * in DBG.TXT — a run must be distinguishable from "never ran" and a no-op
 * from a write (the FW st= lesson):
 *   0x00 never ran          0x01 no-op, page already identical
 *   0x02 written+verified   0xE1 bad file size (must be exactly 4096)
 *   0xE2 verify mismatch after programming (page is now suspect!) */
enum {
    FW_CAL_RESTORE_IDLE      = 0x00,
    FW_CAL_RESTORE_IDENTICAL = 0x01,
    FW_CAL_RESTORE_WRITTEN   = 0x02,
    FW_CAL_RESTORE_BADSIZE   = 0xE1,
    FW_CAL_RESTORE_MISMATCH  = 0xE2,
};

/* data must hold exactly 4096 bytes (pass len anyway; it is the size gate).
 * Returns the status code it also latches for the getters. */
uint8_t  fw_cal_restore(const uint8_t *data, uint32_t len);
uint8_t  fw_cal_restore_status(void);
uint8_t  fw_cal_restore_runs(void);
uint32_t fw_cal_restore_crc(void);

void fw_update_usb_data(uint32_t lba, uint16_t sector_offset, const uint8_t *data, uint16_t len);
void fw_update_note_file(uint32_t base_lba, uint32_t size);
/* Route the next staged file: 0 = firmware image (staged, then installed over
 * the app slot), 1 = FPGA bitstream written straight to its own flash region.
 * Reset to 0 by fw_update_clear(). */
void fw_update_set_blob_mode(uint8_t blob);
uint8_t fw_update_request_apply(void);
void fw_update_clear(void);
void fw_update_service(void);
void fw_update_status(fw_update_status_t *status);
