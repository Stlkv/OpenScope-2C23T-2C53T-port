#ifndef DBGDUMP_H
#define DBGDUMP_H

#include <stdint.h>

/* Render the debug-telemetry text (host-triggered DBGREQ -> DBG.TXT dump).
 * Returns the number of bytes written (no NUL included in the count). */
uint16_t dbgdump_render(char *dst, uint16_t cap);

/* CRC-32, reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF — matches zlib
 * and the host's `crc32`. Shared by the CAL line here and the cal-restore
 * path (fw_update.c) so both sides can never disagree on the algorithm. */
uint32_t dbgdump_crc32(const uint8_t *p, uint32_t len);

#endif
