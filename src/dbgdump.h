#ifndef DBGDUMP_H
#define DBGDUMP_H

#include <stdint.h>

/* Render the debug-telemetry text (host-triggered DBGREQ -> DBG.TXT dump).
 * Returns the number of bytes written (no NUL included in the count). */
uint16_t dbgdump_render(char *dst, uint16_t cap);

#endif
