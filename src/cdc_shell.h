#ifndef CDC_SHELL_H
#define CDC_SHELL_H

#include <stdint.h>

/* Line shell on the CDC-ACM pipe (usb_cdc.c): live telemetry and firmware
 * loading over the SAME cable and the SAME USB device that carries the MSC
 * volume, so nothing about the file-drop update path changes.
 *
 * Two cadences, matching the rest of the port:
 *   cdc_shell_tick(20)   once per UI frame — timekeeping and `mon` pacing
 *   cdc_shell_service()  in the 1 ms idle slots — drain RX, feed TX
 *
 * Both run in the main loop; the endpoints are serviced by the USB interrupt
 * underneath them. */

void cdc_shell_tick(uint16_t ms);
void cdc_shell_service(void);

#endif
