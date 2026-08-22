#pragma once

#include <stdint.h>

void usb_msc_init(void);
void usb_msc_set_enabled(uint8_t enabled);
void usb_msc_poll(void);
/* Second function of the composite device: start a stalled CDC IN transfer and
 * re-arm the CDC OUT endpoint once the shell has drained its ring. Called from
 * the main loop next to usb_msc_poll(); masks the USB interrupt for the moment
 * it touches the endpoint the interrupt also drives. */
void usb_msc_cdc_pump(void);
void usb_msc_debug_counts(uint16_t out[6]);
uint8_t usb_msc_store_screenshot(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
