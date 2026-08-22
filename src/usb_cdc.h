#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

/* CDC-ACM byte stream, the second function of the composite USB device.
 *
 * The endpoint plumbing (descriptors, EP2 bulk pair, EP3 notification) lives
 * in usb_msc.c, because the device owns one descriptor set and the MSC volume
 * — the firmware-update path — must keep working byte for byte. This file is
 * only the two rings between that plumbing and the shell:
 *
 *   shell -> usb_cdc_write()    -> tx ring -> usb_cdc_tx_pull()  -> EP2 IN
 *   EP2 OUT -> usb_cdc_rx_push() -> rx ring -> usb_cdc_read()    -> shell
 *
 * The push/pull side runs in the USB interrupt, the read/write side in the
 * main loop. Each ring therefore has exactly one producer and one consumer,
 * so a volatile head/tail pair is enough and no masking is needed here. */

void usb_cdc_reset(void);

/* Host asserted DTR (SET_CONTROL_LINE_STATE bit 0). A closed port drops
 * writes on the floor instead of filling the ring: an unread stream must not
 * be able to wedge the shell. */
void usb_cdc_set_open(uint8_t open);
uint8_t usb_cdc_is_open(void);

/* Returns the number of bytes accepted, which is less than len when the ring
 * is full. Callers that care check usb_cdc_tx_free() first. */
uint16_t usb_cdc_write(const char *data, uint16_t len);
uint16_t usb_cdc_write_str(const char *s);
uint16_t usb_cdc_tx_free(void);

uint16_t usb_cdc_read(uint8_t *dst, uint16_t max);

/* ─── transport side, called from the USB interrupt ─────────────────── */
void usb_cdc_rx_push(const uint8_t *src, uint16_t len);
uint16_t usb_cdc_tx_pull(uint8_t *dst, uint16_t max);
uint16_t usb_cdc_rx_free(void);
uint8_t usb_cdc_tx_pending(void);

#endif
