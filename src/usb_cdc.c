#include "usb_cdc.h"

#if !defined(USB_CDC_SHELL) || USB_CDC_SHELL

enum {
    CDC_TX_SIZE = 1024, /* one dbg dump is 2 KB; the shell feeds it in chunks */
    CDC_TX_MASK = CDC_TX_SIZE - 1,
    CDC_RX_SIZE = 128,
    CDC_RX_MASK = CDC_RX_SIZE - 1,
};

static uint8_t cdc_tx[CDC_TX_SIZE];
static volatile uint16_t cdc_tx_head; /* written by the shell   */
static volatile uint16_t cdc_tx_tail; /* read by the interrupt  */

static uint8_t cdc_rx[CDC_RX_SIZE];
static volatile uint16_t cdc_rx_head; /* written by the interrupt */
static volatile uint16_t cdc_rx_tail; /* read by the shell       */

static volatile uint8_t cdc_open;

void usb_cdc_reset(void) {
    cdc_tx_head = 0;
    cdc_tx_tail = 0;
    cdc_rx_head = 0;
    cdc_rx_tail = 0;
    cdc_open = 0;
}

void usb_cdc_set_open(uint8_t open) {
    if (!open) {
        /* Drop whatever the host never collected: on the next open the shell
         * should answer the new session, not finish the old dump. */
        cdc_tx_tail = cdc_tx_head;
        cdc_rx_tail = cdc_rx_head;
    }
    cdc_open = open ? 1u : 0u;
}

uint8_t usb_cdc_is_open(void) {
    return cdc_open;
}

uint16_t usb_cdc_tx_free(void) {
    uint16_t head = cdc_tx_head;
    uint16_t tail = cdc_tx_tail;
    return (uint16_t)(CDC_TX_SIZE - 1u - ((head - tail) & CDC_TX_MASK));
}

uint16_t usb_cdc_write(const char *data, uint16_t len) {
    uint16_t head = cdc_tx_head;
    uint16_t written = 0;

    if (!cdc_open || data == 0) {
        return 0;
    }
    while (written < len) {
        uint16_t next = (uint16_t)((head + 1u) & CDC_TX_MASK);
        if (next == (cdc_tx_tail & CDC_TX_MASK)) {
            break; /* full — the caller decides whether to retry */
        }
        cdc_tx[head] = (uint8_t)data[written++];
        head = next;
    }
    cdc_tx_head = head;
    return written;
}

uint16_t usb_cdc_write_str(const char *s) {
    uint16_t len = 0;
    if (s == 0) {
        return 0;
    }
    while (s[len]) {
        ++len;
    }
    return usb_cdc_write(s, len);
}

uint16_t usb_cdc_tx_pull(uint8_t *dst, uint16_t max) {
    uint16_t tail = cdc_tx_tail;
    uint16_t n = 0;

    while (n < max && tail != cdc_tx_head) {
        dst[n++] = cdc_tx[tail];
        tail = (uint16_t)((tail + 1u) & CDC_TX_MASK);
    }
    cdc_tx_tail = tail;
    return n;
}

uint8_t usb_cdc_tx_pending(void) {
    return cdc_tx_head != cdc_tx_tail;
}

void usb_cdc_rx_push(const uint8_t *src, uint16_t len) {
    uint16_t head = cdc_rx_head;
    uint16_t i;

    /* Bytes arriving are proof someone is at the other end, whatever they did
     * with DTR: `screen` and pyserial assert it, a plain `cat` does not, and a
     * shell that answers only half the terminals on the bench is a shell that
     * will be blamed for the wrong thing. */
    if (len) {
        cdc_open = 1;
    }
    for (i = 0; i < len; ++i) {
        uint16_t next = (uint16_t)((head + 1u) & CDC_RX_MASK);
        if (next == (cdc_rx_tail & CDC_RX_MASK)) {
            break; /* the shell is behind; drop the rest of the packet */
        }
        cdc_rx[head] = src[i];
        head = next;
    }
    cdc_rx_head = head;
}

uint16_t usb_cdc_rx_free(void) {
    uint16_t head = cdc_rx_head;
    uint16_t tail = cdc_rx_tail;
    return (uint16_t)(CDC_RX_SIZE - 1u - ((head - tail) & CDC_RX_MASK));
}

uint16_t usb_cdc_read(uint8_t *dst, uint16_t max) {
    uint16_t tail = cdc_rx_tail;
    uint16_t n = 0;

    while (n < max && tail != cdc_rx_head) {
        dst[n++] = cdc_rx[tail];
        tail = (uint16_t)((tail + 1u) & CDC_RX_MASK);
    }
    cdc_rx_tail = tail;
    return n;
}

#else  /* USB_CDC_SHELL == 0 */

void usb_cdc_reset(void) {}

#endif
