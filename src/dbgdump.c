#include "dbgdump.h"

#include "fpga.h"
#include "hw.h"
#include "scope.h"

/* Host-triggered telemetry dump: the host drops an empty DBGREQ file on the
 * USB volume, the idle scan spots it, deletes it and (re)writes DBG.TXT with
 * this text. Everything the LCD debug overlay shows, plus raw GPIO/SPI state,
 * readable without buttons or screenshots. */

static uint16_t put_str(char *dst, uint16_t cap, uint16_t p, const char *s) {
    while (*s && p < cap) {
        dst[p++] = *s++;
    }
    return p;
}

static uint16_t put_hex(char *dst, uint16_t cap, uint16_t p, uint32_t v, uint8_t digits) {
    while (digits--) {
        if (p < cap) {
            dst[p++] = "0123456789ABCDEF"[(v >> (4u * digits)) & 0xFu];
        }
    }
    return p;
}

static uint16_t put_dec(char *dst, uint16_t cap, uint16_t p, uint32_t v) {
    char tmp[10];
    uint8_t n = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && n < sizeof(tmp));
    while (n-- && p < cap) {
        dst[p++] = tmp[n];
    }
    return p;
}

uint16_t dbgdump_render(char *dst, uint16_t cap) {
    fpga53_diag_t dg;
    uint16_t p = 0;

    fpga53_get_diag(&dg);

    p = put_str(dst, cap, p, "OpenScope 2C53T DBG dump v1\n");

    p = put_str(dst, cap, p, "I");
    p = put_dec(dst, cap, p, dg.inited);
    p = put_str(dst, cap, p, " P");
    p = put_dec(dst, cap, p, dg.pc0);
    p = put_str(dst, cap, p, " R");
    p = put_dec(dst, cap, p, dg.reads);
    p = put_str(dst, cap, p, " C");
    p = put_dec(dst, cap, p, dg.forced);
    p = put_str(dst, cap, p, " S");
    p = put_dec(dst, cap, p, scope_hw_last_status());
    p = put_str(dst, cap, p, " F");
    p = put_dec(dst, cap, p, scope_hw_frame_count());
    p = put_str(dst, cap, p, " W");
    p = put_dec(dst, cap, p, scope_hw_wait_count());
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "r0=");
    p = put_hex(dst, cap, p, dg.r0, 2);
    p = put_str(dst, cap, p, " r1=");
    p = put_hex(dst, cap, p, dg.r1, 2);
    p = put_str(dst, cap, p, " r2=");
    p = put_hex(dst, cap, p, dg.r2, 2);
    p = put_str(dst, cap, p, " ch1=");
    p = put_hex(dst, cap, p, dg.smin, 2);
    p = put_str(dst, cap, p, "-");
    p = put_hex(dst, cap, p, dg.smax, 2);
    p = put_str(dst, cap, p, " ch2=");
    p = put_hex(dst, cap, p, dg.smin2, 2);
    p = put_str(dst, cap, p, "-");
    p = put_hex(dst, cap, p, dg.smax2, 2);
    p = put_str(dst, cap, p, " D");
    p = put_dec(dst, cap, p, dg.dup);
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "Q=");
    for (uint8_t i = 0; i < 5u; ++i) {
        p = put_hex(dst, cap, p, dg.cst[i], 2);
        if (i < 4u) {
            p = put_str(dst, cap, p, " ");
        }
    }
    p = put_str(dst, cap, p, " A=");
    p = put_hex(dst, cap, p, dg.sweep_val, 2);
    p = put_str(dst, cap, p, dg.sweep_hit ? "!" : ".");
    p = put_str(dst, cap, p, " X=");
    p = put_hex(dst, cap, p, dg.fe_idx, 2);
    p = put_str(dst, cap, p, " Y=");
    p = put_hex(dst, cap, p, dg.fe_idx_b, 2);
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "V=");
    p = put_hex(dst, cap, p, dg.v04_id, 8);
    p = put_str(dst, cap, p, " B=");
    p = put_hex(dst, cap, p, dg.v04_stb, 8);
    p = put_str(dst, cap, p, " A=");
    p = put_hex(dst, cap, p, dg.v04_sta, 8);
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "calls init=");
    p = put_dec(dst, cap, p, dg.init_calls);
    p = put_str(dst, cap, p, " cfg=");
    p = put_dec(dst, cap, p, dg.cfg_calls);
    p = put_str(dst, cap, p, " poll=");
    p = put_dec(dst, cap, p, dg.poll_calls);
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "IDR A=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOA_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " B=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOB_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " C=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOC_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, " E=");
    p = put_hex(dst, cap, p, GPIO_IDR(GPIOE_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, "\n");

    p = put_str(dst, cap, p, "SPI3 CTRL1=");
    p = put_hex(dst, cap, p, SPI_CTRL1(SPI3_BASE) & 0xFFFFu, 4);
    p = put_str(dst, cap, p, "\n");

    return p;
}
