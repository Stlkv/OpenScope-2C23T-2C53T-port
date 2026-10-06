#include <stdint.h>

#include "app_config.h"
#include "dbgdump.h"

extern uint32_t __data_load;
extern uint32_t __data_start;
extern uint32_t __data_end;
extern uint32_t __bss_start;
extern uint32_t __bss_end;
extern uint32_t __stack_top;

void Reset_Handler(void);
void Reset_Handler_C(void);
void Default_Handler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
void TMR1_UP_IRQHandler(void);
#if HW_TARGET_2C53T
void USART2_IRQHandler(void);
#else
void USART3_IRQHandler(void);
#endif
void power_key_irq_handler(void);
#if HW_TARGET_2C53T
void board_dmm_beep_irq_handler(void);
#endif
void fpga_capture_ready_irq_handler(void);
int main(void);

__attribute__((section(".isr_vector"), used))
void (*const vector_table[128])(void) = {
    [0] = (void (*)(void))&__stack_top,
    [1] = Reset_Handler,
    [2] = Default_Handler,
    [3] = Default_Handler,
    [4] = Default_Handler,
    [5] = Default_Handler,
    [6] = Default_Handler,
    [11] = Default_Handler,
    [12] = Default_Handler,
    [14] = Default_Handler,
    [15] = Default_Handler,
    [16 ... 24] = Default_Handler,
    [25] = EXTI3_IRQHandler,
    [26] = EXTI4_IRQHandler,
    [27 ... 35] = Default_Handler,
    [36] = USB_LP_CAN1_RX0_IRQHandler,
    [37 ... 38] = Default_Handler,
    [39] = EXTI9_5_IRQHandler,
    [40] = Default_Handler,
    [41] = TMR1_UP_IRQHandler,
#if HW_TARGET_2C53T
    [42 ... 53] = Default_Handler,
    [54] = USART2_IRQHandler, /* meter SoC, dmm53.c */
    [55] = Default_Handler,
#else
    [42 ... 54] = Default_Handler,
    [55] = USART3_IRQHandler,
#endif
    [56 ... 127] = Default_Handler,
};

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define SCB_VTOR   0xE000ED08u
#define SYST_CSR   0xE000E010u
#define NVIC_ICER0 0xE000E180u
#define NVIC_ICPR0 0xE000E280u
#define RCC_APB2ENR 0x40021018u
#define GPIOA_CRL   0x40010800u
#define GPIOA_BRR   0x40010814u
#define GPIOB_CRL   0x40010C00u
#define GPIOB_BSRR  0x40010C10u

__attribute__((naked))
void Reset_Handler(void) {
    __asm__ volatile(
        "ldr r0, =__stack_top\n"
        "msr msp, r0\n"
        "cpsid i\n"
        "ldr r0, =0x40021018\n"
        "ldr r1, [r0]\n"
        "movs r2, #0x2c\n"
        "orrs r1, r2\n"
        "str r1, [r0]\n"
        "ldr r1, [r0]\n"
        "ldr r0, =0x40010c00\n"
        "ldr r1, [r0]\n"
        "bic r1, r1, #0x0f00\n"
        "orr r1, r1, #0x0100\n"
        "str r1, [r0]\n"
        "ldr r0, =0x40010c10\n"
        "movs r1, #0x04\n"
        "str r1, [r0]\n"
        "ldr r0, =0x40010800\n"
        "ldr r1, [r0]\n"
        "bic r1, r1, #0x0f\n"
        "orr r1, r1, #0x01\n"
        "str r1, [r0]\n"
        "ldr r0, =0x40010814\n"
        "movs r1, #0x01\n"
        "str r1, [r0]\n"
        "dsb\n"
        "b Reset_Handler_C\n"
    );
}

static void disable_pending_interrupts(void) {
    REG32(SCB_VTOR) = APP_BASE_ADDR;
    REG32(SYST_CSR) = 0;
    for (uint32_t i = 0; i < 8; ++i) {
        REG32(NVIC_ICER0 + i * 4u) = 0xFFFFFFFFu;
        REG32(NVIC_ICPR0 + i * 4u) = 0xFFFFFFFFu;
    }
    __asm__ volatile("dsb\nisb");
}

static void early_panel_blank(void) {
    REG32(RCC_APB2ENR) |= (1u << 2) | (1u << 3); // GPIOA/GPIOB clock
    REG32(GPIOB_CRL) = (REG32(GPIOB_CRL) & ~(0xFu << 8)) | (0x1u << 8); // PB2 power hold
    REG32(GPIOB_BSRR) = 1u << 2;
    REG32(GPIOA_CRL) = (REG32(GPIOA_CRL) & ~0xFu) | 0x1u; // PA0 output
    REG32(GPIOA_BRR) = 1u; // LCD backlight off before clearing large .bss
}

void Reset_Handler_C(void) {
    early_panel_blank();
    disable_pending_interrupts();

    uint32_t *src = &__data_load;
    for (uint32_t *dst = &__data_start; dst < &__data_end;) {
        *dst++ = *src++;
    }

    for (uint32_t *dst = &__bss_start; dst < &__bss_end;) {
        *dst++ = 0;
    }

    (void)main();
    while (1) {
    }
}

/* Every fault and every unexpected interrupt lands here. It used to spin,
 * which froze the unit with nothing to show for it -- the power key is polled
 * by the main loop, so not even that worked. Now it records where the core
 * was and resets; the next boot's dump prints the record (FLT line). */
#define FAULT_REC_MAGIC 0x464C5431u /* "FLT1" */
#define SCB_CFSR  0xE000ED28u
#define SCB_HFSR  0xE000ED2Cu
#define SCB_BFAR  0xE000ED38u
#define SCB_AIRCR 0xE000ED0Cu

__attribute__((section(".fault_rec"), used))
volatile fault_rec_t fault_rec;

__attribute__((used))
void fault_record_and_reset(const uint32_t *frame, uint32_t ipsr) {
    fault_rec.count = fault_rec.magic == FAULT_REC_MAGIC ? fault_rec.count + 1u : 1u;
    fault_rec.magic = FAULT_REC_MAGIC;
    fault_rec.ipsr = ipsr;
    fault_rec.pc = frame[6];
    fault_rec.lr = frame[5];
    fault_rec.cfsr = REG32(SCB_CFSR);
    fault_rec.hfsr = REG32(SCB_HFSR);
    fault_rec.bfar = REG32(SCB_BFAR);
    __asm__ volatile("dsb");
    REG32(SCB_AIRCR) = 0x05FA0004u; /* SYSRESETREQ */
    while (1) {
    }
}

__attribute__((naked))
void Default_Handler(void) {
    __asm__ volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mrs r1, ipsr\n"
        "b fault_record_and_reset\n"
    );
}

void EXTI3_IRQHandler(void) {
#if HW_TARGET_2C53T
    /* PC3, the meter SoC's beep request; POWER is polled on this board. */
    board_dmm_beep_irq_handler();
#else
    power_key_irq_handler();
#endif
}

void EXTI4_IRQHandler(void) {
    fpga_capture_ready_irq_handler();
}
