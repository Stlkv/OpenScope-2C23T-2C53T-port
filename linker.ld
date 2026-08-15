ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08008000, LENGTH = 224K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x36078
}

SECTIONS
{
  .isr_vector ORIGIN(FLASH) :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text.Reset_Handler)
    *(.text*)
    *(.rodata*)
  } > FLASH

  /DISCARD/ :
  {
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }

  /* Survives an MCU reset and the updater's direct jump into the new image,
   * and must land at the SAME address in every build (the writer and the
   * reader are two different firmware images). Hence: first thing in RAM,
   * ahead of .data, and never touched by the startup code. */
  .warm_token ORIGIN(RAM) (NOLOAD) :
  {
    KEEP(*(.warm_token))
    . = ALIGN(8);
  } > RAM

  .data :
  {
    __data_load = LOADADDR(.data);
    __data_start = .;
    *(.data*)
    __data_end = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    __bss_start = .;
    *(.bss*)
    *(COMMON)
    __bss_end = .;
  } > RAM

  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit*)
    . = ALIGN(4);
  } > RAM

  . = ALIGN(8);
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
