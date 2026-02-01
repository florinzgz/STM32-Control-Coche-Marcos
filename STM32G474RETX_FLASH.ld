/* Linker script for STM32G474RE memory layout */

OUTPUT(file.elf)

MEMORY
{
  FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (xrw)     : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  .text : { *(.text) }
  .data : { *(.data) }
  .bss  : { *(.bss) }
}