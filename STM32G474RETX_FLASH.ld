/*
******************************************************************************
**
** File        : LinkerScript.ld
**
** Abstract    : Linker script for STM32G474RETx Device from STM32G4 series
**                      512Kbytes FLASH
**                      128Kbytes RAM
**
**               Set heap size, stack size and stack location according
**               to application requirements.
**
**               Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is without any warranty
**                of any kind.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of "RAM" Ram type memory */

_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400;     /* required amount of stack */

/* Memories definition */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH  (rx)     : ORIGIN = 0x08000000,   LENGTH = 448K
  /* Pages 112-127 reserved for NVM data — not part of FLASH region.
   * Page 112 (0x08070000): tcs_tuning_store.c      TCS tuning (service diag C1)
   * Page 113 (0x08071000): geometry_store.c        Ackermann geometry (service diag C2)
   * Page 114 (0x08072000): shunt_store.c           INA226 shunt mOhm (service diag C3)
   * Page 115 (0x08073000): steering_service_store.c homing/current tuning (service diag C4)
   * Page 116 (0x08074000): wheel_sensor_store.c    wheel encoder tuning (service diag C5)
   * Page 117 (0x08075000): service_diag_report.c   last self-test report (service diag B6)
   * Page 118 (0x08076000): gear_limits_store.c redundant journal page
   * Page 119 (0x08077000): pedal_cal_store.c redundant journal page
   * Page 120 (0x08078000): battery_limits_store.c battery V thresholds
   * Page 121 (0x08079000): drive_tuning_store.c   ramp/creep tuning
   * Page 122 (0x0807A000): gear_limits_store.c   gear power limits (D2/D1/R)
   * Page 123 (0x0807B000): sensor_map_store.c    DS18B20 physIdx→role map
   * Page 124 (0x0807C000): pedal_cal_store.c     pedal ADC calibration
   * Page 125 (0x0807D000): error_log.c           persistent error ring buffer
   * Page 126 (0x0807E000): steering_cal_store.c  steering calibration
   * Page 127 (0x0807F000): eps_params.c          EPS tuning parameters
   *
   * FLASH region was reduced 500K → 496K → 492K → 488K → 480K → 472K → 448K
   * to free pages 124/123 for the pedal calibration NVM slot and the DS18B20
   * sensor map, page 122 for the gear power-limit NVM slot, pages 121/120 for
   * the drive-tuning and battery-limit NVM slots, and pages 117-112 for the
   * six service-autodiagnosis NVM slots (five runtime-tunable parameter
   * stores plus the persisted self-test report).  .text / .rodata / .data
   * init image are guaranteed to end below 0x08070000.                 */
  EPS_PARAMS (r)  : ORIGIN = 0x0807F000,   LENGTH = 4K
}

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { 
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH
  
  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array     :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH
  
  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH
  
  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* ----------------------------------------------------------------------
 * Defense-in-depth: guarantee that the FLASH-resident image (code +
 * rodata + .data init image) never overflows into the NVM region that
 * starts at page 112 (0x08070000).  The MEMORY region `LENGTH = 448K`
 * above already enforces this, but the explicit ASSERT below produces
 * a clearer diagnostic ("Code overlaps NVM page 112") if some future
 * tweak silently widens the FLASH region.  Standard GNU ld syntax,
 * no impact on the existing toolchain.
 * --------------------------------------------------------------------*/
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= 0x08070000,
       "Code overlaps NVM page 112. Reduce code size or revisit FLASH=448K reservation.")
