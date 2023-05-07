MEMORY
{
    FLASH      (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    AXI_SRAM  (rwx) : ORIGIN = 0x24000000, LENGTH = 128K
}

/* The entry point is the reset handler */
ENTRY(_reset_handler);

EXTERN(VECTOR_TABLE);

PROVIDE(NMI             = DefaultExceptionHandler);
PROVIDE(HardFault       = DefaultExceptionHandler);
PROVIDE(MemManage       = DefaultExceptionHandler);
PROVIDE(BusFault        = DefaultExceptionHandler);
PROVIDE(UsageFault      = DefaultExceptionHandler);
PROVIDE(SVCall          = DefaultExceptionHandler);
PROVIDE(DebugMonitor    = DefaultExceptionHandler);
PROVIDE(PendSV          = DefaultExceptionHandler);
PROVIDE(SysTick         = DefaultExceptionHandler);

SECTIONS
{
    .vector_table ORIGIN(FLASH) :
    {
        /* First entry: initial Stack Pointer value */
        LONG(ORIGIN(AXI_SRAM) + LENGTH(AXI_SRAM))

        KEEP(*(.vector_table))
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*);
        . = ALIGN(4);
    } > FLASH

    .rodata : {
        . = ALIGN(4);
        *(.rodata .rodata*)
        . = ALIGN(4);
    } > FLASH

    .data : {
        . = ALIGN(4);
        __SYM_databegin__ = .;
        *(.data .data*)
        . = ALIGN(4);
        __SYM_dataend__ = .;
    } > AXI_SRAM AT > FLASH
    __SYM_loaddatabegin__ = LOADADDR(.data);

    .bss : {
        . = ALIGN(4);
        __SYM_bssbegin__ = .;
        *(.bss .bss*)
        . = ALIGN(4);
        __SYM_bssend__ = .;
    } > AXI_SRAM

    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*);
    }
}
