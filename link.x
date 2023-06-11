MEMORY
{
    FLASH  (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (rwx) : ORIGIN = 0x24000000, LENGTH = 320K
}

ENTRY(Reset);
EXTERN(RESET_VECTOR);
EXTERN(EXCEPTIONS);

SECTIONS
{
    .vector_table ORIGIN(FLASH) :
    {
        /* First entry: initial Stack Pointer value */
        LONG(ORIGIN(RAM) + LENGTH(RAM));

        /* Second entry: reset vector */
        KEEP(*(.vector_table.reset_vector));

        /* The next 14 entries are exception vectors */
        KEEP(*(.vector_table.exceptions));
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*);
        . = ALIGN(4);
    } > FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*);
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .data : AT(ADDR(.rodata) + SIZEOF(.rodata))
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
    } > RAM

    _sidata = LOADADDR(.data);

    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*);
    }
}

PROVIDE(NMI = DefaultExceptionHandler);
PROVIDE(HardFault = DefaultExceptionHandler);
PROVIDE(MemManage = DefaultExceptionHandler);
PROVIDE(BusFault = DefaultExceptionHandler);
PROVIDE(UsageFault = DefaultExceptionHandler);
PROVIDE(SVCall = DefaultExceptionHandler);
PROVIDE(DebugMonitor = DefaultExceptionHandler);
PROVIDE(PendSV = DefaultExceptionHandler);
PROVIDE(SysTick = DefaultExceptionHandler);
