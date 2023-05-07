MEMORY
{
    FLASH      (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    AXI_SRAM  (rwx) : ORIGIN = 0x24000000, LENGTH = 128K
}

/* The entry point is the reset handler */
ENTRY(_reset_handler);

SECTIONS
{
    .vector_table ORIGIN(FLASH) :
    {
        /* First entry: initial Stack Pointer value */
        LONG(ORIGIN(AXI_SRAM) + LENGTH(AXI_SRAM))

        /* Second entry: reset vector */
        KEEP(*(.vector_table))
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*);
    } > FLASH

    /DISCARD/ :
    {
        *(.ARM.exidx .ARM.exidx.*);
    }
}
