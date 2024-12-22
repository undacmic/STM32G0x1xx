ENTRY(Rest_Handler)

MEMORY {
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K   
    SRAM  (xrw)     : ORIGIN = 0x20000000, LENGTH = 144K
}

OUTPUT_FORMAT ("elf32-littlearm")

SECTIONS {
    .isr_vector : {
        KEEP(*(.isr_vector))
    } >FLASH

    .text : {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    _sidata = LOADADDR(.data);

    .data : {
        . = ALIGN(4);
        _data_start = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    } >SRAM AT> FLASH

    .bss : {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        _bss_end = .;
    }

    _estack = ORIGIN(SRAM) + LENGTH(SRAM);
}