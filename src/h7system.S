.type _reset_handler,%function
.thumb_func
.global _reset_handler
_reset_handler:
    bl sysinit
    ldr r1, =main
    blx r1
stop_here:
    bkpt #0
    b stop_here


.section .vector_table, "ax"
.align 2
.global __VECTOR_TABLE
__VECTOR_TABLE:
.word _reset_handler

sysinit:
    // Zero out the BSS
    ldr r1, =__SYM_bssbegin__
    ldr r2, =__SYM_bssend__
    movs r0, #0
    b bss_fill_test
bss_fill_loop:
    stm r1!, {r0}
bss_fill_test:
    cmp r1, r2
    bne bss_fill_loop

    // Copy the DATA section
    ldr r1, =__SYM_databegin__
    ldr r2, =__SYM_dataend__
    ldr r3, =__SYM_loaddatabegin__
    b data_copy_test
data_copy_loop:
    ldm r3!, {r0}
    stm r1!, {r0}
data_copy_test:
    cmp r1, r2
    bne data_copy_loop

    bx lr
