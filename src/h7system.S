.syntax unified
.cpu cortex-m7
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


sysinit:
    // Enabling the FPU
    ldr r0, =0xE000ED88;        // CPACR is located at address 0xE000ED88
    ldr r1, [r0];               // Read CPACR
    orr r1, r1, #(0xF << 20);   // Set bits 20-23 to enable CP10 and CP11 coprocessors
    str r1, [r0];               // wait for store to complete
    dsb;                        // reset pipeline now, the FPU is enabled
    isb

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


.section .vector_table, "ax"
.align 2
.global VECTOR_TABLE
VECTOR_TABLE:
.word _reset_handler
.word NMI
.word HardFault
.word MemManage
.word BusFault
.word UsageFault
.word 0
.word 0
.word 0
.word 0
.word SVCall
.word DebugMonitor
.word 0
.word PendSV
.word SysTick
.word DefaultExceptionHandler
