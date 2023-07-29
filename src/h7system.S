.syntax unified
.cpu cortex-m7
.type _reset_handler,%function
.thumb_func
.global _reset_handler
_reset_handler:
    mov r1, 1
    cmp r1, 1
    beq _reset_handler
    ldr sp, =__SYM_StackTop__
    bl sysinit
    bl mybreakpoint
    ldr r1, =main
    blx r1
stop_here:
    bkpt #0
    b stop_here


.thumb_func
.type mybreakpoint,%function
mybreakpoint:
    push {r0, r1}
mystop:
    ldr.w r0, =stop_required
    ldr.w r1, [r0]
    mov r0, 1
    cmp r0, r1
    beq mystop
    pop {r0, r1}
    bx lr


.thumb_func
.type sysinit,%function
sysinit:
    push {r0-r3}

    // Enabling the FPU
    ldr.w r0, =0xE000ED88;      // CPACR is located at address 0xE000ED88
    ldr r1, [r0];               // Read CPACR
    orr r1, r1, #(0xF << 20);   // Set bits 20-23 to enable CP10 and CP11 coprocessors
    str r1, [r0];               // wait for store to complete
    dsb
    isb;                        // reset pipeline now, the FPU is enabled

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

    pop {r0-r3}
    bx lr


.section .vector_table, "ax"
.align 2
.global _vector_table
_vector_table:
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


.section .data, "aw"
.align 2
stop_required:
    .word 1
