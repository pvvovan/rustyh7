.type _reset_handler,%function
.thumb_func
.global _reset_handler
_reset_handler:
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
