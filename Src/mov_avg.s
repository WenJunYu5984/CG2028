/*
 * mov_avg.s
 *
 * CG2028 Assignment starter file.
 */
.syntax unified
.cpu cortex-m4
.thumb
.global ewma_filter
.type ewma_filter, %function

.text
.align 2

@ CG2028 Assignment
@ (c) ECE NUS
@ Write Student 1's Name here: ABCD (A1234567R)
@ Write Student 2's Name here: WXYZ (A0000007X)
@
@ Function prototype:
@   int ewma_filter(int new_data, int old_output, int alpha_percent);
@
@ ARM calling convention:
@   R0 = new_data       (signed integer sensor sample)
@   R1 = old_output     (previous filtered output)
@   R2 = alpha_percent  (integer from 0 to 100)
@   Return R0 = (alpha_percent * new_data
@                + (100 - alpha_percent) * old_output) / 100
@
@ Notes:
@ - Use signed integer arithmetic.
@ - Integer division must truncate towards zero, matching C integer division.
@ - Preserve all callee-saved registers that you use (R4-R11).
@ - Do not call a C helper function and do not use floating-point instructions.
@
@ Register table:
@   R0 ...
@   R1 ...
@   R2 ...
@   R3 ...
@   R4 ...
@
@ Write your program from here.
ewma_filter:
	@ r0-r3 are caller-saved, so this leaf function needs no stack frame.
	RSB     r3, r2, #100        @ r3 = 100 - alpha_percent
    MUL     r0, r2, r0          @ r0 = alpha_percent * new_data
    /*
    MUL     r3, r3, r1          @ r3 = (100 - alpha_percent) * old_output
    ADD     r0, r0, r3          @ r0 = alpha*new_data + (100-alpha)*old_output
    */
    MLA     r0, r3, r1, r0
    MOV     r1, #100
    SDIV    r0, r0, r1          @ r0 = sum / 100, truncated toward zero (signed)

    BX      lr

.size ewma_filter, .-ewma_filter
