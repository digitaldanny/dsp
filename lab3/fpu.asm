; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; | LAB #3: FPU Assembly Programming, Stack and Interfacing LCD     |
; |                                                                 |
; | AUTHOR: Daniel Hamilton                                         |
; |                                                                 |
; | SUMMARY: fpu.asm                                                |
; | This file runs Lab 3, Part II - Writing FPU Assembly, Line eq.  |
; | The program generates an output vector with the following       |
; | equation..                                                      |
; |                                                                 |
; |                  OUTPUT = (.311 * INPUT) - .181                 |
; |                                                                 |
; | The input vector to the equation is the following...            |
; |                                                                 |
; |                  INPUT = -2.4 : 0.1 : 2.5                       |
; |                          LENGTH = 50                            |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.global _c_int00
	.global out_addr
	.global in_start_value, in_vector_len, in_step_value

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                           REFERENCES                            |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.ref WDREG
	.ref GPIO_CTRL_REGS, GPIO_DATA_REGS
	.ref GPIO_DIR_A, GPIO_MUX1_A, GPIO_GMUX1_A, GPIO_PUD_A
	.ref GPIO_DAT_A, GPIO_SET_A, GPIO_CLR_A

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                              MACROS                             |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

; ------------------------------------------------------------------
; Initialize the Stack Pointer to point at address 0x0400 in the
; .stack section
; ------------------------------------------------------------------
initSP .macro
	EALLOW 	; enable writes to protected registers
	MOV SP, #0x0400 ; initialize the stack pointer for function calls
	EDIS 	; disable writes to protected regsiters
	.endm

; ------------------------------------------------------------------
; Disable the watchdog timer.
; REGISTERS USED: XAR0, AL
; ------------------------------------------------------------------
disableWD .macro
	EALLOW
	MOV	 AL, 	#0x40
	MOVL XAR0, 	#0x0000
	MOV  AR0,	#WDREG
	MOV  *XAR0, AL
	EDIS
	.endm

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                         DATA ALLOCATION                         |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.sect .data

in_addr: .long 0x00008100		; location input vector will be stored
out_addr: .long 0x00008200 	; location output vector will be stored

; -2.4 : 0.1 : 2.5
in_start_value: .float -2.4
in_step_value:  .float 0.1
in_vector_len:  .word 50

; line data -- Y = Ax + B
A: .float 0.311
B: .float -0.181

; Temprorary storage address for subroutines
sub_temp .usect ".ebss", 1

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                         PROGRAM MEMORY                          |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.sect .text

; ===================================================================
; SUMMARY: _c_int00 (MAIN)
; ===================================================================
_c_int00: ; start of boot._asm in RTS library

    initSP      ; SP = 0x400
    disableWD 	; disable watchdog timer

    ; generate the input vector -2.4 : 0.1 : 2.5
    ; ---------------------------------------------------------------
    MOVL XAR0, #in_start_value ; address to float
    PUSH XAR0

    MOVL XAR0, #in_step_value ; address to float
    PUSH XAR0

    MOVL XAR0, #in_vector_len ; address to 16-bit number
    MOVL XAR5, *XAR0 ; store the count value so it can be used in the next loop
  	PUSH XAR0

    LC  GEN_INPUT_VECTOR

	; generate the output vector using the input vector table
	; ---------------------------------------------------------------
	MOVL XAR0, #in_addr ; address to floating point vector
	MOVL XAR0, *XAR0
    PUSH XAR0

    MOVL XAR0, #in_vector_len ; address to 16-bit number
    MOVL *XAR0, XAR5 ; reload the vector_length address with the value before pushing
  	PUSH XAR0

    LC  GEN_OUTPUT_VECTOR

EOP B EOP, UNC ; end of program, infinite loop

; ===================================================================
; SUMMARY: GEN_INPUT_VECTOR (inputStartValue, stepValue, length)
; This subroutine generates the input values for the line equation
; with the following formula...
;
; [inputStartValue : stepValue : inputStartValue + length*stepValue]
;
; INPUTS:
; Stack Push 1 => address of float inputStartValue
; Stack Push 2 => address of float stepValue
; Stack Push 3 => address of 16-bit length
;
; OUTPUTS:
; N/A
;
; REGISTERS AFFECTED: XAR0, XAR1, XAR2, ACC
; ===================================================================
GEN_INPUT_VECTOR:

	; Save the return address so the subroutine can go back to the
	; correct PC value.
	; ---------------------------------------------------------------

    ; holding Return Address
    POP ACC

    ; Return address will be temporarily held in ebss section until end
    ; of the subroutine.
	MOVL XAR0, #sub_temp
	MOVL *XAR0, ACC

	; Save the input values to the subroutine.
	; ---------------------------------------------------------------
	POP XAR2 ; address of in_vector_len
	POP XAR1 ; address of in_step_value
	POP XAR0 ; address of in_start_value

	MOV32 R0H, *XAR0 ; float data of in_START_value
    MOV32 R1H, *XAR1 ; float data of in_STEP_value

    MOVL XAR1, #in_addr ; points to the input vector
    MOVL XAR1, *XAR1

gen_input_vector_loop:
	MOV32 *XAR1++, R0H ; input_vector[i] = nextValue
	DEC *XAR2

	ADDF32 R0H, R0H, R1H ; nextValue = currentValue + 0.1
	NOP ; align the pipelines

	; after 50 numbers have been generated, stop.
	CMP *XAR2, #0
	B gen_input_vector_exit, EQ
	B gen_input_vector_loop, UNC

gen_input_vector_exit:

	; Reload the Return Address from memory and push to the stack so
	; the subroutine can return.
	; ---------------------------------------------------------------
	MOVL XAR0, #sub_temp
	MOVL ACC, *XAR0
    PUSH ACC
	LRET

; ===================================================================
; SUMMARY: GEN_OUTPUT_VECTOR (inputStartAddr, length)
; This subroutine generates the entire output line vector by calling
; the EQUATION subroutine for 'length' number of values starting at
; the inputStartAddr.
;
; INPUTS:
; Stack Push 1 => inputStartAddr
; Stack Push 2 => length
;
; OUTPUTS:
; N/A
;
; REGISTERS AFFECTED: XAR0, ACC
; ===================================================================
GEN_OUTPUT_VECTOR:

	; Save the return address so the subroutine can go back to the
	; correct PC value.
	; ---------------------------------------------------------------

    ; holding Return Address
    POP ACC

    ; Return address will be temporarily held in ebss section until end
    ; of the subroutine.
	MOVL XAR0, #sub_temp
	MOVL *XAR0, ACC

	; Save the input values to the subroutine.
	; ---------------------------------------------------------------
	POP XAR1 ; address of in_vector_len
	POP XAR0 ; address of in_addr
    MOVL XAR2, #out_addr ; points to the output vector
    MOVL XAR2, *XAR2

gen_output_vector_loop:
	MOV32 R0H, *XAR0++ ; next value from input_vector
	DEC *XAR1 ; decrement length

	LC EQUATION ; ACC = (.311 * R0H) - .181
	MOVL *XAR2++, ACC ; store the output 32-bit float into output vector

	; after 50 numbers have been generated, stop.
	CMP *XAR1, #0
	B gen_output_vector_loop, NEQ
	B gen_output_vector_exit, UNC

gen_output_vector_exit:
	; Reload the Return Address from memory and push to the stack so
	; the subroutine can return.
	; ---------------------------------------------------------------
	MOVL XAR0, #sub_temp
	MOVL ACC, *XAR0
    PUSH ACC
	LRET

; ===================================================================
; SUMMARY: EQUATION (inValue)
; This subroutine returns an output based on an input to the following
; function [ACC = (0.311 * inValue) - 0.181].
;
; INPUTS:
; R0H => inValue
;
; OUTPUTS:
; ACC => Calculated floating point value.
;
; REGISTERS AFFECTED: R0H, R1H, R2H, ACC
; ===================================================================
EQUATION:
	PUSH XAR0

	MOVL XAR0, #A
	MOV32 R1H, *XAR0
	MOVL XAR0, #B
    MOV32 R2H, *XAR0

	MPYF32 R0H, R0H, R1H ; 0.311 * input
	NOP ; pipeline alignment

	ADDF32 R0H, R0H, R2H ; 0.311 * input - 0.181
	NOP ; pipeline alignment
	NOP
	NOP

	MOV32 ACC, R0H ; move to output

	POP XAR0
	LRET














