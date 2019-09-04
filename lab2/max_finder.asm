; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; | LAB #2: Assembly programming, GPIO                              |
; |                                                                 |
; | AUTHOR: Daniel Hamilton                                         |
; |                                                                 |
; | SUMMARY: max_finder.asm                                         |
; | This file contains code from programs in Lab 2, parts II - IV.  |
; | A value stored in memory determines which part of the lab is    |
; | run when loading to the board.                                  |
; |                                                                 |
; | PT II:                                                          |
; | This program searches through a vector of signed "test scores"  |
; | ranging from 0-100 and stores the highest grade in the max_addr |
; |                                                                 |
; | PT III:                                                         |
; | This program plays a short pattern on the LEDs using a short    |
; | software delay.                                                 |
; |                                                                 |
; | PT IV:                                                          |
; | This program allows the user to control the output of the codec |
; | LEDs by using the codec buttons and switches.                   |
; |                                                                 |
; |                                                                 |
; | FILE USAGE:                                                     |
; | Setting the "ASSIGNMENT_PART" value in the REFERENCES section   |
; | for values between 1-3 will run the program written for the     |
; | lab's assignments Part 2-4.                                     |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.global _c_int00

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                           REFERENCES                            |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.ref score_addr			; stores an *address* to the vector
	.ref score_vector_len	; stores the length of the score vector
	.ref max_addr			; stores an *address* to the max value

; +=================================================================+
; |=================================================================|
; |=================================================================|
; |=================================================================|
; +=================================================================+
; |                                                                 |
; | 					  ASSIGNMENT_PART:                          |
; | 					1 => Lab 2, Part II                         |
; | 					2 => Lab 2, Part III                        |
; | 					3 => Lab 2, Part IV                         |
; |                                                                 |
; +-----------------------------------------------------------------+
ASSIGNMENT_PART .set 	1
; +-----------------------------------------------------------------+
; |=================================================================|
; |=================================================================|
; |=================================================================|
; +=================================================================+

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                              MACROS                             |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

; Sets active-low LEDs to "SET" value.
; Ex. SET = #0xFFFE, means LED 0 is on.
setLeds .macro SET
		PUSH 	XAR6
		MOVL 	XAR6, 	#GPIO_DAT_A
		MOV 	*XAR6, 	SET
		POP 	XAR6
		.endm

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                            REFERENCES                           |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

CMP_SIGNED 		.set 0
CMP_UNSIGNED 	.set 1

GPIO_CTRL_REGS	.set 0x7C00
GPIO_DATA_REGS 	.set 0x7F00

GPIO_DIR_A      .set GPIO_CTRL_REGS + 0x0A
GPIO_MUX1_A     .set GPIO_CTRL_REGS + 0x06
GPIO_GMUX1_A    .set GPIO_CTRL_REGS + 0x20
GPIO_DAT_A      .set GPIO_DATA_REGS + 0x00
GPIO_PUD_A      .set GPIO_CTRL_REGS + 0x0C
GPIO_SET_A      .set GPIO_DATA_REGS + 0x02
GPIO_CLR_A      .set GPIO_DATA_REGS + 0x04
; ----------------------------------------

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                         DATA ALLOCATION                         |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.data

; selects whether comparing signed or unsigned
; 0 => signed compare
; 1 => unsigned compare
s_or_u: .word 0

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                         PROGRAM MEMORY                          |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.text

; ===================================================================
; SUMMARY: _c_int00 (MAIN)
; ===================================================================
_c_int00: ; start of boot._asm in RTS library

		; Move the stack pointer to valid RAM location
		EALLOW ; enable protected register write
		MOV 	SP, #0x8400
		EDIS ; disable protected register write

		LC DISABLE_WATCHDOG

		; Determine which part of the file to run
		.if ASSIGNMENT_PART == 1
			B PART1, UNC
		.else
			.if ASSIGNMENT_PART == 2
				B PART2, UNC
			.else
				B PART3, UNC
			.endif
		.endif

PART1: ; ***********************************************************************************

		MOVL 	XAR0, 	#score_vector_len 	; load the vector length into this register to be decremented later
		MOVL 	XAR1, 	#score_addr 	    ; score vector POINTER
		MOVL 	XAR2, 	#max_addr 			; max value POINTER

part1_loop:
		; ---------------------------------------------------------------------------------
		DEC		*AR0				; decrement number of values remaining in the vector
		MOV 	AR6, 	AR1       	; move the next score in the vector to the input argument register
		LC      SUB_COMP_MAX	 	; determine if the input value AR6 is greater than the current max
		INC 	AR1     			; point to the next score in the vector

		; if there are no more values remaining in the vector, exit.
		; ---------------------------------------------------------------------------------
		CMP		*AR0, 	#0
		B		part1_loop,	NEQ

		; exit the program
		; ---------------------------------------------------------------------------------
		B       EOP, 	UNC

PART2: ; ***********************************************************************************

		EALLOW

		; dir - set GPIO(0-7) as outputs
		MOVL XAR6, #GPIO_DIR_A
		MOV  *XAR6, #0x00FF

		; gmux1 - GPIO mode
		MOVL XAR6, #GPIO_GMUX1_A
		MOV  *XAR6, #0x00

		; mux1
		MOVL XAR6, #GPIO_MUX1_A
		MOV  *XAR6, #0x00

		; data - Output all lights on for active low LEDs
		MOVL XAR6, #GPIO_DAT_A
		MOV  *XAR6, #0x0000

part2_loop:

		setLeds #0x00FF
		LC SW_DELAY

		setLeds #0x00FC
		LC SW_DELAY

		setLeds #0x00F3
		LC SW_DELAY

		setLeds #0x00CF
		LC SW_DELAY

		setLeds #0x003F
		LC SW_DELAY

		; loop pattern forever
		B 	part2_loop, UNC

PART3:

		EALLOW

		; dir
		; GPIO [7:0] - outputs
		; GPIO [31:8] - inputs
		MOVL 	XAR6, #GPIO_DIR_A
		MOV  	*XAR6++, #0x00FF
		MOV 	*XAR6, #0x0000

		; gmux1 - GPIO mode
		MOVL XAR6, #GPIO_GMUX1_A
		MOV  *XAR6++, #0x0000
		MOV	*XAR6, #0x0000

		; mux1
		MOVL XAR6, #GPIO_MUX1_A
		MOV  *XAR6++, #0x0000
		MOV	*XAR6, #0x0000

		; pullup - All input pullups are turned off
		MOVL XAR6, #GPIO_PUD_A
		MOV  *XAR6++, #0x0000
		MOV	*XAR6, #0x0000

		; data - Output all lights on for active low LEDs
		; LED4 is constantly on
		MOVL XAR6, #GPIO_DAT_A
		MOV  *XAR6, #0xFFEF

		; dirset reg
		MOVL XAR4, #GPIO_SET_A
		MOV	*XAR4, #0x0000

		; dirclr reg
		MOVL XAR7, #GPIO_CLR_A
		MOV *XAR7, #0x0000

part3_loop:

		; Read data from switches, bitwise-AND them
		; with the LEDs that should be affected by switches.
		MOVL 	ACC, 	*XAR6 	; AL = GPIO[15:0]
		AND 	ACC, 	#0x0F00 ; AND with bits 11:8 to store in LEDs
		OR 		ACC, 	#0x1000 ; OR with bits 12, which will be moved to bit 4 as constant LED
		MOV 	T, 		#8 		; number of bits to shift accumulator right by
		LSR 	AL, 	T		; shift bits 11:8 into bits 3:0

		MOVL	*XAR3, 	ACC		; temp storage

		; Read the data from buttons. Bitwise-AND them
		; with the LEDs that should be affected by the buttons.
		MOVL 	ACC, 	*XAR6 	; AH = GPIO[31:16], AL = GPIO[15:0]
		AND 	AH, 	#0x0001 ; Button 2 = Bit 16
		AND 	AL, 	#0xC000 ; Buttons 0, 1 = bit 14, 15
		MOV 	T, 		#9 		; number of bits to shift accumulator right by
		LSRL 	ACC,	T		; shift entire ACC reg by 10 bits
		XOR 	ACC, 	#0x00E0 ; Invert bits 7:5 for the buttons' LEDs

		OR 		ACC, 	*XAR3	; combine with stored data

		MOV		*XAR7, 	ACC		; dir clear
		NOT 	ACC 		 	; handle for active low leds
		MOV		*XAR4, 	ACC  	; dir set

		B part3_loop, UNC

; ===================================================================
; SUMMARY: SUB_COMP_MAX
; This subroutine compares two values and outputs the larger of the
; two values in the return register. If the input argument is greater
; than the previous max, the input argument is stored at the max_addr.
;
; The s_or_u setting in the config file determines whether this
; function does comparison for SIGNED or UNSIGNED numbers.
;
; DEFINES USED:
; s_or_u - signed or unsigned
;
; INPUTS:
; XAR6 (A) 		- address of value to test against the current max value.
; XAR2 (MAX) 	- stores a pointer to the current max value
;
; OUTPUTS:
; N/A
; ===================================================================
SUB_COMP_MAX:
	PUSH 	AL
	PUSH	AH
	PUSH 	AR0

	; ---------------------------------------------------------------
	; determine if the inputs are signed or unsigned 16-bit values
	MOV		AR0,			#s_or_u
	CMP     *AR0,        	#CMP_SIGNED
	B       j_signed,       EQ		; if configured to compare as signed numbers, jump.
	B       j_unsigned,     UNC 	; if configured to compare as unsigned numbers, jump.

j_signed: ; signed comparison ---------------------------------------
	MOV 	AL,             *XAR6       ; move input argument to ACC so comparison can be done.
	MOV 	AH,				*XAR2 	    ; loads address of test value into ACC for comparison.
	CMP     AL,      		AH			; test value - max value <= 0 means test value is NOT new max value
	B		j_exit, 		LEQ 		; less than or equal - used for signed comparison
	MOV   	*AR2,      		AL			; store the new max value to the max address if greater.
	B       j_exit,         UNC

j_unsigned: ; unsigned comparison -----------------------------------
	MOV 	AL,             *XAR6       ; move input argument to ACC so comparison can be done.
	MOV 	AH,				*XAR2 	    ; loads address of test value into ACC for comparison.
	CMP     AL,      		AH			; test value - max value <= 0 means test value is NOT new max value
	B		j_exit, 		LOS			; lower or same - used for unsigned comparison
	MOV   	*AR2,      		AL			; store the new max value to the max address if greater.
	B       j_exit,         UNC

j_exit:
	POP AR0
	POP AH
    POP AL
    LRET

; ===================================================================
; SUMMARY: DISABLE_WATCHDOG
; This subroutine compares two values and outputs the larger of the
; two values in the return register. If the input argument is greater
; than the previous max, the input argument is stored at the max_addr.
; ===================================================================
DISABLE_WATCHDOG:
		PUSH	AL
		PUSH 	XAR6
		EALLOW 					; enable protected register write
		MOV		AL,		#0x40 	; data to disable watchdog
		MOV		AR6,	#0x7029 ; Load watchdog timer address
		MOV		*XAR6,	AL
		EDIS 					; disable protected register write
		POP		XAR6
		POP 	AL
		LRET

; ===================================================================
; SUMMARY: SW_DELAY
; This subroutine delays for a variable amount of time.
; ===================================================================
SW_DELAY:
	push 	ACC
	MOV		ACC, #0xFFFF

sw_delay_loop:
	LC		sw_nested_delay
	SUB 	ACC, #1
	BF 		sw_delay_loop, GEQ

	pop 	ACC
	LRET

sw_nested_delay:
	push 	ACC
	MOV 	ACC, #100

sw_nested_delay_loop:
	SUB 	ACC, #1
	BF 		sw_nested_delay_loop, GEQ

	pop 	ACC
	LRET

; ===================================================================
; SUMMARY: EOP - End of program infinite loop.
; ===================================================================
EOP: 	B 		EOP, UNC ; branches unconditionally

