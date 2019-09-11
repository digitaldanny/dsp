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

	.ref Quiz_Values, Quiz_Values_Length, Min_Value, Max_Value

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
; |                     4 => Quiz practice                          |
; |                     5 => Real Quiz                              |
; |                                                                 |
; +-----------------------------------------------------------------+
ASSIGNMENT_PART .set 	5
; +-----------------------------------------------------------------+
; |=================================================================|
; |=================================================================|
; |=================================================================|
; +=================================================================+

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                              MACROS                             |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

loadMax .macro VAL32
	PUSH XAR0
	MOVL XAR0,   #Max_Value ; points to target register
	MOV *XAR0++, #(VAL32 & 0xFFFF) ; load the low word of the reg first
	MOV *XAR0,   #(VAL32 >> 16) ; load the high word of the reg
	POP XAR0
	.endm

loadMin .macro VAL32
	PUSH XAR0
	MOVL XAR0,   #Min_Value ; points to target register
	MOV *XAR0++, #(VAL32 & 0xFFFF) ; load the low word of the reg first
	MOV *XAR0,   #(VAL32 >> 16) ; load the high word of the reg
	POP XAR0
	.endm

; Sets active-low LEDs to "SET" value.
; Ex. SET = #0xFFFE, means LED 0 is on.
setLeds .macro SET
	PUSH 	XAR6
	MOVL 	XAR6, 	#GPIO_DAT_A
	MOV 	*XAR6, 	SET
	POP 	XAR6
	.endm

initLeds .macro
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
	.endm

debounceB0 .macro
	PUSH AL
	MOV AL, #1
	LC DEBOUNCE
	POP AL
	.endm

debounceB1 .macro
	PUSH AL
	MOV AL, #2
	LC DEBOUNCE
	POP AL
	.endm

debounceB2 .macro
	PUSH AL
	MOV AL, #3
	LC DEBOUNCE
	POP AL
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
		MOV 	SP, #0x0400
		EDIS ; disable protected register write

		LC DISABLE_WATCHDOG

		; Determine which part of the file to run
		.if ASSIGNMENT_PART == 1
			B PART1, UNC
		.else
			.if ASSIGNMENT_PART == 2
				B PART2, UNC
			.else
				.if ASSIGNMENT_PART == 3
					B PART3, UNC
				.else
					B PART5, UNC
				.endif
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

PART3: ; ***********************************************************************************

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

PART4: ; ***********************************************************************************
		EALLOW
		initLeds

		; AL - GPIO 15:0
		; AH - GPIO 31:16
		MOVL XAR0, #GPIO_DAT_A
		MOV	*XAR0, #0xFFFF ; initialize LEDs to be off
		MOVL XAR1, #GPIO_DAT_A
		INC AR1

part4_loop:

		; load in the new button values
		MOV AL, *XAR0
		MOV AH, *XAR1
		NOT AL
		NOT AH

		; check if the reset button was hit first
		MOV AR2, AL ; backup for GPIO 15:0
		AND AL, #0x8000
		CMP AL, #0x8000
		B RESET, EQ

		; check if the increment button (PB0) was hit first
		MOV AL, AR2 ; move the backup into the ACC register
		AND AL, #0x4000
		CMP AL, #0x4000
		B INCREMENT, EQ

		; check if the decrement button (PB2) was hit first
		AND AH, #0x0001
		CMP AH, #0x0001
		B DECREMENT, EQ
		B part4_loop, UNC
RESET:
		debounceB1
		MOV *XAR0, #0xFFFF ; all LEDs turn off
		B part4_exit, UNC

INCREMENT:
		debounceB0
		MOV AL, *XAR0
		NOT AL		; 0xFFFF => 0x0000
		INC AL		; 0x0000 => 0x0001
		NOT AL 		; 0x0001 => 0xFFFE
		mov *XAR0, AL
		B part4_exit, UNC

DECREMENT:
		debounceB2
		MOV AL, *XAR0
		NOT AL		; 0xFFFF => 0x0000
		DEC AL		; 0x0000 => 0x0001
		NOT AL 		; 0x0001 => 0xFFFE
		mov *XAR0, AL
		B part4_exit, UNC

part4_exit:
		B part4_loop, UNC

PART5: ; ***********************************************************************************

		EALLOW
		MOVL 	XAR0, 	#Quiz_Values_Length 	; load the vector length into this register to be decremented later
		MOVL 	XAR1, 	#Quiz_Values 	    ; score vector POINTER
		MOVL 	XAR2, 	#Max_Value 			; max value POINTER
		MOVL	XAR3, 	#Min_Value			; min value pointer

		; initialize the max/min value
		loadMax 0x00000000
		loadMin 0xFFFFFFFF

part5_min_max_loop:
		; ---------------------------------------------------------------------------------
		DEC		*XAR0				; decrement number of values remaining in the vector
		MOVL 	XAR6, 	XAR1       	; move the next score in the vector to the input argument register

		; ---------------------------------------------------------------------------------
		; load the max value into the ACC to be tested
		MOV AL, *XAR1 ; load the low word of the next test value
		INC AR1
		MOV AH, *XAR1 ; load the high word of the next test value
		INC AR1		; prepare for next iteration

CompMax:
		MOV	DP, #0x280 ; #Max_Value>>6 ; compare with the next 32-bit number
		CMPL ACC, @Max_Value ; compare the current max value with the ACC test value

		B CompMin, LOS ; if the old max value is higher or the same, skip to comparing as min
		MOV *XAR2, AL; store the new max value
		INC AR2 ; point to high word
		MOV *XAR2, AH ; store high part of new max value
		DEC AR2 ; prepare for next iteration

CompMin:
		; load the max value to be tested
		MOV	DP, #0x280 ; #Min_Value>>6
		CMPL ACC, @Min_Value ; compare the current min value with the ACC value

		B NextValue, HIS ; if the old min value is lower or the same, skip to next value
		MOV *XAR3, AL; store the new max value
		INC AR3 ; point to high word
		MOV *XAR3, AH ; store high part of new max value
		DEC AR3 ; prepare for next iteration

		B 		NextValue, UNC

	    ; ---------------------------------------------------------------------------------
NextValue:

		; if there are no more values remaining in the vector, continue to part 2.
		; ---------------------------------------------------------------------------------
		CMP		*AR0, 	#0
		B		part5_min_max_loop,	NEQ

; -----------------------------------------------------------------------------------------

		; AL - GPIO 15:0
		; AH - GPIO 31:16
		EALLOW
		initLeds
		MOVL XAR0, #GPIO_DAT_A
		MOV	*XAR0, #0xFFFF ; initialize LEDs to be off
		MOVL XAR1, #GPIO_DAT_A
		INC AR1

part5_echo_loop:

		; load in the new button values
		MOV AL, *XAR0
		MOV AH, *XAR1
		NOT AL
		NOT AH

		; check if the middle button was hit first
		MOV AR2, AL ; backup for GPIO 15:0
		AND AL, #0x8000
		CMP AL, #0x8000
		B MIDDLE, EQ

		; check if the right button (PB0) was hit first
		MOV AL, AR2 ; move the backup into the ACC register
		AND AL, #0x4000
		CMP AL, #0x4000
		B RIGHT, EQ

		; check if the left button (PB2) was hit first
		AND AH, #0x0001
		CMP AH, #0x0001
		B LEFT, EQ
		B part5_echo_loop, UNC

MIDDLE: ; echo switches on both sides
		PUSH AR2
		PUSH AL
		PUSH AH

		; Read data from switches, bitwise-AND them
		; with the LEDs that should be affected by switches.
		MOV 	AL, 	AR2 	; AL = GPIO[15:0]
		AND 	AL, 	#0x0F00 ; AND with bits 11:8 to store in LEDs
		MOV 	T, 		#8 		; number of bits to shift accumulator right by
		LSR 	AL, 	T		; shift bits 11:8 into bits 3:0

		MOV 	AH, 	AR2 	; AL = GPIO[15:0]
		AND 	AH, 	#0x0F00 ; AND with bits 11:8 to store in LEDs
		MOV 	T, 		#4 		; number of bits to shift accumulator right by
		LSR 	AH, 	T		; shift bits 11:8 into bits 7:4

		OR		AL, AH			; combine for GPIO 7:0
		MOV 	*XAR0, AL ; push the shifted values into LEDs

		POP AH
		POP AL
		POP AR2
		B part5_exit, UNC

RIGHT: ; echo switches on GPIO 3:0
		PUSH AR2
		PUSH AL
		PUSH AH

		; Read data from switches, bitwise-AND them
		; with the LEDs that should be affected by switches.
		MOV 	AL, 	AR2 	; AL = GPIO[15:0]
		AND 	AL, 	#0x0F00 ; AND with bits 11:8 to store in LEDs
		MOV 	T, 		#8 		; number of bits to shift accumulator right by
		LSR 	AL, 	T		; shift bits 11:8 into bits 3:0
		OR		AL, 	#0x00F0 ; clear bits 7:4

		OR		AL, AH			; combine for GPIO 7:0
		MOV 	*XAR0, AL ; push the shifted values into LEDs

		POP AH
		POP AL
		POP AR2
		B part5_exit, UNC

LEFT: ; echo switches on GPIO 7:4
		PUSH AR2
		PUSH AL
		PUSH AH

		; Read data from switches, bitwise-AND them
		; with the LEDs that should be affected by switches.
		MOV 	AL, 	AR2 	; AL = GPIO[15:0]
		AND 	AL, 	#0x0F00 ; AND with bits 11:8 to store in LEDs
		MOV 	T, 		#4 		; number of bits to shift accumulator right by
		LSR 	AL, 	T		; shift bits 11:8 into bits 7:4
		OR		AL, 	#0x000F ; clear bits 3:0

		OR		AL, AH			; combine for GPIO 7:0
		MOV 	*XAR0, AL ; push the shifted values into LEDs

		POP AH
		POP AL
		POP AR2
		B part5_exit, UNC

part5_exit:
		B part5_echo_loop, UNC
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
; SUMMARY: DEBOUNCE - debounces selected push button.
; INPUTS: AL - #1:PB0, #2:PB1, #3:PB2
; ===================================================================
DEBOUNCE:
	PUSH AL
	PUSH AR0
	MOV AR0, #GPIO_DAT_A ; points at the button/switch register

	CMP AL, #1 			; debounce button 0
	B debounce_b0, EQ
	CMP AL, #2 			; debounce button 1
	B debounce_b1, EQ
	B debounce_b2, UNC 	; debounce button 2

debounce_b0:
	MOV AL, *AR0 ; loads AR0 <= GPIO 15:0
	LC SDELAY ; wait for button to finish bouncing
	CMP AL, #0x4000 ; check if the button has gone high yet
	B debounce_b0, EQ
	B debounce_exit, UNC

debounce_b1:
	MOV AL, *AR0 ; loads AR0 <= GPIO 15:0
	LC SDELAY ; wait for button to finish bouncing
	CMP AL, #0x8000 ; check if the button has gone high yet
	B debounce_b1, EQ
	B debounce_exit, UNC

debounce_b2:
	INC AR0 ; points at the upper word of the button/switch register
	MOV AL, *AR0 ; loads AR0 <= GPIO 15:0
	LC SDELAY ; wait for button to finish bouncing
	CMP AL, #0x0001 ; check if the button has gone high yet
	B debounce_b2, EQ
	B debounce_exit, UNC

debounce_exit:
	POP AR0
	POP AL
	LRET

; ===================================================================
; SUMMARY: SDELAY - short delay used for debouncing.
; ===================================================================
SDELAY:
	push 	AL
	MOV		AL, #0xFFFF
sdelay_loop:
	SUB 	AL, #1
	BF 		sdelay_loop, GEQ
	pop 	AL
	LRET

; ===================================================================
; SUMMARY: EOP - End of program infinite loop.
; ===================================================================
EOP: 	B 		EOP, UNC ; branches unconditionally

