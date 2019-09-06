; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; | LAB #3: FPU Assembly Programming, Stack and Interfacing LCD     |
; |                                                                 |
; | AUTHOR: Daniel Hamilton                                         |
; |                                                                 |
; | SUMMARY: lcd.asm                                                |
; | This file runs Lab 3 - Part III (Add a Serial LCD).             |
; | The program sends out the following lines..                     |
; |                                                                 |
; |	                      "Daniel Hamilton"                         |
; |                       "EEL4511        "                         |
; |                                                                 |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.global _c_int00

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                           REFERENCES                            |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.ref WDREG
	.ref GPIO_CTRL_REGS, GPIO_DATA_REGS
	.ref GPIO_DIR_A, GPIO_MUX1_A, GPIO_GMUX1_A, GPIO_PUD_A
	.ref GPIO_DAT_A, GPIO_SET_A, GPIO_CLR_A
	.ref GPIO_DIR_D, GPIO_MUX1_D, GPIO_GMUX1_D, GPIO_PUD_D, GPIO_DAT_D, GPIO_SET_D, GPIO_CLR_D

lcd_addr .set 0x3F

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

; ------------------------------------------------------------------
; SUMMARY: Load a GPIO register with a 32-bit immediate value
;
; REGISTERS USED: ACC, XAR0

; INPUTS:
; REG_ADDR - #GPIO_PUD_A
; VAL32 - #0xAAAABBBB
; ------------------------------------------------------------------
loadReg .macro REG_ADDR, VAL32
	MOVL XAR6,   #REG_ADDR ; points to target register
	MOV *XAR6++, #(VAL32 & 0xFFFF) ; load the low word of the reg first
	MOV *XAR6,   #(VAL32 >> 16) ; load the high word of the reg
	.endm

; ------------------------------------------------------------------
; SUMMARY: Set/Clear i2c clk (pin 105)
; ------------------------------------------------------------------
clkHi .macro COND
	loadReg GPIO_SET_D, 0x00020000
	.endm

clkLo .macro COND
	loadReg GPIO_CLR_D, 0x00020000
	.endm

; ------------------------------------------------------------------
; SUMMARY: Set/Clear i2c data (pin 104)
; ------------------------------------------------------------------
dataHi .macro
	loadReg GPIO_SET_D, 0x00010000
	.endm

dataLo .macro
	loadReg GPIO_CLR_D, 0x00010000
	.endm

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                         DATA ALLOCATION                         |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.sect .data

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

main_loop:

	LC INIT_I2C_PINS

	; ACC = 0x0000.00CD
	MOV AL, #0xA
	LC I2C_TX_4BITS
	B main_loop, UNC

INIT_I2C_PINS:
	EALLOW
	loadReg GPIO_DIR_D,   0x00030000 ; Pins 103, 104 = outputs
	loadReg GPIO_GMUX1_D, 0x00000000 ; Pins 103, 104 = GPIO
	loadReg GPIO_MUX1_D,  0x00000000 ; Pins 103, 104 = GPIO
	loadReg GPIO_PUD_D,   0x00000000 ; Pins 103, 104 = Pull up turned on
	loadReg GPIO_DAT_D,   0x00030000 ; SDA = 1, SCL = 1
	EDIS
	LRET

; ===================================================================
; SUMMARY: I2C_TX_4BITS
; This subroutine transfers 4-bits using the I2C backpack for the LCD.
; The 4 bits are transferred out with the enable high, then again with the
; enable low.
;
; INPUTS:
; AL[3:0] - 4 bits transferred to data bus.
;
; REGISTER USAGE:
; AL[3:0] - 4 bits transferred to the data bus.
; AH - used as a shift register to test specific bits
; T - used as a shift #
; AR0 - counter for loops
; ===================================================================
I2C_TX_4BITS:
	PUSH AR0
	PUSH AH
	PUSH T
	EALLOW

	; load the 4 bit counter into XAR0
	MOV AR0, #4 ; for iterating through loop
	MOV AH, #0 ; for storing shifted AL values

i2c_tx_4bits_en_hi_loop:
	DEC AR0

	; check if the next bit of the 4 bit value
	; is a 1 or 0 before setting data line hi or lo
	MOV AH, AL ; AR1 = 0xDATA
	MOV T, AR0
	LSR AH, T ; AR1 = 0xDATA >> 3, 2, 1, 0
	ANDB AH, #1 ; AH = AH & 0x01 to compare only the end bit

	; If next bit is a 1, set SDA high. Otherwise, set SDA low
	CMP AH, #1
	B setLo_0, NEQ

	dataHi
	B dSent_0, UNC
setLo_0: dataLo
dSent_0:

	; if all 4 bits have been transferred with
	; the enable HIGH, go to the next loop.
	CMP AR0, #0
	B i2c_tx_4bits_reload, EQ
	B i2c_tx_4bits_en_hi_loop, UNC

i2c_tx_4bits_reload:
	; load the 4 bit counter into XAR0 to be used again
	; for the second loop.
	MOV AR0, #4 ; for iterating through loop
	MOV AH, #0 ; for storing shifted AL values

i2c_tx_4bits_en_lo_loop:
	DEC AR0

	; check if the next bit of the 4 bit value
	; is a 1 or 0 before setting data line hi or lo
	MOV AH, AL ; AR1 = 0xDATA
	MOV T, AR0
	LSR AH, T ; AR1 = 0xDATA >> 3, 2, 1, 0
	ANDB AH, #1 ; AH = AH & 0x01 to compare only the end bit

	; If next bit is a 1, set SDA high. Otherwise, set SDA low
	CMP AH, #1
	B setLo_1, NEQ

	dataHi
	B dSent_1, UNC
setLo_1: dataLo
dSent_1:

	; if all 4 bits have been transferred with
	; the enable LOW, exit the subroutine.
	CMP AR0, #0
	B i2c_tx_4bits_exit, EQ
	B i2c_tx_4bits_en_lo_loop, UNC

i2c_tx_4bits_exit:
	EDIS
	POP T
	POP AH
	POP AR0
	LRET

; ===================================================================
	; 100 - 400 KHz clock speed

	; Upper nibble w/ Enable HIGH
	; Upper nibble w/ Enable LOW

	; Lower nibble w/ Enable HIGH
	; Lower nibble w/ Enable LOW

	; R/~W bit always LOW

	; LED bit on, backlight stays on

	; RS => 1 when we write data
	; RS => 0 when we write commands
; ===================================================================
WRITE_COMMAND_REG:
	; RS => 0 to write commands
	LRET

WRITE_DATA_REG:
	; RS => 1 to write data
	LRET

INITIALIZE_LCD:
	; 0x33 => CMD, Base LCD init command
	; 0x32 => CMD, Base LCD init command
	; 0x28 => CMD, 4 bit mode, 2 line mode
	; 0x0F => CMD, Display on, Cursor on, Position on
	; 0x01 => CMD, Clear screen
	LRET

WRITE_CHAR_STRING:
	; Null ('\0') terminated string => DATA
	LRET

EOP B EOP, UNC ; end of program, infinite loop
