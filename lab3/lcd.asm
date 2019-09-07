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

delay .macro
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	.endm

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
; REGISTERS USED: XAR0

; INPUTS:
; REG_ADDR - ex. GPIO_PUD_A
; VAL32 - ex. 0xAAAABBBB
; ------------------------------------------------------------------
loadReg .macro REG_ADDR, VAL32
	PUSH XAR0
	MOVL XAR0,   #REG_ADDR ; points to target register
	MOV *XAR0++, #(VAL32 & 0xFFFF) ; load the low word of the reg first
	MOV *XAR0,   #(VAL32 >> 16) ; load the high word of the reg
	POP XAR0
	.endm

; ------------------------------------------------------------------
; SUMMARY: Set/Clear i2c clk (pin 105) without affecting the
; data bits.
; ------------------------------------------------------------------
clkHi .macro
	;loadReg GPIO_SET_D, 0x00000200
	EALLOW
	PUSH AL ; holds DIR data
	PUSH AR0 ; points to the GPIO_DIR_D register

	MOV AR0, #GPIO_DIR_D
	MOV AL, *AR0 ; AR0 = data in GPIO_DIR_D
	AND AL, #0x0100 ; sets bit 105 to an input without changing data
	MOV *AR0, AL

	POP AR0
	POP AL
	delay
	EDIS
	.endm

clkLo .macro
	;loadReg GPIO_CLR_D, 0x00000200
	EALLOW
	PUSH AL ; holds DIR data
	PUSH AR0 ; points to the GPIO_DIR_D register

	MOV AR0, #GPIO_DIR_D
	MOV AL, *AR0 ; AR0 = data in GPIO_DIR_D
	OR AL, #0x0200 ; sets bit 105 to an output without changing data
	MOV *AR0, AL

	POP AR0
	POP AL
	delay
	EDIS
	.endm

; ------------------------------------------------------------------
; SUMMARY: Set/Clear i2c data (pin 104)
; Clock is always low when the data switches between 0 or 1.
; ------------------------------------------------------------------
dataHi .macro
	;loadReg GPIO_SET_D, 0x00000100
	EALLOW
	PUSH AL ; holds DIR data
	PUSH AR0 ; points to the GPIO_DIR_D register

	MOV AR0, #GPIO_DIR_D
	MOV AL, *AR0 ; AR0 = data in GPIO_DIR_D
	AND AL, #0x0200 ; sets bit 105 to an input without changing clock
	MOV *AR0, AL

	POP AR0
	POP AL
	delay
	EDIS
	.endm

dataLo .macro
	;loadReg GPIO_CLR_D, 0x00000100
	EALLOW
	PUSH AL ; holds DIR data
	PUSH AR0 ; points to the GPIO_DIR_D register

	MOV AR0, #GPIO_DIR_D
	MOV AL, *AR0 ; AR0 = data in GPIO_DIR_D
	OR AL, #0x0100 ; sets bit 104 to an output without changing clock
	MOV *AR0, AL

	POP AR0
	POP AL
	delay
	EDIS
	.endm

; ------------------------------------------------------------------
; SUMMARY: Transfer a byte over I2C interface.
;
; INPUTS:
; BYTE - ex. #0xAC
; ------------------------------------------------------------------
txByte .macro BYTE
	PUSH AL
	MOV AL, BYTE
	LC I2C_TX_BYTE
	POP AL
	.endm

; ------------------------------------------------------------------
; Start and Stop for data transfer
; ------------------------------------------------------------------
start .macro
	dataLo
	clkLo
	delay
	.endm

stop .macro
	clkHi
	dataHi
	delay
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
	LC INIT_I2C_PINS ; initialize the I2C pins 104, 105

main_loop:

	LC INITIALIZE_LCD

	B main_loop, UNC

INIT_I2C_PINS:
	EALLOW
	loadReg GPIO_DIR_D,   0x00000300 ; Pins 103, 104 = outputs
	loadReg GPIO_GMUX1_D, 0x00000000 ; Pins 103, 104 = GPIO
	loadReg GPIO_MUX1_D,  0x00000000 ; Pins 103, 104 = GPIO
	loadReg GPIO_PUD_D,   0x00000000 ; Pins 103, 104 = Pull up turned on
	loadReg GPIO_DAT_D,   0x00000000 ; SDA = 0, SCL = 0
	EDIS
	LRET

; ===================================================================
; SUMMARY: I2C_TX_4BITS
; This subroutine transfers 4-bits using the I2C backpack for the LCD.
; The 4 bits are transferred out with the enable high, then again with the
; enable low.
;
; INPUTS:
; AL[7:0] - 8 bits transferred to data bus.
;
; REGISTER USAGE:
; AL[7:0] - 8 bits transferred to the data bus.
; AH - used as a shift register to test specific bits
; T - used as a shift #
; AR0 - counter for loops
; ===================================================================
I2C_TX_BYTE:
	PUSH AR0
	PUSH AH
	PUSH T
	EALLOW

	; load the 4 bit counter into XAR0
	MOV AR0, #8 ; for iterating through loop
	MOV AH, #0 ; for storing shifted AL values

i2c_tx_byte_loop:
	DEC AR0

	; check if the next bit of the 4 bit value
	; is a 1 or 0 before setting data line hi or lo
	MOV AH, AL ; AR1 = 0xDATA
	MOV T, AR0
	LSR AH, T ; AR1 = 0xDATA >> 3, 2, 1, 0
	ANDB AH, #1 ; AH = AH & 0x01 to compare only the end bit

	; If next bit is a 1, set SDA high. Otherwise, set SDA low
	CMP AH, #1
	B i2c_tx_byte_set, EQ ; next bit is a 1
	B i2c_tx_byte_clr, UNC ; next bit is a 0

i2c_tx_byte_set:
	dataHi
	B i2c_tx_byte_data_sent, UNC
i2c_tx_byte_clr:
	dataLo
i2c_tx_byte_data_sent:

	; Rising clock edge
	clkLo
	clkHi
	clkLo

	; if all 4 bits have been transferred with
	; the enable HIGH, go to the next loop.
	CMP AR0, #0
	B i2c_tx_byte_exit, EQ
	B i2c_tx_byte_loop, UNC

i2c_tx_byte_exit:

	; Receive the ack bit by releasing the pins
	; and waiting for an acknowledge to return
	dataHi
	clkHi

; while(CLK != 1)
i2c_tx_byte_wait_ack:
	MOV AR0, #GPIO_DAT_D
	MOV AL, *AR0
	AND AL, #0x0200
	CMP AL, #0x0200
	B i2c_tx_byte_wait_ack, NEQ

	clkLo
	delay
	delay
	delay

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
	start
	txByte #((lcd_addr << 1) ^ (0x00)) ; RW = 0, LCD Addr = 0x3F
	txByte #0x33 ; 0x33 => CMD, Base LCD init command
	;txByte #0x32 ; 0x32 => CMD, Base LCD init command
	;txByte #0x28 ; 0x28 => CMD, 4 bit mode, 2 line mode
	;txByte #0x0F ; 0x0F => CMD, Display on, Cursor on, Position on
	;txByte #0x01 ; 0x01 => CMD, Clear screen
	stop
	LRET

WRITE_CHAR_STRING:
	; Null ('\0') terminated string => DATA
	LRET
