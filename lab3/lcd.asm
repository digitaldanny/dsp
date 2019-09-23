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
	LC SW_DELAY
	;LC SW_DELAY
	;LC SW_DELAY
	;LC SW_DELAY
	;LC SW_DELAY
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
; Start and Stop for data transfer
; ------------------------------------------------------------------
start .macro
	dataLo
	clkLo
	.endm

stop .macro
	clkHi
	dataHi
	.endm

; ------------------------------------------------------------------
; SUMMARY: Transfer a byte over I2C interface without knowledge
; of 4-bit transfer mode. This is an immediate load.
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
; SUMMARY: Transfer a byte over I2C interface to the LCD WITH knowledge
; of 4-bit transfer mode. This is an immediate load.
;
; INPUTS:
; BYTE - ex. #0xAC
; ------------------------------------------------------------------
txByteLCD .macro BYTE
	PUSH AL
	MOV AL, BYTE
	LC LCD_TX_BYTE
	POP AL
	.endm

; ------------------------------------------------------------------
; SUMMARY: Transfer a byte over I2C interface WITH knowledge
; of 4-bit transfer mode. This is an immediate load.
;
; INPUTS:
; CHAR/CMD - #'A'
; ------------------------------------------------------------------
wrDataImm .macro CHAR
	PUSH AL
	MOV AL, CHAR
	LC WRITE_DATA_REG
	POP AL
	.endm

wrCmdImm .macro CMD
	PUSH AL
	MOV AL, CMD
	LC WRITE_COMMAND_REG
	POP AL
	.endm

; ------------------------------------------------------------------
; SUMMARY: Store the string into memory, then read the memory to
; output characters to LCD.
;
; INPUTS:
; STRING - "Hello world"
;
; OUTPUTS:
; AR0 - points to beginning of the string
; ------------------------------------------------------------------
writeString .macro STRING

	.endm

lcdRow1 .macro
	MOV AL, #(0x80 ^ 0x00)	; row 2 offset = 0x40, row 1 offset = 0x00
	LC WRITE_COMMAND_REG 	; go to home line
	.endm

lcdRow2 .macro
	MOV AL, #(0x80 ^ 0x40)	; row 2 offset = 0x40, row 1 offset = 0x00
	LC WRITE_COMMAND_REG 	; go to home line
	.endm

lcdCursorBlinkOff .macro
	MOV AL, #0x0E
	LC WRITE_COMMAND_REG
	.endm

lcdCursorBlinkOn .macro
	MOV AL, #0x0F
	LC WRITE_COMMAND_REG
	.endm

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                         DATA ALLOCATION                         |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.sect .data

stringA:
	.word "Daniel Hamilton"
	.word '\0'
stringB:
	.word "EEL4511"
	.word '\0'

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
	LC INITIALIZE_LCD ; set to 4 bit mode, init screen
	;lcdCursorBlinkOff

	; Writes out "Daniel Hamilton" on the first line
	MOV AL, #stringA
	LC WRITE_CHAR_STRING

	lcdRow2 ; sets cursor to second row of the lcd

	; Writes out "EEL4511" on the second line
	MOV AL, #stringB
	LC WRITE_CHAR_STRING

	;writeString "Hamilton"

EOP	B EOP, UNC

; ===================================================================
; SUMMARY: INIT_I2C_PINS
; This subroutine initializes pins 104 and 105 to work for bit-banging
; I2C for the LCD.
; ===================================================================
INIT_I2C_PINS:
	EALLOW
	loadReg GPIO_DIR_D,   0x00000300 ; Pins 103, 104 = outputs
	loadReg GPIO_GMUX1_D, 0x00000000 ; Pins 103, 104 = GPIO
	loadReg GPIO_MUX1_D,  0x00000000 ; Pins 103, 104 = GPIO
	loadReg GPIO_PUD_D,   0x00000000 ; Pins 103, 104 = Pull up turned on
	loadReg GPIO_DAT_D,   0x00000000 ; SDA = 0, SCL = 0
	EDIS
	stop
	LRET

; ===================================================================
; SUMMARY: I2C_TX_BYTE
; This subroutine transfers 8-bits using the I2C backpack. This subroutine
; works WITHOUT knowledge of 4-bit mode, enable, RS, or RW.
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

	dataHi ; release to see if acknowledge is received

	delay ; delays waiting for the I2C device to process
	delay
	delay
	delay

	clkLo ; clock in the acknowledge
	clkHi
	clkLo

	; required delay between transfers
	delay
	delay
	delay
	delay

	EDIS
	POP T
	POP AH
	POP AR0
	LRET

; ===================================================================
; SUMMARY: WRITE_COMMAND_REG
; Write a byte to the command registers (RS = 0).
;
; REGISTERS AFFECTED:
; AL - Input byte
; ===================================================================
WRITE_COMMAND_REG:
	PUSH AR0
	MOV AR0, AL ; copy original byte for later use
	start
	txByte #((lcd_addr << 1) ^ (0x00)) ; send i2c address - RW = 0, LCD Addr = 0x3F

	; Upper nibble transfer
	; ---------------------------------------------------------------

	; Enable high
	AND AL, #0xF0 	; 0xAB & 0xF0 = 0xA0, makes sure that theres only 4 bits transferring
	OR AL, #0x0C	; 0xAC = upper nibble tx, RS = 0, back light on, enable HIGH, RW low
	LC I2C_TX_BYTE

	; Enable low
	MOV AL, AR0
	AND AL, #0xF0 	; 0xAB & 0xF0 = 0xA0, makes sure that theres only 4 bits transferring
	OR AL, #0x08	; 0xB8 = upper nibble tx, RS = 0, back light on, enable LOW, RW low
	LC I2C_TX_BYTE

	; Lower nibble transfer
	; ---------------------------------------------------------------

	; Enable high
	MOV AL, AR0
	LSL AL, #4		; 0xAB << 4 = 0xB0, shifts lower nibble into D7-D4 pins
	AND AL, #0xF0 	; 0xB0 & 0xF0 = 0xB0, makes sure that theres only 4 bits transferring
	OR AL, #0x0C	; 0xB8 = upper nibble tx, RS = 0, back light on, enable HIGH, RW low
	LC I2C_TX_BYTE

	; Enable low
	MOV AL, AR0
	LSL AL, #4		; 0xAB << 4 = 0xB0, shifts lower nibble into D7-D4 pins
	AND AL, #0xF0 	; 0xB0 & 0xF0 = 0xB0, makes sure that theres only 4 bits transferring
	OR AL, #0x08	; 0xB8 = upper nibble tx, RS = 0, back light on, enable LOW, RW low
	LC I2C_TX_BYTE

	stop
	POP AR0
	LRET

; ===================================================================
; SUMMARY: WRITE_DATA_REG
; Write a byte to the data registers (RS = 1).
;
; REGISTERS AFFECTED:
; AL - Input byte
; ===================================================================
WRITE_DATA_REG:
	PUSH AR0
	MOV AR0, AL ; copy original byte for later use
	start
	txByte #((lcd_addr << 1) ^ (0x00)) ; send i2c address - RW = 0, LCD Addr = 0x3F

	; Upper nibble transfer
	; ---------------------------------------------------------------

	; Enable high
	AND AL, #0xF0 	; 0xAB & 0xF0 = 0xA0, makes sure that theres only 4 bits transferring
	OR AL, #0x0D	; 0xAC = upper nibble tx, RS = 1, back light on, enable HIGH, RW low
	LC I2C_TX_BYTE

	; Enable low
	MOV AL, AR0
	AND AL, #0xF0 	; 0xAB & 0xF0 = 0xA0, makes sure that theres only 4 bits transferring
	OR AL, #0x09	; 0xB8 = upper nibble tx, RS = 1, back light on, enable LOW, RW low
	LC I2C_TX_BYTE

	; Lower nibble transfer
	; ---------------------------------------------------------------

	; Enable high
	MOV AL, AR0
	LSL AL, #4		; 0xAB << 4 = 0xB0, shifts lower nibble into D7-D4 pins
	AND AL, #0xF0 	; 0xB0 & 0xF0 = 0xB0, makes sure that theres only 4 bits transferring
	OR AL, #0x0D	; 0xB8 = upper nibble tx, back light on, enable HIGH, RW low
	LC I2C_TX_BYTE

	; Enable low
	MOV AL, AR0
	LSL AL, #4		; 0xAB << 4 = 0xB0, shifts lower nibble into D7-D4 pins
	AND AL, #0xF0 	; 0xB0 & 0xF0 = 0xB0, makes sure that theres only 4 bits transferring
	OR AL, #0x09	; 0xB8 = upper nibble tx, back light on, enable LOW, RW low
	LC I2C_TX_BYTE

	stop
	POP AR0
	LRET

; ===================================================================
; SUMMARY: WRITE_DATA_REG
; Sets LCD in 4-bit, 2 line mode.. clears screen, turns display and
; cursor on.
; ===================================================================
INITIALIZE_LCD:
	delay
	delay
	delay
	delay
	delay
	delay

	start
	txByte #((lcd_addr << 1) ^ (0x00)) ; RW = 0, LCD Addr = 0x3F
	wrCmdImm #0x33 ; 0x33 => CMD, Base LCD init command
	wrCmdImm #0x32 ; 0x32 => CMD, Base LCD init command
	wrCmdImm #0x28 ; 0x28 => CMD, 4 bit mode, 2 line mode
	wrCmdImm #0x0F ; 0x0F => CMD, Display on, Cursor on, Position on
	wrCmdImm #0x01 ; 0x01 => CMD, Clear screen
	stop

	delay
	delay
	delay
	delay
	delay
	delay
	LRET

; ===================================================================
; SUMMARY: WRITE_DATA_REG
; Writes NULL ('\0') terminated string from memory out to the LCD.
;
; INPUTS: AL - pointer to beginning of the string (not used as a pointer
; 	throughout subroutine).
; ===================================================================
WRITE_CHAR_STRING:
	PUSH AR0
	MOV AR0, AL	; move address into a pointer register

write_char_string_loop:
	; Exit if NULL character found. Otherwise, output the next
	; character.
	MOV AL, *AR0	; load the next character from the string
	CMP AL, #'\0'
	B write_char_string_exit, EQ

	INC AR0			; point to the next character
	LC WRITE_DATA_REG ; send character to LCD
	B write_char_string_loop, UNC

write_char_string_exit:
	POP AR0
	LRET

; ===================================================================
; SUMMARY: SW_DELAY
; This subroutine delays for a large # of clock cycles.
; ===================================================================
SW_DELAY:
	push 	AL
	MOV		AL, #0x3FF

sw_delay_loop:
	SUB 	AL, #1
	BF 		sw_delay_loop, GEQ

	pop 	AL
	LRET
