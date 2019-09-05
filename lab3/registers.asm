; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; | AUTHOR: Daniel Hamilton                                         |
; |                                                                 |
; | SUMMARY: registers.asm                                          |
; | This file contains addresses for registers used in ASM code for |
; | the LaunchpadXL                                                 |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+

	.def WDREG
	.def GPIO_CTRL_REGS, GPIO_DATA_REGS
	.def GPIO_DIR_A, GPIO_MUX1_A, GPIO_GMUX1_A, GPIO_PUD_A
	.def GPIO_DAT_A, GPIO_SET_A, GPIO_CLR_A

; watchdog timer address
WDREG .set 0x7029

; GPIO vector starting addresses
GPIO_CTRL_REGS 	.set 0x7C00
GPIO_DATA_REGS 	.set 0x7F00

; GPIO vector offset addresses
GPIO_DIR_A      .set GPIO_CTRL_REGS + 0x0A
GPIO_MUX1_A     .set GPIO_CTRL_REGS + 0x06
GPIO_GMUX1_A    .set GPIO_CTRL_REGS + 0x20
GPIO_PUD_A      .set GPIO_CTRL_REGS + 0x0C

GPIO_DAT_A      .set GPIO_DATA_REGS + 0x00
GPIO_SET_A      .set GPIO_DATA_REGS + 0x02
GPIO_CLR_A      .set GPIO_DATA_REGS + 0x04
