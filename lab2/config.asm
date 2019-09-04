	.def score_addr, max_addr, score_vector_len

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                    PROGRAM CONFIGURATIONS                       |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
	.data
score_addr:
	.word 76
	.word 15
	.word 1
	.word 96
	.word 67
	.word -1 ; testing signed vs unsigned, 0xFFFF in hex
	.word 75
	.word 79
	.word 100
	.word 0

max_addr:
	.word 0x0000

score_vector_len:
	.word 10

; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; |                           TEST PROGRAM                          |
; +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+
; Example program for testing #, @, * usage
; ; --------------------------------
; MOV 	AL, #results ; AL = address (0xA000)
; MOV 	AH, @results ; AL = data (0x4000)
; ; --------------------------------
; ADD 	AL, #1 ; modify (0xA001)
; ADD 	AH, #2 ; modify (0x4002)
; MOV 	AR1, AL ; indirect addressable register
; ; --------------------------------
; MOV 	*AR1, AH ; move data 0x4002 into address 0xA000
; ; --------------------------------
