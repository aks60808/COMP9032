
; project-para
; written z5219960@unsw.edu.au   Heng-Chuan Lin
; Board settings: 
; 1.  LCD data pins D0-D7 to PORTF0-7.
; 2.  LCD control pins BE-RS to PORTA4-7.
; 3.  LED BAR pins to PL3,4 PE3,5
; 4.  KEYPAD R0-3 to PORTC7-4 , C0-3 to PORTC4-0 
  
.include "m2560def.inc"
;; keypad 
.def row			=r16		; current row number
.def col			=r17		; current column number
.def rmask			=r18		; mask for current row
.def cmask			=r19		; mask for current column
.def temp1			=r20		
.def temp2			=r21
.def LCD_reg		=r22
.def DDRMSK			=r23

.equ PORTCDIR = 0b00010000			; use PortC for input/output from keypad: PC7-4, output, PC3-0, input
.equ INITCOLMASK = 0xEF		; scan from the leftmost column, the value to mask output
.equ INITROWMASK = 0x01		; scan from the bottom row
.equ ROWMASK  =0x0F			; low four bits are output from the keypad. This value mask the high 4 bits.
.equ LCD_RS = 7				; set bits for each LCD control position
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

;;;;;;;;;  MACRO SECTION ;;;;;;;;;;

.macro OpaLV_Local	
		
	; description: Individual adjustment of each window
	; argument: @0 addr. of that Window , @1 up or down , @2 Lock

	ldi YH,high(@2)			; fetch the variable ( Lock_L )
	ldi YL,low(@2)
	ld r24,Y
	cpi r24,0				; check if it's locked
	breq notlocked
	rjmp end
notlocked:
	OpaLV_set @0,@1			; operation up/down
	end:
.endmacro

.macro OpaLV_set	
		
	; descirption: operation up/down
	; argument: @0 addr. of that Window , @1 up or down 

	ldi YH,high(@0)				; load the addr of window to addr pointer
	ldi YL,low(@0)
	ld r24,Y+					; reg <- (Y)
	ld r25,Y
	sbci r24,@1					; up -(-1) or down -1 
	cpi r24,0
	brsh OpaLV_set_sub1			; OpaLv cant be < 0
	rjmp end
OpaLV_set_sub1:
	cpi r24,4					; OpaLv cant be >3
	brlo OpaLV_set_sub2
	rjmp end
OpaLV_set_sub2:
	window 'L',' '				; refresh the display of State to LOCAL
	st Y,r25					; store the current OpaLV value to Data Memory 
	st -Y,r24

end:
.endmacro

.macro OpaLV_Central_Clear
	
	; description: set all window clear

	window 'C',' '			; change the state to Central
	Clear W1				; Turn each window Clear
	Clear W2
	Clear W3
	Clear W4

.endmacro

.macro OpaLV_Central_Dark
		
	; description: set all window clear

	window 'C',' '			; change the state to Central
	Dark W1				; Turn each window dark
	Dark W2
	Dark W3
	Dark W4
.endmacro

.macro Led_check	
		
	; descirption: Central Control Clear or Dark 
	; This function would change the PWM duty cycle of that OCXX
	; argument: @0 value of OpaLv , @1 OCRXXL, @2 temp reg  
 
	cpi @0,0		; value inspection
	brne led_br0
	rjmp mode0
led_br0:
	cpi @0,1
	brne led_br1
	rjmp mode1
led_br1:
	cpi @0,2
	brne mode3
	rjmp mode2
mode3:				; Dark mode (0x00)
	clr @2
	sts @1,@2		; store this value to the mem-mapped port(output)
	rjmp led_end
mode0:				; Clear mode (0xFF)
	ser @2
	sts @1,@2		; store this value to the mem-mapped port(output)
	rjmp led_end
mode1:
	ldi @2,0x3A		; Less Clear mode (0x3A)
	sts @1,@2		; store this value to the mem-mapped port(output)
	rjmp led_end
mode2:				; faint mode (0x1A)
	ldi @2,0x1A
	sts @1,@2		; store this value to the mem-mapped port(output)
led_end:
	
.endmacro


.macro window
		
	; descirption: display on LED 
	; argument: @0 status , @1 additonal char
 
	do_lcd_command 0b0010000000		; target on specific position
	do_lcd_data_const @0			; display the data
	do_lcd_command 0b0010000001
	do_lcd_data_const @1
	do_lcd_command 0b0010000010
	do_lcd_data_const 'W'
	do_lcd_command 0b0010000011
	do_lcd_data_const '1'
	do_lcd_command 0b0010000101
	do_lcd_data_const 'W'
	do_lcd_command 0b0010000110
	do_lcd_data_const '2'
	do_lcd_command 0b0010001000
	do_lcd_data_const 'W'
	do_lcd_command 0b0010001001
	do_lcd_data_const '3'
	do_lcd_command 0b0010001011
	do_lcd_data_const 'W'
	do_lcd_command 0b0010001100
	do_lcd_data_const '4'
.endmacro


.macro D2ASCII
		
	; descirption: convert digit to ASCII
	; argument: @0 resgister
 		
	subi @0, -48	; convert the digit in reg to ASCII word
	do_lcd_data @0	; print it out on LCD
.endmacro
	
.macro Dark
	ldi YL,low(@0)	; load address into addr pointer
	ldi YH,high(@0)
	ldi r22,3		; clean the value
	st Y+,r22		; store it back
	st Y,r22		; store it back
.endmacro

.macro Set1		;clear the value in that word address
	ldi YL,low(@0)	; load address into addr pointer
	ldi YH,high(@0)
	ldi r22,1		; clean the value
	st Y+,r22		; store it back
	st Y,r22		; store it back
.endmacro

.macro Clear		;clear the value in that word address
	ldi YL,low(@0)	; load address into addr pointer
	ldi YH,high(@0)
	clr r22		; clean the value
	st Y+,r22		; store it back
	st Y,r22		; store it back
.endmacro

;;;;;;;;;;;;; These following macros are mostly from COMP9032 Course website
 
.macro lcd_set				; set operation, ref: sample code
	sbi PORTA, @0
.endmacro

.macro lcd_clr				; clr operation, ref: sample code
	cbi PORTA, @0
.endmacro

.macro do_lcd_command		; write the command to MCU of LCD, ref: sample code
	ldi LCD_reg, @0		
	call lcd_command
	call lcd_wait		
.endmacro

.macro do_lcd_data			; write the data to be displayed on LCD , modified based on sample code
	mov LCD_reg, @0			; modify to take value from register
	call lcd_data
	call lcd_wait
.endmacro

.macro do_lcd_data_const	; write the data to be displayed on LCD, ref: sample code
	ldi LCD_reg, @0			; take imidiate value to register
	call lcd_data
	call lcd_wait
.endmacro

.dseg
.org 0x0200 
	W1: .byte 1				; OpaLv of Windows
	W2:	.byte 1
	W3: .byte 1
	W4: .byte 1	
	Lock_L: .byte 1			; Lock for lock/unlock individual operation

.cseg
.org 0x0000						; define where is start addr of program address
	jmp RESET		
.org INT0addr					; setup the Emergency trigger  (higher priority)
	jmp EXT_INT0





RESET:
;;	initiation of PWM duty cycle for LED

	ldi temp1, 0b00011000		; enable PL3(5A) and PL4(5B)
	sts DDRL, temp1
	ldi temp1,0b00101000		; enable PE3(3C) and PE5(3A)
	out DDRE,temp1

	clr temp1					; setup the value fo controling the PWM duty cycle
	sts OCR3AH, temp1			
	sts OCR3CH, temp1
	sts OCR5AH, temp1
	sts OCR5BH, temp1

	ldi temp1, 0xFF				; initial OpaLv: clear
	sts OCR5AL, temp1			; W1
	sts OCR5BL, temp1			; W2
	sts OCR3CL, temp1			; W3
	sts OCR3AL, temp1			; W4

	; setup timer3,5 to Phase Correct PWM

	ldi temp1, (1 << CS50)		;setup timer clock frequency
	sts TCCR5B, temp1
	sts TCCR3B, temp1
	ldi temp1, (1<< WGM50)|(1<<COM5A1)|(1<<COM5B1)  ; set each phase correct and control pins for bot timer3 and timer5
	sts TCCR5A, temp1 
	ldi temp1, (1<< WGM30)|(1<<COM3C1)|(1<<COM3A1)
	sts TCCR3A, temp1 

;;initial the windows

	Clear W4					
	Clear W1
	Clear W2
	Clear W3
	
;; keyPAd SECTION

	ldi temp1, PORTCDIR			; columns are outputs, rows are inputs
	out	DDRC, temp1

;; LCD initiation   most of the code in this section are from COMP9032 course website

	ldi LCD_reg, low(RAMEND)	; setup a stack
	out SPL, LCD_reg
	ldi LCD_reg, high(RAMEND)
	out SPH, LCD_reg
	ser LCD_reg
	out DDRF, LCD_reg			; set PORTF (LCD) , A (LCD) as output
	out DDRA, LCD_reg
	out PORTB, LCD_reg			; pull-up PORTB ( for trigger EXT_INT1) 
	clr LCD_reg
	out DDRB, LCD_reg			; set PORTB as output ( for trigger EXT_INT1) 
	out PORTF, LCD_reg			; initiation of each port
	out PORTA, LCD_reg			
	do_lcd_command 0b00111000 ; 2x5x7
	call sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	call sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink
	; the initiation of LCD is completed.


;; Interrupt	

	ldi temp1,(2<<ISC00) ; set INT0(emergency) as falling edge
	sts EICRA,temp1					
	in temp1,EIMSK					;enable INT0 
	ori temp1,(1<<INT0)
	out EIMSK,temp1
	sei								; enable Global interrupt
	call unlock				; initial the Lock_L
	window 'S',' '			;intial Status of windows
	call OpaLV_show			; display the OpaLV on LCD
	call LED_show			; display the OpaLV on LED
	jmp main


EXT_INT0:					; for emergency status -> all winows should be clear
	push temp1				; push conflict register
	in temp1,SREG			; record the status Reg
	push temp1
	push temp2
	window '!','!'			; change status on LCD
	Clear W4				; CLear all the windows  OPaLv -> 0
	Clear W1
	Clear W2
	Clear W3
	call Locker				; Lock the local adjustment
	call OpaLV_show			; refresh the OpaLv on both LCD and LED
	call Led_show
	pop temp2				; pop
	pop temp1
	out SREG,temp1
	pop temp1
	reti


main:
	;; backbone of keypad scanning is from COMP9032 course website 

	ldi DDRMSK, 0b00010000		; load the DDR mask for parallel input
	out	DDRC, DDRMSK
	ldi cmask, INITCOLMASK		; initial column mask
	clr	col						; initial column
colloop:
	cpi col, 4					; scaning the col
	brne colloop1
	jmp endofOP					; if all cols are scanned -> go to endofOP
colloop1:
	out	PORTC, cmask			; set column to mask value (one column off)
	ldi temp1, 0xFF
delay:
	dec temp1			
	brne delay
	in	temp1, PINC				; read the input
	andi temp1, ROWMASK	
	cpi temp1, 0xF				; check if any rows are on
	brne currentcol
	jmp nextcol								
currentcol:
	clr	row						; initial row
rowloop:						; now no more rmsk, read each pin direclty
	cpi row,4
	brne rowloop_sub1
	jmp nextcol
rowloop_sub1:					; identify which row it is 
	Cpi row,0
	brne row1_3 
	rjmp row0
row1_3:
	Cpi row,1
	brne row2_3 
	rjmp row1
row2_3:
	Cpi row,2
	brne row3 
	jmp row2
row3:
	inc row						; row++ 	
	sbrc temp1,3				; if bit is still set -> go to nextrow
	jmp rowloop
	cpi col,0
	brne r3_sub1
	rjmp r3c0
r3_sub1:
	cpi col,1
	brne r3_sub2
	rjmp r3c1
r3_sub2:
	cpi col,2
	brne r3c3
	rjmp r3c2
r3c3:							; keyPAd 'D'
	 OpaLV_Central_Dark			; Central OPeration Clear
	 call Locker	 			; Lock the local function					
	jmp rowloop					; check next row
r3c0:							; KeyPad '*'
	call Locker					; Lock the local function 									
	jmp rowloop
r3c1:							; keyPad '0'
	call unlock					; Release unlock	 										
	jmp rowloop
r3c2:							; keypad '#'							
	jmp rowloop
row0:
	inc row
	sbrc temp1,0
	jmp rowloop
	cpi col,0
	brne r0_sub1
	rjmp r0c0
r0_sub1:
	cpi col,1
	brne r0_sub2
	rjmp r0c1
r0_sub2:
	cpi col,2
	brne r0c3
	rjmp r0c2
r0c3:
	OpaLV_Local W4,-1,Lock_L	; keypad 'A' Operation: W4_UP 									
	jmp rowloop
r0c0:
	OpaLV_Local W1,-1,Lock_L	; keypad '1' Operation: W1_UP  									
	jmp rowloop
r0c1:
	OpaLV_Local W2,-1,Lock_L	; keypad '2' Operation: W2_UP  										
	jmp rowloop
r0c2:
	OpaLV_Local W3,-1,Lock_L	; keypad '3' Operation: W3_UP 	 									
	jmp rowloop

row1:
	inc row
	sbrc temp1,1
	jmp rowloop
	cpi col,0
	brne r1_sub1
	rjmp r1c0
r1_sub1:
	cpi col,1
	brne r1_sub2
	rjmp r1c1
r1_sub2:
	cpi col,2
	brne r1c3
	rjmp r1c2
r1c3:
	OpaLV_Local W4,1,Lock_L		; keypad 'B' Operation: W4_down									
	jmp rowloop
r1c0:
	OpaLV_Local W1,1,Lock_L		; keypad '4' Operation: W1_down	 									
	jmp rowloop
r1c1:
	OpaLV_Local W2,1,Lock_L		; keypad '5' Operation: W2_down	 										
	jmp rowloop
r1c2:
	OpaLV_Local W3,1,Lock_L		; keypad '6' Operation: W3_down									
	jmp rowloop
row2:
	inc row
	sbrc temp1,2
	jmp rowloop
	cpi col,0
	brne r2_sub1
	rjmp r2c0
r2_sub1:
	cpi col,1
	brne r2_sub2
	rjmp r2c1
r2_sub2:
	cpi col,2
	brne r2c3
	rjmp r2c2
r2c3:
	OpaLV_Central_Clear			; keypad 'C', Central_Operation Dark
	 call Locker								
	jmp rowloop
r2c0:							; keypad '7'								
	jmp rowloop
r2c1:							; keypad '8' 										
	jmp rowloop
r2c2:							; keypad '9'									
	jmp rowloop
	
nextcol:
	lsl DDRMSK					; left shift the DDRMSK and load to DDRC
	out	DDRC, DDRMSK
	lsl cmask					; else get new mask by shifting and 
	inc col						; increment column value
	jmp colloop					; and check the next column
endofOP:
	call OpaLV_show				; display the value on LCD
	call LED_show				; dispay the value on LED
	call delayalot
	jmp main

;;; function

delayalot:		; set up 0.5 second delay
	push r21
	ldi r21,100		; temp2 <- 50    100* 5ms =~ 0.5s
	loop:
		dec r21
		breq end
		call sleep_5ms
		rjmp loop
	end: 
	pop r21
	ret

OpaLV_show:
	
	; description: read Opalv of each window and convert it to LCD

	push YH						; push conflict register
	push YL
	push r24
	push r25
	ldi YH,high(W1)				; load the addr to addr pointer
	ldi YL,low(W1)
	ld r24,Y					; load the (Y) to reg
	do_lcd_command 0b0011000010	; set the right position on 
	D2ASCII r24					; convert the digit to ASCII and display it
	
	ldi YH,high(W2)	
	ldi YL,low(W2)
	ld r24,Y					
	do_lcd_command 0b0011000101
	D2ASCII r24
	
	ldi YH,high(W3)	
	ldi YL,low(W3)
	ld r24,Y					
	do_lcd_command 0b0011001000
	D2ASCII r24
	
	ldi YH,high(W4)	
	ldi YL,low(W4)
	ld r24,Y					
	do_lcd_command 0b0011001011
	D2ASCII r24

	pop r25
	pop r24
	pop YL
	pop YH
	ret


LED_show:
	
	; description: read Opalv of each window and convert it to LED

	push YH
	push YL
	push r24
	push r25
	push temp1	
	ldi YH,high(W1)				; load the addr to addr pointer
	ldi YL,low(W1)
	ld r24,Y					; load the (Y) to reg
	LED_check r24,OCR5AL,temp1	; renew the PWM duty cycle of Opalv for each window
	ldi YH,high(W2)	
	ldi YL,low(W2)
	ld r24,Y					
	LED_check r24,OCR5BL,temp1
	ldi YH,high(W3)
	ldi YL,low(W3)
	ld r24,Y					
	LED_check r24,OCR3CL,temp1
	ldi YH,high(W4)	
	ldi YL,low(W4)
	ld r24,Y					
	LED_check r24,OCR3AL,temp1			
	pop temp1
	pop r25
	pop r24
	pop YL
	pop YH
	ret

Locker:
	
	; description: setup the locker

	push YL
	push YH
	push r24		
	do_lcd_command 0b0011001110
	do_lcd_data_const 'L'
	do_lcd_data_const 'K'
	ldi YL,low(Lock_L)	; load address into addr pointer
	ldi YH,high(Lock_L)
	ldi r24,1		; clean the value
	st Y+,r24		; store it back
	st Y,r24		; store it back
	pop r24
	pop YH
	pop YL
	ret
Unlock:
	
	; description: release locker

	push YL
	push YH
	push r24		
	do_lcd_command 0b0011001110
	do_lcd_data_const 'U'
	do_lcd_data_const 'L'
	ldi YL,low(Lock_L)	; load address into addr pointer
	ldi YH,high(Lock_L)
	ldi r24,0		; clean the value
	st Y+,r24		; store it back
	st Y,r24		; store it back
	pop r24
	pop YH
	pop YL
	ret

; the code below is provided from sample code
; Send a command to the LCD (r22)
;

lcd_command:
	out PORTF, LCD_reg
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	ret
lcd_data:
	out PORTF, LCD_reg
	lcd_set LCD_RS
	nop
	nop
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	lcd_clr LCD_RS
	ret
lcd_wait:
	push LCD_reg
	clr LCD_reg
	out DDRF, LCD_reg
	out PORTF, LCD_reg
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E
	nop
	nop
    nop
	in LCD_reg, PINF
	lcd_clr LCD_E
	sbrc LCD_reg, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser LCD_reg
	out DDRF, LCD_reg
	pop LCD_reg
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret





