;====================================================================================================
;
;    Filename:       Q_EncoderH.inc
;    Created:        12/8/2019
;    File Revision:  1.0.2d1 3/8/2020
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    Quadrature Encoder Interface.
;
; This file is mostly notes and example code.
;
; The file "Q_EncoderISR.inc" contains only the ISR portion of the Quadrature Encoder Interface.
; The file "Q_EncoderInit.inc" contains only the initialization code for the Quadrature Encoder Interface.
;   	
;
;    History:
; 1.0.2d1 3/8/2020     Updated for 'F18854
; 1.0.1d2 12/8/2019    Updated for 'F18346
; 1.0d2  09/19/2018    Working A and B
; 1.0d1  01/22/2014	First Code
;
;====================================================================================================
;=========================================================================================
; uController pinout (PIC16F18854):
;
; Pin 1 (RE3/MCLR*)		VPP/MCLR*
; Pin 2 (RA0/ANA0)		ANA0 Motor Current
; Pin 3 (RA1/ANA1)		ANA1 Motor Supply Voltage
; Pin 4 (RA2/ANA2)		ANA2 Logic Supply Voltage
; Pin 5 (RA3)		TTL Serial TX
; Pin 6 (RA4)		Hall U (Digital Input)
; Pin 7 (RA5)		Hall V (Digital Input)
; Pin 8 (Vss)		Ground
; Pin 9 (RA7)		LED1 (Active Low Input/Output)(System Button/System LED)
; Pin 10 (RA6)		Hall W (Digital Input)
; Pin 11 (RC0)		Home Switch (Active Low Input/Output)
; Pin 12 (RC1)		MagEnc_CSBit (Active Low Output)/IEnc A (Digital Input)
; Pin 13 (RC2)		MagEnc_MOSI (Digital Output)
; Pin 14 (RC3)		MagEnc_CLKBit (Digital Output)/IEnc B (Digital Input)
;
; Pin 15 (RC4)		MagEnc_MISO (Digital Input)/IEnc Z (Digital Input)
; Pin 16 (RC5)		Reverse Limit (Active Low Input/Output)
; Pin 17 (RC6)		Forward Limit (Active Low Input/Output)
; Pin 18 (RC7)		TTL Serial RX
; Pin 19 (Vss)		Ground
; Pin 20 (Vdd)		+5 Volts
; Pin 21 (RB0)		WH CCP3  Active High PWM Coil W High Side MOSFET
; Pin 22 (RB1)		WL       Active High Coil W Low Side MOSFET
; Pin 23 (RB2)		VH CCP2  Active High PWM Coil V High Side MOSFET
; Pin 24 (RB3)		VL       Active High Coil V Low Side MOSFET
; Pin 25 (RB4)		UH CCP1  Active High PWM Coil U High Side MOSFET
; Pin 26 (RB5)		UL       Active High Coil U Low Side MOSFET
; Pin 27 (RB6/ICSPCLK)		ICSPCLK
; Pin 28 (RB7/ICSPDAT)		ICSPDAT
;
;====================================================================================================
; Memory used in Bank5
;
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
	cblock	0x2A0	;<<<< copy to main.asm
;
	BitsTemp		;Copy of Q_Enc_Port data
	EdgeFlags		;Copy of IOCBF
	QEnc_OldValue
	QEnc_PrevValue
	QEnc_NextValue
	QEnc_Counter:4		;Used only by ISR
	QEnc_Position:4		;Updated by ISR if QEnc_Semaphore is clear	
;
	QEnc_Flags
	endc	
;
QEnc_Bank	EQU	5	;Same as IOC registers
;
#Define	QEnc_Z	QEnc_Flags,3	;Set by ISR when Index is seen
#Define	QEnc_Counted	QEnc_Flags,4	;Set by ISR when inc or dec counter
#Define	QEnc_Semaphore	QEnc_Flags,7	;Set to stop ISR from changing QEnc_Position
;
; PortB config
;  RB0,RB6,RB7 Inputs, Interrupt on change
;
#Define	Q_Enc_Port	PORTC
#Define	Q_Enc_Tris	TRISC
#Define	Q_Enc_Z_Bit	4
#Define	Q_Enc_A_Bit	1
#Define	Q_Enc_B_Bit	3
;
QEnc_BitMask	EQU	0xC0                   ;A and B
;
;=====================================================================================================
; Using the position data sample code
;
;	movlb	QEnc_Flags
;	BSF	QEnc_Semaphore
;	BTFSS	QEnc_Semaphore	;Wait for Semaphore
;	GOTO	$-2
; Do stuff with the position data
;
;	BCF	QEnc_Semaphore
;
;==================================================================================================
