;=========================================================================================
;
;   Filename:	SMRRC_Output.asm
;   Created:	7/25/2026
;   File Revision:	0.9.1d   7/25/2026
;   Project:	Serial Model Railroad Control
;   Author:	David M. Flynn
;   Company:	DMF-Enterprises
;   E-Mail:	dflynn.oxfordvue@gmail.com
;   License:	MIT License
;
;=========================================================================================
;   Serial Model Railroad Control:
;
; RS485 Node. Sits on the RS485 bus and does stuff for the Railroad.
; This version is for the Lighting Daughter Board.
;
;    History:
; 0.9.1d   7/25/2026   Copied from SMRRC.asm 0.9.3a
;
;=========================================================================================
; Options:
;
;=========================================================================================
;=========================================================================================
; What happens next:
;   At power up the system LED will blink.
;
; When a command packet is received from RX1 (RS485):
;  Source=any, Destination=Me: Parse and Process the command. 
;     Send any requested data back to the sender on TX1 (RS485).
;
;=========================================================================================
; uController pinout (PIC16F15345):
;
; Pin 1  (Vdd)		+5 Volts
; Pin 2  (RA5)		LED2 (Active Low Input/Output)(LED2/Button2)
; Pin 3  (RA4)		LED1 (Active Low Input/Output)(LED1/Button1)
; Pin 4  (RA3/MCLR*)		VPP/MCLR*
; Pin 5  (RC5)		I/O    Daughter Card Pin 10
; Pin 6  (RC4)		I/O    Daughter Card Pin 9
; Pin 7  (RC3)		I/O    Daughter Card Pin 8
; Pin 8  (RC6)		I/O    Daughter Card Pin 11
; Pin 9  (RC7)		I/O    Daughter Card Pin 12
; Pin 10 (RB7/TX1)		RS-485 Driver
; Pin 11 (RB6)		RS-485 Driver Enable (Active High Output)
; Pin 12 (RB5)		RS-485 Receiver
; Pin 13 (RB4)		RS-485 Receiver Enable (Active Low Output)
; Pin 14 (RC2)		I/O    Daughter Card Pin 7
; Pin 15 (RC1)		RX2 or I/O Daughter Card Pin 6
; Pin 16 (RC0)		TX2 or I/O Daughter Card Pin 5
; Pin 17 (RA2)		SYSLED (Active Low Output)(System LED)
; Pin 18 (RRA/ICSPCLK)		ICSPCLK
; Pin 19 (RA0/ICSPDAT)		ICSPDAT
; Pin 20 (Vss)		Ground
;
;=========================================================================================
;
	list	p=16F15345,r=hex,W=1	; list directive to define processor
	nolist
	include	p16f15345.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1, b'0001111110001100'
; FCMEM=0, RSTOSC=000 (32MHz), FEXTOSC=100
;
	__CONFIG _CONFIG2, b'0011111111111110'
; MCLRE=0
;
	__CONFIG _CONFIG3, b'0011111110011111'
;WDTE=00
;
	__CONFIG _CONFIG4, b'0001111111101111'
; LVP=0, nSAFEN=0
;
	__CONFIG _CONFIG5, b'0011111111111111'
;
;=========================================================================================
;
	constant	oldCode=0
	constant	useRS232=0	;TX2/RX2 to USB
	constant	useRS485=1	;TX1/RX1 to RS-485 SMRRC devices
	constant	useRS232PacketCmds=0
	constant	useRS485PacketCmds=1
	constant	useBootloader=0	;TX2/RX2 only.
	constant	UseEEParams=1
;
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
;
	constant	RP485_LongAddr=0
	constant	RP485_AddressBytes=1
	constant	RP485_DataBytes=4
	constant	UseRS485SyncBytes=1
kRS485SyncByteValue	EQU	0xDD
	constant	UseRS485Chksum=1
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x03	;This Slave's Address
;
kRS485_Address	EQU	0x03	;my Address on the RS-485 bus
;
kSysMode	EQU	.0	;Default Mode
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;====================================================================================================
	nolist
	include	F15345_Macros.inc
	list
;
;    Port A bits
PortA_Tris_Bits	EQU	b'11111111'	;All inputs
PortA_Init_Value	EQU	b'00000000'
PortA_ANSel_Value	EQU	b'00000000'	;All digital
;
SysLED_Bit	EQU	2	;LED1 (Active Low Output)
SysLEDPort	EQU	PORTA
#Define	SW1_In	PORTA,SysLED_Bit
#Define	SysLEDLat	LATA,SysLED_Bit
#Define	SysLEDTris	TRISA,SysLED_Bit
;
LED1_Bit	equ	4
#Define	LED1_TRIS	TRISA,LED1_Bit
#Define	LED1_LAT	LATA,LED1_Bit
#Define	SW1_PORT	PORTA,LED1_Bit
LED2_Bit	equ	5
#Define	LED2_TRIS	TRISA,LED2_Bit
#Define	LED2_LAT	LATA,LED2_Bit
#Define	SW2_PORT	PORTA,LED2_Bit
;
;    Port B bits
PortB_Tris_Bits	EQU	b'10101111'	;RS-485, TX/RX are controlled by serial	
PortB_Init_Value	EQU	b'00000000'
PortB_ANSel_Value	EQU	b'00000000'	;All digital
;
#Define	RS485nRE	LATB,4
#Define	RS485DE	LATB,6
;
;    Port C bits
PortC_Tris_Bits	EQU	b'00000000'	;RC0=TX2, RC1=RX2
PortC_Init_Value	EQU	b'00000000'
PortC_ANSel_Value	EQU	b'00000000'	;All digital
;
#Define	RC0In	PORTC,0	;I/O or TTL Serial TX
#Define	RC1In	PORTC,1	;I/O or TTL Serial RX
;
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
T2CON_Value	EQU	b'11101001'	;T2 On, /64 pre, /10 post
PR2_Value	EQU	.125	; 100/Sec
;
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
LEDFastTime	EQU	d'20'
;
;2MHz timebase for R/C servos
T1CON_Val	EQU	b'00100001'	;Fosc=32MHz, PreScale=4,Fosc/4,Timer ON
;
TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, high speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
BAUD2CON_Value	EQU	b'00001000'	;BRG16=1
BAUD1CON_Value	EQU	b'00001000'	;BRG16=1
; 32MHz clock low speed (BRGH=1,BRG16=1)
Baud_300	EQU	.26666	;300, 0.00%
Baud_1200	EQU	.6666	;1200, 0.00%
Baud_2400	EQU	.3332	;2400, +0.01%
Baud_9600	EQU	.832	;9604, +0.04%
Baud_19200	EQU	.416	;19.18k, -0.08%
Baud_38400	EQU	.207	;38.46k, +0.16%
Baud_57600	EQU	.138	;57.55k, -0.08%
BaudRate	EQU	Baud_38400
RS485BaudRate	EQU	Baud_38400
;
kSysFlags	EQU	.0
;
DebounceTime	EQU	.10
;
kMaxMode	EQU	.3
;
;
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are1024 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F,
;  Bank3 0x1A0..0x1EF, Bank4 0x220..0x26F, Bank5 0x2A0..0x2EF, Bank6 0x320..0x36F, 
;  Bank7 0x3A0..0x3EF, Bank8 0x420..0x46F, Bank9 0x4A0..0x4EF, Bank10 0x520..0x56F, 
;  Bank11 0x5A0..0x5EF, Bank12 0x620..0x64F
; there are 128 bytes of Storage Area Flash starting at 0x1F80 to 0x1FFF
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	SysLED_Time		;sys LED time
	SysLED_Blinks		;0=1 flash,1,2,3
	SysLED_BlinkCount
	SysLEDCount		;sys LED Timer tick count
; LED1, LED2 blinking
	LED1_Blinks		;0=off,1,2,3
	LED2_Blinks
	LED1_BlinkCount		;LED2_Blinks..0
	LED2_BlinkCount
	LED1_Count		;tick count
	LED2_Count
;
	ssStatus:4
	SysFlags1
	SysFlags2
;
	EEAddrTemp		;SAF address to read or write
	EEDataTemp		;Data to be writen to SAF
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; 50 mS RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		; 50 mS RX485 timer
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
; RS-232
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
; RS-485
	TX485Byte		;Next byte to send
	RX485Byte		;Last byte received
	Ser485Flags
;
;-----------------------
;Below here are saved in SAF
	SysMode
	RS232_MasterAddr:RP_AddressBytes
	RS232_SlaveAddr:RP_AddressBytes
;
	RS485_Address:RP485_AddressBytes	;my address on the RS-485 bus
;
	SysFlags		
;
	endc
;--------------------------------------------------------------
;---SysFlags1 bits---
#Define	SW1_Active	SysFlags1,2
#Define	SW1_Debounce	SysFlags1,3
#Define	LED1_Active	SysFlags1,4
#Define	SW2_Active	SysFlags1,5
#Define	SW2_Debounce	SysFlags1,6
#Define	LED2_Active	SysFlags1,7
;
;
;---SerFlags bits---
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
#Define	ssRX_Timeout	SerFlags,3	;cleared by host read
;
#define	RS485TXActive	Ser485Flags,0
#Define	RS485DataReceivedFlag	Ser485Flags,1
#Define	RS485DataSentFlag	Ser485Flags,2
#Define	RS485RX_Timeout	Ser485Flags,3	;cleared by host read
;
;---------------
#Define	FirstRAMParam	SysMode
#Define	LastRAMParam	SysFlags
;
;
;=========================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes, RS-232 Packet Serial
;
	cblock	0x0A0
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_SrcAdd:RP_AddressBytes	;1 or 2
	RX_DstAdd:RP_AddressBytes	;1 or 2
	RX_TempData:RP_DataBytes	;4
	RX_Data:RP_DataBytes		;4
	TX_Data:RP_DataBytes		;4
;
	endc
;
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes RS-232 Packet Serial
;
#Define	Ser_Buff_Bank	2
;
	cblock	0x120
	Ser_In_Bytes		;Bytes in Ser_In_Buff
	Ser_Out_Bytes		;Bytes in Ser_Out_Buff
	Ser_In_InPtr
	Ser_In_OutPtr
	Ser_Out_InPtr
	Ser_Out_OutPtr
	Ser_In_Buff:20
	Ser_Out_Buff:20
	endc
;
;================================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
MathAddress	EQU	0x1A0
;	include	MathEQUs.inc
;=========================================================================================
;  Bank4 Ram 220h-26Fh 80 Bytes, RS-485 Packet Serial
	include	RS485_Parse.H
;=========================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
;
;=========================================================================================
;  Bank6 Ram 320h-32Fh 16 Bytes, 512 bytes total ram
;================================================================================================
;  Bank7 No Ram beyond bank 6
;=======================================================================================================
;  Common Ram 70-7F same for all banks 0..23, 56..63
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs _IDLOC0, 0x10d1
;
	if UseEEParams
;==============================================================================================
; SAF locations (HEF) 0x00..0x7F (offsets)
;
; default values
SAFAddress	EQU	0x1F80
	ORG	SAFAddress
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr, 0x0F
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr, 0x10
;
	de	kRS485_Address	;me
;
	de	kSysFlags	;nvSysFlags
;
	ORG	0x1FFF
	de	0x00	;Skip BootLoader
;
	endif
; SAF Addresses
SAFStart	equ	0x1F80
	cblock	0x0000
;
	nvSysMode
	nvRS232_MasterAddr		;for RS-232 packet serial
	nvRS232_SlaveAddr
;
	nvRS485_Address
	nvRS485_MasterAddr
;
	nvSysFlags
;
	endc
;
#Define	nvFirstParamByte	nvSysMode
#Define	nvLastParamByte	nvSysFlags
;
;
;============================================================================================
; ******************* Reset Vector *****************************
;============================================================================================
;
	ORG	0x000	; processor reset vector
;
	if useBootloader
BootLoaderStart	EQU	0x1D00
;
	movlp	BootLoaderStart
	goto	BootLoaderStart
	endif
;
ProgStartVector	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	PCLATH
	movlb	0	; bank 0
;
;=============================
; Timer 0 is 100/s
;
	movlb	PIR0	; bank14
	btfss	PIR0,TMR0IF
	bra	SystemTick_end
	bcf	PIR0,TMR0IF
	movlb	0	; bank 0
;
;------------------
; These routines run 100 times per second
;
;	
;------------------
;Decrement timers until they are zero
;
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;	
	bsf	SysLEDTris	;LED off
	bsf	LED1_TRIS	;LED off
	bsf	LED2_TRIS	;LED off
;
;
;--------------------
; Sys LED time
	DECFSZ	SysLEDCount,F	;Is it time?
	bra	SystemBlink_end	; No, not yet
;
;
SystemBlink_Std	MOVF	SysLED_Time,W
	MOVWF	SysLEDCount
	bcf	SysLEDTris	;LED ON
;
SystemBlink_end:
;--------------------
; LED 1
	bcf	SW1_Active
	btfss	SW1_PORT
	bsf	SW1_Active
;
	decfsz	LED1_Count,F
	bra	LED1Blink_end
;
	movlw	.100	;1 second
	movwf	LED1_Count
	btfsc	LED1_Active
	bcf	LED1_TRIS	;LED ON
LED1Blink_end:
;--------------------
; LED 2
	bcf	SW2_Active
	btfss	SW2_PORT
	bsf	SW2_Active
;
	decfsz	LED2_Count,F
	bra	LED2Blink_end
;
	movlw	.100	;1 second
	movwf	LED2_Count
	btfsc	LED2_Active
	bcf	LED2_TRIS	;LED ON
LED2Blink_end:
;	
;
SystemTick_end:
;
;-----------------------------------------------------------------------------------------
	if useRS232
;-----------------------------------------------------------------------------------------
;EUSART Serial ISR RS-232 Packet Serial
;
IRQ_Ser	movlb	PIR3
	BTFSS	PIR3,RC2IF	;RX has a byte?
	BRA	IRQ_Ser_End
	CALL	RX_TheByte
IRQ_Ser_End:
;
	endif
;-----------------------------------------------------------------------------------------
	if useRS485
;-----------------------------------------------------------------------------------------
;EUSART Serial ISR RS-485 Packet Serial on RX1/TX1
;
IRQ_RS_485	movlb	PIR3
	BTFSS	PIR3,RC1IF	;RX has a byte?
	BRA	IRQ_RS_485_End
	CALL	RX_The485Byte
IRQ_RS_485_End:
;
	endif
;
;-----------------------------------------------------------------------------------------
	retfie
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	include	F15345_Common.inc
	if useRS232
	include	SerBuff15345.inc
	include	RS232_Parse.inc
	endif
;
start	call	InitializeIO
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
MainLoop	nop
;	CLRWDT
	nop

;----------------------
;
	movlb	0	;bank 0
	movf	SysMode,W
	brw
	goto	DoModeZero
	goto	DoModeOne
	goto	DoModeTwo
	goto	DoModeThree
;
ModeReturn:
;
;
	movlb	0
;
;-----------------------------------------------------------------------------------------
	if useRS232
	include	RS232MLCode.inc
	endif
;-----------------------------------------------------------------------------------------
	if useRS485
	include	RS484MLCode.inc
	endif
;-----------------------------------------------------------------------------------------
;
;
	bra	MainLoop
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
; Mode 0  Master Control, RS-232 to USB, RS-485 to Railroad
;
DoModeZero	goto	ModeReturn
;
;=========================================================================================
; 
;
DoModeOne	goto	ModeReturn
;
;=========================================================================================
;
;
DoModeTwo	goto	ModeReturn
;
;=========================================================================================
; 
;
DoModeThree	goto	ModeReturn
;
;=========================================================================================
; ***************************************************************************************
;=========================================================================================
; Initialization routine for PIC16F18854 based BLDC_Drive.
; Call once before starting main loop.
;
InitializeIO	movlb	WDTCON0
	movlw	b'00100110'	;longest and off
	movwf	WDTCON0
;
	movlb	ANSELA                 ;bank 30
	movlw	PortA_ANSel_Value
	movwf	ANSELA
	clrf	SLRCONA	;No slew
	clrf	INLVLA	;TTL levels
	movlw	PortB_ANSel_Value
	movwf                  ANSELB
	movlw	PortC_ANSel_Value
	movwf	ANSELC
	clrf	SLRCONC	;No slew
	clrf	INLVLC	;TTL levels
;
	movlb	0	;bank 0
	movlw	PortA_Init_Value
	movwf	LATA
	movlw                  PortB_Init_Value
	movwf                  LATB
	movlw	PortC_Init_Value
	movwf	LATC
;
                       movlw                  PortA_Tris_Bits
                       movwf                  TRISA
                       movlw                  PortB_Tris_Bits
                       movwf                  TRISB
                       movlw                  PortC_Tris_Bits
                       movwf                  TRISC
;
; clear memory to zero
	CALL	ClearRam
	CLRWDT
	if UseEEParams
	CALL	CopyToRam
	endif
;
;-----------------------
;Setup T0 for 100/s
;
T0CON0_Value	equ	b'10001001'	;T0EN, 8bit timer, 1:10 Postscaler
T0CON1_Value	equ	b'01010110'	;Fosc/4, sync, 1:64 perscaler
TMR0H_Value	equ	.125
;
	movlb	T0CON0
	movlw	T0CON0_Value
	movwf	T0CON0
	movlw	T0CON1_Value
	movwf	T0CON1
	movlw	TMR0H_Value
	movwf	TMR0H
	bsf	T0CON0,T0EN
	movlb	PIE0
	bsf	PIE0,TMR0IE
	movlb	0
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
	MOVLW	LEDTIME
	MOVWF	SysLED_Time
	clrf	SysLED_Blinks
	clrf	SysLED_BlinkCount
	movlw	0x01
	movwf	SysLEDCount	;start blinking right away
	bcf	SysLEDTris	;LED ON
	MOVLW	LEDTIME
	MOVWF	SysLED_Time
	movlw	.100
	movwf	Timer4Lo	;ignor buttons for 1st second
;
;=========================================================================================
	if useRS232
	call	RS232_Init
	endif
;=========================================================================================
	if useRS485
	call	RS485_Init
	endif
;
;=========================================================================================
;
;
	movlb	0
	return
;
;=========================================================================================
;
	if useRS485
	include	SerBuffRS485_15345.inc
	include	RS485_Parse.inc
	endif
;
	if useRS232PacketCmds
	org	0x0800
	include	PacketSerialCmds.inc
	endif
;
	if useRS485PacketCmds
	org	0x0800
	include	RS485_OutputDC_Cmds.inc
	endif
;
	if useBootloader
	org	BootLoaderStart
	include	BootLoader15345.inc
	endif
;
	END
;
;























