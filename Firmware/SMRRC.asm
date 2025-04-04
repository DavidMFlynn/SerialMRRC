;=========================================================================================
;
;   Filename:	SMRRC.asm
;   Created:	4/1/2025
;   File Revision:	0.9a   4/1/2025
;   Project:	Serial Model Railroad Control
;   Author:	David M. Flynn
;   Company:	DMF-Enterprises
;   E-Mail:	dflynn@oxfordvue.com
;
;=========================================================================================
;   Serial Model Railroad Control:
;
;
;    History:
; 0.9a   4/1/2025	Copied from BLDC_Servo, Blink an LED
;
;=========================================================================================
; Options:
;
;=========================================================================================
;=========================================================================================
; What happens next:
;   At power up the system LED will blink.
;
;=========================================================================================
; uController pinout (PIC16F18854):
;
; Pin 1  (Vdd)		+5 Volts
; Pin 2  (RA5)		LED2 (Active Low Input/Output)(LED2/Button2)
; Pin 3  (RA4)		LED1 (Active Low Input/Output)(LED1/Button1)
; Pin 4  (RA3/MCLR*)		VPP/MCLR*
; Pin 5  (RC5)		I/O
; Pin 6  (RC4)		I/O
; Pin 7  (RC3)		I/O
; Pin 8  (RC6)		I/O
; Pin 9  (RC7)		I/O
; Pin 10 (RB7/TX1)		RS-485 Driver
; Pin 11 (RB6)		RS-485 Driver Enable (Active High Output)
; Pin 12 (RB5)		RS-485 Receiver
; Pin 13 (RB4)		RS-485 Receiver Enable (Active Low Output)
; Pin 14 (RC2)		I/O
; Pin 15 (RC1)		RX2 or I/O
; Pin 16 (RC0)		TX2 or I/O
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
	constant	useRS232=0
	constant	useBootloader=0
	constant	UseEEParams=0
	constant	UseQEnc=0
	constant	UsePID=0
	constant	UseAuxLEDBlinking=0
	constant	UseAnalogInputs=0
;
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x02	;This Slave's Address
kAux0Config	EQU	0x00
kAux1Config	EQU	0x00
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
#Define	Aux0_LED1_TRIS	TRISA,LED1_Bit
#Define	Aux0_LED1_LAT	LATA,LED1_Bit
#Define	Aux0_SW1_PORT	PORTA,LED1_Bit
LED2_Bit	equ	5
#Define	Aux1_LED2_TRIS	TRISA,LED2_Bit
#Define	Aux1_LED2_LAT	LATA,LED2_Bit
#Define	Aux1_SW2_PORT	PORTA,LED2_Bit
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
PortC_Tris_Bits	EQU	b'11111111'	;RC0=TX2, RC1=RX2
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
BAUDCON_Value	EQU	b'00001000'	;BRG16=1
; 32MHz clock low speed (BRGH=1,BRG16=1)
Baud_300	EQU	.26666	;300, 0.00%
Baud_1200	EQU	.6666	;1200, 0.00%
Baud_2400	EQU	.3332	;2400, +0.01%
Baud_9600	EQU	.832	;9604, +0.04%
Baud_19200	EQU	.416	;19.18k, -0.08%
Baud_38400	EQU	.207	;38.46k, +0.16%
Baud_57600	EQU	.138	;57.55k, -0.08%
BaudRate	EQU	Baud_38400
;
kSysFlags	EQU	.0
;
DebounceTime	EQU	.10
;
; ssAuxNConfig bits
;------------------------
; AuxIO modes, Mode is bits 0..2
kAuxIOnone	EQU	0x00
kAuxIOLEDBtn	EQU	0x01
kAuxIODigitalIn	EQU	0x02
kAuxIODigitalOut	EQU	0x03
kAuxIOAnalogIn	EQU	0x04
kAuxIOHomeSw	EQU	0x05
kAuxIOFwdLimit	EQU	0x06
kAuxIORevLimit	EQU	0x07
;
AuxConfigSwInvert	EQU	0x03
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
	if UseAuxLEDBlinking
; Aux0..Aux2 LED blinking
	LED1_Blinks		;0=off,1,2,3
	LED2_Blinks
	LED1_BlinkCount		;LED2_Blinks..0
	LED2_BlinkCount
	LED1_Count		;tick count
	LED2_Count
	endif
;
	SysFlags1
	SysFlags2
;
	EEAddrTemp		;SAF address to read or write
	EEDataTemp		;Data to be writen to SAF
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; 50 mS RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
; RS-232
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
;
;-----------------------
;Below here are saved in SAF
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
			; default 1.0 = 100 counts / sec^2
	ssAux0Config		;kAuxIO0
	ssAux1Config		;kAuxIO1
	SysFlags		
;
	endc
;--------------------------------------------------------------
;---SysFlags1 bits---
#Define	Aux0_SW1_Active	SysFlags1,2
#Define	Aux0_SW1_Debounce	SysFlags1,3
#Define	Aux0_LED1_Active	SysFlags1,4
#Define	Aux1_SW2_Active	SysFlags1,5
#Define	Aux1_SW2_Debounce	SysFlags1,6
#Define	Aux1_LED2_Active	SysFlags1,7
;
;
;---SerFlags bits---
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
;
;
;---------------
#Define	FirstRAMParam	SysMode
#Define	LastRAMParam	SysFlags
;
;
;=========================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes, RS-232 Packet Serial
	cblock	0x0A0
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_SrcAdd:RP_AddressBytes
	RX_DstAdd:RP_AddressBytes
	RX_TempData:RP_DataBytes
	RX_Data:RP_DataBytes
	TX_Data:RP_DataBytes
;
	ANFlags		;New Data flags
	ANxActive		;Skip if 0
	ANCount		;Current AN being serviced
	Cur_AN0:2		;Motor_I
	Cur_AN1:2		;Motor_V, 7815 output volts
	Cur_AN2:2		;Batt_V, J1.1 +12V
	Cur_AN3:2		;For_Limit, SW2_LED2
	Cur_AN4:2		;Rev_Limit, SW3_LED3
	Cur_AN5:2		;Home, SW4_LED4
;
	endc
;
LastAN	EQU	.5	;Sevice 3 AN inputs
FirstANData	EQU	Cur_AN0
;
AN_MotorCurrent	EQU	Cur_AN0
AN_MotorVolts	EQU	Cur_AN1
AN_BattVolts	EQU	Cur_AN2
AN_Aux0	EQU	Cur_AN3
AN_Aux1	EQU	Cur_AN4
AN_Aux2	EQU	Cur_AN5
;
;---ANFlags bits---
#Define	NewDataAN0	ANFlags,0
#Define	NewMotorCurrentData	ANFlags,0
#Define	NewDataAN1	ANFlags,1
#Define	NewDataAN2	ANFlags,2
#Define	NewDataAN3	ANFlags,3
#Define	NewDataAN4	ANFlags,4
#Define	NewDataAN5	ANFlags,5
;
;---ANxActive bits---
#Define	AN0_ActiveBit	ANxActive,0
#Define	AN1_ActiveBit	ANxActive,1
#Define	AN2_ActiveBit	ANxActive,2
#Define	AN3_ActiveBit	ANxActive,3
#Define	AN4_ActiveBit	ANxActive,4
#Define	AN5_ActiveBit	ANxActive,5
#Define	AN_Aux0_ActiveBit	ANxActive,3	;Names used by Serial Comms
#Define	AN_Aux1_ActiveBit	ANxActive,4
#Define	AN_Aux2_ActiveBit	ANxActive,5
;
AN0_Val	EQU	0x00	;ANA0
AN1_Val	EQU	0x01	;ANA1
AN2_Val	EQU	0x02	;ANA2
AN3_Val	EQU	0x03	;ANA3, For_Limit
AN4_Val	EQU	b'00010101'	;ANC5, Rev_Linit
AN5_Val	EQU	b'00010000'	;ANC0, Home
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
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
;  Bank4 Ram 220h-26Fh 80 Bytes
;=========================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
;	include	Q_EncoderH.inc
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
	de	kAux0Config	;nvssAux0Config
	de	kAux1Config	;nvssAux1Config
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
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
;
	nvssAux0Config
	nvssAux1Config
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
BootLoaderStart	EQU	0x0E00
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
;
;	
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
;	
	bsf	SysLEDTris	;LED off
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
;
SystemTick_end:
;
;-----------------------------------------------------------------------------------------
	retfie
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
;	include	F15345_Common.inc
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

	nop
	nop
;
	bra	MainLoop
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
;	clrf	SLRCONC	;No slew
;	clrf	INLVLC	;TTL levels
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
	return
;
;
;=============================
;
;
	END
;
;























