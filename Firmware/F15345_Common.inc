;=========================================================================================
; Commonly used routines PIC16F18854 version
;
;    Filename:      F18854_Common.inc
;    Date:          3/26/2020
;    File Version:  1.0.4
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
;    History:
;
; 1.0.4  3/26/2020     Working with 16F18854
; 1.0.3  2/23/2020     Copied from F1847_Common and modified.
; 1.0.2  4/6/2019	Fixes to EERead/EEwrite
; 1.0.2 2/16/2019	Fix: added FSR0H to DecTimer.
; 1.0.1 11/21/2015	Updates DecTimer.
; 1.0   11/16/2013	Updated from F648A Common.inc
;
;=========================================================================================
; Routines:
;
; ClearRam	(2+0) Clears all RAM, call once before initializing variables, FSR0
; CopyToRam	(1+0) copy param memory (EEPROM) to ram, call once, FSR0
; SaveParams	(1+0) copy ram to param memory (EEPROM), FSR0
;
;  Set FSR0H before calling these routines
; DecTimer4	(0+0) Decrement routine for 16 bit timers, FSR0
; DecTimer3
; DecTimer2
; DecTimer1
; DecTimer	(0+0) High byte of counter address in W
;
; TestT4_Zero	Test for 16 bit timers = zero
; TestT3_Zero	If Timer is zero return Z flag,1 else Z=0
; TestT2_Zero
; TestT1_Zero
;
; Delay10uS	(0+0)Delay uS    1 cycle = 1uS, 8Mhz clock version
; Delay100uS
; Delay40uS
; DelayWuS
;
; EEReadW	(0+0) Read EEPROM address in W
; EERead	(0+0) Read EEPROM address in EEAddrTemp
; EEWriteW	(0+0) Write EEPROM address in W, Data in EEDataTemp
; EEWrite	(0+0) Write EEPROM address in EEAdrTemp, Data in EEDataTemp, FSR0
;
;=========================================================================================
	ifndef UseEEParams
	constant UseEEParams=0
	endif
;=========================================================================================
; Clears all RAM
; Entry: none
; Exit: none
; RAM used: All
; Calls:(2+0) ClearRam_L2
;
; Banks 0..11 80 bytes, Bank 12 48 bytes
;
ClearRam	MOVLB	0x00                   ;bank 0
	MOVLW	0x5F	;Clear 20h-7Eh, 95 bytes
	MOVWF	Param7F
	MOVLW	0x20
	MOVWF	FSR0L
	CLRF	FSR0H
	CALL	ClearRam_L2
;
	MOVLW	0x50	;Clear Bank 1 A0h-EFh, 80 bytes
	MOVWF	Param7F
	MOVLW	0xA0
	MOVWF	FSR0L
	CALL	ClearRam_L2
;
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 2,3
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 4,5
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 6,7
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 8,9
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 10,11
;
	INCF	FSR0H,F	;Bank 12, 48 bytes
	movlw                  0x30
	MOVWF	Param7F
	MOVLW	0x20
	MOVWF	FSR0L
	bra                    ClearRam_L2
;	
ClearRam_2	MOVLW	0x50	;Clear 120h-16Fh, 80 bytes
	MOVWF	Param7F
	MOVLW	0x20
	MOVWF	FSR0L
	CALL	ClearRam_L2
;
	MOVLW	0x50	;Clear A0h-EFh, 80 bytes
	MOVWF	Param7F
	MOVLW	0xA0
	MOVWF	FSR0L
;
ClearRam_L2	CLRF	INDF0
	INCF	FSR0L,F
	DECFSZ	Param7F,F
	bra	ClearRam_L2
	RETURN
;
	if UseEEParams
;==========================================================================
; copy param memory to ram
;
CopyToRam	MOVLW	nvFirstParamByte
	MOVWF	EEAddrTemp
	MOVLW	FirstRAMParam
	MOVWF	FSR0L
	CLRF	FSR0H
CopyToRam_L1	CALL	EERead
	movwi	FSR0++
	INCF	EEAddrTemp,F
	MOVLW	LastRAMParam+1
	SUBWF	FSR0L,W
	SKPZ
	BRA	CopyToRam_L1
	RETURN
;
;===========================================================================
; copy ram to param memory
;
SaveParams	MOVLW	nvFirstParamByte
	MOVWF	EEAddrTemp
	MOVLW	FirstRAMParam
	MOVWF	FSR0L
	CLRF	FSR0H
SaveParams_L1	moviw	FSR0++
	MOVWF	EEDataTemp
	CALL	EEWrite
	INCF	EEAddrTemp,F
	MOVLW	LastRAMParam+1	;last byte
	SUBWF	FSR0L,W
	SKPZ
	BRA	SaveParams_L1
	RETURN
;
	endif
;=====================================================================================================
;=========================================================================================================
; Decrement routine for 16 bit timers
; Set FSR0H before calling these routines.
;
DecTimer4	movlw	Timer4Hi
	bra	DecTimer
DecTimer3	movlw	Timer3Hi
	bra	DecTimer
DecTimer2	movlw	Timer2Hi
	bra	DecTimer
DecTimer1	movlw	Timer1Hi
;DecTimer
; entry: FSR=Timer(n)Hi
DecTimer	MOVWF	FSR0L
	movlw	High Timer4Hi
	movwf	FSR0H
	MOVIW	FSR0--	;TimerNHi
	IORWF	INDF0,W	;TimerNLo
	SKPNZ
	RETURN
	MOVLW	0x01
	SUBWF	INDF0,F	;TimerNLo
	INCF	FSR0L,F
	CLRW
	SUBWFB	INDF0,F	;TimerNHi
	RETURN
;
;==============================================================================================
; Test for 16 bit timers = zero
;If Timer is zero return Z flag,1 else Z=0
;
TestT4_Zero	movlb	0
	movf	Timer4Lo,W
	iorwf	Timer4Hi,W
	return
;
TestT3_Zero	movlb	0
	movf	Timer3Lo,W
	iorwf	Timer3Hi,W
	return
;
TestT2_Zero	movlb	0
	movf	Timer2Lo,W
	iorwf	Timer2Hi,W
	return
;
TestT1_Zero	movlb	0
	movf	Timer1Lo,W
	iorwf	Timer1Hi,W
	return	
;
	if oldCode
;======================================================================================
;Delay uS    1 cycle = 1uS, 8MHz clock version
; RAM used: Param77
; Calls:(0) none
;
Delay10uS	MOVLW	0x05	;(2*3+5)/2=10
	bra	DelayWuS
Delay100uS	MOVLW	d'65'	;(28*3+5)/2=100
	bra	DelayWuS
Delay40uS	MOVLW	d'25'	;(11*3+5)=40
DelayWuS	MOVWF	Param77
DelayWuS_Loop	DECFSZ	Param77,F
	bra	DelayWuS_Loop
	RETURN
;
	endif
;======================================================================================
;Delay uS    1 cycle = .125uS, 32MHz clock version
; RAM used: Param77
; Calls:(0) none
;
Delay10uS	MOVLW	0x05	;(2*3+5)/2=10
	bra	DelayWuS
Delay100uS	MOVLW	d'65'	;(28*3+5)/2=100
	bra	DelayWuS
Delay40uS	MOVLW	d'25'	;(11*3+5)=40
DelayWuS	MOVWF	Param77
DelayWuS_Loop	nop
	nop
	DECFSZ	Param77,F
	bra	DelayWuS_Loop
	RETURN
;
	if UseEEParams
;==============================================================================================
; Read EEPROM
; entry: EEPROM address to read in W
;        Bank 0 selected
; exit: W=EEDATA, Bank 0 selected
;
EEReadW	movwf	EEAddrTemp
;
;==============================================================================================
; Read EEPROM
; entry: EEPROM address to read in EEAddrTemp
;        Bank 0 selected
; exit: W=EEDATA, Bank 0 selected
;
EERead	movf	EEAddrTemp,W
	addlw	low SAFAddress
	movwf	FSR0L
	movlw	high SAFAddress
	bsf	WREG,7
	movwf	FSR0H
	movfw	INDF0
	MOVLB	0x00	;Bank 0
	return
;
;==============================================================================================
; Write EEPROM
; entry: EEPROM address to write in W
;        EEPROM data to write in EEDataTemp
;        Bank 0 selected
; exit: Bank 0 selected
;
EEWriteW	movwf	EEAddrTemp
;
;==============================================================================================
; Write EEPROM
; Cannot be used by ISR.
; entry: EEPROM address to write in EEAdrTemp
;        EEPROM data to write in EEDataTemp
;        Bank 0 selected
; exit: Bank 0 selected
;
EEWrite	MOVF	EEAddrTemp,W
	
                       movlb                  NVMADRL                ;bank 16
                       addlw	low SAFAddress
                       movwf                  NVMADRL
	movlw                  high SAFAddress
	movwf                  NVMADRH
	movlb	0	;bank 0
	movf	EEDataTemp,W
	movlb	NVMADRL                ;bank 16
	movwf                  NVMDATL
	clrw
	movwf                  NVMDATH
;
                       bcf                    NVMCON1,NVMCON1_NVMREGS	;Prog mem
;                      bsf                    NVMCON1,NVMCON1_FREE
	
;
EEWrite_L1	bcf	INTCON,GIE	;Disale Ints
	btfsc	INTCON,GIE
	bra	EEWrite_L1
;
	bsf                    NVMCON1,NVMCON1_WREN
	movlw                  0x55
	movwf                  NVMCON2
	movlw                  0xAA
	movwf                  NVMCON2
	bsf                    NVMCON1,NVMCON1_WR
;
EEWriteLoop	btfsc	NVMCON1,NVMCON1_WR	
	bra	EEWriteLoop
	bcf	NVMCON1,NVMCON1_WREN
	bsf	INTCON,GIE
	MOVLB	0x00	;Bank 0
	return	
;
	endif
;	
;=========================================================================================
;=========================================================================================
;
;
;
