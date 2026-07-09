;=========================================================================================
; 
;    Filename:      PacketSerialCmds.inc
;    created:       4/4/2025
;    File Version:  1.0    4/4/2025
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
; Notes:
;  Command interpreter for Serial Model Railroad Control RS-232 Packet Serial
;
; History
; 1.0    4/4/2025	Copied from BLDCServo 1.2
;=========================================================================================
;
kCmd_SetMode	EQU	0x81	;+1 data (SysMode), return ACK
kCmd_GetMode	EQU	0x01
;
kCmd_SetAuxConfig	EQU	0xB2	;+2 data (AuxNum,Value), return ACK
kCmd_GetAuxConfig	EQU	0x22	;+1 data (AuxNum)
kCmd_SetAuxNValue	EQU	0xB3	;+2 data (AuxNum,Value), return ACK
kCmd_GetAuxNValue	EQU	0x23	;(AuxN,ValueL,ValueH)
;
kCmd_ReBoot	EQU	0x93	;ReBoot the controller
kCmd_SaveParams	EQU	0x94	;Save all eeprom params, return ACK
kCmd_RestoreParams	EQU	0x95	;Copy to ram, return ACK
kCmd_GetStatus	EQU	0x98	;return ssStatus
kCmd_RunBootloader	EQU	0x99	;Set EEPROM (0xFF) = 0xFF and reboot
;
; these commands save params and return a 0,0,0,0 packet with the new address
kCmd_SetMasterAddr	EQU	0xA1	;+1 data, return a zero packet
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;Special packet (0xDD, 0xDD, 0x01, 0x02, 0xAA, 0x00, 0x00, 0x00, checksum)
kCmd_SendIDString	EQU	0xAA	; returns "DMFE BLDC Servo\n"
;
;=========================================================================================
;
RS232_Send_S0	mLongCall	RS232_Send
	return
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRXData	movlb	RX_Flags	;bank 1
	bcf	RXDataIsNew
	btfsc	RXDataValidFlag	;from master to me?
	bra	HandleRXData_1
HandleRXData_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;
HandleRXData_1	movlb	0	;bank 0
	mMOVLF	RX_Data,FSR1
	mMOVLF	TX_Data,FSR0
;Dispatcher
	moviw	FSR1++
	movwf	Param78	;aka RX_Data
	andlw                  0xF0
	SKPZ
	bra                    Dispatcher_1
	movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x00 to 0x0F
	brw
	return                                        ;0x00 not used
	bra                    Cmd_GetMode
	return
	return
	return
	return
	return
	return
	return
	return
	return
	return
	return
	return
	return
	return
;
Dispatcher_1           sublw                  0x80
                       SKPZ
                       bra                    Dispatcher_2
                       movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x80 to 0x8F
                       brw
                       return                                        ;0x80 not used
                       bra                    Cmd_SetMode            ;kCmd_SetMode
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return		;0x8E
                       return
;
;Jump table for 0x20 to 0x24
Dispatcher_2           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x20
                       SKPZ
                       bra                    Dispatcher_3
                       movlw                  0x07
                       andwf                  Param78,W
                       brw
                       return
                       return
                       goto                   Cmd_GetAuxConfig
                       goto                   Cmd_GetAuxNValue
                       return
                       return
                       return
                       return
;
;Jump table for 0xB0 to 0xB4
Dispatcher_3           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xB0
                       SKPZ
                       bra                    Dispatcher_4
                       movlw                  0x07
                       andwf                  Param78,W
                       brw
                       return
                       return
                       goto                   Cmd_SetAuxConfig
                       goto                   Cmd_SetAuxNValue
                       return
                       return
                       return
                       return
;
;Jump table for 0x90 to 0x9F
Dispatcher_4           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x90
                       SKPZ
                       bra                    Dispatcher_5
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
                       return                                        ;0x90 not used
                       return
                       return
                       reset                                         ;kCmd_ReBoot
                       goto                   Cmd_SaveParams
                       goto                   Cmd_RestoreParams
                       return
                       return		;0x97
                       goto                   Cmd_GetStatus
                       goto                   Cmd_RunBootloader
                       return		;0x9A
                       return
                       return
                       return
                       return
                       return
;
Dispatcher_5           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xA0
                       SKPZ
                       bra                    Dispatcher_6
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
Dispatcher_6           return                                        ;0xA0 not used
                       goto                   Cmd_SetMasterAddr
                       goto                   Cmd_SetSlaveAddr
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       goto                   Cmd_SendIDString       ;0xAA
                       return
                       return
                       return
                       return
                       return
;
;---kCmd_SetMode----------------
Cmd_SetMode	movlw	kMaxMode+1
	subwf	INDF1,W
	SKPB		;kMaxMode+1>Data
	bra	HandleRXData_Rtn
;
	moviw	FSR1++
	movwf	SysMode
	goto	TX_ACK
;
;---kCmd_GetMode------------------
Cmd_GetMode	movf	SysMode,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;
;---kCmd_SetAuxConfig-----------	;+2 data (AuxNum,Value), return ACK
; Set Aux N Config
Cmd_SetAuxConfig	movf	INDF1,W
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	moviw	FSR1++
	addlw	low ssAux0Config
	movwf	FSR0L
	movlw	high ssAux0Config
	movwf	FSR0H
	moviw	FSR1++	;New Config Value
	movwi	FSR0++
	goto	TX_ACK
;
;---kCmd_GetAuxConfig-----------
; Get Aux N Config
Cmd_GetAuxConfig	movf	INDF1,W
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	movf	INDF1,W
	addlw	low ssAux0Config
	movwf	FSR1L
	movlw	high ssAux0Config
	movwf	FSR1H
;
	movf	INDF1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetAuxNValue-----------	;+2 data (AuxNum,Value), return ACK
; Set Aux N Value
Cmd_SetAuxNValue	moviw	FSR1++
	movwf	Param78	;AuxNum
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	movf	Param78,W	;AuxNum
	addlw	low ssAux0Config
	movwf	FSR0L
	movlw	high ssAux0Config
	movwf	FSR0H
;
	movf	INDF0,W	;ssAuxXConfig
	andlw	0x07
	brw
	goto	TX_ACK	;kAuxIOnone
	bra	Cmd_SetAuxNLED	;kAuxIOLEDBtn
	goto	TX_ACK	;kAuxIODigitalIn
	bra	Cmd_SetAuxNDigOut	;kAuxIODigitalOut
	goto	TX_ACK	;kAuxIOAnalogIn
	goto	TX_ACK	;kAuxIOHomeSw
	goto	TX_ACK	;kAuxIOFwdLimit
	goto	TX_ACK	;kAuxIORevLimit
;
Cmd_SetAuxNDigOut:
Cmd_SetAuxNLED:
                       if UseAuxLEDBlinking
	movf	Param78,W	;AuxNum
	addlw	low LED2_Blinks
	movwf	FSR0L
	movlw	high LED2_Blinks
	movwf	FSR0H
	moviw	FSR1++	;AuxVal
	movwi	FSR0++
	endif
	goto	TX_ACK
;
;---kCmd_GetAuxNValue-----------	;(AuxN,ValueL,ValueH)
; Get Aux N Value
Cmd_GetAuxNValue	movf	INDF1,W
	movwf	Param78	;AuxNum
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	movf	Param78,W	;AuxNum
	addlw	low ssAux0Config
	movwf	FSR1L
	movlw	high ssAux0Config
	movwf	FSR1H
;
	movf	INDF1,W	;ssAuxXConfig
	andlw	0x07
	brw
	goto	HandleRXData_Rtn	;kAuxIOnone, not valid
	bra	Cmd_GetAuxNValue_Btn	;kAuxIOLEDBtn
	bra	Cmd_GetAuxNValue_Dig	;kAuxIODigitalIn
	bra	Cmd_GetAuxNValue_Dig	;kAuxIODigitalOut
	bra	Cmd_GetAuxNValue_Ana	;kAuxIOAnalogIn
	bra	Cmd_GetAuxNValue_Dig	;kAuxIOHomeSw
	bra	Cmd_GetAuxNValue_Dig	;kAuxIOFwdLimit
	bra	Cmd_GetAuxNValue_Dig	;kAuxIORevLimit
;
Cmd_GetAuxNValue_Ana	lslf	Param78,W	;AuxNum X2
	addlw	low AN_Aux0
	movwf	FSR1L
	movlw	high AN_Aux0
	movwf	FSR1H
	moviw	FSR1++
	movwi	FSR0++
	moviw	FSR1++
	movwi	FSR0++
	goto	RS232_Send_S0
;
; Send the value of the port bit 0x00 or 0x01
Cmd_GetAuxNValue_Dig	movf	Param78,W	;AuxNum
	SKPZ
	bra	Cmd_GetAuxNValue_Dig1
	btfsc	Aux0_SW1_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Dig1	sublw	0x01
	SKPZ
	bra	Cmd_GetAuxNValue_Dig2
	btfsc	Aux1_SW2_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Dig2	clrw
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
; Send the Button value
Cmd_GetAuxNValue_Btn	movf	Param78,W	;AuxNum
	SKPZ
	bra	Cmd_GetAuxNValue_Btn1
	btfsc	Aux0_SW1_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Btn1	sublw	0x01
	SKPZ
	bra	Cmd_GetAuxNValue_Btn2
	btfsc	Aux1_SW2_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Btn2	clrw
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
;
;---kCmd_SaveParams-----------------
Cmd_SaveParams	mLongCall	SaveParams
	goto	TX_ACK
;
;---kCmd_RestoreParams--------------
Cmd_RestoreParams	mLongCall	CopyToRam
	goto	TX_ACK
;
;
;---kCmd_GetStatus----------------
Cmd_GetStatus	movf	ssStatus,W
	movwi	FSR0++
	movf	ssStatus+1,W
	movwi	FSR0++
	movf	ssStatus+2,W
	movwi	FSR0++
	movf	ssStatus+2,W
	movwi	FSR0++
;reset flags
	bcf	ssRX_Timeout
;
	clrf	ssStatus+1
;	
	goto	RS232_Send_S0
;
;---kCmd_RunBootloader--------------
Cmd_RunBootloader	movlw	0xFF
	movwf	EEDataTemp
	mLongCall	EEWriteW	;Write EEPROM address in W
	reset
;
;
;---kCmd_SetMasterAddr------------
Cmd_SetMasterAddr	moviw	FSR1++
	movwf	RS232_MasterAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
;
;---kCmd_SetSlaveAddr-------------
Cmd_SetSlaveAddr	moviw	FSR1++
	movwf	RS232_SlaveAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
;
;---kCmd_SendIDString------------
;
Cmd_SendIDString	clrf	Param79
Cmd_SendIDString_L1	movf	Param79,W
	call	IDString
	iorlw	0x00
	SKPNZ
	return
	call	StoreSerOut_S0
	incf	Param79,F
	bra	Cmd_SendIDString_L1
;
IDString	brw
	retlw	'D'
	retlw	'M'
	retlw	'F'
	retlw	'E'
	retlw	' '
	retlw	'S'
	retlw	'M'
	retlw	'R'
	retlw	'R'
	retlw	'C'
	retlw	'\n'
	retlw	0x00
;
;--------------------------------
	return
;
TX_ACK	movlw	0xFF
StoreSerOut_S0	mLongCall	StoreSerOut
	return
;
;=========================================================================================
