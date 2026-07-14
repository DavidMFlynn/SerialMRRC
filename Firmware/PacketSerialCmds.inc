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
kCmd_SendIDString	EQU	0xAA	; returns "DMFE SMRRC Node\n"
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
	btfsc	RXDataValidFlag	;From master to me?
	bra	HandleRXData_1	; Yes
;
	btfss	RXSrcIsMaster	;from master?
	bra	HandleRXData_Rtn	; No, ignore it
	btfsc	RXDstIsMe	;to Me?
	bra	HandleRXData_Rtn	; Yes, must be a bad CS or ?, ignore it
; forward packet on RS485 bus
;	RX_CSUM		;bank1
;	RX_SrcAdd:RP_AddressBytes	;1 or 2
;	RX_DstAdd:RP_AddressBytes	;1 or 2
;	RX_TempData:RP_DataBytes	;4
;
	call	RS485_Forward
;
HandleRXData_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;
; Data is from Master to Me, handle it.
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
                       return
                       return
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
	return
                       return
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
;Jump table for 0xA0 to 0xAF
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
	bsf	LED1_Active	;tc
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
	retlw	' '
	retlw	'N'
	retlw	'o'
	retlw	'd'
	retlw	'e'
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
