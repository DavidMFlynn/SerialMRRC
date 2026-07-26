;=========================================================================================
; 
;   Filename:      RS485_OutputDC_Cmds.inc
;   created:       7/25/2026
;   File Version:  1.0.0   7/25/2026
;
;   Author: David M. Flynn
;   Company: DMF-Enterprises
;   E-Mail: dflynn.oxfordvue@gmail.com
;   License: MIT License
;
;=========================================================================================
; Notes:
;  Command interpreter for Serial Model Railroad Control RS-485 Packet Serial.
;  Output Daughter Board version
;
; History
; 1.0.0   7/25/2026	Copied from PacketSerialCmds.inc
;=========================================================================================
;
kCmd_SetMode	EQU	0x81	;+1 data (SysMode), return ACK
kCmd_GetMode	EQU	0x01
;
kCmd_SetOutputs	EQU	0xB4	;+1 data (the 8 output bits)
kCmd_GetOutputs	EQU	0x24	;(the 8 output bits)
kCmd_SetOutput	EQU	0xB5	;+2 data (Output Number, Bit Value)
kCmd_GetOutput	EQU	0x25	;+1 data (Output Number, Bit Value)
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
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;Special packet (0xDD, 0xDD, 0x01, 0x02, 0xAA, 0x00, 0x00, 0x00, checksum)
kCmd_SendIDString	EQU	0xAA	; returns "DMFE SMRRC Node\n"
;
;=========================================================================================
;
RS485_Send_S0:
;Set Source and Destination addresses
	movlb	0	;Bank 0
	movf	RS485_Address,W	;Source is Me
	movlb	Ser485_Buff_Bank	;Bank 4
	movwf	TX485_SrcAdd
	if RP485_LongAddr
	movlb	0	;Bank 0
	movf	RS485_Address+1,W
	movlb	Ser485_Buff_Bank	;Bank 4
	movwf	TX485_SrcAdd+1
	endif
;Return to sender
	movf	RX485_SrcAdd,W	;TX Dest is RX source
	movwf	TX485_DstAdd
	if RP485_LongAddr
	movf	RX485_SrcAdd+1,W
	movwf	TX485_DstAdd+1
	endif
;
	mLongCall	RS485_Send
	return
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRX485Data	movlb	RX485_Flags	;bank 4
	bcf	RX485DataIsNew
	btfsc	RX485DataValidFlag	;Destination is me?
	bra	HandleRX485Data_1	; Yes
;
HandleRX485Data_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;
; Data is to Me, handle it.
HandleRX485Data_1:
;
; Setup indirect pointers
	movlb	0	;bank 0
	mMOVLF	RX485_Data,FSR1
	mMOVLF	TX485_Data,FSR0
;RS485Dispatcher
	moviw	FSR1++	;Command
	movwf	Param78	;aka RX_Data
	andlw                  0xF0
	SKPZ		;0x0X?
	bra                    RS485_Dispatcher_1	; No
	movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x00 to 0x0F
	brw
	return                                        ;0x00 not used
	bra                    RS485Cmd_GetMode
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
RS485_Dispatcher_1     sublw                  0x80
                       SKPZ		;Command=0x8X?
                       bra                    RS485Dispatcher_2	; No
                       movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x80 to 0x8F
                       brw
                       return                                        ;0x80 not used
                       bra                    RS485Cmd_SetMode            ;kCmd_SetMode
                       return
                       return
                       return
                       return
                       goto	RS485Cmd_SetOutputs	;kCmd_SetOutputs
                       goto	RS485Cmd_SetOutput	;kCmd_SetOutput
                       return
                       return
                       return
                       return
                       return
                       return
                       return		;0x8E
                       return
;
;Jump table for 0x20 to 0x2F
RS485Dispatcher_2      movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x20
                       SKPZ		;Command=0x2X?
                       bra                    RS485Dispatcher_3	; No
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
                       return
                       return
                       return
                       return
                       goto	RS485Cmd_GetOutputs	;kCmd_GetOutputs
                       goto	RS485Cmd_GetOutput	;kCmd_GetOutput
                       return
                       return
;
;Jump table for 0xB0 to 0xBF
RS485Dispatcher_3      movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xB0
                       SKPZ		;Command=0xBX?
                       bra                    RS485Dispatcher_4	; No
                       movlw                  0x0F
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
RS485Dispatcher_4      movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x90
                       SKPZ		;Command=0x9X?
                       bra                    RS485Dispatcher_5	; No
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
                       return                                        ;0x90 not used
                       return
                       return
                       reset                                         ;kCmd_ReBoot
                       goto                   RS485Cmd_SaveParams
                       goto                   RS485Cmd_RestoreParams
                       return
                       return		;0x97
                       goto                   RS485Cmd_GetStatus
                       return                   	;RS485Cmd_RunBootloader
                       return		;0x9A
                       return
                       return
                       return
                       return
                       return
;
;Jump table for 0xA0 to 0xAF
RS485Dispatcher_5      movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xA0
                       SKPZ		;Command=0xAX?
                       bra                    RS485Dispatcher_6	; No
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
RS485Dispatcher_6      return                                        ;0xA0 not used
                       return		;RS485Cmd_SetMasterAddr
                       goto                   RS485Cmd_SetAddr
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       return		; RS485Cmd_SendIDString       ;0xAA
                       return
                       return
                       return
                       return
                       return
;
;---kCmd_SetMode----------------
RS485Cmd_SetMode	movlw	kMaxMode+1
	subwf	INDF1,W
	SKPB		;kMaxMode+1>Data
	bra	HandleRX485Data_Rtn
;
	moviw	FSR1++
	movwf	SysMode
	return
;
;---kCmd_GetMode------------------
RS485Cmd_GetMode	movf	SysMode,W
	movwi	FSR0++
	goto	RS485_Send_S0
;
;---kCmd_SetOutputs---------------
RS485Cmd_SetOutputs	movf	INDF1,W
	movwf	LATC
	return
;
;---kCmd_SetOutput-----------------
RS485Cmd_SetOutput	moviw	FSR1++	;Bit #, 0..7
	andlw	0x07
	movwf	Param79
	moviw	FSR1++	;Value 0 or 1
	andlw	0x01
	movwf	Param7A
	movlw	0x01
RS485Cmd_SetOutput_L1	movf	Param79,F
	SKPNZ
	bra	RS485Cmd_SetOutput_1
	lslf	WREG,F
	decf	Param79,F
	bra	RS485Cmd_SetOutput_L1
;
RS485Cmd_SetOutput_1	movlb	0	;Bank 0
	btfss	Param7A,0	;Setting the bit?
	bra	RS485Cmd_SetOutput_2	; No
	iorwf	LATC,F
	return
;
RS485Cmd_SetOutput_2	xorlw	0xFF	;invert the bits
	andwf	LATC,F
	return
;
;---kCmd_GetOutputs----------------
RS485Cmd_GetOutputs	movlb	0	;Bank 0
	movf	LATC,W
	movwi	FSR0++
	goto	RS485_Send_S0
;
;---kCmd_GetOutput-----------------
RS485Cmd_GetOutput	moviw	FSR1++	;Bit #, 0..7
	andlw	0x07
	movwf	Param79
	clrf	Param7A
	movlw	0x01
RS485Cmd_GetOutput_L1	movf	Param79,F
	SKPNZ
	bra	RS485Cmd_GetOutput_1
	lslf	WREG,F
	decf	Param79,F
	bra	RS485Cmd_GetOutput_L1	
;
RS485Cmd_GetOutput_1	movlb	0	;Bank 0
	andwf	LATC,W
	SKPZ
	bsf	Param7A,0
	movf	Param7A,W
	movwi	FSR0++
	goto	RS485_Send_S0
;
;---kCmd_SaveParams-----------------
RS485Cmd_SaveParams	mLongCall	SaveParams
	return
;
;---kCmd_RestoreParams--------------
RS485Cmd_RestoreParams	mLongCall	CopyToRam
	return
;
;
;---kCmd_GetStatus----------------
RS485Cmd_GetStatus	movf	ssStatus,W
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
	goto	RS485_Send_S0
;
;
;---kCmd_SetSlaveAddr-------------
RS485Cmd_SetAddr	moviw	FSR1++
	movwf	RS485_Address
;
	if RP485_LongAddr
	moviw	FSR1++
	movwf	RS485_Address+1
	endif
;
	mLongCall	SaveParams
	goto	RS485_Send_S0
;
;
;=========================================================================================
