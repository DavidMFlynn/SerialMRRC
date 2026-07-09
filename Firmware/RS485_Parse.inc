;=========================================================================================
;
;    Filename:  RS485_Parse.inc
;    Created:   4/5/2025
;    Revision:  1.0d1  4/5/2025
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;
;=========================================================================================
; Routines:
;
; RS485_Parse	Call from main loop
; RS485_ClearTXData	Clear the TX data, FSR1
; RS485_Send	Send the data at TX_data, FSR0, FSR1
;
;=========================================================================================
; Notes:
;
; History:
; 1.0d1  4/5/2025	Copied from RS232_Parse 1.0b5
;
; RS485_Parse	(X/0) Parse any bytes received by the serial port
; Packet format: (DD DD) DA SA D0 D1 D2 D3 D4 D5 D6 D7 (CS)
;  DD = Sync byte added if address is received
;  DA = Destination Address, SA = Source Address, 
;       Dn = Data, CS Check Sum (xor of Address and Data bytes)
;  SA RS485_SAddr ( the sender )
;  DA must be RS485_MyAddr ( this program, must match )
;  If the checksum matches put the data bytes in the RX485_Data buffer,
;   set RX485DataValidFlag and RX485DataIsNew.
;=========================================================================================
;
; default to a 6 byte packet 0xDD 0xDD DA SA D0 D1 D2 D3 CS
;
;
	ifndef RP485_DataBytes
	constant RP485_DataBytes=4
	endif
;
	ifndef UseRS485Chksum
	constant UseRS485Chksum=0
	endif
;
	constant RX485Timeout=5	;50 mS max between incoming bytes
;=========================================================================================
; Bank0 variable used by RS232_Parse:
;
;	cblock
;	RS485_Address
;	endc
;
; Bank4 variables used by RS485_Parse:
;#Define	Ser485_Buff_Bank	4
;
;	cblock
;	RX485_ParseFlags
;	RX485_Flags
;	RX485_DataCount
;	RX485_CSUM
;	RX485_SrcAdd:RP485_AddressBytes
;	RX485_DstAdd:RP485_AddressBytes
;	RX485_TempData:RP_DataBytes
;	RX485_Data:RP485_DataBytes
;	TX485_Data:RP485_DataBytes
;	endc
;RX485_ParseFlags Bits
#Define	SyncByte1RXd	RX_ParseFlags,0
#Define	SyncByte2RXd	RX_ParseFlags,1
#Define	RS485DestAddrRXd	RX485_ParseFlags,2
#Define	RS485SourceAddrRXd	RX485_ParseFlags,3
#Define	RS485AllDataRXd	RX485_ParseFlags,4
;
;RX485_Flags Bits
#Define	RX485DataValidFlag	RX485_Flags,0
#Define	RX485DataIsNew	RX485_Flags,1
#Define	RX485SrcIsMaster	RX485_Flags,2
#Define	RX485DstIsMe	RX485_Flags,3
;
;
; *** put in main loop
;	CALL	RS485_Parse
; if RX485DataValidFlag is set data may be used
;
;=========================================================================================
; Parse any bytes received by the serial port
;
; Receive 0xDD,0xDD,DA0,SA0,Cmd,Data0,Data1,Data2,CS
;
;
; Entry: none
; Exit: none
; Ram Used: Param78, FSR0
; Calls (1+0): GetSer485In
;
RS485_Parse	movlb	Ser485_Buff_Bank
	movf	RX485_ParseFlags,F
	SKPNZ		;Receive in process?
	bra	RS485_Parse_Byte	; no
;
	movlb	0	;bank 0
	movf	Timer2Lo,F
	SKPZ		;Timed out?
	bra	RS485_Parse_Byte	; no
;timed out
	bsf	RS485RX_Timeout
	movlb	Ser485_Buff_Bank
	clrf	RX485_ParseFlags
;
RS485_Parse_Byte	CALL	GetSer485In
	BTFSS	Param78,0	;Got a byte?
	RETURN		; No
;
	MOVWF	Param78	;Data Byte RXd
	movlw	RX485Timeout	;Reset time-out timer
	movwf	Timer2Lo
; Command inturpreter
;
;
; Destination Address
RS485_Parse_DALo	btfsc	RS485DestAddLoRXd
	bra	RS485_Parse_DALo_End
	bsf	RX485DstIsMe
	movlb	0
	movf	RS485_Address,W
	movlb	Ser485_Buff_Bank
	subwf	Param78,W
	SKPZ
	bcf	RX485DstIsMe
	bsf	RS485DestAddLoRXd
	movf	Param78,W
	movwf	RX485_DstAdd
	xorwf	RX485_CSUM,F
	bra	RS485_Parse_Rtn
RS485_Parse_DALo_End:
;
; Source Address
RS485_Parse_SALo	movlb	Ser485_Buff_Bank
	btfsc	RS485SourceAddLoRXd
	bra	RS485_Parse_SALo_end
;
	clrf	RX485_CSUM
	clrf	RX485_DataCount
	bsf	RX485SrcIsMaster
	movlb	0
	movf	RS485_MasterAddr,W
	movlb	Ser485_Buff_Bank
	subwf	Param78,W
	SKPZ
	bcf	RX485SrcIsMaster
	bsf	RS485SourceAddLoRXd
	movf	Param78,W
	movwf	RX485_SrcAdd
	xorwf	RX485_CSUM,F
RS485_Parse_Rtn	movlb	0x00	;bank 0
	return
RS485_Parse_SALo_end:
;
;
RS485_Parse_Data	btfsc	RS485AllDataRXd
	bra	RS485_Parse_Data_end
	movf	RX485_DataCount,W
	addlw	low RX485_TempData
	movwf	FSR0L
	movlw	high RX485_TempData
	movwf	FSR0H
	movf	Param78,W
	xorwf	RX485_CSUM,F
	movwf	INDF0
	incf	RX485_DataCount,F
	movlw	RP_DataBytes
	subwf	RX485_DataCount,W
	SKPZ
	bra	RS485_Parse_Rtn
	bsf	RS485AllDataRXd
;
	if UseRS485Chksum
	bra	RS485_Parse_Rtn
	endif
RS485_Parse_Data_end
;
	if UseRS485Chksum
; checksum
	movf	Param78,W
	subwf	RX485_CSUM,W
	SKPZ		;Checksum matches?
	bra	RS485_Parse_Error	; No
;
	endif
;
	movlw	low RX485_TempData
	movwf	FSR0L
	movlw	high RX485_TempData
	movwf	FSR0H
;
	movlw	low RX485_Data
	movwf	FSR1L
	movlw	high RX485_Data
	movwf	FSR1H
;
	movlw	RP_DataBytes
	movwf	Param78
RS485_Parse_L1	moviw	FSR0++
	movwi	FSR1++
	decfsz	Param78,F
	bra	RS485_Parse_L1
;
	bsf	RX485DataIsNew	;good packet rx'd
	bsf	RX485DataValidFlag
	btfss	RX485SrcIsMaster	;from master?
	bcf	RX485DataValidFlag	; no
	btfss	RX485DstIsMe	;to me?
	bcf	RX485DataValidFlag	; no
;
	clrf	RX485_ParseFlags
	movlb	0x00
	return
;
;
; We didn't get the sync byte we were expecting so restart the parser
RS485_Parse_Error	CLRF	RX485_ParseFlags
	goto	FlushInBuff
;
;=========================================================================================
; Clear the TX data, unused bytes are sent as 0x00
; Entry: none
; Exit: Bank 0
; Ram used: Param78, FSR1
; Calls: none
RS485_ClearTXData	movlw	1	;bank 1
	movlw	RP_DataBytes
	movwf	Param78
	movlw	low TX_Data
	movwf	FSR1L
	movlw	high TX_Data
	movwf	FSR1H
	movlw	0x00
RS485_ClearTXData_L1	movwi	FSR1++
	decfsz	Param78,F
	bra	RS485_ClearTXData_L1
	movlb	0x00
	return
;
;=========================================================================================
; Send a data packet
;
;Entry: TX_Data = Data to send 
;Exit: TX_Data is cleared
;Ram used: Param78,Param79,Param7A,FSR0,FSR1
;Calls: (1+0) StoreSer485Out
;
RS485_Send:
;
	movlb	0x00	;bank 0
	clrf	Param7A	;CSum
	movf	RS485_Address,W	;source address is Me
	xorwf	Param7A,F
	call	StoreSer485Out
;
	if RP_LongAddr
	movf	RS485_Address+1,W	;source address is Me
	xorwf	Param7A,F
	call	StoreSer485Out
	endif
;
	movf	RS485_MasterAddr,W	;destination address
	xorwf	Param7A,F
	call	StoreSer485Out
;
	if RP_LongAddr
	movf	RS485_MasterAddr+1,W	;destination address
	xorwf	Param7A,F
	call	StoreSer485Out
	endif
;
	movlb	Ser485_Buff_Bank
	movlw	RP_DataBytes
	movwf	Param79
	movlw	low TX_Data
	movwf	FSR1L
	movlw	high TX_Data
	movwf	FSR1H
;
RS485_Send_L1	moviw	FSR1++
	xorwf	Param7A,F
	call	StoreSer485Out
	decfsz	Param79,F
	bra	RS485_Send_L1
;
	if UseRS485Chksum
	movf	Param7A,W
	call	StoreSer485Out
	endif
;
	goto	RS485_ClearTXData
;
;=========================================================================================
;






















