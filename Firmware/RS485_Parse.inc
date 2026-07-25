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
#Define	RS485SyncByte1RXd	RX485_ParseFlags,0
#Define	RS485SyncByte2RXd	RX485_ParseFlags,1
#Define	RS485SourceAddrLoRXd	RX485_ParseFlags,2
#Define	RS485SourceAddrHiRXd	RX485_ParseFlags,3
#Define	RS485DestAddrLoRXd	RX485_ParseFlags,4
#Define	RS485DestAddrHiRXd	RX485_ParseFlags,5
#Define	RS485AllDataRXd	RX485_ParseFlags,6
;
;RX485_Flags Bits
#Define	RX485DataValidFlag	RX485_Flags,0
#Define	RX485DataIsNew	RX485_Flags,1
#Define	RX485DstIsMe	RX485_Flags,2
;
;
; *** put in main loop
;	CALL	RS485_Parse
; if RX485DataValidFlag is set data may be used
;
;=========================================================================================
; Parse any bytes received by the serial port
;
; Receive 0xDD,0xDD,SA0,DA0,Cmd,Data0,Data1,Data2,CS
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
	if UseRS485SyncBytes
;
	movlb	Ser485_Buff_Bank	;bank 4
	MOVF	RX485_ParseFlags,F
	SKPZ		;Beginning new packet?
	BRA	RS485_Parse_Sync2	; No
	MOVLW	kRS485SyncByteValue
	SUBWF	Param78,W
	SKPNZ		;Sync byte OK?
	BSF	RS485SyncByte1RXd	; Yes
	bra	RS485_Parse_Rtn
;
RS485_Parse_Sync2	btfsc	RS485SyncByte2RXd
	bra	RS485_Parse_SALo
	movlw	kRS485SyncByteValue
	subwf	Param78,W
	SKPZ
	goto	RS485_Parse_Error
	bsf	RS485SyncByte2RXd
	bra	RS485_Parse_Rtn
;
	endif
;
;
; Source Address
RS485_Parse_SALo	movlb	0
	btfsc	RS485SourceAddLoRXd
	bra	RS485_Parse_SALo_end
;
	bsf	RS485SourceAddLoRXd
	movlb	Ser485_Buff_Bank	;bank 4
	clrf	RX485_CSUM
	clrf	RX485_DataCount
	movf	Param78,W
	movwf	RX485_SrcAdd
	addwf	RX485_CSUM,F
RS485_Parse_Rtn	movlb	0	;bank 0
	return
RS485_Parse_SALo_end:
;
; MSB of Source Address
	if RP485_LongAddr
;
RS485_Parse_SAHi	btfsc	RS485SourceAddHiRXd
	bra	RS485_Parse_SAHi_end
;
	bsf	RS485SourceAddHiRXd
	movlb	Ser485_Buff_Bank	;bank 4
	movf	Param78,W
	movwf	RX485_SrcAdd+1	;Save for later
	addwf	RX485_CSUM,F
	bra	RS232_Parse_Rtn
RS485_Parse_SAHi_end:
	endif
;
; Destination Address
RS485_Parse_DALo	movlb	0	;Bank 0
	btfsc	RS485DestAddLoRXd
	bra	RS485_Parse_DALo_End
;
	bsf	RX485DstIsMe	;default to it being my message
	movf	RS485_Address,W
	movlb	Ser485_Buff_Bank	;Bank 4
	subwf	Param78,W
	SKPZ		;RS485_Address=RXd byte?
	bcf	RX485DstIsMe	; No
	bsf	RS485DestAddLoRXd
	movf	Param78,W
	movwf	RX485_DstAdd	;Save RXd byte as dest addr
	addwf	RX485_CSUM,F
	bra	RS485_Parse_Rtn
RS485_Parse_DALo_End:
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
	addwf	RX485_CSUM,F
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
RS485_Parse_Data_end:
;
	if UseRS485Chksum
; checksum
	movlw	0xFF
	xorwf	RX485_CSUM,W
	subwf	Param78,W
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
	btfss	RX485DstIsMe	;to me?
	bcf	RX485DataValidFlag	; no
;
	clrf	RX485_ParseFlags
	movlb	0x00
;tc
;	bsf	LED2_Active
;etc
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
RS485_ClearTXData	movlb	Ser485_Buff_Bank	;bank 4
	clrf	TX485_SrcAdd
	clrf	TX485_DstAdd
;
	if RP485_LongAddr
	clrf	TX485_SrcAdd+1
	clrf	TX485_DstAdd+1
	endif
;
	movlw	RP485_DataBytes
	movwf	Param78
	movlw	low TX485_Data
	movwf	FSR1L
	movlw	high TX485_Data
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
;  from the TX485 buffer (TX485_SrcAdd,TX485_DstAdd,TX485_Data)
;
;Entry: TX_Data = Data to send 
;Exit: TX_Data is cleared
;Ram used: Param78,Param79,Param7A,FSR0,FSR1
;Calls: (1+0) StoreSer485Out
;
RS485_Send:
;
	if UseRS485SyncBytes
;
	movlw	kRS485SyncByteValue
	call	StoreSer485Out
	movlw	kRS485SyncByteValue
	call	StoreSer485Out
;
	endif
;
	clrf	Param7A	;CSum=CSum + data
; Source Address
	movlb	Ser485_Buff_Bank	;bank 4
	movf	TX485_SrcAdd,W	;source address is Me
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
;
	if RP485_LongAddr
	movlb	Ser485_Buff_Bank	;bank 4
	movf	TX485_SrcAdd+1,W	;source address is Me
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
	endif
;
; Destination Address
	movlb	Ser485_Buff_Bank	;bank 4
	movf	TX485_DstAdd,W	;destination address
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
;
	if RP485_LongAddr
	movlb	Ser485_Buff_Bank	;bank 4
	movf	TX485_DstAdd+1,W	;destination address
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
	endif
;
	movlb	Ser485_Buff_Bank
	movlw	RP485_DataBytes
	movwf	Param79
	movlw	low TX485_Data
	movwf	FSR1L
	movlw	high TX485_Data
	movwf	FSR1H
;
RS485_Send_L1	moviw	FSR1++
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
	decfsz	Param79,F
	bra	RS485_Send_L1
;
	if UseRS485Chksum
	movlw	0xFF
	xorwf	Param7A,W	;CSum
	call	StoreSer485Out
	endif
;
	goto	RS485_ClearTXData
;
;=========================================================================================
; Forward a data packet from RS232 (USB)
;  send data from RS232 (RX_SrcAdd, RX_DstAdd, RX_TempData)
;  to the RS485 bus 
;
;Entry: TX_Data = Data to send 
;Exit: TX_Data is cleared
;Ram used: Param78,Param79,Param7A,FSR0,FSR1
;Calls: (1+0) StoreSer485Out
;
RS485_Forward:
	if UseRS485SyncBytes
;
	movlw	kRS485SyncByteValue
	call	StoreSer485Out
	movlw	kRS485SyncByteValue
	call	StoreSer485Out
;
	endif
;
	clrf	Param7A	;CSum
;
	movlb	RX_SrcAdd	;bank 1
	movf	RX_SrcAdd,W	;source address
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
;
	if RP485_LongAddr
	movlb	RX_SrcAdd	;bank 1
	movf	RX_SrcAdd+1,W	;source address high byte
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
	endif
;
	movlb	RX_DstAdd	;bank 1
	movf	RX_DstAdd,W	;destination address
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
;
	if RP_LongAddr
	movlb	RX_DstAdd	;bank 1
	movf	RX_DstAdd+1,W	;destination address
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
	endif
;
	movlw	RP485_DataBytes
	movwf	Param79
	movlw	low RX_TempData
	movwf	FSR1L
	movlw	high RX_TempData
	movwf	FSR1H
;
RS485_Forward_L1	moviw	FSR1++
	addwf	Param7A,F	;CSum=CSum + data
	call	StoreSer485Out
	decfsz	Param79,F
	bra	RS485_Forward_L1
;
	if UseRS485Chksum
	movlw	0xFF
	xorwf	Param7A,W
	call	StoreSer485Out
	endif
;
;tc
;	bsf	LED1_Active
;etc
	return
;
;=========================================================================================
;






















