;=========================================================================================
;
;   Filename:  RS485_Parse.inc
;   Created:   4/5/2025
;   Revision:  1.0.2  7/26/2026
;
;   Author:  David M. Flynn
;   Company: DMF-Enterprises
;   E-Mail: dflynn.oxfordvue@gmail.com
;   License: MIT License
;
;=========================================================================================
; Routines:
;
; RS485_Parse	Call from main loop
; RS485_ClearTXData	Clear the TX data, FSR1
; RS485_Send	Send the data at TX_data, FSR0, FSR1
; RS485_Init	Setup serial I/O on RB7/TX1, RB5/RX1, RB4/nRE, RB6/DE
;
;=========================================================================================
; Notes:
;
; History:
; 1.0.2  7/26/2026	Code cleanup. Moved header stuff to RS485_Parse.H
; 1.0.1  7/25/2026	Moved RS485_Init into this file.
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
	btfss	Param78,0	;Got a byte?
	return		; No
;
	movwf	Param78	;Data Byte RXd
	movlw	RX485Timeout	;Reset time-out timer
	movwf	Timer2Lo
; Command inturpreter
	if UseRS485SyncBytes
;
	movlb	Ser485_Buff_Bank	;bank 4
	movf	RX485_ParseFlags,F
	SKPZ		;Beginning new packet?
	bra	RS485_Parse_Sync2	; No
;
	movlw	kRS485SyncByteValue
	subwf	Param78,W
	SKPNZ		;Sync byte OK?
	bsf	RS485SyncByte1RXd	; Yes
	bra	RS485_Parse_Rtn
;
RS485_Parse_Sync2	btfsc	RS485SyncByte2RXd	;Second sync byte rec'd?
	bra	RS485_Parse_SALo	; Yes
;
	movlw	kRS485SyncByteValue
	subwf	Param78,W
	SKPZ		;Got the second sync byte?
	goto	RS485_Parse_Error	; No, need 2
	bsf	RS485SyncByte2RXd
	bra	RS485_Parse_Rtn
;
	endif
;
;
; Source Address
RS485_Parse_SALo	btfsc	RS485SourceAddrLoRXd	;Looking for the source address?
	bra	RS485_Parse_SALo_end	; No
;My data is echoed back
	movlb	0	;Bank 0
	movf	RS485_Address,W
	movlb	Ser485_Buff_Bank	;Bank 4
	bsf	RX485SrcIsMe	;default to it being my message
	subwf	Param78,W
	SKPZ		;RS485_Address=RXd byte?
	bcf	RX485SrcIsMe	; No, it's not for me.
;
; Save the source address
	clrf	RX485_DataCount
	movf	Param78,W
	movwf	RX485_SrcAdd	;Save the source address
	movwf	RX485_CSUM	;Start a checksum
	bsf	RS485SourceAddrLoRXd
;
RS485_Parse_Rtn	movlb	0	;bank 0
	return
RS485_Parse_SALo_end:
;
	if RP485_LongAddr
;
; MSB of Source Address
RS485_Parse_SAHi	btfsc	RS485SourceAddrHiRXd	;Have a high source address byte?
	bra	RS485_Parse_SAHi_end	; Yes
;
	movlb	0	;Bank 0
	movf	RS485_Address+1,W
	movlb	Ser485_Buff_Bank	;Bank 4
	subwf	Param78,W
	SKPZ		;RS485_Address=RXd byte?
	bcf	RX485SrcIsMe	; No, it's not for me.
;
	movf	Param78,W
	movwf	RX485_SrcAdd+1	;Save for later
	addwf	RX485_CSUM,F
	bsf	RS485SourceAddrHiRXd
	bra	RS485_Parse_Rtn
RS485_Parse_SAHi_end:
	endif
;
; Destination Address
RS485_Parse_DALo	bsf	RX485RXDataEquTXData
	btfss	RX485SrcIsMe
	bcf	RX485RXDataEquTXData
;
	btfsc	RS485DestAddrLoRXd	;Have a source address low byte?
	bra	RS485_Parse_DALo_End	; Yes
;
	movlb	0	;Bank 0
	movf	RS485_Address,W
	movlb	Ser485_Buff_Bank	;Bank 4
	bsf	RX485DstIsMe	;default to it being my message
	subwf	Param78,W
	SKPZ		;RS485_Address=RXd byte?
	bcf	RX485DstIsMe	; No, it's not for me.
;
	movf	Param78,W
	movwf	RX485_DstAdd	;Save RXd byte as dest addr
	addwf	RX485_CSUM,F
	bsf	RS485DestAddrLoRXd
	bra	RS485_Parse_Rtn
RS485_Parse_DALo_End:
;
	if RP485_LongAddr
; MSB of Destination Address
RS485_Parse_DAHi	btfsc	RS485DestAddrHiRXd	;Have a source address low byte?
	bra	RS485_Parse_DAHi_End	; Yes
;
	movlb	0	;Bank 0
	movf	RS485_Address+1,W
	movlb	Ser485_Buff_Bank	;Bank 4
	subwf	Param78,W
	SKPZ		;RS485_Address=RXd byte?
	bcf	RX485DstIsMe	; No, it's not for me.
;
	movf	Param78,W
	movwf	RX485_DstAdd+1	;Save RXd byte as dest addr
	addwf	RX485_CSUM,F
	bsf	RS485DestAddrHiRXd
	bra	RS485_Parse_Rtn
RS485_Parse_DAHi_End:
;
	endif
;
; Receive the data bytes, store them in RX485_TempData
RS485_Parse_Data	btfsc	RS485AllDataRXd	;Got all data bytes?
	bra	RS485_Parse_Data_end	; Yes
;FSR0 is data ptr
	movf	RX485_DataCount,W
	addlw	low RX485_TempData
	movwf	FSR0L
	movlw	high RX485_TempData
	movwf	FSR0H
;
	movf	Param78,W
	addwf	RX485_CSUM,F	;Update ckecksum
	movwf	INDF0	;Save the data byte
	incf	RX485_DataCount,F
	movlw	RP485_DataBytes
	subwf	RX485_DataCount,W
	SKPZ		;All data bytes RX'd?
	bra	RS485_Parse_Rtn	; No
	bsf	RS485AllDataRXd	; Yes
;
	if UseRS485Chksum
	bra	RS485_Parse_Rtn	;Wait for a checksum
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
;Copy data from RX485_TempData to RX485_Data
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
	movlw	RP485_DataBytes
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
	movlb	0	;Bank 0
	return
;
;
; We didn't get the sync byte we were expecting so restart the parser
RS485_Parse_Error	CLRF	RX485_ParseFlags
	goto	Flush485InBuff
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
;
	clrf	TX485_Csum
	movlb	0	;Bank 0
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
	clrf	Param7A	;CSum=0
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
	movlb	Ser485_Buff_Bank	;bank 4
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
	movlb	Ser485_Buff_Bank	;bank 4
	movwf	TX485_Csum	;Save ckecksum for later.
	call	StoreSer485Out
	endif
;
	goto	RS485_ClearTXData
;
	if useRS232
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
	return
;
	endif
;=========================================================================================
; Setup serial I/O on RB7/TX1, RB5/RX1, RB4/nRE, RB6/DE
;
;Note:using dfault values
; Configure pins
RS485_Init	movlb	RX1DTPPS	;bank 29
	movlw	0x0D	;RB5, default
	movwf	RX1DTPPS
	movlw	0x0F	;RB7, default
	movwf	TX1CKPPS
;
	movlb	RB7PPS                 ;bank 30, Output pin routing
	movlw	0x0F	;TX1/CK1  signal
	movwf	RB7PPS
; Set baud rate
	movlb	BAUD1CON	;bank 2
	movlw	BAUD1CON_Value
	movwf	BAUD1CON
	MOVLW	low RS485BaudRate
	MOVWF	SP1BRGL
	MOVLW	high RS485BaudRate
	MOVWF	SP1BRGH
;
	bsf	RC1STA,RC1STA_SPEN	;Enable Serial port
	bsf	RC1STA,RC1STA_CREN	;Continuous Receive
;
	bcf	TX1STA,TX1STA_SYNC	;Asynchronous
	bsf	TX1STA,TX1STA_BRGH	;High Speed
	bcf	TX1STA,TX1STA_TXEN	;Transmitter Off
;
; Enable Receive
	movlb	0
	bcf	RS485DE	;DriverEnable=0
	bcf	RS485nRE	;nRecieverEnable=0
	bcf	RS485TXActive
;
;
	movlb	PIE3	; bank 14
	BSF	PIE3,RC1IE	; Serial Receive interupt
	movlb	0
	bsf	RS485DataSentFlag
	return;
;
;=========================================================================================
;






















