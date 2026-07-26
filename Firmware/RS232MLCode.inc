;-----------------------
; Handle serial from Mac on USB
; If there are byte in the input buffer send them to the parser.
	call	GetSerInBytes
	SKPZ		;Any data?
	CALL	RS232_Parse	; yes
;
;---------------------
; Handle Serial Communications
;
	if useRS232PacketCmds
;
	movlb	RX_Flags               ;bank 1
	btfss	RXDataIsNew
	bra	RS232ML_1
	mLongCall	HandleRXData
RS232ML_1:
	endif
;
	movlb	PIR3                   ;bank 14
	BTFSC	PIR3,TX2IF	;TX done?
	CALL	TX_TheByte	; Yes
;
; move any serial data received into the 32 byte input buffer
                       movlb                  0                      ;bank 0
	BTFSS	DataReceivedFlag
	BRA	RS232ML_Ser_Out
	MOVF	RXByte,W
	BCF	DataReceivedFlag
	CALL	StoreSerIn
;
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
RS232ML_Ser_Out	BTFSS	DataSentFlag
	BRA	RS232ML_Ser_End
	CALL	GetSerOut
	BTFSS	Param78,0
	BRA	RS232ML_Ser_End
	MOVWF	TXByte
	BCF	DataSentFlag
RS232ML_Ser_End:
;