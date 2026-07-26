; If there are byte in the input buffer send them to the parser.
	call	GetSer485InBytes
	SKPZ		;Any data?
	CALL	RS485_Parse	; yes
;
;---------------------
; Handle Serial Communications
	if useRS485PacketCmds
;
	movlb	RX485_Flags               ;bank 4
	btfss	RX485DataIsNew
	bra	RS485ML_1
	mLongCall	HandleRX485Data
RS485ML_1:
	endif
;
; Handle RS-485 Serial Communications
;
	CALL	TX_The485Byte
;
; move any serial data received into the 32 byte input buffer
	BTFSS	RS485DataReceivedFlag
	bra	ML_Ser485_End
	MOVF	RX485Byte,W
	BCF	RS485DataReceivedFlag
	CALL	StoreSer485In
;
ML_Ser485_End:
;