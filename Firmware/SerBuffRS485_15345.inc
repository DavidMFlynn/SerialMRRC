;=========================================================================================
;
;    Filename:      SerBuffRS485_15345.inc
;    Created:  4/5/2025                               
;    Revision:  1.0d1      4/5/2025
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;
;=========================================================================================
;=========================================================================================
; Notes:
;  Handles RS-485 packet serial I/O on TX1/RX1
;
; History:
; 1.0d1      4/5/2025	Converted from SerBuff_18854 Rev 1.0
;
;=========================================================================================
; Bank0 Variables:
;
;	cblock	0x20
;	TX485Byte		;Next byte to send
;	RX485Byte		;Last byte received
;	Ser485Flags
;	endc
;
;#define	RS485TXActive	Ser485Flags,0
;#Define	RS485DataReceivedFlag	Ser485Flags,1
;#Define	RS485DataSentFlag	Ser485Flags,2
;#Define	RS485RX_Timeout	Ser485Flags,3	;cleared by host read
;#Define	RS485RX_IsAddress	Ser485Flags,4	;Data in RX485Byte is and address
;#Define	RS485DestAddLoRXd	Ser485Flags,5
;#Define	RS485SourceAddLoRXd	Ser485Flags,6
;
; Bank4 Variables:
;#Define	Ser485_Buff_Bank	4
;	cblock	0x120
;	Ser485_In_Bytes		;Bytes in Ser_In_Buff
;	Ser485_Out_Bytes		;Bytes in Ser_Out_Buff
;	Ser485_In_InPtr
;	Ser485_In_OutPtr
;	Ser485_Out_InPtr
;	Ser485_Out_OutPtr
;	Ser485_In_Buff:20
;	Ser485_Out_Buff:20
;	endc
;
;=========================================================================================
; Routines	(Bank In/Bank Out)
;
; TX_The485Byte	(X/0) Send one byte to UART
; RX_The485Byte	(X/0) Receive one byte from UART
; StoreSer485In	(X/0) Put the byte in W into the serial input buffer, FSR0
; GetSer485In	(X/0) Get a byte from the serial input buffer, FSR0
; Flush485InBuff	(X/0) Flush the serial incoming buffer.
; GetSer485InBytes	(X/0) Get the number of bytes in the serial input buffer
; GetSer485OutBytes	(X/0) Get the number of bytes in the serial ouput buffer
; StoreSer485Out	(X/0) Put the byte in W into the serial output buffer, FSR0
; POP_Ser485Out	(X/0) Remove the last char stored in the output buffer
; GetSer485Out	(X/0) Get a byte from the serial Output buffer, FSR0
;	       W=Byte, Param78:0=ValidFlag
; RS485ActivateRX	(X/0) If TX is Idle switch mode to receive, RS485TXActive=0
; RS485ActivateTX	(X/0) RS485TXActive=1
;=========================================================================================
;=========================================================================================
; *** Put in main loop ***
;
; Handle RS-485 Serial Communications
;	btfss	RS485TXActive	;In transmit mode?
;	bra	RS485NoTX	; No
;	movlb	PIR3
;	BTFSS	PIR3,TX1IF	;TX done?
;	BRA	RS485TX_End
;	CALL	TX_The485Byte	; Yes
;RS485TX_End:
;
; move any serial data received into the 32 byte input buffer
;	BTFSS	RS485DataReceivedFlag
;	BRA	ML_Ser485_Out
;	MOVF	RX485Byte,W
;	BCF	RS485DataReceivedFlag
;	CALL	StoreSer485In
;
;=========================================================================================
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
;ML_Ser485_Out	BTFSS	RS485DataSentFlag
;	BRA	ML_Ser485_End
;	CALL	GetSer485Out
;	BTFSS	Param78,0
;	BRA	ML_Ser485_End
;	MOVWF	TX485Byte
;	BCF	RS485DataSentFlag
;ML_Ser485_End
;
;=========================================================================================
; *** Put in Init section ***
;BAUDCON_Value	EQU	b'00001000'
;TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, High speed
;RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
;; 8MHz clock low speed (BRGH=0,BRG16=1)
;;Baud_300	EQU	d'1666'	;0.299, -0.02%
;;Baud_1200	EQU	d'416'	;1.199, -0.08%
;;Baud_2400	EQU	d'207'	;2.404, +0.16%
;;Baud_9600	EQU	d'51'	;9.615, +0.16%
;; 8MHz clock high speed (BRGH=1,BRG16=1)
;Baud_300	EQU	d'6666'	;0.299, -0.02%
;Baud_1200	EQU	d'1666'	;1.199, -0.08%
;Baud_2400	EQU	d'832'	;2.404, +0.16%
;Baud_9600	EQU	d'207'	;9.615, +0.16%
;Baud_19_2	EQU	d'103'	;19.23k, +0.16
;Baud_57_6	EQU	d'34'	;57.14k, -0.79
;BaudRate	EQU	Baud_9600
;-------------
;
; setup serial I/O
;	movlb	TX1STA	; bank 3
;	MOVLW	TX1STA_Value
;	MOVWF	TX1STA
;	MOVLW	low BaudRate
;	MOVWF	SP1BRGL
;	MOVLW	high BaudRate
;	MOVWF	SP1BRGH
;	MOVLW	RC1STA_Value
;	MOVWF	RC1STA
;
;	movlb	PIE3	; bank ?
;	BSF	PIE3,RC1IE	; Serial Receive interupt
;	movlb	0x00	; bank 0
;	BSF	INTCON,PEIE	;Peripheral IRQ Enable
;	BSF	INTCON,GIE
;
;=========================================================================================
; *** Put in ISR ***
;-----------------------------------------------------------------------------------------
;AUSART Serial ISR
;
;IRQ_RS_485	BTFSS	PIR3,RC1IF	;RX has a byte?
;	BRA	IRQ_RS_485_End
;	CALL	RX_The485Byte
;
;IRQ_RS_485_End
;
;=========================================================================================
; ***************************************************************************************
;=========================================================================================
Ser485_Out	movlb	0
	BTFSS	RS485DataSentFlag
	bra	Ser485_End
	CALL	GetSer485Out
	BTFSS	Param78,0
	bra	Ser485_End
	MOVWF	TX485Byte
	BCF	RS485DataSentFlag
Ser485_End	return
;
;=========================================================================================
; If TX is Idle switch mode to receive
;
RS485ActivateRX	movlb	0
	btfss	RS485TXActive	;TX mode?
	bra	RS485ActivateRX_1	; No
; finish TX if there are bytes in the buffer
	call	Ser485_Out
	btfss	RS485DataSentFlag	;Byte is waiting?
	return		; Yes
	movlb	PIR3
	btfss	PIR3,TX1IF	;TX done?
	bra	RS485ActivateRX_End	; No
;
RS485ActivateRX_1	movlb	TX1STA
	bcf	TX1STA,TX1STA_TXEN	;Transmitter Off
;
	bsf	RC1STA,RC1STA_SPEN	;Enable Serial port
	bsf	RC1STA,RC1STA_RX9	;Enable 9 bit receive
	bsf	RC1STA,RC1STA_CREN	;Continuous Receive
	bsf	RC1STA,RC1STA_ADDEN	;Address detect on
;
	movlb	0	;bank 0
	bcf	RS485TXActive
	bcf	RS485DE	;DriverEnable=0
	bcf	RS485nRE	;nRecieverEnable=0
RS485ActivateRX_End	movlb	0	
	return
;
;=========================================================================================
;
;
RS485ActivateTX	movlb	0
RS485ActivateTX_End	movlb	0	
	return
;
;=========================================================================================
; Send one byte
; Entry: 
; Exit: Bank0
;
;
TX_The485Byte	movlb                  0                      ;bank 0
                       BTFSC	RS485DataSentFlag
	RETURN
	MOVF	TX485Byte,W	
	movlb	TX1REG                  ;bank ?
	MOVWF	TX1REG
	MOVLB	0                      ;bank 0
	BSF	RS485DataSentFlag
	RETURN
;
;=========================================================================================
;
RX_The485Byte	movlb	RC1STA	; bank ?
	MOVF	RC1STA,W
	ANDLW	0x06
	SKPZ		;Frame or Overrun Err?
	bra	RX_The485Byte_Err	; yes
; Is Address?
	btfss	RC1STA,RC1STA_ADDEN
	bra	RX_The485Byte_Data
	btfss	RC1STA,RC1STA_RX9D	;1=Address, 0=Data, Is an Address?
	bra	RX_The485Byte_Data	; No
; An Address was received
	bcf	RC1STA,RC1STA_ADDEN	;Address detect off, receive all data
	movlb	0
	bsf	RS485RX_IsAddress
	bra	RX_The485Byte_1
;
RX_The485Byte_Data	movlb	0
	bcf	RS485RX_IsAddress
;
RX_The485Byte_1	movlb	RC1REG
	movf	RC1REG,W
	movlb	0	; bank 0
	MOVWF	RX485Byte
	bsf	RS485DataReceivedFlag
	return
;
RX_The485Byte_Err	MOVF	RC1REG,W	;bad byte so drop it
	BCF	RC1STA,RC1STA_CREN     ;disable receiver
	BCF	RC1STA,RC1STA_FERR
	BCF	RC1STA,RC1STA_OERR
	BSF	RC1STA,RC1STA_CREN     ;enable receiver
	movlb	0                      ;bank 0
	clrf	RX485Byte
	return
;
;=========================================================================================
;=========================================================================================
; Put the byte in W into the serial input buffer
;
; Entry: W = byte to store
; Exit: none
; Ram Used: Param78, FSR
;
StoreSer485In	MOVWF	Param78
	MOVLB	Ser485_Buff_Bank
	INCF	Ser485_In_Bytes,F
;	
	MOVF	Ser485_In_InPtr,W
	ANDLW	0x1F
	INCF	Ser485_In_InPtr,F
	ADDLW	low Ser485_In_Buff
	MOVWF	FSR0L
	MOVLW	high Ser485_In_Buff
	MOVWF	FSR0H
	MOVF	Param78,W
	MOVWF	INDF0
	MOVLB	0x00
	RETURN
;
;=========================================================================================
; Get a byte from the serial input buffer
;
; Entry: none
; Exit: W=byte from buffer, Param78,0=valid
;
GetSer485In	MOVLB	Ser485_Buff_Bank
	BCF	Param78,0	;no data
	MOVF	Ser485_In_Bytes,F
	SKPNZ
	bra	GetSer485In_End
	BSF	Param78,0
	DECF	Ser485_In_Bytes,F
	MOVF	Ser485_In_OutPtr,W
	ANDLW	0x1F
	INCF	Ser485_In_OutPtr,F
	ADDLW	low Ser485_In_Buff
	MOVWF	FSR0L
	MOVLW	high Ser485_In_Buff
	MOVWF	FSR0H
	MOVF	INDF0,W
GetSer485In_End	movlb	0
	return
;
;=========================================================================================
; Flush the serial incoming buffer
; Entry: none
; Exit: bank 0
;
Flush485InBuff	movlb	Ser485_Buff_Bank
	clrf	Ser485_In_Bytes
	clrf	Ser485_In_OutPtr
	clrf	Ser485_In_InPtr
	movlb	0
	return
;
;=========================================================================================
; Get the number of bytes in the serial input buffer
;
; Entry: none
; Exit: W=bytes in buffer, Z
;
GetSer485InBytes	MOVLB	Ser485_Buff_Bank
	MOVF	Ser485_In_Bytes,W
	movlb	0
	return
;
;=========================================================================================
; Get the number of bytes in the serial ouput buffer
;
; Entry: none
; Exit: W=bytes in buffer, Z
;
GetSer485OutBytes	MOVLB	Ser485_Buff_Bank
	MOVF	Ser485_Out_Bytes,W
	movlb	0
	return
;
;=========================================================================================
; Put the byte in W into the serial output buffer
;
; Entry: W = byte to store
; Exit: Param78 & W = byte stored, Bank 0
; Ram used: Param78, FSR0
; Calls: (0) none
;
StoreSer485Out	MOVWF	Param78
	MOVLB	Ser485_Buff_Bank
	INCF	Ser485_Out_Bytes,F
;	
	MOVF	Ser485_Out_InPtr,W
	ANDLW	0x1F
	INCF	Ser485_Out_InPtr,F
	ADDLW	low Ser485_Out_Buff
	MOVWF	FSR0L
	MOVLW	high Ser485_In_Buff
	MOVWF	FSR0H
	MOVF	Param78,W
	MOVWF	INDF0
	movlb	0
	return
;
;=========================================================================================
; Remove the last char stored in the output buffer
;
POP_Ser485Out	MOVLB	Ser485_Buff_Bank
	MOVF	Ser485_Out_Bytes,F
	SKPNZ
	bra	POP_Ser485Out_End
	DECF	Ser485_Out_Bytes,F
	DECF	Ser485_Out_InPtr,F
POP_Ser485Out_End	movlb	0
	return
;
;=========================================================================================
; Get a byte from the serial Output buffer
;
; Entry: none
; Exit: W=byte from buffer, Param78,0=valid
;
GetSer485Out	MOVLB	Ser485_Buff_Bank
	BCF	Param78,0	;no data
	MOVF	Ser485_Out_Bytes,F
	SKPNZ
	bra	GetSer485Out_End
	BSF	Param78,0
	DECF	Ser485_Out_Bytes,F
	MOVF	Ser485_Out_OutPtr,W
	ANDLW	0x1F
	INCF	Ser485_Out_OutPtr,F
	ADDLW	low Ser485_Out_Buff
	MOVWF	FSR0L
	MOVLW	high Ser485_In_Buff
	MOVWF	FSR0H
	MOVF	INDF0,W
GetSer485Out_End	movlb	0
	return
;
;=========================================================================================
;


















