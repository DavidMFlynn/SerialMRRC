;====================================================================================================
;
;    Filename:      Q_EncoderISR.inc
;    Created:       12/8/2019
;    File Version:  1.0.2d1 3/8/2020
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;
; This file contains only the ISR portion of the Quadrature Encoder Interface.
;
;    History:
;
; 1.0.2d1 3/8/2020     Updated for 'F18854
; 1.0.1d2 12/8/2019    Updated for 'f18346
; 1.0d2  09/19/2018    Working A and B
; 1.0d1  01/22/2014	First Code
;
;====================================================================================================
;====================================================================================================
;
ISR_Q_Enc	movlb	QEnc_Bank              ;bank 5
	movf                   IOCBF,W
	SKPNZ                  	;Our interrupt?
	bra	ISR_Q_Enc_End	; No
;
                       xorlw                  0xFF
 	ANDWF	IOCBF,F	;Clear interrupt flag(s)
	XORLW	0xFF
	MOVWF	EdgeFlags	;Save edges detected                      
;
                       movlb                  0                      ;bank 0
	MOVF	Q_Enc_Port,W
	andlw	QEnc_BitMask
;
	movlb	QEnc_Bank              ;bank 5
	movwf	BitsTemp	;Save current state
	swapf	BitsTemp,F	;move A and B to bits 0 and 1
	lsrf	BitsTemp,F
	lsrf	BitsTemp,F
;
;
; x4 Encoding
ISR_Q_Enc_x4	BTFSC	EdgeFlags,Q_Enc_A_Bit	;Has Ch-A changed?
	bra	ISR_Q_Enc_Test	; Yes
	BTFSS	EdgeFlags,Q_Enc_B_Bit	;Has Ch-B changed?
	bra	ISR_Q_Enc_Update	; No
;
ISR_Q_Enc_Test	movf	BitsTemp,W
	subwf	QEnc_OldValue,W
	SKPNZ		;Any change?
	bra	ISR_Q_Enc_Update	; No
;
	movf	BitsTemp,W
	subwf	QEnc_NextValue,W
	SKPNZ		;Moved forward?
	bra	ISR_Q_Enc_Up	; Yes
;
	movf	BitsTemp,W
	subwf	QEnc_PrevValue,W
	SKPZ		;Moved reverse?
	bra	ISR_Q_Enc_Update	; No
;
ISR_Q_Enc_Down	MOVLW	0x01
	SUBWF	QEnc_Counter,F
	CLRW
	SUBWFB	QEnc_Counter+1,F
	SUBWFB	QEnc_Counter+2,F
	SUBWFB	QEnc_Counter+3,F
	BSF	QEnc_Counted
	bra	ISR_Q_Enc_Update	
;
ISR_Q_Enc_Up	MOVLW	0x01
	ADDWF	QEnc_Counter,F
	CLRW
	ADDWFC	QEnc_Counter+1,F
	ADDWFC	QEnc_Counter+2,F
	ADDWFC	QEnc_Counter+3,F
	BSF	QEnc_Counted
;
ISR_Q_Enc_Update	movf	BitsTemp,W
	movwf	QEnc_OldValue
;
	movf	BitsTemp,W
	call	ISR_Q_GetNext	
	movwf	QEnc_NextValue
;
	movf	BitsTemp,W
	call	ISR_Q_GetPrev
	movwf	QEnc_PrevValue
	bra	ISR_Q_Enc_CopyVal
;
ISR_Q_GetPrev	brw
	retlw	0x02
	retlw	0x00
	retlw	0x03
	retlw	0x01
;
ISR_Q_GetNext	brw
	retlw	0x01
	retlw	0x03
	retlw	0x00
	retlw	0x02
;
ISR_Q_Enc_CopyVal	btfss	QEnc_Counted
	bra	ISR_Q_Enc_Index
	BTFSC	QEnc_Semaphore	;OK to update counter?
	bra	ISR_Q_Enc_Index	; No
	bcf	QEnc_Counted
	MOVF	QEnc_Counter,W
	MOVWF	QEnc_Position
	MOVF	QEnc_Counter+1,W
	MOVWF	QEnc_Position+1
	MOVF	QEnc_Counter+2,W
	MOVWF	QEnc_Position+2
	MOVF	QEnc_Counter+3,W
	MOVWF	QEnc_Position+3
;
ISR_Q_Enc_NoCount:
ISR_Q_Enc_Index:
;	BTFSC	EdgeFlags,Q_Enc_Z_Bit	;Any Index edge was seen?
;	BSF	QEnc_Z	; Yes
;	BTFSC	BitsTemp,Q_Enc_Z_Bit	;Index is still High?
;	BSF	QEnc_Z	; Yes
ISR_Q_Enc_End:
	movlb	0
;
;
;==================================================================================================
