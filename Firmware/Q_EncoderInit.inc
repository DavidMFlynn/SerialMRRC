;====================================================================================================
;
;    Filename:      Q_EncoderInit.inc
;    Date:          1/22/2014
;    File Version:  1.0.1d1  03/08/2020
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;
; This file contains only the initialization code for the Quadrature Encoder Interface.
;
;    History:
;
; 1.0.1d1  03/08/2020  Updated for 'F18854
; 1.0d1    01/22/2014	First Code
; 1.0d2    09/19/2018  Working A and B
;
;====================================================================================================
;====================================================================================================
;Q_Encoder Initialization Code
;
	movlb	WPUC                   ;bank 30
	BCF	WPUC,Q_Enc_Z_Bit	;No weak pull-up
	BCF	WPUC,Q_Enc_A_Bit
	BCF	WPUC,Q_Enc_B_Bit
;
	movlb	Q_Enc_Tris             ;bank 0
	BSF	Q_Enc_Tris,Q_Enc_Z_Bit	;Inputs
	BSF	Q_Enc_Tris,Q_Enc_A_Bit
	BSF	Q_Enc_Tris,Q_Enc_B_Bit
;
	movlb                  QEnc_Bank              ;bank 5
	BCF	QEnc_Semaphore
;
	movlb	IOCCP                  ;bank 30
	BSF	IOCCP,Q_Enc_A_Bit	;Rising edge of channel-A
	BSF	IOCCP,Q_Enc_B_Bit	;Rising edge of channel-B
	BSF	IOCCN,Q_Enc_A_Bit	;Falling edge of channel-A
	BSF	IOCCN,Q_Enc_B_Bit	;Falling edge of channel-B
;
;	BSF	IOCCN,Q_Enc_Z_Bit	;Falling edge of channel-Z
;	BSF	IOCCP,Q_Enc_Z_Bit	;Rising edge of channel-Z
;
                       movlb                  PIE0
	BSF	PIE0,IOCIE
;
	movlb                  0                      ;bank 0
;
;====================================================================================================
;====================================================================================================
;