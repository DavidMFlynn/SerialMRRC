;=========================================================================================
;
;  Filename: RS485_Parse.H
;  Created: 7/26/2026
;  Revision: 1.0.0   7/26/2026
;
;   Author:  David M. Flynn
;   Company: DMF-Enterprises
;   E-Mail: dflynn.oxfordvue@gmail.com
;   License: MIT License
;=========================================================================================
; Notes:
;  RAM locations used by RS485_Parse.inc
;=========================================================================================
;  Bank4 Ram 220h-26Fh 80 Bytes, RS-485 Packet Serial
;
; Bank4 variables used by RS485_Parse:
#Define	Ser485_Buff_Bank	4
;
	cblock	0x220
	RX485_ParseFlags
	RX485_Flags
	RX485_DataCount
	RX485_CSUM
	RX485_SrcAdd:RP485_AddressBytes
	RX485_DstAdd:RP485_AddressBytes
	RX485_TempData:RP485_DataBytes	;Incoming data is stored here
	RX485_Data:RP485_DataBytes
;
	TX485_SrcAdd:RP485_AddressBytes
	TX485_DstAdd:RP485_AddressBytes
	TX485_Data:RP485_DataBytes
	TX485_Csum
;
	Ser485_In_Bytes		;Bytes in Ser_In_Buff
	Ser485_Out_Bytes		;Bytes in Ser_Out_Buff
	Ser485_In_InPtr
	Ser485_In_OutPtr
	Ser485_Out_InPtr
	Ser485_Out_OutPtr
	Ser485_In_Buff:20
	Ser485_Out_Buff:20
	endc
;
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
#Define	RX485DstIsMe	RX485_Flags,2	;Data for me!?
#Define	RX485SrcIsMe	RX485_Flags,3	;I just sent this?
#Define	RX485RXDataEquTXData	RX485_Flags,4	;RX485SrcIsMe && TX Csum=RX Csum
;
;=========================================================================================