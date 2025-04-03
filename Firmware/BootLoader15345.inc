;************************************************
; BootLoader for PIC16F18854
; filename: Bootloader18854.inc
; Created: 3/21/2019
; Revision: 1.0b1 3/26/2020
;************************************************
; History
; 1.0b1 3/26/2020      Seems to work.
; 1.0a2 3/25/2020      Copied from Bootloader1847.inc, updated for F18854
; 1.0a1 4/7/2019	First working version.
; 1.0d1 3/21/2019	first code
;************************************************
; Notes:
;
; WDT is not suported.
;
;  Does not program data EEProm, or other than prog mem 0..BootLoaderStart-1
;
; Must be the last thing in prog mem, will not overwrite itself or anything after it.
;
;BootLoaderStart	EQU	0x1E00
;	org BootLoaderStart
;	include <Bootloader18854.inc>
;
;  Reset vector:
;  	movlp	high BootLoaderStart
;    	goto	BootLoaderStart
;  ProgStartVector	CLRF	PCLATH
;	goto	Start
;
;  DataEEProm (0xFF) = 0xFF = run bootloader
;
; What happens next:
;  If Data EEProm address (0xFF) <> 0xFF then goto ProgStartVector
;  Setup  UART for 38.4KBaud
;  Parse incoming text as .hex file
;   if program memory address > 0 and < BootLoader then
;    read block from prog mem
;    if compare blocks then
;      if prog mem is not blank then erase block
;    write block
;
;  DataEEProm (0xFF) = 0x00
;
;  reset
;************************************************
; .hex
;:020000040000FA 
;       ^^^^^^ Set high word of address to 0x0000
;:100010005A23582356235423422021000C16200033
;:100020003A100C1E3A14A10B053221000C122000CC
;^ Start code                             ^^ Checksum
; ^^ Bytes of data (0x10)
;   ^^^^ Address low word (0x0020)
;       ^^ Record type 0x00 = Data
;         ^^^^ LSB MSB of data for Addr / 2
;                                         
; Checksum is 2's complement of all other bytes added together
; Valid address range 0000 0000 .. 0000 3BFE (bootloader is at 0000 3C00 .. 0000 3FFF, 512 words)
;************************************************
;
;
	cblock 0x20
	PROG_ADDR_LO                                  ;word address PC
	PROG_ADDR_HI                                  ; high 7 bits of PC
	ProgWordCount                                 ;Words in SourceBuffer
	
	IH_Addr_Lo:2		;
	IH_Addr_Hi:2		;extended linear address, alway 0x0000 for Prog mem
	IH_Bytes
	IH_ByteCount
	IH_Type		;Record Type 0..5
	IH_State
	IH_TempVal
	IH_Flags
	ChkSum
	endc
;
#Define	MSN_Done	IH_Flags,0	;Most Significant Nibble
#Define	DataInBuffer	IH_Flags,1             ;SourceBuffer has data
;
;IH_Type: Intel Hex Types
IHT_Data	EQU	0x00
IHT_End	EQU	0x01	;ie :00000001FF
IHT_XSA	EQU	0x02	;extended segment address, not supported
IHT_SSA	EQU	0x03	;start segment address, not supported
IHT_HighAddr	EQU	0x04	;extended linear address, :02000004FFFFFC
IHT_SLA	EQU	0x05	;start linear address, not supported, :04000005000000CD2A
;
;IH_State values
IHS_WaitForColon	EQU	0x00
IHS_Length	EQU	0x01
IHS_AddrMSB	EQU	0x02
IHS_AddrLSB	EQU	0x03
IHS_DataType	EQU	0x04
IHS_Data	EQU	0x05
IHS_CSum	EQU	0x06	;waiting for check sum
;
IH_ErrorBadLen	EQU	'+'	;>64 Too big
IH_ErrorBadCSum	EQU	'-'	;Checksum didn't match
IH_ErrorBadType        EQU                    '='                    ;IH_Type not recognized 
IH_Ack	EQU	'*'	;We got a good Intel Hex line
;
BlockSize	EQU	0x40
BlockWords	EQU	BlockSize/2	;0x20
AddressMask	EQU	0xE0
;
;Contiguous ram 0x2000..0x21EF, 496 bytes
	cblock 0x2100		;0x0100 after start of ram
	SourceBuffer:BlockSize
	DestBuffer:BlockSize
	SerialBuffer:BlockSize
	endc
;
;=========================================================================================
;
; Entry: DataEEProm255
; Exit: goto ProgStartVector
; Calls: none
; RAM used: ALL
;
BootLoader	call	InitializeBLIO	;returns w/ bank 0 selected
;
	call	ReadEEData	;Get Data from EEPROM (0xFF) 
	xorlw	0xFF
	SKPZ		;0xFF xor 0xFF = 0
	bra	BootLoader_End
;
; Bootloader is active send "**"
	movlw	IH_Ack
	call	SendChar
	movlw	IH_Ack
	call	SendChar
;
;Initialize
	clrf	PROG_ADDR_LO
	clrf	PROG_ADDR_HI
	clrf	IH_Addr_Hi
	clrf	IH_Addr_Hi+1
	call	EraseSourceBuffer
;
	call	BL_Parse
;	
;if successful then clear EEProm(255)
	call	WriteEEData
;
BootLoader_End	movlp	0x00
	goto	ProgStartVector
;
;=========================================================================================
; Parse incoming data and do stuff
;
BL_Parse	bcf	DataInBuffer
BL_ParseNextLine	clrf	IH_State
	clrf	IH_ByteCount
	clrf	ChkSum
	call                   FSR0_SerialBuffer
;
BLP_L1	bcf	MSN_Done
;
BLP_L2	call	GetChar
	movwf	Param78
	movlw	':'
	subwf	Param78,W
	SKPZ		;Char=":"
	bra	BLP_TestNum	; No
; It's a ":" so set state=1, wait for length
	movlw	IHS_Length
	movwf	IH_State
	bra	BLP_L1
;
BLP_TestNum	movf	IH_State,F
	SKPNZ		;Are we waiting for a number?
	bra	BLP_L2	; No
;
	movlw	0x30	;'0'
	subwf	Param78,W
	SKPNB		;Char>="0"?
	bra	BL_ParseNextLine	; No start over
	movlw	0x3A	;':'
	subwf	Param78,W
	SKPNB                                         ;>"9"?
	bra	BLP_TestNum_1          ; No, it's 0..9
	movlw	0x41	;'A'
	subwf	Param78,W
	SKPNB		;Char>="A"?
	bra	BL_ParseNextLine	; No start over
	movlw	0x47	;'G'
	subwf	Param78,W
	SKPB		;Char<"G"
	bra	BL_ParseNextLine	; No, start over
	movlw	0x07	; Yes, A..F
	subwf	Param78,F              ;  make "A" "0"+.10
;
BLP_TestNum_1	movlw	0x30
	subwf	Param78,F	;'0' >> 0x00
	btfsc	MSN_Done
	bra	BLP_TestNum_2	;do low nibble
	swapf	Param78,W
	movwf	IH_TempVal
	bsf	MSN_Done
	bra	BLP_L2                 ;go get the next char
;
BLP_TestNum_2	movf	Param78,W
	addwf	IH_TempVal,F           ;add the low nibble
;
; We have a number in IH_TempVal
                       movf                   IH_State,W
                       andlw                  0x07
                       brw
                       bra                    BL_ParseNextLine       ;IHS_WaitForColon already handled
                       bra                    BLP_IHS_Length         ;Waiting for length byte
                       bra                    BLP_IHS_AddrMSB        ;IHS_AddrMSB
                       bra                    BLP_IHS_AddrLSB        ;IHS_AddrLSB
                       bra                    BLP_IHS_DataType       ;IHS_DataType
                       bra                    BLP_IHS_Data           ;IHS_Data
                       bra                    BLP_IHS_CSum           ;IHS_CSum
                       bra                    BL_ParseNextLine       ;oops
;	
BLP_IHS_Length	movlw	.65	; Yes, is it in range 0..64?
	subwf	IH_TempVal,W
	SKPNB		;<=64?
	bra	BLP_SaveIH_Length
;
	movlw	IH_ErrorBadLen
	call	SendChar
	goto	BL_ParseNextLine
;
BLP_SaveIH_Length	movf	IH_TempVal,W
	movwf	IH_Bytes
	addwf	ChkSum,F
BLP_NextState	incf	IH_State,F
	goto	BLP_L1
;
BLP_IHS_AddrMSB	movf	IH_TempVal,W
	movwf	IH_Addr_Lo+1
	addwf	ChkSum,F
	bra	BLP_NextState
;
BLP_IHS_AddrLSB	movf	IH_TempVal,W
	movwf	IH_Addr_Lo
	addwf	ChkSum,F
	bra	BLP_NextState
;
BLP_IHS_DataType	movf	IH_TempVal,W
	movwf	IH_Type
	addwf	ChkSum,F
; Ship IHS_Data if length = 0
	movf	IH_Bytes,F
	SKPNZ		;Expecting zero length data?
	incf	IH_State,F	; Yes
	bra	BLP_NextState
;
BLP_IHS_Data	movf	IH_TempVal,W
	movwi	FSR0++
	addwf	ChkSum,F
	incf	IH_ByteCount,F
; if IH_ByteCount=IH_Bytes then next state
	movf	IH_ByteCount,W
	subwf	IH_Bytes,W
	SKPZ		;Done with data?
	goto	BLP_L1	; No
	bra	BLP_NextState	; Yes
;
BLP_IHS_CSum:
; get 2's comp
	clrf	Param78
	movf	ChkSum,W
	subwf	Param78,W
	subwf	IH_TempVal,W
	SKPNZ		;Checksum matches?
	bra	BLP_Validate           ; Yes
;
	movlw	IH_ErrorBadCSum
	call	SendChar
	goto	BL_ParseNextLine
;
;===================
; We have received a complete Intel Hex command
;
BLP_Validate	movf                   IH_Type,W
                       andlw                  0xF8
                       SKPZ
                       bra                    BLPV_BadType
;
                       movf                   IH_Type,W
                       brw
                       bra                    BLPV_Data              ;IHT_Data
                       bra                    BLPV_EndCmd            ;IHT_End
                       bra                    BLPV_BadType           ;IHT_XSA
                       bra                    BLPV_BadType           ;IHT_SSA
                       bra                    BLPV_HighAddr          ;IHT_HighAddr
                       bra                    BLPV_BadType           ;IHT_SLA
                       bra                    BLPV_BadType           ;6
                       bra                    BLPV_BadType           ;7
;---------------------
; handle end command
BLPV_EndCmd	btfsc	DataInBuffer
	call	BLP_WriteBuffer
	movlw	IH_Ack
                       goto	SendChar
;
;---------------------
; handle data
BLPV_Data	call	BufferIHData           ;Move to source buffer
                       bra                    BLPV_Exit_Ack
;
;---------------------
; handle high address change
BLPV_HighAddr	call                   FSR0_SerialBuffer
	moviw	FSR0++
	movwf	IH_Addr_Hi+1
	moviw	FSR0++
	movwf	IH_Addr_Hi
BLPV_Exit_Ack	movlw	IH_Ack
BLPV_Exit	call	SendChar
	goto	BL_ParseNextLine
;---------------------
BLPV_BadType           movlw                  IH_ErrorBadType
                       bra                    BLPV_Exit
;
;===========================================
; Validate address is in range and move to SourceBuffer
;
; Entry: DataInBuffer, IH_ByteCount, Data in SerialBuffer, 32bit address IH_Addr_Hi:IH_Addr_Lo
; Exit: SourceBuffer, DataInBuffer, ProgWordCount, PROG_ADDR_HI:PROG_ADDR_LO
; 
; check address range, max 32K program words
BufferIHData	movf	IH_Addr_Hi,W
	iorwf	IH_Addr_Hi+1,W
	SKPZ		;High address = 0x0000 ?
	return		; No, > 64KB or 32K words
;
; convert to words, IH_Addr_Lo = IH_Addr_Lo / 2
	lsrf	IH_Addr_Lo+1,F
	rrf	IH_Addr_Lo,F
	movlw	high BootLoaderStart   ;0x0E
	subwf	IH_Addr_Lo+1,W
	SKPB		;BootLoaderStart > IH_Addr_Lo?
	return		; No, don't allow overwrite of bootloader
;
	btfss	DataInBuffer	;Any old data in the buffer?
	bra	BIHD_PutInBuff	; No
;
; Continue filling buffer or flush and add to new buffer?
;
	movf	IH_Addr_Lo+1,W
	subwf	PROG_ADDR_HI,W
	SKPZ		;MSB of Address Matches?
	bra	BIHD_FlushBuff	; No, write current data to PFM
;
	movlw	AddressMask
	andwf	PROG_ADDR_LO,W
	movwf	Param78                ;Param78 = Prog Address LSB & 0xE0
	movlw	AddressMask
	andwf	IH_Addr_Lo,W           ;W = IH Word Address LSB & 0xE0
	subwf	Param78,W
	SKPNZ		;Row Address Matches?
	bra	BIHD_PutInBuff_E2	; Yes
;
BIHD_FlushBuff	call	BLP_WriteBuffer
;
; Entry: IH_Addr_Lo=Word Address
; 
BIHD_PutInBuff	movf	IH_Addr_Lo,W           ;was converted to words
	andlw	AddressMask            ; 0xE0
	movwf	PROG_ADDR_LO
	movf	IH_Addr_Lo+1,W
	movwf	PROG_ADDR_HI
; ProgWordCount = 0x1F & IH_Addr_Lo
	movlw	0xFF
	xorlw	AddressMask	;0x1F
	andwf	IH_Addr_Lo,W	;Offset within block
	movwf	ProgWordCount
;
BIHD_PutInBuff_E2	call                   FSR0_SerialBuffer
;
; FSR1 = SourceBuffer + ProgWordCount * 2
	lslf	ProgWordCount,W	;ProgWordCount * 2
	addlw	low SourceBuffer
	movwf	FSR1L
	movlw	high SourceBuffer
	movwf	FSR1H
;
;Param79 = IH_ByteCount
	movf	IH_ByteCount,W
	movwf	Param79
;ProgWordCount += IH_ByteCount / 2
	lsrf	IH_ByteCount,W	; divide by 2
	addwf	ProgWordCount,F
;
BIHD_PutInBuff_L1	moviw	FSR0++                 ;move SerialBuffer
	movwi	FSR1++                 ;to SourceBuffer
	decfsz	Param79,F
	bra	BIHD_PutInBuff_L1
;
	bsf	DataInBuffer
;
	movlw	BlockWords             ;0x20
	subwf	ProgWordCount,W        ;W = ProgWordCount - 0x20
	SKPNZ		;buffer full?
	bra	BLP_WriteBuffer	; Yes, ProgWordCount = 0x20
	SKPNB		;Overflow? This should not happen.
	return		; No, ProgWordCount < 0x20
;
; fall thru to BLP_WriteBuffer
;=============================================
; Write SourceBuffer to Prog memory
;
; Entry: SourceBuffer, ProgWordCount, PROG_ADDR_HI:PROG_ADDR_LO
;
BLP_WriteBuffer:
; copy old prog mem to DestBuffer
	call                   FSR1_DestBuff
	call	ReadProgBlock
;
	movlw	low SourceBuffer	;new data
	movwf	FSR0L
	movlw	high SourceBuffer
	movwf	FSR0H	
	call                   FSR1_DestBuff          ;old data
	call	CompareBlock
;
	btfsc	WREG,0	;Matches?
	bra	BLP_WriteBuffer_end	; Yes
;	
	call                   FSR1_DestBuff
	call	TestErased
	btfss	WREG,0	;Erased?
	call	EraseProgBlock	; No, Erase it.
;
	call                   FSR1_SourceBuffer	
	call	WriteProgBlock
;
BLP_WriteBuffer_end	goto	EraseSourceBuffer
;
;---------------------
FSR0_SerialBuffer      movlw	low SerialBuffer
	movwf	FSR0L
	movlw	high SerialBuffer
	movwf	FSR0H
	return
;
;---------------------
FSR1_DestBuff          movlw	low DestBuffer
	movwf	FSR1L
	movlw	high DestBuffer
	movwf	FSR1H
	return
;
;---------------------
FSR1_SourceBuffer      movlw	low SourceBuffer
	movwf	FSR1L
	movlw	high SourceBuffer
	movwf	FSR1H
	return
;
;==============================================
; Get a byte from UART, waits forever
;
; Entry: none
; Exit: W=Char
;
GetChar	movlb	PIR3                   ;bank 14
	btfss	PIR3,RCIF	;RX has a byte?
	bra	GetChar
;
	movlb	RC1STA	; bank 2
	movf	RC1STA,W
	andlw	0x06
	SKPZ		;Frame or Overrun Err?
	bra	GetChar_Err	; yes
	movf	RC1REG,W
;
	movlb	0	;bank 0
	return
;
GetChar_Err	MOVF	RC1REG,W	;bad byte so drop it
	bcf	RC1STA,CREN
	bcf	RC1STA,FERR
	bcf	RC1STA,OERR
	bsf	RC1STA,CREN
	bra	GetChar
;
;==============================================
; Send a byte to UART
;
; Entry: W=Char to send
SendChar               movlb	PIR3                   ;bank 14
SendChar_L1	btfss	PIR3,TXIF	;TX done?
	bra	SendChar_L1            ; No
	movlb	TX1REG
	movwf	TX1REG
	movlb	0                      ;bank 0
	return
;	
;==============================================
; Read EEProm(255)
;
; entry: none
; exit: W=EEDATA, Bank 0 selected
;
ReadEEData	movlb	NVMADRH	;Bank 16
	bsf                    NVMCON1,NVMREGS        ; Select Configuration Space, EEPROM, etc
	movlw                  0xFF
	movwf                  NVMADRL
	movlw                  0xF0
	movwf                  NVMADRH
	bsf                    NVMCON1,RD
	movf                   NVMDATL,W
	movlb	0	;Bank 0
	return
;
;==============================================
; Clear EEProm(255)
;
; entry: none
; exit: Bank 0 selected
;
WriteEEData	movlb                  NVMADRL                ;bank 16
                       bsf                    NVMCON1,NVMREGS
                       movlw                  0xFF
                       movwf                  NVMADRL
	movlw                  0xF0
	movwf                  NVMADRH
	clrf                   NVMDATL
;
	bsf                    NVMCON1,WREN
	call                   NVM_WriteSequence
;
WriteEEData_L1	btfsc	NVMCON1,WR	
	bra	WriteEEData_L1
	bcf	NVMCON1,WREN
	movlb	0x00	;Bank 0
	return	
;
;==============================================
; Test if block is erased
;
; Entry: FSR1 -> current data
; Exit: W=1 if new = blank else W=0
;
TestErased	movlw	BlockWords
	movwf	Param79
TestErased_L1	moviw	FSR1++	;LSB
	sublw	0xFF
	SKPZ		;=0xFF?
	retlw	0x00	; No
;
	moviw	FSR1++	;MSB
	sublw	0x3F
	SKPZ		;=0x3F?
	retlw	0x00	; No
;
	decfsz	Param79,F
	bra	TestErased_L1
	retlw	0x01                   ;All words are 0x3FFF
;
;==============================================
; Compare Block
;
; Entry: FSR0 -> new data, FSR1 -> current data
; Exit: W=1 if new = current else W=0
;
CompareBlock	movlw	BlockWords
	movwf	Param79
CompareBlock_L1	moviw	FSR0++	;LSB
	movwf	Param77
	moviw	FSR0++	;MSB
	movwf	Param78
; skip?
	sublw	0x3F
	SKPZ
	bra	CB_NotBlank
	incf	Param77,W
	SKPZ
	bra	CB_NotBlank
	moviw	FSR1++
	moviw	FSR1++
	bra	CB_Next
;
CB_NotBlank	moviw	FSR1++
	subwf	Param77,W
	SKPZ		;Matches?
	retlw	0x00	; No
	moviw	FSR1++
	subwf	Param78,W
	SKPZ		;Matches?
	retlw	0x00	; No
;
CB_Next	decfsz	Param79,F
	bra	CompareBlock_L1
	retlw	0x01	;It's a match!
;
;==============================================
; Fill SourceBuffer w/ 0x3FFF
;
EraseSourceBuffer	call                   FSR1_SourceBuffer
                       bcf	DataInBuffer           ;mark SourceBuffer as empty
;
; Fall thru to EraseBlock
;==============================================
; Erase Block (Fill with 0x3FFF)
;
; Entry: FSR1
;
EraseBlock	movlw	BlockWords
	movwf	Param79
EraseBlock_L1	movlw	0xFF
	movwi	FSR1++
	movlw	0x3F
	movwi	FSR1++
	decfsz	Param79,F
	bra	EraseBlock_L1
	return
;	
;==============================================
; Read Program memory block
;
; Entry: FSR1,PROG_ADDR_HI:PROG_ADDR_LO
; Exit: FSR1 changed, 32 words >> (FSR1++)
;
ReadProgBlock	call	LoadProgAddr
	bcf                    NVMCON1,NVMREGS        ; Do not select Configuration Space
	movlw	BlockWords	;Block is .32 words
	movwf	Param79
ReadProgBlock_L1	bsf	NVMCON1,RD	; Initiate read
	movf	NVMDATL,W	; Get LSB of word
	movwi	FSR1++	; Store in user location
	MOVF	NVMDATH,W	; Get MSB of word
	movwi	FSR1++	; Store in user location
	decfsz	Param79,F
	bra	ReadProgBlock_1
	movlb	0
	return
;
ReadProgBlock_1	incf	NVMADRL,F
	bra	ReadProgBlock_L1
;
;=====================================================
; Erase Program memory block
;
; Entry: PROG_ADDR_HI:PROG_ADDR_LO
;
EraseProgBlock	call	LoadProgAddr
                       bcf                    NVMCON1,NVMREGS        ; Do not select Configuration Space
	bsf	NVMCON1,FREE	;Specify an erase operation, automaticaly cleared
	bsf	NVMCON1,WREN	;Enable writes
	call                   NVM_WriteSequence      ;Set WR bit to begin erase
	bcf	NVMCON1,WREN	;Disable writes
	movlb	0
	return
;
;=========================================================
; Load the program address PROG_ADDR_HI:PROG_ADDR_LO into EEADRH:EEADRL
;
; Entry: PROG_ADDR_HI:PROG_ADDR_LO, bank 0
; Exit: bank 16
;
LoadProgAddr	movf	PROG_ADDR_HI,W
	movlb	NVMADRH	;bank 16
	movwf	NVMADRH	; Store MSB of address
	movlb	0	;bank 0
	movf	PROG_ADDR_LO,W
	movlb	NVMADRL	;bank 16
	movwf	NVMADRL	; Store LSB of address
	return
;
;==========================================================
; Write Program memory block
;
; Entry: PROG_ADDR_HI:PROG_ADDR_LO, FSR1
; Exit: FSR1 changed
; 
WriteProgBlock	call	LoadProgAddr
                       bcf                    NVMCON1,NVMREGS        ; Do not select Configuration Space
	bsf	NVMCON1,WREN	;Enable writes
	bsf	NVMCON1,LWLO	;Only Load Write Latches
	movlw	BlockWords
	movwf	Param79
;
WriteProgBlock_L1	moviw	FSR1++
	movwf	NVMDATL
	moviw	FSR1++
	movwf	NVMDATH
;
	movf	NVMADRL,W
	xorlw	0x1F
	andlw	0x1F
	btfsc	_Z	;EEADRL = xxx1 1111?
	bra	WriteProgBlock_Wr	; Yes
;
	call                   NVM_WriteSequence	; Set WR bit to begin write to latches
	incf	NVMADRL,F
	bra	WriteProgBlock_L1
;
WriteProgBlock_Wr	bcf	NVMCON1,LWLO	; Write memory not latches
                       call                   NVM_WriteSequence
	bcf	NVMCON1,WREN	;Disable writes
;
	movlb	0
	return	
;
;===================
NVM_WriteSequence      movlw	0x55	; Start of required write sequence:
	movwf	NVMCON2
	movlw	0xAA
	movwf	NVMCON2
	bsf	NVMCON1,WR	; Set WR bit to begin write
	return
;
;============================================================
; call once
;
InitializeBLIO	movlb	ANSELA                 ;bank 30
	clrf	ANSELA
	clrf                   ANSELB
	clrf	ANSELC
;
	movlb	0	;bank 0
	movlw	PortA_Init_Value
	movwf	LATA
	movlw                  PortB_Init_Value
	movwf                  LATB
	movlw	PortC_Init_Value
	movwf	LATC
;
                       movlw                  PortA_Tris_Bits
                       movwf                  TRISA
                       movlw                  PortB_Tris_Bits
                       movwf                  TRISB
                       movlw                  PortC_Tris_Bits
                       movwf                  TRISC
;
;=============================
;
; setup serial I/O
	movlb	RXPPS                  ;bank 29
	movlw	0x17	;RC7, default
	movwf	RXPPS
	movlw	0x16	;RC6, default
	movwf	TXPPS
;
	movlb	RC6PPS                 ;bank 30
	movlw	0x10	;Tx/CK signal
	movwf	RC6PPS
;
	movlb	BAUD1CON	;bank 2
	movlw	BAUDCON_Value
	movwf	BAUD1CON
	MOVLW	low BaudRate
	MOVWF	SP1BRGL
	MOVLW	high BaudRate
	MOVWF	SP1BRGH
;
	bcf	TX1STA,SYNC_TX1STA
	bsf	TX1STA,BRGH
	bsf	TX1STA,TXEN
;
	bsf	RC1STA,SPEN
	bsf	RC1STA,CREN
;
	movlb	0	; bank 0
;
	return
;
;
 



 



 

















