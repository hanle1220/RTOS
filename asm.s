; Han Le

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

   .def setPSP
   .def getPSP
   .def setASPBit
   .def pushR4toR11toPSP
   .def popR4toR11fromPSP
   .def pushToPSP
   .def setLR
   .def getMSP
   .def setTMPL

;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

setPSP:
				MSR PSP, R0
                BX	LR                     ; return from subroutine

getPSP:
               	MRS R0, PSP
              	BX  LR

getMSP:
  	MRS R0, MSP
   	BX  LR

pushR4toR11toPSP:
	MRS R0, PSP
	SUB R0, #4
	STR R11, [R0]
	SUB R0, #4
	STR R10, [R0]
	SUB R0, #4
	STR R9, [R0]
	SUB R0, #4
	STR R8, [R0]
	SUB R0, #4
	STR R7, [R0]
	SUB R0, #4
	STR R6, [R0]
	SUB R0, #4
	STR R5, [R0]
	SUB R0, #4
	STR R4, [R0]
	MSR PSP, R0
	BX LR

popR4toR11fromPSP:
	MRS R0, PSP
	LDR R4, [R0]
	ADD R0, #4
	LDR R5, [R0]
	ADD R0, #4
	LDR R6, [R0]
	ADD R0, #4
	LDR R7, [R0]
	ADD R0, #4
	LDR R8, [R0]
	ADD R0, #4
	LDR R9, [R0]
	ADD R0, #4
	LDR R10, [R0]
	ADD R0, #4
	LDR R11, [R0]
	ADD R0, #4
	MSR PSP, R0
	BX LR

setASPBit:
				MRS	R0, CONTROL
		   		ORR R0, R0, #0x00000002
		   		MSR CONTROL, R0
		    	ISB
				BX	LR

setTMPL:
	MRS R0, CONTROL
	ORR R0, R0, #0x00000001
	MSR CONTROL, R0

pushToPSP:
				MRS R1, PSP
				SUB R1, #4
				STR R0, [R1]
				MSR PSP, R1
		        BX	LR

setLR:
				MRS R1, PSP
				ADD R1, #20
				LDR R1, [R0]
				MSR PSP, R0
               	BX	LR

.endm
