  ORG &H000

ConvertToUnits:
  LOADI Points 
  ADDI Idx
  ADDI Idx
  STORE Offset
  CALL FeetToUnits
  LOAD Offset
  ADDI 1
  STORE Offset
  CALL FeetToUnits
  LOAD Idx
  ADDI 1
  STORE Idx
  ADDI -12
  JZERO StartSort
  JUMP ConvertToUnits

FeetToUnits:
  ILOAD Offset
  ; Now we have the value that we want to modify in our AC
  STORE m16sA
  LOADI 2093 ; There are 209.2857... robot units in a foot
  STORE m16sB
  CALL Mult16s ; Because we know that we won't be given values higher than 6
  LOAD mres16sL ; We only need to load the low word 
  STORE d16sN ; and / 10 and do a round
  LOADI 10
  STORE d16sD
  CALL Div16s
  LOAD dres16sR ; and round
  ADDI -5
  JNEG SkipRound
  LOAD dres16sQ
  ADDI 1
  STORE dres16sQ

SkipRound:
  LOAD dres16sQ
  ISTORE Offset
  RETURN

Idx: DW 0 ; index for loop
Offset: DW 0

ConvertToUnits:
  JUMP ConvertToUnits

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOADI  246       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOADI  104       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

Points:
  DW -4 ; Entry 00 x
  DW 1 ; Entry 00 y
  DW 3 ; Entry 01 x
  DW -2 ; Entry 01 y
  DW -3 ; Entry 02 x
  DW -1 ; Entry 02 y
  DW 1 ; Entry 03 x
  DW 4 ; Entry 03 y
  DW -2 ; Entry 04 x
  DW -4 ; Entry 04 y
  DW 0 ; Entry 05 x
  DW -5 ; Entry 05 y
  DW -3 ; Entry 06 x
  DW 2 ; Entry 06 y
  DW 2 ; Entry 07 x
  DW -3 ; Entry 07 y
  DW 2 ; Entry 08 x
  DW 5 ; Entry 08 y
  DW -2 ; Entry 09 x
  DW 4 ; Entry 09 y
  DW 3 ; Entry 10 x
  DW -3 ; Entry 10 y
  DW -4 ; Entry 11 x
  DW -2 ; Entry 11 y

OutPoints:
  DW 0 ; Entry 00 x
  DW 0 ; Entry 00 y
  DW 0 ; Entry 00 count
  DW 0 ; Entry 01 x
  DW 0 ; Entry 01 y
  DW 0 ; Entry 01 count
  DW 0 ; Entry 02 x
  DW 0 ; Entry 02 y
  DW 0 ; Entry 02 count
  DW 0 ; Entry 03 x
  DW 0 ; Entry 03 y
  DW 0 ; Entry 03 count
  DW 0 ; Entry 04 x
  DW 0 ; Entry 04 y
  DW 0 ; Entry 04 count
  DW 0 ; Entry 05 x
  DW 0 ; Entry 05 y
  DW 0 ; Entry 05 count
  DW 0 ; Entry 06 x
  DW 0 ; Entry 06 y
  DW 0 ; Entry 06 count
  DW 0 ; Entry 07 x
  DW 0 ; Entry 07 y
  DW 0 ; Entry 07 count
  DW 0 ; Entry 08 x
  DW 0 ; Entry 08 y
  DW 0 ; Entry 08 count
  DW 0 ; Entry 09 x
  DW 0 ; Entry 09 y
  DW 0 ; Entry 09 count
  DW 0 ; Entry 10 x
  DW 0 ; Entry 10 y
  DW 0 ; Entry 10 count
  DW 0 ; Entry 11 x
  DW 0 ; Entry 11 y
  DW 0 ; Entry 11 count

