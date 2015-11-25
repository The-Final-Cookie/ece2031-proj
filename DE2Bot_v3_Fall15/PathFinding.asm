  ORG &H000

PathFind:
; We should clear out data registers that we might have left data hanging around
; inside

; TODO We can do better, use loops and such to take care of this etc
  LOADI 0
  LOADI BestPoint
  STORE Offset
  LOADI 0
  ISTORE Offset
  LOAD Offset
  ADDI 1
  STORE Offset
  LOADI 0
  ISTORE Offset
  LOAD Offset
  ADDI 1
  STORE Offset
  LOADI 0
  ISTORE Offset

  LOADI CurrentPoint
  STORE Offset
  LOADI 0
  ISTORE Offset
  LOAD Offset
  ADDI 1
  STORE Offset
  LOADI 0
  ISTORE Offset
  STORE Idx
  STORE Jdx
  STORE Offset
  STORE HeadingTheta

; First we need to convert the data points into robot units
ConvertToUnits:
  LOADI Points 
  ADD Idx
  ADD Idx
  STORE Offset
  LOADI ConvertedPoints
  ADD Idx
  ADD Idx
  ADD Idx
  STORE ConvertedOffset
  CALL FeetToUnits
  LOAD Offset
  ADDI 1
  STORE Offset
  LOAD ConvertedOffset
  ADDI 1
  STORE ConvertedOffset
  CALL FeetToUnits
  ; Now store the index along with the point
  LOAD ConvertedOffset
  ADDI 1
  STORE ConvertedOffset
  LOAD Idx
  ADDI 1
  ISTORE ConvertedOffset
  LOAD Idx
  ADDI 1
  STORE Idx
  ADDI -12
  JZERO StartSort
  JUMP ConvertToUnits

; This is dumb and slow, TODO rewrite with a >> 3 store lsb >> 1 then add
; lsb instead of idiv
FeetToUnits:
  ILOAD Offset
  ; Now we have the value that we want to modify in our AC
  STORE m16sA
  LOAD SixteenthUnitsInFoot
  STORE m16sB
  CALL Mult16s ; Because we know that we won't be given values higher than 6
  ;LOAD mres16sL ; We only need to load the low word ; already loaded!
  SHIFT -3     ; And round, we have 4 low bits of fractional
  AND One
  STORE Temp
  LOAD mres16sL
  SHIFT -1
  ADD Temp
  ISTORE ConvertedOffset
  RETURN

Temp: DW 0 ; dumb name
Idx: DW 0 ; index for loop
Jdx: DW 0 ; index for 2nd loop
Offset: DW 0
OffsetTo: DW 0
HeadingTheta: DW 0 ; atan2 radians units (8 fractional bits)
TempTheta: DW 0 ; temporarily stores the candidate new heading during CalcCost
ConvertedOffset: DW 0
SixteenthUnitsInFoot: DW 4644 ; There are 209.2857... robot units in a foot,
                              ; using four fractional bits

; Copies point from one offset to another
; Accepts input from Offset and OffsetTo, copies the contents of Offset to
; OffsetTo, points are three memory addresses in contiguous memory (x, y, count)
; This function modifies Offset and OffsetTo and CopyPointIdx
CopyPoint:
  ILOAD Offset  ; from P1 x
  ISTORE OffsetTo  ; to P2 x
  LOAD Offset
  ADDI 1
  STORE Offset
  LOAD OffsetTo
  ADDI 1
  STORE OffsetTo
  ILOAD Offset  ; from P1 y
  ISTORE OffsetTo  ; to P2 y
  LOAD Offset
  ADDI 1
  STORE Offset
  LOAD OffsetTo
  ADDI 1
  STORE OffsetTo
  ILOAD Offset  ; from P1 count
  ISTORE OffsetTo  ; to P2 count
  RETURN

StartSort:
  ; Starting point is (0,0), should be initialized at the top

  ;LOADI 1
  ;OUT XLEDS ; Signal to the simulator that we're finished
  LOADI 0
  STORE Idx ; loop variable

  OuterDistLoop:

    ; pointsLeft = 12 - i
    LOADI 12
    SUB Idx
    STORE PointsLeft

    LOAD c7FFF ; max int
    STORE BestCost

    LOADI 0
    STORE Jdx ; start the loop for Jdx
    InnerDistLoop:
      ; Calculate cost reads from Offset indirectly and the next memory location
      ; after.  It also reads from the Current Point, which ought to already be
      ; set up from the previous loop part
      LOADI ConvertedPoints
      ADD Jdx
      STORE Offset
      CALL CalculateCost ; outputs to ThisCost
      LOAD BestCost
      SUB ThisCost
      JNEG UpdateInnerLoop 
      LOADI ConvertedPoints ; if this point is better than previous
      ADD Jdx
      STORE Offset
      LOADI BestPoint
      STORE OffsetTo
      CALL CopyPoint
      LOAD TempTheta
      STORE BestTheta
      LOAD ThisCost
      STORE BestCost
      LOAD Jdx
      STORE BestIdx

      UpdateInnerLoop:
      LOAD Jdx
      ADDI 3 ; Our entry size is 3, so we have to jump 3 per iteration
      STORE Jdx
      SUB PointsLeft
      SUB PointsLeft
      SUB PointsLeft
      JNEG InnerDistLoop

    ; Copy the current best point to our index
    LOADI BestPoint
    STORE Offset
    LOADI RealOutPoints
    ADD Idx
    ADD Idx
    ADD Idx
    STORE OffsetTo
    CALL CopyPoint

    ; Copy the last point in the list to the area we just copied (effectively
    ; reduces the size of the list by one and removes the point we just used
    ; from consideration)

    LOADI ConvertedPoints
    ADD BestIdx ; already multiplied by 3
    STORE OffsetTo ; pointer to the point we just copied

    LOADI ConvertedPoints
    ADD PointsLeft
    ADD PointsLeft
    ADD PointsLeft
    ADDI -1
    ADDI -1
    ADDI -1
    STORE Offset

    CALL CopyPoint

    ; currentPoint = bestPoint
    LOADI BestPoint
    STORE Offset
    LOADI CurrentPoint
    STORE OffsetTo
    CALL CopyPoint

    LOAD Idx
    ADDI 1
    STORE Idx
    ADDI -12
    JNEG OuterDistLoop

LoopDone:
  LOADI 1
  OUT XLEDS ; signal emulator that we're done
  JUMP LoopDone

; Calculate cost reads from Offset indirectly and the next memory location
; after.  It also reads from the Current Point, which ought to already be
; set up from the previous loop part
CalculateCost:
  ILOAD Offset ; Grab xdiff
  SUB CurrentPoint
  STORE XDiff
  STORE L2X

  LOAD Offset ; And ydiff, note this is inconsistent but it doesn't matter
  ADDI 1
  STORE Offset
  ILOAD Offset
  STORE YDiff
  LOADI CurrentPoint
  ADDI 1
  STORE Offset
  ILOAD Offset
  SUB YDiff
  STORE YDiff
  STORE L2Y

  CALL L2Estimate
  STORE ThisCost

  LOADI 1
  STORE A2retrad
  LOADI 402 ; pi/2 with 8 fractional bits
  STORE PosModuloD

  CALL Atan2
  SUB HeadingTheta
  STORE TempTheta
  CALL PosModulo ; mod pi/2
  STORE m16sA

  LOAD TwoPi
  STORE PosModuloD
  LOAD TempTheta
  CALL PosModulo
  STORE TempTheta

  LOADI 113 ; the axle track radius in robot units
  STORE m16sB

  CALL Mult16s ; 113 * fmod(atan2(ydiff, xdiff) - theta, pi/2)
               ; The angle cost of the point

  ; LOAD mres16sL ; already loaded!
  SHIFT -7
  AND One
  STORE Temp
  LOAD mres16sL
  SHIFT -1
  ADD Temp ; Now we have the 8 lsb
  STORE mres16sL
  LOAD mres16sH
  SHIFT 8
  AND mres16sL ; Now we have a full 16 bits rounded shouldn't overflow, I hope
  ADD ThisCost
  STORE ThisCost

  LOADI 0        ; And be nice to other callers of atan2
  STORE A2retrad

  RETURN

XDiff: DW 0
YDiff: DW 0
ThisCost: DW 0
PointsLeft: DW 0
BestCost: DW 0
BestTheta: DW 0 ; stored with 8 fractional bits
BestIdx: DW 0
BestPoint:
  DW 0 ; x
  DW 0 ; y
  DW 0 ; index + 1 (for reporting)
CurrentPoint:
  DW 0
  DW 0
  DW 0

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

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;*******************************************************************************
; Mod180: modulo 180
; Returns AC%180 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************	
Mod180:
	JNEG   Mod180n      ; handle negatives
Mod180p:
	ADDI   -180
	JPOS   Mod180p      ; subtract 180 until negative
	ADDI   180          ; go back positive
	RETURN
Mod180n:
	ADDI   180          ; add 180 until positive
	JNEG   Mod180n
	ADDI   -180         ; go back negative
	RETURN

;*******************************************************************************
; PosModulo: modulo
; Returns AC%ModuloD in AC
; The modulo is strictly positive in the range 0, ModuloD-1
;*******************************************************************************	
PosModulo:
  JZERO  PosModulo_bail
	JNEG   PosModuloN      ; handle negatives
PosModuloP:
	SUB    PosModuloD
	JPOS   PosModuloP      ; subtract until negative
	ADD    PosModuloD      ; go back positive
	RETURN
PosModuloN:
	ADD    PosModuloD      ; add until positive
	JNEG   PosModuloN
	RETURN
PosModulo_bail:
  RETURN

PosModuloD: DW 0

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW -1                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
  LOAD   A2retrad     ; If we should return radians
  JPOS   A2_rad       ; Just return this
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_rad:
  LOAD m16sA
  RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits
A2retrad:   DW 0        ; bool flag to just return the radians

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

TwoPi:    DW 1608 ; 8 fractional bits

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

ORG &H300

Points:
  DW 5 ; Entry 00 x
  DW -4 ; Entry 00 y
  DW 4 ; Entry 01 x
  DW 1 ; Entry 01 y
  DW -3 ; Entry 02 x
  DW 4 ; Entry 02 y
  DW 0 ; Entry 03 x
  DW 4 ; Entry 03 y
  DW 0 ; Entry 04 x
  DW -1 ; Entry 04 y
  DW -2 ; Entry 05 x
  DW -6 ; Entry 05 y
  DW 0 ; Entry 06 x
  DW -6 ; Entry 06 y
  DW 0 ; Entry 07 x
  DW -5 ; Entry 07 y
  DW -2 ; Entry 08 x
  DW 4 ; Entry 08 y
  DW 2 ; Entry 09 x
  DW 0 ; Entry 09 y
  DW 5 ; Entry 10 x
  DW -2 ; Entry 10 y
  DW -4 ; Entry 11 x
  DW -6 ; Entry 11 y

MemoryDumpMarker:
  DW &H4141
  DW &H4141
  DW &H4141
  DW &H4141

ConvertedPoints:
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

MemoryDumpMarker2:
  DW &H4141
  DW &H4141
  DW &H4141
  DW &H4141

OutPoints:
  DW 0 ; Entry 00 x
  DW 0 ; Entry 00 y
  DW 0 ; Entry 00 count
RealOutPoints:
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
  DW 0 ; Entry 12 x
  DW 0 ; Entry 12 y
  DW 0 ; Entry 12 count

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H99  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
