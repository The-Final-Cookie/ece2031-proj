; StartingCodeWithLog.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program includes:
; - Wireless position logging using timer interrupt
; - Subroutines for
; -- 16x16 signed multiplication
; -- 16/16 signed division
; -- Arctangent (in appropriate robot units)
; -- Distance (L2 norm) approximation
; - Example of using tables (ILOAD, ISTORE)
; - Additional misc. examples


;***************************************************************
;* Jump Table
;***************************************************************
; When an interrupt occurs, execution is redirected to one of
; these addresses (depending on the interrupt source), which
; needs to either contain RETI (return from interrupt) if not
; used, or a JUMP instruction to the desired interrupt service
; routine (ISR).  The first location is the reset vector, and
; should be a JUMP instruction to the beginning of your normal
; code.
ORG        &H000       ; Jump table is located in mem 0-4
	JUMP   Init        ; Reset vector
	JUMP   CTimer_ISR  ; Timer interrupt

;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping

	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD

WaitForSafety:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety

WaitForUser:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
	JUMP   Main

;***************************************************************
;* Main code
;***************************************************************

Main:
  OUT RESETPOS

  LOADI 0
	STORE isPoint0
Loop:  
	LOAD done;
	JPOS Die;
	CALL SetLoopValues

	;now we have generic values that are stored in 
	;curPointx, curPointy, nextPointx, nextPointy

	LOAD	nextPointx
	SUB		curPointx
	STORE  m16sA       ; Converting Feet to ticks
	LOAD   TicksPerFoot 
	STORE  m16sB        
	CALL   Mult16s    
	LOAD   mres16sL      
	STORE  AtanX      ; input to atan subroutine
	STORE  L2X    	  ; input to distance estimation subroutine
	LOAD	nextPointy
	SUB		curPointy
	STORE  m16sA		; Converting Feet to ticks
	LOAD   TicksPerFoot
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sL
	STORE  AtanY      ; input to atan subroutine
	STORE  L2Y        ; input to distance estimation subroutine
	CALL   Atan2      ; find the angle
	STORE  CurrAngle
	JPOS DontAdjust
	ADDI 360
	STORE CurrAngle
	OUT LCD
	DontAdjust:
	CALL   L2Estimate ; estimate the distance
	STORE  CurrDist
	SUB	   OneFoot
	STORE OneFootLess

	CALL DirectionAndAngle
	LOAD DirectionToGo
	OUT LCD
	JPOS callTurnCW
	JZERO callTurnCCW 
	callTurnCW: CALL TurnCW
				JUMP Loop
	callTurnCCW: CALL TurnCCW
				 JUMP Loop

Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD         ; An indication that we are dead

Forever:
	JUMP   Forever      ; Do this forever.
	DEAD:  DW &HDEAD    ; Example of a "local" variable

;***************************************************************
;* Subroutines
;***************************************************************

; Direction and Angle, to call set CurrAngle, this function will read from THETA
; directly, output is in DirectionToGo (clockwise = 1, ccw = 0) and AngleToGo
; range (0-359)

DirectionAndAngle:
  LOADI 360
  STORE PosModuloD
  IN THETA
  STORE DirAndAngTemp
  LOAD CurrAngle
  SUB DirAndAngTemp
  CALL PosModulo
  ADDI -180 ; diff = ((CurrAngle - CurrTheta) % 360) - 180
  JNEG DirectionAndAngle_CCW
  ADDI -180
  CALL Abs
  STORE AngleToGo
  LOADI 1
  STORE DirectionToGo
  RETURN

  DirectionAndAngle_CCW:
  LOADI 0
  STORE DirectionToGo
  IN THETA
  STORE DirAndAngTemp
  LOAD CurrAngle
  SUB DirAndAngTemp
  CALL PosModulo ; %360
  STORE AngleToGo
  RETURN

; Turning clockwise and move forward

 TurnCW: 
  LOADI 0
  STORE amtTurned
 
 TurnCWLoop:
  IN THETA
  LOAD thetaold
  LOADI -100 
  OUT RVelCmd
  LOADI 100
  OUT LVelCmd
  IN THETA
  LOAD thetanew
  SUB thetaold
  CALL Mod360
  ADD amtTurned
  STORE amtTurned
  SUB AngleToGo
  JPOS CWWaiting
  
			; Change this to a appropriate jump later
waitCW: JUMP TurnCWLoop

 CWWaiting:
 LOADI 0
 OUT RVelCmd
 OUT LvelCmd
 IN LVEL
 JNEG CWWaiting
 IN LPOS
 STORE OLDPOS

 CWForward:   ; Moves Forward in a straight line. Moves quickly until DE2Bot is 1 foot away from the destination. 
  LOADI 1023
  OUT LEDS
  IN THETA
  OUT LCD
  LOADI 350
  OUT LVelCmd
  OUT RVelCmd
  IN LPOS
  SUB OLDPOS
  SUB OneFootLess
  JPOS CWForward2
  JUMP CWForward

CWForward2: ; Move forward the remaining foot
  IN THETA
  OUT LCD
  LOADI 100
  OUT LVelCmd
  OUT RVelCmd
  IN LPOS
  SUB OLDPOS
  SUB CurrDist
  JPOS CWForward3
  JUMP CWForward2

CWForward3: RETURN

; Turning counter-clockwise and move forward

TurnCCW: 
  LOADI 0
  STORE amtTurned
 
 TurnCCWLoop: 
  IN THETA
  LOAD thetaold
  LOADI 100 
  OUT RVelCmd
  LOADI -100
  OUT LVelCmd
  IN THETA
  LOAD thetanew
  LOAD thetanew
  SUB thetaold
  CALL Mod360
  ADD amtTurned
  STORE amtTurned
  SUB AngleToGo
  JPOS CCWWaiting
 
waitCCW: JUMP TurnCCWLoop

 CCWWaiting:
 LOADI 0
 OUT RVelCmd
 OUT LvelCmd
 IN LVEL
 JNEG CCWWaiting
 IN LPOS
 STORE OLDPOS

 CCWForward:   ; Moves Forward in a straight line. Moves quickly until DE2Bot is 1 foot away from the destination. 
  LOADI 1023
  OUT LEDS
  IN THETA
  OUT LCD
  LOADI 350
  OUT LVelCmd
  OUT RVelCmd
  IN LPOS
  SUB OLDPOS
  SUB OneFootLess
  JPOS CCWForward2
  JUMP CCWForward

CCWForward2: ; Move forward the remaining foot
  IN THETA
  OUT LCD
  LOADI 100
  OUT LVelCmd
  OUT RVelCmd
  IN LPOS
  SUB OLDPOS
  SUB CurrDist
  JPOS CCWForward3
  JUMP CCWForward2

CCWForward3: RETURN

;Change current x and y to match loop
SetLoopValues:
	LOAD isPoint0
	JPOS P1
	LOAD Point0X
	STORE curPointx;
	LOAD Point0Y
	STORE curPointy;
	LOAD Point1X
	STORE nextPointx;
	LOAD Point1Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint0
	LOADI 0
	STORE isPoint1
	RETURN
P1:
	LOAD isPoint1
	JPOS P2
	LOAD Point1X
	STORE curPointx;
	LOAD Point1Y
	STORE curPointy;
	LOAD Point2X
	STORE nextPointx;
	LOAD Point2Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint1
	LOADI 0
	STORE isPoint2
	RETURN
P2:
	LOAD isPoint2
	JPOS P3
	LOAD Point2X
	STORE curPointx;
	LOAD Point2Y
	STORE curPointy;
	LOAD Point3X
	STORE nextPointx;
	LOAD Point3Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint2
	LOADI 0
	STORE isPoint3
	RETURN
P3:
	LOAD isPoint3
	JPOS P4
	LOAD Point3X
	STORE curPointx;
	LOAD Point3Y
	STORE curPointy;
	LOAD Point4X
	STORE nextPointx;
	LOAD Point4Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint3
	LOADI 0
	STORE isPoint4
	RETURN
P4:
	LOAD isPoint4
	JPOS P5
	LOAD Point4X
	STORE curPointx;
	LOAD Point4Y
	STORE curPointy;
	LOAD Point5X
	STORE nextPointx;
	LOAD Point5Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint4
	LOADI 0
	STORE isPoint5
	RETURN
P5:
	LOAD isPoint5
	JPOS P6
	LOAD Point5X
	STORE curPointx;
	LOAD Point5Y
	STORE curPointy;
	LOAD Point6X
	STORE nextPointx;
	LOAD Point6Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint5
	LOADI 0
	STORE isPoint6
	RETURN
P6:
	LOAD isPoint6
	JPOS P7
	LOAD Point6X
	STORE curPointx;
	LOAD Point6Y
	STORE curPointy;
	LOAD Point7X
	STORE nextPointx;
	LOAD Point7Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint6
	LOADI 0
	STORE isPoint7
	RETURN
P7:
	LOAD isPoint7
	JPOS P8
	LOAD Point7X
	STORE curPointx;
	LOAD Point7Y
	STORE curPointy;
	LOAD Point8X
	STORE nextPointx;
	LOAD Point8Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint7
	LOADI 0
	STORE isPoint8
	RETURN
P8:
	LOAD isPoint8
	JPOS P9
	LOAD Point8X
	STORE curPointx
	LOAD Point8Y
	STORE curPointy
	LOAD Point9X
	STORE nextPointx
	LOAD Point9Y
	STORE nextPointy
	LOADI 1
	STORE isPoint8
	LOADI 0
	STORE isPoint9
	RETURN
P9:
	LOAD isPoint9
	JPOS P10
	LOAD Point9X
	STORE curPointx;
	LOAD Point9Y
	STORE curPointy;
	LOAD Point10X
	STORE nextPointx;
	LOAD Point10Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint9
	LOADI 0
	STORE isPoint10
	RETURN
P10:
	LOAD isPoint10
	JPOS P11
	LOAD Point10X
	STORE curPointx;
	LOAD Point10Y
	STORE curPointy;
	LOAD Point11X
	STORE nextPointx;
	LOAD Point11Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint10
	LOADI 0
	STORE isPoint11
	RETURN
P11:
	LOAD isPoint11	
	JPOS P12
	LOAD Point11X
	STORE curPointx;
	LOAD Point11Y
	STORE curPointy;
	LOAD Point12X
	STORE nextPointx;
	LOAD Point12Y;
	STORE nextPointy;
	LOADI 1
	STORE isPoint11
	RETURN
P12:
	LOAD 1
	STORE DONE
	RETURN

; Subroutine to wait the number of timer counts currently in AC
WaitAC:
	STORE  WaitTime
	OUT    Timer
WACLoop:
	IN     Timer
	SUB    WaitTime
	JNEG   WACLoop
	RETURN
	WaitTime: DW 0     ; "local" variable.

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	JUMP   DeadBatt    ; repeat forever

; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN

; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

; Subroutines to send AC value through the UART,
; Calling UARTSend2 will send both bytes of AC
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Calling UARTSend1 will only send the low byte.
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend2:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
UARTSend1:
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

; Subroutine to send a space to the computer log
UARTNBSP:
	LOAD   NBSP
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NBSP: DW &H201B

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN     UART_DAT
	JNEG   UARTClear
	RETURN

; Subroutine to tell the server that this position is one
; of the destinations.  Use AC=0 for generic indication,
; or AC=#1-12 for specific indication
IndicateDest:
	; AC contains which destination this is
	AND    LowNibl    ; keep only #s 0-15
	STORE  IDNumber
	LOADI  1
	STORE  IDFlag     ; set flag for indication
	RETURN
	IDNumber: DW 0
	IDFlag: DW 0


; Timer interrupt, used to send position data to the server
CTimer_ISR:
	CALL   UARTNL ; newline
	IN     XPOS
	CALL   UARTSend2
	IN     YPOS
	CALL   UARTSend2
	LOAD   IDFlag ; check if user has request a destination indication
	JPOS   CTIndicateDest ; if yes, do it; otherwise...
CTIndicateDest:
	LOAD   IDNumber
	CALL   UARTSend1 ; send the indicated destination
	LOADI  0
	STORE  IDFlag

; Configure the interrupt timer and enable interrupts
StartLog:
	; See supporting information on the powersof2 site for how
	; SCOMP's communication system works.
	CALL   UARTNL      ; send a newline to separate data
	LOADI  0
	STORE  IDFlag      ; clear any pending flag
	LOADI  50
	OUT    CTIMER      ; configure timer for 0.01*50=0.5s interrupts
	RETURN

; Disable the interrupt timer and interrupts
StopLog:
	LOADI  0
	OUT    CTIMER      ; reset configurable timer
	RETURN

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
; Mod360: modulo 360
; Returns AC%360 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************	
Mod360:
	JNEG   Mod180n      ; handle negatives
Mod360p:
	ADDI   -360
	JPOS   Mod360p      ; subtract 360 until negative
	ADDI   360         ; go back positive
	RETURN
Mod360n:
	ADDI   360          ; add 360 until positive
	JNEG   Mod360n
	ADDI   -360         ; go back negative
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

;***************************************************************
;* Variables
;***************************************************************
Temp:  DW 0 ; "Temp" is not a great name, but can be useful
Temp2: DW 0
Temp3: DW 0
CDX: DW 0      ; current desired X
CDY: DW 0      ; current desired Y
CDT: DW 0      ; current desired angle
CX:  DW 0      ; sampled X
CY:  DW 0      ; sampled Y
CT:  DW 0      ; sampled theta
CurrDist:   DW 0      ; Current Distance
OneFootLess: DW 0   ; One foot less than current dist
OLDPOS:     DW 0      ; 

DirAndAngTemp: DW 0
DirAndAngDiff: DW 0
DirectionToGo: DW 0
AngleToGo: DW 0
CurrAngle: DW 0

curPointx: DW 0
curPointy: DW 0
nextPointx: DW 0
nextPointy: DW 0

isPoint0: DW 1
Point0X: DW 0 
Point0Y: DW 0
Point0R: DW 0

isPoint1: DW 1
Point1X: DW 2
Point1Y: DW 1
Point1R: DW 0

isPoint2: DW 1
Point2X: DW 1
Point2Y: DW 2
Point2R: DW 0

isPoint3: DW 1
Point3X: DW 0
Point3Y: DW 0 
Point3R: DW 0

isPoint4: DW 1
Point4X: DW 2
Point4Y: DW 2 
Point4R: DW 0

isPoint5: DW 1
Point5X: DW 1 
Point5Y: DW 1 
Point5R: DW 0

isPoint6: DW 1
Point6X: DW 0 
Point6Y: DW 0 
Point6R: DW 0

isPoint7: DW 1
Point7X: DW 0 
Point7Y: DW 0 
Point7R: DW 0

isPoint8: DW 1
Point8X: DW 0 
Point8Y: DW 0 
Point8R: DW 0

isPoint9: DW 1
Point9X: DW 0 
Point9Y: DW 0 
Point9R: DW 0

isPoint10: DW 1
Point10X: DW 0 
Point10Y: DW 0 
Point10R: DW 0

isPoint11: DW 1
Point11X: DW 0 
Point11Y: DW 0 
Point11R: DW 0

isPoint12: DW 1   ; This is our STOPPING case
Point12X: DW 0 
Point12Y: DW 0 
Point12R: DW 0

thetaold: DW 0
thetanew: DW 0
amtTurned: DW 0
DONE: DW -1



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
SignBit:  DW &H8000
NOTSignBit: DW &H7FFF 

; some useful movement values
OneMeter: DW 952       ; ~1m in 1.05mm units
HalfMeter: DW 476      ; ~0.5m in 1.05mm units
OneFoot:  DW 290       ; ~1ft in 1.05mm robot units
TwoFeet:  DW 581       ; ~2ft in 1.05mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500
Deg45:	  DW 45
Deg315:    DW 315

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

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
UART_RDY: EQU &H98  ; UART status
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

TicksPerFoot: DW 290 ;from 1.05 mm / tick, ~304 mm / foot
Deg: DW 270
