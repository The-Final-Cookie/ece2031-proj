; Direction and Angle, to call set CurrAngle, this function will read from THETA
; directly, output is in DirectionToGo (clockwise = 1, ccw = 0) and AngleToGo
; range (0-359)

DirectionAndAngle:
  LOADI 360
  STORE ModuloD
  IN THETA
  STORE DirAndAngTemp
  LOAD CurrAngle
  SUB DirAndAngTemp
  CALL Modulo
  ADDI -180 ; diff = ((CurrAngle - CurrTheta) % 360) - 180
  JNEG DirectionAndAngle_CCW
  ADDI 90
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
  STORE AngleToGo
  RETURN


DirAndAngTemp: DW 0
DirAndAngDiff: DW 0
DirectionToGo: DW 0
AngleToGo: DW 0
CurrAngle: DW 0

;*******************************************************************************
; Modulo: modulo
; Returns AC%ModuloD in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************	
Modulo:
  JZERO  Modulo_bail
	JNEG   ModuloN      ; handle negatives
ModuloP:
	ADD    ModuloD
	JPOS   ModuloP      ; subtract until negative
	ADD    ModuloD      ; go back positive
	RETURN
ModuloN:
	ADDI   ModuloD      ; add until positive
	JNEG   ModuloN
	ADDI   ModuloD      ; go back negative
	RETURN
Modulo_bail:
  RETURN

ModuloD: DW 0

THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
