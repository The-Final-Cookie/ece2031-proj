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
  CALL PosModulo ; %360
  STORE AngleToGo
  RETURN


DirAndAngTemp: DW 0
DirAndAngDiff: DW 0
DirectionToGo: DW 0
AngleToGo: DW 0
CurrAngle: DW 0

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

THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
