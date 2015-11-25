// Given deceleration, and current speed, output the distance of deceleration before the robot stops at one point.
CalcDecDist: LOAD LVEL
	ADD    RVEL
	SHIFT  -1
	STORE  m16sA      
	LOAD   LVEL
	ADD    RVEL
	SHIFT  -1
	STORE  m16sB        
	CALL   Mult16s ; The low word is already in AC
    SHIFT  -10 ; / 1024
	STORE  DecDist ; Low 6 bits only
	LOAD mres16sH
	SHIFT 6
	OR DecDist
	STORE DecDist
	RETURN 
	
DecDist: DW 0
