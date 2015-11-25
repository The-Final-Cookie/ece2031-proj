CalcDecDist:
  IN LVEL
  STORE Mean2Arg
  IN RVEL
  CALL Mean2

  STORE  m16sA      
  STORE  m16sB        
  CALL   Mult16s ; The low word is already in AC

  SHIFT  -10 ; / 1024
  STORE  DecDist ; Low 6 bits only
  LOAD mres16sH
  SHIFT 6
  OR DecDist
  STORE DecDist
  RETURN 
