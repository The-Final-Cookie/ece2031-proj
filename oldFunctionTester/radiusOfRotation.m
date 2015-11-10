%% radiusOfRotation.m

function point = radiusOfRotation(leftWheelPower, rightWheelPower)
  % This is a bit intricate.  We need to determine the center point of rotation
  % based on the speed of both wheels.  For a rough idea of how the shape of
  % this function:
  % - If the numbers are the same then the answer is indeterminate, it could
  %   be either negative or positive infinity.  This must be handled through a
  %   special case.
  % - If the values are the same magnitude but have opposite sign, then the value
  %   of point is zero, representing a rotation about the position.
  % - If one value is zero and the other is non-zero, then the point of rotation
  %   is either one (if the right wheel is motionless) or negative one (if the
  %   left wheel if motionless)

  % special case indeterminate form, return boolean false
  if leftWheelPower == rightWheelPower
    point = false;
    return;
  end

  % otherwise the sign depends on both the slower value and the overall sign of
  % the system
  % read ~= as xor.
  isNeg = (leftWheelPower < rightWheelPower) ~= (leftWheelPower + rightWheelPower < 0);

  % because the direction is determined all that matters is the magnitude of
  % the larger vs the smaller value, so let's extract that
  larger = max([abs(leftWheelPower), abs(rightWheelPower)]);
  smaller = min([abs(leftWheelPower), abs(rightWheelPower)]);

  % next let's find the magnitude of the point of rotation
  % this formula comes from considering two concentric circles, we know that
  % the ratio of the circumferences from the larger to smaller.  And we know
  % the distance between the circles, so we can solve for the radius of the
  % larger, then subtract 1 to find the radius at the position.
  point = 2 / (1 - sign(leftWheelPower)*sign(rightWheelPower)*smaller/larger) - 1;

  % and copy the sign
  if isNeg
    point = -point;
  end
end
