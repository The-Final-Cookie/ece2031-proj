function [leftWheelPower, rightWheelPower] = functionA(position, destination, heading)
  destVector = destination - position;
  destHeading = atan2(destVector(2), destVector(1));

  headingError = normalizeAngle(destHeading - heading);

  leftWheelPower = min(1 - 2/pi * headingError, 1);
  rightWheelPower = min(1 + 2/pi * headingError, 1);
end
