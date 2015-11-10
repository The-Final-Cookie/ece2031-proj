function [leftWheelPower, rightWheelPower] = functionB(position, destination, heading)
  destVector = destination - position;
  destHeading = atan2(destVector(2), destVector(1));

  headingError = normalizeAngle(destHeading - heading);

  leftWheelPower = min(1 - 1/pi * headingError, 1);
  rightWheelPower = min(1 + 1/pi * headingError, 1);
end
