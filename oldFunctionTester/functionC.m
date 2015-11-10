function [leftWheelPower, rightWheelPower] = functionC(position, destination, heading)
  destVector = destination - position;
  destHeading = atan2(destVector(2), destVector(1));

  headingError = normalizeAngle(destHeading - heading);

  if (headingError < pi/6)
    leftWheelPower = 1;
  else
    leftWheelPower = -1;
  end

  if (headingError > -pi/6)
    rightWheelPower = 1;
  else
    rightWheelPower = -1;
  end
end
