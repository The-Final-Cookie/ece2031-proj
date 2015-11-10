function [leftWheelPower, rightWheelPower] = functionD(position, destination, heading)
  destVector = destination - position;
  destHeading = atan2(destVector(2), destVector(1));

  headingError = normalizeAngle(destHeading - heading);

  if (abs(headingError) <= pi/2)
    leftWheelPower = min(1 - 4/pi * headingError, 1);
    rightWheelPower = min(1 + 4/pi * headingError, 1);
  else
    % handle the moving backward case, first we need to make sure that our area
    % is continuous, so we renormalize
    headingError = normalizeAngle(headingError + pi);

    leftWheelPower = max(-1 - 4/pi * headingError, -1);
    rightWheelPower = max(-1 + 4/pi * headingError, -1);
  end

end
