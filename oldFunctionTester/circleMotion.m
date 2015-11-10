function [position, heading] = circleMotion(position, heading, radians, radius)
  % special case to avoid division by zero, need to handle separately
  if radius == 0
    position = false;
    heading = false;
    return;
  end

  angle = normalizeAngle(heading + pi/2);
  traversedAngle = -radians / radius;
  
  newAngle = angle + traversedAngle;
  positionDelta = [cos(newAngle) - cos(angle), sin(newAngle) - sin(angle)] * radius;

  position = position + positionDelta;
  heading = normalizeAngle(newAngle - pi/2);
end
