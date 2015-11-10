%% runSimulation
%
%  runSimulation is a function that simulates a Braitenberg machine with a
%  given stepFunction until the destination becomes close to the goal.  Where
%  close is defined by default as within 1.1x of a step at max velocity to the
%  destination, or spending more than 100 steps within 2x (a failsafe to
%  prevent an infinite loop), or running maxSteps steps (a failsafe to prevent
%  broken functions).
%
%  Inputs:
%    stepFunc - (function handle) The step function.  Expected to take three
%               arguments and return two as follows:
%      Inputs:
%        position - (double) A 1x2 matrix of the current position
%        destination - (double) A 1x2 matrix of the destination position
%        heading - (double) A scalar value from -pi to pi representing the
%                  current facing direction of the robot. Standard mathematics
%                  conventions apply.
%      Outputs:
%        leftWheelPower - (double) A scalar value of the power to the left
%                         wheel from -1 to 1
%        rightWheelPower - (double) A scalar value of the power to the right
%                          wheel from -1 to 1
%
%    origin - (double) a 1x2 matrix of the initial position in x,y coord form
%    destination - (double) A 1x2 matrix of the sought position in x,y coord
%                  form
%    initialHeading - (double) A scalar value from -pi to pi representing the
%                     initial heading of the robot. Standard mathematics
%                     conventions apply.
%    tdelta - (optional double) A scalar value of the timestep.  This is the
%             number of time units between two steps.  A good value is probably
%             1/100 or so, the smaller the more accurate.
%    distanceBetweenWheels - (optional double) A scalar value in the same units
%                            as origin and destination that specify how far the
%                            left wheel is from the right wheel.  The position
%                            of the vehicle is given as the midpoint between
%                            the two wheels.
%    leftWheelColors - (optional double) A  3x3 grid of color information, each
%                      row represents a color specified as an RGB triplet from
%                      0 to 1.  The top row represents the color of max power,
%                      the bottom row represents minimum power, and the middle
%                      row, represents the zero power color.  For values
%                      between these a linear interpolation is performed.  The
%                      left wheel is the wheel on the y-positive axis when the
%                      robot is at the origin at a heading of 0.
%    rightWheelColors - (optional double) The same as leftWheelColors, except
%                       for the right wheel.
%    goalRadius - (optional double) A scalar value representing the goal
%                 radius.  Defaults to 1.1 x tdelta
%    failsafeRadius - (optional double) A 1x2 double, the first number is the
%                     radius, should be larger than goalRadius to be effective.
%                     The second number represents the number of steps to spend
%                     in this radius before aborting.  This is to combat
%                     potential orbiting behavior.
%    maxSteps - (optional double) Abort computation if this number is exceeded.
%               Use -1 to force running forever.
%    bounds - (optional double) A 2x2 matrix, the first row represents the x,y
%             coords of the minimum bounds, the second represents the x,y
%             coords of the maximum bounds.  If the robot exits the rectangle
%             given by these coords then the simulation aborts.  Default is the
%             square centered on the destination that is has a side length of
%             10x the distance between the origin and the destination.
%
%  Outputs:
%    status - (double) A scalar value representing the result of the simulation.
%             A value of 0 is considered successful.
%             A value of 1 means that the simulation was aborted because it
%             entered the failsafeRadius but did not reach the goal in time.
%             A value of 2 means that the simulation aborted due to too many
%             steps being taken.
%             A value of 3 means that the simulation aborted due to the robot
%             going out of bounds.
%    leftWheelSteps - (double) An nx2 matrix representing the path the left
%                     wheel took over time
%    rightWheelSteps - (double) An nx2 matrix representing the path the right
%                      wheel took over time
%    leftWheelOutColors - (double) An nx3 matrix representing the output colors of
%                      the left wheel over time
%    rightWheelOutColors - (double) An nx3 matrix representing the output colors of
%                      the right wheel over time

function [status, leftWheelSteps, rightWheelSteps, leftWheelOutColors, rightWheelOutColors] = runSimulation(stepFunc, origin, destination, initialHeading, tdelta, distanceBetweenWheels, leftWheelColors, rightWheelColors, goalRadius, failsafeRadius, maxSteps, bounds)
  if nargin < 13
    sideDelta = 5*norm(origin - destination);
    bounds = [ destination - sideDelta; destination + sideDelta ];
  end
  if nargin < 11
    maxSteps = 10000;
  end
  if nargin < 8
    rightWheelColors = [0 1 0.5; 0 0 0; 1 0 0.5];
  end
  if nargin < 7
    leftWheelColors = [0 0.5 1; 0 0 0; 1 0.5 0];
  end
  if nargin < 6
    distanceBetweenWheels = 1;
  end
  if nargin < 5
    tdelta = 1/100;
  end
  if nargin < 10
    failsafeRadius(1) = 2 * tdelta;
    failsafeRadius(2) = 100;
  end
  if nargin < 9
    goalRadius = 1.1 * tdelta;
  end

  leftWheelSteps = [leftWheelPosition(origin, initialHeading)];
  rightWheelSteps = [rightWheelPosition(origin, initialHeading)];
  leftWheelOutColors = [colorFromPower(0, leftWheelColors)];
  rightWheelOutColors = [colorFromPower(0, rightWheelColors)];
  position = origin;
  currentHeading = initialHeading;

  numSteps = 0;
  failsafeSteps = 0;

  status = 0;

  while 1
    [leftWheelPower, rightWheelPower] = stepFunc(position, destination, currentHeading);

    % traveled is the distance traveled around the circle in radians (convenient!)
    traveled = mean([leftWheelPower, rightWheelPower]) * tdelta;

    radius = radiusOfRotation(leftWheelPower, rightWheelPower);

    % if we're traveling with the same power given to both wheels
    if islogical(radius)
      position = position + [cos(currentHeading), sin(currentHeading)] * traveled;
    else

      [newPosition, newHeading] = circleMotion(position, currentHeading, traveled, radius);

      if islogical(newPosition)
        changeInAngle = rightWheelPower / pi;
        currentHeading = currentHeading + changeInAngle;
      else
        position = newPosition;
        currentHeading = newHeading;
      end

      % need to normalize to between pi and -pi because other functions are expecting this
      currentHeading = normalizeAngle(currentHeading);
    end

    leftWheelSteps(end+1,:) = leftWheelPosition(position, currentHeading);
    rightWheelSteps(end+1,:) = rightWheelPosition(position, currentHeading);
    leftWheelOutColors(end+1,:) = colorFromPower(leftWheelPower, leftWheelColors);
    rightWheelOutColors(end+1,:) = colorFromPower(rightWheelPower, rightWheelColors);

    % finally let's check if we need to bail
    distToGoal = norm(destination - position);
    if distToGoal < goalRadius
      break;
    end
    if distToGoal < failsafeRadius(1)
      if failsafeSteps >= failsafeRadius(2)
        status = 1;
        break;
      else
        failsafeSteps = failsafeSteps + 1;
      end
    % one would expect these lines; however, if we enter and then leave the
    % goal area, we are probably on an elliptical orbit, we don't want to clear
    % progress in this case

    % else
    %   failsafeSteps = 0;
    end

    numSteps = numSteps + 1;
    if numSteps >= maxSteps || maxSteps == -1
      status = 2;
      break;
    end

    if any(position < bounds(1,:)) || any(position > bounds(2,:))
      status = 3;
      break;
    end
  end
end

function pos = leftWheelPosition(position, heading)
  pos(1) = position(1) + cos(heading + pi/2);
  pos(2) = position(2) + sin(heading + pi/2);
end

function pos = rightWheelPosition(position, heading)
  pos(1) = position(1) + cos(heading - pi/2);
  pos(2) = position(2) + sin(heading - pi/2);
end

function color = colorFromPower(power, colorSet)
  color(1) = interp1([1, 0, -1], colorSet(:,1)', power);
  color(2) = interp1([1, 0, -1], colorSet(:,2)', power);
  color(3) = interp1([1, 0, -1], colorSet(:,3)', power);
end
