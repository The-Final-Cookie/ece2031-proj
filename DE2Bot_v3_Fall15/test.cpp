{
  currentPoint = {0, 0, 0}

  for (int i = 0; i < 12; ++i) {
  
    pointsLeft = 12 - i;

    bestCost = 0x7FFF
    bestPoint = {0, 0, 0}

    for (int j = 0; j < pointsLeft; ++j) {
      thisCost = calculateCost(currentPoint, points[j])
      if (thisCost < bestCost)
        bestPoint = points[j]
    }

    outPoints[i] = bestPoint

    swap(points[pointsLeft - 1], points[bestPoint[2] - 1]) // remove the point from
    // consideration these points are already tagged, so this is NBD as far as
    // order is concerned

    currentPoint = bestPoint
  }
}

{
  calculateCost()

  xdiff = currentPoint[0] - points[i][0]
  ydiff = currentPoint[1] - points[i][1]
  cost = pyth(a, b) + mod(atan2(ydiff, xdiff) - theta, pi/2) * 113 // 227 is axle_track radius in robounits
}

{
  // CurrAngle = atan2(destY - Y, destX - X)
  // Theta 

  currentTheta = IN THETA
  diff = ((CurrAngle - currentTheta) % 360) - 180
  if (diff > 0) {
    direction = 1 // clockwise
    diff += 90
  } else {
    direction = 0 // counterclockwise
    diff = currAngle - currentTheta
  }
}
