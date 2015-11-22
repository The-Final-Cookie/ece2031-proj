#ifndef UTIL_HPP
#define UTIL_HPP

#include <limits>

namespace TFC {

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
  almost_equal(T x, T y, int ulp = 100) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::abs(x-y) < std::numeric_limits<T>::epsilon() * std::abs(x+y) * ulp
    // unless the result is subnormal
    || std::abs(x-y) < std::numeric_limits<T>::min();
}

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
  nearZero(T x, int ulp = 100) {
  return almost_equal(x, (T)0, ulp);
}

template<typename T>
T clamp(T const val, T const min, T const max) {
    return std::min(std::max(val, min), max);
}

template<typename T>
T approach(T goal, T current, T rate) {
  if (goal < current) {
    return max(current - rate, goal);
  } else if (goal > current) {
    return min(current + rate, goal);
  } else {
    return current;
  }
}

template <typename T>
T normalizeAngle(T in) {
  T out = fmod(in, 2*M_PI);
  if (out > M_PI) {
    out = out - 2*M_PI;
  }

  return out;
}

}

#endif
