#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

#include "Config.hpp"

namespace TFC {

// Moves the given value and into an rvalue.  Works whether or not the type has
// a valid move constructor or not.  Always leaves the given value in its
// default constructed state.
template<typename T>
T take(T& t) {
  T t2{};
  std::swap(t, t2);
  return t2;
}

}

#endif
