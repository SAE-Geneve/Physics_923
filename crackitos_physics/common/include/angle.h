//
// Created by Mat on 3/11/2025.
//

#ifndef PHYSICSENGINE_CRACKITOS_PHYSICS_COMMON_INCLUDE_ANGLE_H_
#define PHYSICSENGINE_CRACKITOS_PHYSICS_COMMON_INCLUDE_ANGLE_H_

#include <iostream>
#include <cmath>

#include "commons.h"

namespace crackitos_physics::angles {
// Strongly-typed structures
struct Rad {
  double value;
  constexpr explicit Rad(double new_value) : value(new_value) {}
};

struct Deg {
  double value;
  constexpr explicit Deg(double new_value) : value(new_value) {}
};

// Non-type template function for conversion
template<typename From, typename To>
constexpr To Convert(From from);

template<>
constexpr Rad Convert(Deg deg) {
  return Rad{deg.value * (commons::kPi / 180.0)};
}

template<>
constexpr Deg Convert(Rad rad) {
  return Deg{rad.value * (180.0 / commons::kPi)};
}
}

#endif //PHYSICSENGINE_CRACKITOS_PHYSICS_COMMON_INCLUDE_ANGLE_H_
