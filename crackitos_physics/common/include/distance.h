#ifndef CRACKITOS_PHYSICS_COMMON_DISTANCE_H_
#define CRACKITOS_PHYSICS_COMMON_DISTANCE_H_

#include "commons.h"

namespace crackitos_physics::distance {
// Strongly-typed structures
struct Meter {
  double value;
  commons::fp pixel_per_meter_ratio = 100.0f;
  constexpr explicit Meter(double new_value) : value(new_value) {}
  constexpr explicit Meter(double new_value, float ratio) : value(new_value), pixel_per_meter_ratio(ratio) {}
};

struct Pixel {
  double value;
  commons::fp meter_per_pixel_ratio = 0.01f;
  constexpr explicit Pixel(double new_value) : value(new_value) {}
  constexpr explicit Pixel(double new_value, float ratio) : value(new_value), meter_per_pixel_ratio(ratio) {}
};

// Non-type template function for conversion
template<typename From, typename To>
constexpr To Convert(From from);

template<>
constexpr Meter Convert(Pixel pix) {
  return Meter{pix.value * pix.meter_per_pixel_ratio};
}

template<>
constexpr Pixel Convert(Meter met) {
  return Pixel{met.value * met.pixel_per_meter_ratio};
}
}

#endif //CRACKITOS_PHYSICS_COMMON_DISTANCE_H_
