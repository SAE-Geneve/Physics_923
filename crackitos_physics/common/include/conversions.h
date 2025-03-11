#ifndef CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_
#define CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_

#include "vec2.h"
#include "commons.h"

namespace crackitos_physics::conversions {
static constexpr commons::fp kMetersPerPixel = 0.01f;
static constexpr commons::fp kRatioPixelsPerMeter = 100.0f;
//TODO std::ratio

[[nodiscard]]constexpr commons::fp PixelsToMeters(const commons::fp pixels) noexcept {
  return pixels * kMetersPerPixel;
}

[[nodiscard]] constexpr commons::fp MetersToPixels(
    const commons::fp meters) noexcept {
  return meters * kRatioPixelsPerMeter;
}

[[nodiscard]] constexpr math::Vec2f PixelsToMeters(const math::Vec2f pixels_pos) {
  return pixels_pos * kMetersPerPixel;
}

[[nodiscard]] constexpr math::Vec2f MetersToPixels(const math::Vec2f pixels_pos) {
  return pixels_pos * kRatioPixelsPerMeter;
}
} // namespace conversions
#endif // CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_
