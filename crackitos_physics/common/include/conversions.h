#ifndef CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_
#define CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_

#include "vec2.h"
#include "commons.h"

namespace crackitos_physics::conversions
{
    static constexpr crackitos_physics::commons::fp kMetersPerPixel = 0.01f;
    static constexpr crackitos_physics::commons::fp kRatioPixelsPerMeter = 100.0f;

    [[nodiscard]]

    constexpr crackitos_physics::commons::fp PixelsToMeters(const crackitos_physics::commons::fp pixels) noexcept
    {
        return pixels * kMetersPerPixel;
    }

    [[nodiscard]] constexpr crackitos_physics::commons::fp MetersToPixels(
        const crackitos_physics::commons::fp meters) noexcept
    {
        return meters * kRatioPixelsPerMeter;
    }

    [[nodiscard]] constexpr math::Vec2f PixelsToMeters(const math::Vec2f pixels_pos)
    {
        return pixels_pos * kMetersPerPixel;
    }

    [[nodiscard]] constexpr math::Vec2f MetersToPixels(const math::Vec2f pixels_pos)
    {
        return pixels_pos * kRatioPixelsPerMeter;
    }

    //input a radian value and it returns a degree value
    [[nodiscard]] constexpr crackitos_physics::commons::fp RadToDegree(const crackitos_physics::commons::fp& rad_value)
    {
        return rad_value * 180 / commons::kPi;
    }

    //input a degree value and it returns a radian
    [[nodiscard]] constexpr crackitos_physics::commons::fp DegreeToRad(
        const crackitos_physics::commons::fp& degrees_value)
    {
        return degrees_value * commons::kPi / 180;
    }
} // namespace conversions
#endif // CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_
