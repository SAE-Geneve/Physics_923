#ifndef CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_
#define CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_

#include "vec2.h"
#include "commons.h"

namespace crackitos_physics::conversions
{
    static constexpr commons::fp kMetersPerPixel = 0.01f;
    static constexpr commons::fp kRatioPixelsPerMeter = 100.0f;
    //TODO switch to nttp (non type template parameter) & std::ratio
    // helpers
    // struct of statics

    [[nodiscard]]

    constexpr commons::fp PixelsToMeters(const commons::fp pixels) noexcept
    {
        return pixels * kMetersPerPixel;
    }

    [[nodiscard]] constexpr commons::fp MetersToPixels(
        const commons::fp meters) noexcept
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

    //TODO class degree and radian with constructors for both & cast operator

    //input a radian value and it returns a degree value
    [[nodiscard]] constexpr commons::fp RadToDegree(const commons::fp& rad_value)
    {
        return rad_value * 180 / commons::kPi;
    }

    //input a degree value and it returns a radian
    [[nodiscard]] constexpr commons::fp DegreeToRad(const commons::fp& degrees_value)
    {
        return degrees_value * commons::kPi / 180;
    }
} // namespace conversions
#endif // CRACKITOS_PHYSICS_COMMON_CONVERSIONS_H_
