#ifndef PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
#define PHYSICS_923_LIB_COMMON_CONVERSIONS_H_

#include "vec2.h"

namespace physics923::conversions
{
    //TODO rename conversion
    //TODO radians & angles
    static constexpr float kMetersPerPixel = 0.01f;
    static constexpr float kRatioPixelsPerMeter = 100.0f;

    [[nodiscard]] constexpr float ConvertToMeters(const float pixels) noexcept
    {
        return pixels * kMetersPerPixel;
    }

    [[nodiscard]] constexpr float ConvertToPixels(const float meters) noexcept
    {
        return meters * kRatioPixelsPerMeter;
    }

    [[nodiscard]] constexpr math::Vec2f ConvertToMeters(const math::Vec2f pixels_pos)
    {
        return pixels_pos * kMetersPerPixel;
    }

    [[nodiscard]] constexpr math::Vec2f ConvertToPixels(const math::Vec2f pixels_pos)
    {
        return pixels_pos * kRatioPixelsPerMeter;
    }
}

#endif //PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
