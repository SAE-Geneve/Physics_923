#ifndef PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
#define PHYSICS_923_LIB_COMMON_CONVERSIONS_H_

#include "vec2.h"
#include "commons.h"

namespace physics923::conversions
{
    //TODO rename conversion
    //TODO radians & angles
    static constexpr physics923::commons::fp kMetersPerPixel = 0.01f;
    static constexpr physics923::commons::fp kRatioPixelsPerMeter = 100.0f;

    [[nodiscard]] constexpr physics923::commons::fp ConvertToMeters(const physics923::commons::fp pixels) noexcept
    {
        return pixels * kMetersPerPixel;
    }

    [[nodiscard]] constexpr physics923::commons::fp ConvertToPixels(const physics923::commons::fp meters) noexcept
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
