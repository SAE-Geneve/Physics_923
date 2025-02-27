#ifndef PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
#define PHYSICS_923_LIB_COMMON_CONVERSIONS_H_

#include "vec2.h"
#include "commons.h"

namespace physics923::conversions {
    static constexpr physics923::commons::fp kMetersPerPixel = 0.01f;
    static constexpr physics923::commons::fp kRatioPixelsPerMeter = 100.0f;

    [[nodiscard]]

    constexpr physics923::commons::fp PixelsToMeters(const physics923::commons::fp pixels) noexcept {
        return pixels * kMetersPerPixel;
    }

    [[nodiscard]] constexpr physics923::commons::fp MetersToPixels(const physics923::commons::fp meters) noexcept {
        return meters * kRatioPixelsPerMeter;
    }

    [[nodiscard]] constexpr math::Vec2f PixelsToMeters(const math::Vec2f pixels_pos) {
        return pixels_pos * kMetersPerPixel;
    }

    [[nodiscard]] constexpr math::Vec2f MetersToPixels(const math::Vec2f pixels_pos) {
        return pixels_pos * kRatioPixelsPerMeter;
    }

    //input a radian value and it returns a degree value
    [[nodiscard]] constexpr physics923::commons::fp RadToDegree(const physics923::commons::fp &rad_value) {
        return rad_value * 180 / commons::kPi;
    }

//input a degree value and it returns a radian
    [[nodiscard]] constexpr physics923::commons::fp DegreeToRad(const physics923::commons::fp &degrees_value) {
        return degrees_value * commons::kPi / 180;
    }
}

#endif //PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
