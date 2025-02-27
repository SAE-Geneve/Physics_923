#ifndef PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
#define PHYSICS_923_LIB_COMMON_CONVERSIONS_H_

#include "vec2.h"
#include "commons.h"

namespace physics923::conversions {
    static constexpr physics923::commons::fp kMetersPerPixel = 0.01f;
    static constexpr physics923::commons::fp kRatioPixelsPerMeter = 100.0f;
    static constexpr physics923::commons::fp kPi = 3.14159265358979323846f;

    [[nodiscard]]

    constexpr physics923::commons::fp ConvertToMeters(const physics923::commons::fp pixels) noexcept {
        return pixels * kMetersPerPixel;
    }

    [[nodiscard]] constexpr physics923::commons::fp ConvertToPixels(const physics923::commons::fp meters) noexcept {
        return meters * kRatioPixelsPerMeter;
    }

    [[nodiscard]] constexpr math::Vec2f ConvertToMeters(const math::Vec2f pixels_pos) {
        return pixels_pos * kMetersPerPixel;
    }

    [[nodiscard]] constexpr math::Vec2f ConvertToPixels(const math::Vec2f pixels_pos) {
        return pixels_pos * kRatioPixelsPerMeter;
    }

    //input a radian value and it returns a degree value
    [[nodiscard]] constexpr physics923::commons::fp ToDegree(const physics923::commons::fp &rad_value) {
        return rad_value * 180 / kPi;
    }

//input a degree value and it returns a radian
    [[nodiscard]] constexpr physics923::commons::fp ToRadians(const physics923::commons::fp &degrees_value) {
        return degrees_value * kPi / 180;
    }
}

#endif //PHYSICS_923_LIB_COMMON_CONVERSIONS_H_
