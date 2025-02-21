#ifndef PHYSICS_923_LIB_MATH_FOUR_VEC4_H_
#define PHYSICS_923_LIB_MATH_FOUR_VEC4_H_

#include <array>

#include "vec4.h"

namespace physics923::math
{
    template <typename T>
    struct FourVec4
    {
        std::array<T, 4> x{};
        std::array<T, 4> y{};
        std::array<T, 4> z{};
        std::array<T, 4> w{};

        FourVec4() = default;

        explicit constexpr FourVec4(const std::array<Vec4<T>, 4>& vec)
        {
            for (int i = 0; i < 4; i++)
            {
                x[i] = vec[i].x;
                y[i] = vec[i].y;
                z[i] = vec[i].z;
                w[i] = vec[i].w;
            }
        }

        FourVec4<T> operator+(const FourVec4<T>& other) const;
        FourVec4<T> operator-(const FourVec4<T>& other) const;
        FourVec4<T> operator-() const; //Opposite
        FourVec4<T> operator*(const float scalar) const; //Multiply by scalar
        FourVec4<T> operator/(const float scalar) const; //Divide by scalar
        std::array<float, 4> Dot(const FourVec4<T>& other) const; //Dot
        std::array<float, 4> SquareMagnitude() const; //SquareMagnitude
        std::array<float, 4> Magnitude() const; //Magnitude/Sqrroot
        FourVec4<T> Normalize() const; //Normalize
    };

    using FourVec4i = FourVec4<int32_t>;
    using FourVec4f = FourVec4<float>;

    //Specialization for float
    template <>
    FourVec4f FourVec4<float>::operator+(const FourVec4f& other) const;
    template <>
    FourVec4f FourVec4<float>::operator-(const FourVec4f& other) const;
    template <>
    FourVec4f FourVec4<float>::operator-() const;
    template <>
    FourVec4f FourVec4<float>::operator*(const float scalar) const;
    template <>
    FourVec4f FourVec4<float>::operator/(const float scalar) const;
    template <>
    std::array<float, 4> FourVec4<float>::Dot(const FourVec4f& other) const;
    template <>
    std::array<float, 4> FourVec4<float>::SquareMagnitude() const;
    template <>
    std::array<float, 4> FourVec4<float>::Magnitude() const;
    template <>
    FourVec4f FourVec4<float>::Normalize() const;
}

#endif //KUMA_ENGINE_LIB_MATH_FOUR_VEC4_H_
