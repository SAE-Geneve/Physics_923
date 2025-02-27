#ifndef CRACKITOS_PHYSICS_MATH_FOUR_VEC4_H_
#define CRACKITOS_PHYSICS_MATH_FOUR_VEC4_H_

#include <array>

#include "vec4.h"
#include "commons.h"

namespace crackitos_physics::math
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
        FourVec4<T> operator*(const crackitos_physics::commons::fp scalar) const; //Multiply by scalar
        FourVec4<T> operator/(const crackitos_physics::commons::fp scalar) const; //Divide by scalar
        std::array<crackitos_physics::commons::fp, 4> Dot(const FourVec4<T>& other) const; //Dot
        std::array<crackitos_physics::commons::fp, 4> SquareMagnitude() const; //SquareMagnitude
        std::array<crackitos_physics::commons::fp, 4> Magnitude() const; //Magnitude/Sqrroot
        FourVec4<T> Normalize() const; //Normalize
    };

    using FourVec4i = FourVec4<int32_t>;
    using FourVec4f = FourVec4<crackitos_physics::commons::fp>;

    //Specialization for physics923::commons::fp
    template <>
    FourVec4f FourVec4<crackitos_physics::commons::fp>::operator+(const FourVec4f& other) const;
    template <>
    FourVec4f FourVec4<crackitos_physics::commons::fp>::operator-(const FourVec4f& other) const;
    template <>
    FourVec4f FourVec4<crackitos_physics::commons::fp>::operator-() const;
    template <>
    FourVec4f FourVec4<crackitos_physics::commons::fp>::operator*(const crackitos_physics::commons::fp scalar) const;
    template <>
    FourVec4f FourVec4<crackitos_physics::commons::fp>::operator/(const crackitos_physics::commons::fp scalar) const;
    template <>
    std::array<crackitos_physics::commons::fp, 4> FourVec4<crackitos_physics::commons::fp>::Dot(
        const FourVec4f& other) const;
    template <>
    std::array<crackitos_physics::commons::fp, 4> FourVec4<crackitos_physics::commons::fp>::SquareMagnitude() const;
    template <>
    std::array<crackitos_physics::commons::fp, 4> FourVec4<crackitos_physics::commons::fp>::Magnitude() const;
    template <>
    FourVec4f FourVec4<crackitos_physics::commons::fp>::Normalize() const;
} // namespace math
#endif // CRACKITOS_PHYSICS_MATH_FOUR_VEC4_H_
