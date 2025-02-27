#ifndef CRACKITOS_PHYSICS_MATH_FOUR_VEC3_H_
#define CRACKITOS_PHYSICS_MATH_FOUR_VEC3_H_

#include <array>

#include "vec3.h"
#include "commons.h"

namespace crackitos_physics::math
{
    template <typename T>
    struct FourVec3
    {
        std::array<T, 4> x{};
        std::array<T, 4> y{};
        std::array<T, 4> z{};

        FourVec3() = default;

        explicit constexpr FourVec3(const std::array<Vec3<T>, 4>& vec)
        {
            for (int i = 0; i < 4; i++)
            {
                x[i] = vec[i].x;
                y[i] = vec[i].y;
                z[i] = vec[i].z;
            }
        }

        FourVec3<T> operator+(const FourVec3<T>& other) const;
        FourVec3<T> operator-(const FourVec3<T>& other) const;
        FourVec3<T> operator-() const; //Opposite
        FourVec3<T> operator*(const crackitos_physics::commons::fp scalar) const; //Multiply by scalar
        FourVec3<T> operator/(const crackitos_physics::commons::fp scalar) const; //Divide by scalar
        std::array<crackitos_physics::commons::fp, 4> Dot(const FourVec3<T>& other) const; //Dot
        std::array<crackitos_physics::commons::fp, 4> SquareMagnitude() const; //SquareMagnitude
        std::array<crackitos_physics::commons::fp, 4> Magnitude() const; //Magnitude/Sqrroot
        FourVec3<T> Normalize() const; //Normalize
    };

    using FourVec3i = FourVec3<int32_t>;
    using FourVec3f = FourVec3<crackitos_physics::commons::fp>;

    //Specialization for physics923::commons::fp
    template <>
    FourVec3f FourVec3<crackitos_physics::commons::fp>::operator+(const FourVec3f& other) const;
    template <>
    FourVec3f FourVec3<crackitos_physics::commons::fp>::operator-(const FourVec3f& other) const;
    template <>
    FourVec3f FourVec3<crackitos_physics::commons::fp>::operator-() const;
    template <>
    FourVec3f FourVec3<crackitos_physics::commons::fp>::operator*(const crackitos_physics::commons::fp scalar) const;
    template <>
    FourVec3f FourVec3<crackitos_physics::commons::fp>::operator/(const crackitos_physics::commons::fp scalar) const;
    template <>
    std::array<crackitos_physics::commons::fp, 4> FourVec3<crackitos_physics::commons::fp>::Dot(
        const FourVec3f& other) const;
    template <>
    std::array<crackitos_physics::commons::fp, 4> FourVec3<crackitos_physics::commons::fp>::SquareMagnitude() const;
    template <>
    std::array<crackitos_physics::commons::fp, 4> FourVec3<crackitos_physics::commons::fp>::Magnitude() const;
    template <>
    FourVec3f FourVec3<crackitos_physics::commons::fp>::Normalize() const;
} // namespace math
#endif // CRACKITOS_PHYSICS_MATH_FOUR_VEC3_H_
