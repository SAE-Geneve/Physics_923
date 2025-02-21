#ifndef PHYSICS_923_LIB_MATH_QUATERNION_H_
#define PHYSICS_923_LIB_MATH_QUATERNION_H_

#include "vec3.h"

namespace physics923::math
{
    template <typename T>
    struct Quaternion
    {
        // W is the real part, V is the imaginary part comprised of x, y, z
        // Default is the identity quaternion
        T W{1};
        Vec3<T> V{0,0,0};

        Quaternion() = default;

        Quaternion(T w, const Vec3<T>& v) : W{w}, V{v} {}
        Quaternion(T w, T x, T y, T z) : W{w}, V{x, y, z} {};

        static constexpr Quaternion Identity()
        {
            return Quaternion(1, 0, 0, 0);
        }

        constexpr Quaternion operator*(const Quaternion& other) const
        {
            Quaternion result;
            result.W = W * other.W - Vec3<T>::Dot(V, other.V);
            result.V = V*other.W + other.V*W + Vec3<T>::Cross(V, other.V);
            return result;
        }

        [[nodiscard]] constexpr Quaternion Conjugate() const
        {
            Quaternion result;
            result.W = W;
            result.V.x = -V.x;
            result.V.y = -V.y;
            result.V.z = -V.z;
            return result;
        }

        //TODO: modify once Angle is implemented. Also implement own trigonometry (cos & sin) to be able to constexpr
        [[nodiscard]] Quaternion EulerToQuaternion(const float x, const float y, const float z) const
        {
            const float cosX = std::cos(x * 0.5f);
            const float sinX = std::sin(x * 0.5f);
            const float cosY = std::cos(y * 0.5f);
            const float sinY = std::sin(y * 0.5f);
            const float cosZ = std::cos(z * 0.5f);
            const float sinZ = std::sin(z * 0.5f);

            Quaternion result;
            result.W = cosX*cosY*cosZ + sinX*sinY*sinZ;
            result.V.x = sinX*cosY*cosZ - cosX*sinY*sinZ;
            result.V.y = cosX*sinY*cosZ + sinX*cosY*sinZ;
            result.V.z = cosX*cosY*sinZ - sinX*sinY*cosZ;

            return result;
        }
    };
} // namespace math


#endif //PHYSICS_923_LIB_MATH_QUATERNION_H_
