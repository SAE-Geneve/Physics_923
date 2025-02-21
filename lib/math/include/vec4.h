#ifndef PHYSICS_923_LIB_MATH_VEC4_H_
#define PHYSICS_923_LIB_MATH_VEC4_H_

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace physics923::math
{
    /**
     * \brief Vec4 is a mathematical object represented by four values of type T
     */
    template <typename T>
    struct Vec4
    {
        T x = 0, y = 0, z = 0, w = 0;

        // Default constructor
        Vec4() = default; // Default constructor

        // Constructor with parameters
        constexpr Vec4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w)
        {
        }

        constexpr Vec4(std::initializer_list<T> values)
        {
            assert(values.size() == 4 && "Vector4 requires 4 values");
            auto it = values.begin();
            x = *it++;
            y = *it++;
            z = *it++;
            w = *it;
        }

        static constexpr Vec4 zero()
        {
            return Vec4(static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0));
        }

        static constexpr Vec4 one()
        {
            return Vec4(static_cast<T>(1), static_cast<T>(1), static_cast<T>(1), static_cast<T>(1));
        }

        //Addition
        constexpr Vec4 operator+(const Vec4& other) const
        {
            return Vec4{x + other.x, y + other.y, z + other.z, w + other.w};
        }

        //Subtraction
        constexpr Vec4 operator-(const Vec4& other) const
        {
            return {x - other.x, y - other.y, z - other.z, w - other.w};
        }

        //Opposite
        constexpr Vec4 operator-() const
        {
            return {-x, -y, -z, -w};
        }

        //Multiplication by scalar
        constexpr Vec4 operator*(T scalar) const
        {
            return {x * scalar, y * scalar, z * scalar, w * scalar};
        }

        //Division by scalar
        constexpr Vec4 operator/(T scalar) const
        {
            if (scalar == 0) return {0, 0, 0, 0};
            return {x / scalar, y / scalar, z / scalar, w / scalar};
        }

        T& operator[](const int index)
        {
            if (index == 0) return x;
            if (index == 1) return y;
            if (index == 2) return z;
            if (index == 3) return w;
            throw std::out_of_range("Index out of range for Vec4");
        }

        const T& operator[](const int index) const
        {
            if (index == 0) return x;
            if (index == 1) return y;
            if (index == 2) return z;
            if (index == 3) return w;
            throw std::out_of_range("Index out of range for Vec4");
        }

        //Dot product
        static constexpr T Dot(const Vec4& v1, const Vec4& v2)
        {
            return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
        }

        //Vector square length
        [[nodiscard]] constexpr T SquareMagnitude() const
        {
            return x * x + y * y + z * z + w * w;
        }

        //Vector length
        [[nodiscard]] constexpr T Magnitude() const
        {
            return std::sqrt(SquareMagnitude());
        }

        //Normalized vector
        [[nodiscard]] constexpr Vec4 Normalized() const
        {
            float mag = Magnitude();
            if (mag == 0) return {0, 0, 0, 0};
            return {x / mag, y / mag, z / mag, w / mag};
        }

        //Interpolate between this Vec4 and other by t amount
        [[nodiscard]] constexpr Vec4 LERP(const Vec4& other, T t) const
        {
            return {x + t * (other.x - x), y + t * (other.y - y), z + t * (other.z - z), w + t * (other.w - w)};
        }
    }; // struct Vec4

    //Operator overload for multiplication
    template <typename T>
    constexpr Vec4<T> operator*(T scalar, const Vec4<T>& vec)
    {
        return {scalar * vec.x, scalar * vec.y, scalar * vec.z, scalar * vec.w};
    }
} // namespace math
#endif // PHYSICS_923_LIB_MATH_VEC4_H_
