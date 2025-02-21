#ifndef PHYSICS_923_LIB_MATH_VEC2_H_
#define PHYSICS_923_LIB_MATH_VEC2_H_

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace physics923::math
{
    /**
     * \brief Vec2 is a mathematical object represented by two values of type T
     */
    template <typename T>
    struct Vec2
    {
        T x{}, y{};

        // Default constructor
        Vec2() = default; // Default constructor

        // Constructor with parameters
        constexpr Vec2(T x, T y) : x(x), y(y)
        {
        }

        constexpr Vec2(std::initializer_list<T> values)
        {
            assert(values.size() == 2 && "Vector2 requires 2 values");
            auto it = values.begin();
            x = *it++;
            y = *it;
        }

        static constexpr Vec2 Zero()
        {
            return Vec2(static_cast<T>(0), static_cast<T>(0));
        }

        static constexpr Vec2 One()
        {
            return Vec2(static_cast<T>(1), static_cast<T>(1));
        }

        //Addition
        constexpr Vec2 operator+(const Vec2& other) const
        {
            return Vec2{x + other.x, y + other.y};
        }

        constexpr Vec2<T> operator+=(const Vec2<T> vec) noexcept
        {
            x += vec.x;
            y += vec.y;

            return *this;
        }

        //Subtraction
        constexpr Vec2 operator-(const Vec2 other) const
        {
            return {x - other.x, y - other.y};
        }

        //Opposite
        constexpr Vec2 operator-() const
        {
            return {-x, -y};
        }

        //Multiplication by scalar
        constexpr Vec2 operator*(T scalar) const
        {
            return {x * scalar, y * scalar};
        }

        //Division by scalar
        constexpr Vec2 operator/(T scalar) const
        {
            if (scalar == 0) return {0, 0};
            return {x / scalar, y / scalar};
        }


        // other option:
        // T& Vec::GetElement(int index){if(index ≥ 3 || index <0){throw out of range} return (&x)[index];}
        //this is safe because we have a check but we could get rid of it to gain speed_multiplier
        //(context is everything, we know our classes and won’t use an index that is out of range)

        T& operator[](const int index)
        {
            if (index == 0) return x;
            if (index == 1) return y;
            throw std::out_of_range("Index out of range for Vec2");
        }

        const T& operator[](const int index) const
        {
            if (index == 0) return x;
            if (index == 1) return y;
            throw std::out_of_range("Index out of range for Vec2");
        }

        bool operator==(const Vec2& vec2) const
        {
            return x == vec2.x && y == vec2.y;
        };

        //Dot product
        static constexpr T Dot(Vec2 v1, Vec2 v2)
        {
            return v1.x * v2.x + v1.y * v2.y;
        }

        [[nodiscard]] constexpr T Dot(Vec2 v) const
        {
            return x * v.x + y * v.y;
        }

        [[nodiscard]] constexpr T CrossProduct(const Vec2& v1, const Vec2& v2) const
        {
            return v1.x * v2.y - v1.y * v2.x;
        }

        [[nodiscard]] constexpr Vec2 CrossProduct(const Vec2& v1, float s) const
        {
            return Vec2(s * v1.y, -s * v1.x);
        }

        [[nodiscard]] constexpr Vec2 CrossProduct(float s, const Vec2& v1) const
        {
            return Vec2(-s * v1.y, s * v1.x);
        }


        //First perpendicular vector
        [[nodiscard]] constexpr Vec2 Perpendicular() const
        {
            return {-y, x};
        }

        //Second perpendicular vector
        [[nodiscard]] constexpr Vec2 Perpendicular2() const
        {
            return {y, -x};
        }

        //Vector squarelength
        [[nodiscard]] constexpr T SquareMagnitude() const
        {
            return x * x + y * y;
        }

        //Vector length

        [[nodiscard]] constexpr T Magnitude() const
            requires std::is_floating_point_v<T>
        {
            return std::sqrt(SquareMagnitude());
        }

        /**
         * \brief Normalized vector
         * \return if the vector's length is zero, return zero
         */
        [[nodiscard]] constexpr Vec2 Normalized() const
        {
            float mag = Magnitude();
            if (mag == 0) return {0, 0};
            return {x / mag, y / mag};
        }

        //Interpolate between this Vec2 and other by t amount
        [[nodiscard]] constexpr Vec2 LERP(Vec2 other, T t) const
        {
            return {x + t * (other.x - x), y + t * (other.y - y)};
        }
    }; // struct Vec2

    //Operator overload for multiplication
    template <typename T>
    constexpr Vec2<T> operator*(T scalar, const Vec2<T>& vec)
    {
        return {scalar * vec.x, scalar * vec.y};
    }

    using Vec2f = Vec2<float>;
    using Vec2i = Vec2<int>;
    using Vec2d = Vec2<double>;
} // namespace math
#endif //KUMA_ENGINE_LIB_MATH_VEC2_H_
