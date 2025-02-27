#ifndef CRACKITOS_PHYSICS_MATH_MATRIX2_H_
#define CRACKITOS_PHYSICS_MATH_MATRIX2_H_

#include <array>
#include <cassert>

#include "vec2.h"
#include "commons.h"

namespace crackitos_physics::math
{
    template <typename T>
    struct matrix2
    {
        std::array<Vec2<T>, 2> rows;

        // Default constructor
        matrix2() = default;

        // Constructor with parameters
        constexpr matrix2(const Vec2<T>& row1, const Vec2<T>& row2) : rows{{row1, row2}}
        {
        }

        // Constructors with initializer lists (values and vectors)
        constexpr matrix2(std::initializer_list<T> values)
        {
            assert(values.size() == 4 && "Matrix2 requires 4 values");
            auto it = values.begin();
            rows[0].x = *it++;
            rows[0].y = *it++;
            rows[1].x = *it++;
            rows[1].y = *it;
        }

        constexpr matrix2(std::initializer_list<Vec2<T>> vectors)
        {
            assert(vectors.size() == 2 && "Matrix2 requires 2 Vec2");
            auto it = vectors.begin();
            rows[0] = *it++;
            rows[1] = *it;
        }

        constexpr matrix2 identity() const
        {
            return matrix2{Vec2<T>{1, 0}, Vec2<T>{0, 1}};
        }

        constexpr matrix2 zero() const
        {
            return matrix2{Vec2<T>{0, 0}, Vec2<T>{0, 0}};
        }

        // Access row by index
        Vec2<T>& operator[](const int index)
        {
            if (index == 0) return rows[0];
            if (index == 1) return rows[1];
            throw std::out_of_range("Index out of range for matrix2");
        }

        const Vec2<T>& operator[](const int index) const
        {
            if (index == 0) return rows[0];
            if (index == 1) return rows[1];
            throw std::out_of_range("Index out of range for matrix2");
        }

        // Access specific item
        T& operator()(int x, int y)
        {
            return rows[x][y];
        }

        const T& operator()(int x, int y) const
        {
            return rows[x][y];
        }

        // Addition
        constexpr matrix2 operator+(const matrix2& other) const
        {
            return matrix2(rows[0] + other[0], rows[1] + other[1]);
        }

        // Subtraction
        constexpr matrix2 operator-(const matrix2& other) const
        {
            return matrix2(rows[0] - other[0], rows[1] - other[1]);
        }

        // Multiplication by scalar
        constexpr matrix2 operator*(T scalar) const
        {
            return matrix2(rows[0] * scalar, rows[1] * scalar);
        }

        // Multiplication by vector
        constexpr Vec2<T> operator*(const Vec2<T>& vec) const
        {
            return Vec2<T>(rows[0].x * vec.x + rows[0].y * vec.y, rows[1].x * vec.x + rows[1].y * vec.y);
        }

        // Determinant 1 using the Laplace Method
        [[nodiscard]] constexpr T determinant() const
        {
            return rows[0].x * rows[1].y - rows[0].y * rows[1].x;
        }

        // Transpose of the Matrix
        [[nodiscard]] constexpr matrix2 transpose() const
        {
            return matrix2(Vec2<T>(rows[0].x, rows[1].x), Vec2<T>(rows[0].y, rows[1].y));
        }

        // Inverse
        [[nodiscard]] constexpr matrix2 inverse() const
        {
            T det = determinant();
            if (det == 0) throw std::runtime_error("Matrix is not invertible");
            return matrix2(Vec2<T>(rows[1].y / det, -rows[0].y / det), Vec2<T>(-rows[1].x / det, rows[0].x / det));
        }

        // Rotation Matrix
        [[nodiscard]] constexpr matrix2 rotation(T angle) const
        {
            T c = std::cos(angle);
            T s = std::sin(angle);
            return matrix2(Vec2<T>(c, -s), Vec2<T>(s, c));
        }
    };

    // Multiplication by scalar (outside the struct)
    template <typename T>
    constexpr matrix2<T> operator*(T scalar, const matrix2<T>& mat)
    {
        return mat * scalar;
    }

    using matrix2f = matrix2<crackitos_physics::commons::fp>;
    using matrix2i = matrix2<int>;
    using matrix2d = matrix2<double>;
} // namespace math
#endif // CRACKITOS_PHYSICS_MATH_MATRIX2_H_
