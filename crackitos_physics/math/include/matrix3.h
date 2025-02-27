#ifndef CRACKITOS_PHYSICS_MATH_MATRIX3_H_
#define CRACKITOS_PHYSICS_MATH_MATRIX3_H_

#include <array>
#include <cassert>
#include <stdexcept>

#include "matrix2.h"
#include "vec3.h"

namespace crackitos_physics::math
{
    template <typename T>
    struct matrix3
    {
        std::array<Vec3<T>, 3> rows;

        // Default constructor
        matrix3() = default;

        // Constructor with parameters
        constexpr matrix3(const Vec3<T>& row1, const Vec3<T>& row2, const Vec3<T>& row3) : rows{{row1, row2, row3}}
        {
        }

        // Constructors with initializer lists (values and vectors)
        constexpr matrix3(std::initializer_list<T> values)
        {
            assert(values.size() == 9 && "Matrix3 requires 9 values");
            auto it = values.begin();
            rows[0].x = *it++;
            rows[0].y = *it++;
            rows[0].z = *it++;
            rows[1].x = *it++;
            rows[1].y = *it++;
            rows[1].z = *it++;
            rows[2].x = *it++;
            rows[2].y = *it++;
            rows[2].z = *it;
        }

        constexpr matrix3(std::initializer_list<Vec3<T>> vectors)
        {
            assert(vectors.size() == 3 && "Matrix3 requires 3 Vec3");
            auto it = vectors.begin();
            rows[0] = *it++;
            rows[1] = *it++;
            rows[2] = *it;
        }

        static constexpr matrix3 identity()
        {
            return matrix3{Vec3<T>(1, 0, 0), Vec3<T>(0, 1, 0), Vec3<T>(0, 0, 1)};
        }

        static constexpr matrix3 zero()
        {
            return matrix3{Vec3<T>(0, 0, 0), Vec3<T>(0, 0, 0), Vec3<T>(0, 0, 0)};
        }

        // Access row by index
        Vec3<T>& operator[](const int index)
        {
            if (index == 0) return rows[0];
            if (index == 1) return rows[1];
            if (index == 2) return rows[2];
            throw std::out_of_range("Index out of range for matrix3");
        }

        const Vec3<T>& operator[](const int index) const
        {
            if (index == 0) return rows[0];
            if (index == 1) return rows[1];
            if (index == 2) return rows[2];
            throw std::out_of_range("Index out of range for matrix3");
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
        constexpr matrix3 operator+(const matrix3& other) const
        {
            return matrix3(rows[0] + other[0], rows[1] + other[1], rows[2] + other[2]);
        }

        // Subtraction
        constexpr matrix3 operator-(const matrix3& other) const
        {
            return matrix3(rows[0] - other[0], rows[1] - other[1], rows[2] - other[2]);
        }

        // Multiplication by scalar
        constexpr matrix3 operator*(T scalar) const
        {
            return matrix3(rows[0] * scalar, rows[1] * scalar, rows[2] * scalar);
        }

        // Multiplication by vector
        constexpr Vec3<T> operator*(const Vec3<T>& vec) const
        {
            return Vec3<T>(rows[0].x * vec.x + rows[0].y * vec.y + rows[0].z * vec.z,
                           rows[1].x * vec.x + rows[1].y * vec.y + rows[1].z * vec.z,
                           rows[2].x * vec.x + rows[2].y * vec.y + rows[2].z * vec.z);
        }

        // Determinant 1 using the Laplace Method
        [[nodiscard]] constexpr T determinant() const
        {
            // Calculate the determinant using the Laplace expansion
            // along the first row
            return rows[0].x * math::matrix2<T>(Vec2<T>(rows[1].y, rows[1].z), Vec2<T>(rows[2].y, rows[2].z)).
                determinant()
                - rows[0].y * math::matrix2<T>(Vec2<T>(rows[1].x, rows[1].z), Vec2<T>(rows[2].x, rows[2].z)).
                determinant()
                + rows[0].z * math::matrix2<T>(Vec2<T>(rows[1].x, rows[1].y), Vec2<T>(rows[2].x, rows[2].y)).
                determinant();
        }

        // Determinant 2 using the Sarrus Method
        [[nodiscard]] constexpr T determinantSarrus() const
        {
            // Calculate the determinant using the Sarrus rule
            return rows[0].x * rows[1].y * rows[2].z
                + rows[0].y * rows[1].z * rows[2].x
                + rows[0].z * rows[1].x * rows[2].y
                - rows[0].x * rows[1].z * rows[2].y
                - rows[0].y * rows[1].x * rows[2].z
                - rows[0].z * rows[1].y * rows[2].x;
        }

        // Transpose of the Matrix
        [[nodiscard]] constexpr matrix3 transpose() const
        {
            return matrix3(Vec3<T>(rows[0].x, rows[1].x, rows[2].x),
                           Vec3<T>(rows[0].y, rows[1].y, rows[2].y),
                           Vec3<T>(rows[0].z, rows[1].z, rows[2].z));
        }

        // Inverse
        [[nodiscard]] constexpr matrix3 inverse() const
        {
            T det = determinant();
            if (det == 0) throw std::runtime_error("Matrix is singular");

            // Calculate the cofactor matrix
            matrix3 cofactor;
            cofactor[0].x = rows[1].y * rows[2].z - rows[1].z * rows[2].y;
            cofactor[0].y = rows[1].z * rows[2].x - rows[1].x * rows[2].z;
            cofactor[0].z = rows[1].x * rows[2].y - rows[1].y * rows[2].x;
            cofactor[1].x = rows[0].z * rows[2].y - rows[0].y * rows[2].z;
            cofactor[1].y = rows[0].x * rows[2].z - rows[0].z * rows[2].x;
            cofactor[1].z = rows[0].y * rows[2].x - rows[0].x * rows[2].y;
            cofactor[2].x = rows[0].y * rows[1].z - rows[0].z * rows[1].y;
            cofactor[2].y = rows[0].z * rows[1].x - rows[0].x * rows[1].z;
            cofactor[2].z = rows[0].x * rows[1].y - rows[0].y * rows[1].x;

            // Calculate the adjugate matrix
            matrix3 adjugate = cofactor.transpose();

            // Calculate the inverse matrix
            return adjugate * (1 / det);
        }

        // Rotation Matrix
        [[nodiscard]] constexpr matrix3 rotation(T angle) const
        {
            // Calculate the rotation matrix
            T cosAngle = std::cos(angle);
            T sinAngle = std::sin(angle);

            return matrix3(Vec3<T>(cosAngle, -sinAngle, 0),
                           Vec3<T>(sinAngle, cosAngle, 0),
                           Vec3<T>(0, 0, 1));
        }
    };

    // Multiplication by scalar (outside the struct)
    template <typename T>
    constexpr matrix3<T> operator*(T scalar, const matrix3<T>& mat)
    {
        return mat * scalar;
    }
} // namespace math
#endif // CRACKITOS_PHYSICS_MATH_MATRIX3_H_
