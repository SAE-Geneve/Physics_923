#ifndef PHYSICS_923_LIB_MATH_MATRIX4_H_
#define PHYSICS_923_LIB_MATH_MATRIX4_H_

#include <array>
#include <cassert>
#include <stdexcept>

#include "matrix3.h"
#include "vec4.h"

namespace physics923::math
{
    template <typename T>
    struct matrix4
    {
        std::array<Vec4<T>, 4> rows;

        // Default constructor
        matrix4() = default;

        // Constructor with parameters
        constexpr matrix4(const Vec4<T>& row1, const Vec4<T>& row2, const Vec4<T>& row3, const Vec4<T>& row4) : rows{
            {row1, row2, row3, row4}
        }
        {
        }

        // Constructors with initializer lists (values and vectors)
        constexpr matrix4(std::initializer_list<T> values)
        {
            assert(values.size() == 16 && "Matrix4 requires 16 values");
            auto it = values.begin();
            rows[0].x = *it++;
            rows[0].y = *it++;
            rows[0].z = *it++;
            rows[0].w = *it++;
            rows[1].x = *it++;
            rows[1].y = *it++;
            rows[1].z = *it++;
            rows[1].w = *it++;
            rows[2].x = *it++;
            rows[2].y = *it++;
            rows[2].z = *it++;
            rows[2].w = *it++;
            rows[3].x = *it++;
            rows[3].y = *it++;
            rows[3].z = *it++;
            rows[3].w = *it;
        }

        constexpr matrix4(std::initializer_list<Vec4<T>> vectors)
        {
            assert(vectors.size() == 4 && "Matrix4 requires 4 Vec4");
            auto it = vectors.begin();
            rows[0] = *it++;
            rows[1] = *it++;
            rows[2] = *it++;
            rows[3] = *it;
        }

        static constexpr matrix4 identity()
        {
            return matrix4{Vec4<T>(1, 0, 0, 0), Vec4<T>(0, 1, 0, 0), Vec4<T>(0, 0, 1, 0), Vec4<T>(0, 0, 0, 1)};
        }

        static constexpr matrix4 zero()
        {
            return matrix4{Vec4<T>(0, 0, 0, 0), Vec4<T>(0, 0, 0, 0), Vec4<T>(0, 0, 0, 0), Vec4<T>(0, 0, 0, 0)};
        }

        // Access row by index
        Vec4<T>& operator[](const int index)
        {
            if (index == 0) return rows[0];
            if (index == 1) return rows[1];
            if (index == 2) return rows[2];
            if (index == 3) return rows[3];
            throw std::out_of_range("Index out of range for matrix4");
        }

        const Vec4<T>& operator[](const int index) const
        {
            if (index == 0) return rows[0];
            if (index == 1) return rows[1];
            if (index == 2) return rows[2];
            if (index == 3) return rows[3];
            throw std::out_of_range("Index out of range for matrix4");
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
        constexpr matrix4 operator+(const matrix4& other) const
        {
            return matrix4(rows[0] + other[0], rows[1] + other[1], rows[2] + other[2], rows[3] + other[3]);
        }

        // Subtraction
        constexpr matrix4 operator-(const matrix4& other) const
        {
            return matrix4(rows[0] - other[0], rows[1] - other[1], rows[2] - other[2], rows[3] - other[3]);
        }

        // Multiplication by scalar
        constexpr matrix4 operator*(T scalar) const
        {
            return matrix4(rows[0] * scalar, rows[1] * scalar, rows[2] * scalar, rows[3] * scalar);
        }

        // Multiplication by vector
        constexpr Vec4<T> operator*(const Vec4<T>& vec) const
        {
            return Vec4<T>(rows[0].x * vec.x + rows[0].y * vec.y + rows[0].z * vec.z + rows[0].w * vec.w,
                           rows[1].x * vec.x + rows[1].y * vec.y + rows[1].z * vec.z + rows[1].w * vec.w,
                           rows[2].x * vec.x + rows[2].y * vec.y + rows[2].z * vec.z + rows[2].w * vec.w,
                           rows[3].x * vec.x + rows[3].y * vec.y + rows[3].z * vec.z + rows[3].w * vec.w);
        }

        // Determinant 1 using the Laplace Method
        [[nodiscard]] constexpr T determinant() const
        {
            // Calculate the determinant using the Laplace expansion
            // along the first row
            return rows[0].x * math::matrix3<T>(Vec3<T>(rows[1].y, rows[1].z, rows[1].w),
                                                Vec3<T>(rows[2].y, rows[2].z, rows[2].w),
                                                Vec3<T>(rows[3].y, rows[3].z, rows[3].w)).determinant()
                - rows[0].y * math::matrix3<T>(Vec3<T>(rows[1].x, rows[1].z, rows[1].w),
                                               Vec3<T>(rows[2].x, rows[2].z, rows[2].w),
                                               Vec3<T>(rows[3].x, rows[3].z, rows[3].w)).determinant()
                + rows[0].z * math::matrix3<T>(Vec3<T>(rows[1].x, rows[1].y, rows[1].w),
                                               Vec3<T>(rows[2].x, rows[2].y, rows[2].w),
                                               Vec3<T>(rows[3].x, rows[3].y, rows[3].w)).determinant()
                - rows[0].w * math::matrix3<T>(Vec3<T>(rows[1].x, rows[1].y, rows[1].z),
                                               Vec3<T>(rows[2].x, rows[2].y, rows[2].z),
                                               Vec3<T>(rows[3].x, rows[3].y, rows[3].z)).determinant();
        }

        // Determinant 2 using the Sarrus Method
        [[nodiscard]] constexpr T determinantSarrus() const
        {
            // Calculate the determinant using the Sarrus rule
            return rows[0].x * rows[1].y * rows[2].z * rows[3].w
                + rows[0].y * rows[1].z * rows[2].w * rows[3].x
                + rows[0].z * rows[1].w * rows[2].x * rows[3].y
                + rows[0].w * rows[1].x * rows[2].y * rows[3].z
                - rows[0].x * rows[1].z * rows[2].w * rows[3].y
                - rows[0].y * rows[1].w * rows[2].x * rows[3].z
                - rows[0].z * rows[1].x * rows[2].y * rows[3].w
                - rows[0].w * rows[1].y * rows[2].z * rows[3].x;
        }

        // Transpose of the Matrix
        [[nodiscard]] constexpr matrix4 transpose() const
        {
            return matrix4(Vec4<T>(rows[0].x, rows[1].x, rows[2].x, rows[3].x),
                           Vec4<T>(rows[0].y, rows[1].y, rows[2].y, rows[3].y),
                           Vec4<T>(rows[0].z, rows[1].z, rows[2].z, rows[3].z),
                           Vec4<T>(rows[0].w, rows[1].w, rows[2].w, rows[3].w));
        }
    };
} // namespace math

#endif // PHYSICS_923_LIB_MATH_MATRIX4_H_
