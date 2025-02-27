#include <gtest/gtest.h>

#include <numbers>
#include "commons.h"
#include "matrix4.h"

namespace crackitos_physics::math
{
    struct Matrix4Fixture : public testing::Test
    {
    };

    TEST_F(Matrix4Fixture, MatrixAddition)
    {
        matrix4<crackitos_physics::commons::fp> mat1{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
        matrix4<crackitos_physics::commons::fp> mat2{{17, 18, 19, 20}, {21, 22, 23, 24}, {25, 26, 27, 28}, {29, 30, 31, 32}};
        auto result = mat1 + mat2;

        EXPECT_FLOAT_EQ(result[0].x, 18);
        EXPECT_FLOAT_EQ(result[0].y, 20);
        EXPECT_FLOAT_EQ(result[0].z, 22);
        EXPECT_FLOAT_EQ(result[0].w, 24);
        EXPECT_FLOAT_EQ(result[1].x, 26);
        EXPECT_FLOAT_EQ(result[1].y, 28);
        EXPECT_FLOAT_EQ(result[1].z, 30);
        EXPECT_FLOAT_EQ(result[1].w, 32);
        EXPECT_FLOAT_EQ(result[2].x, 34);
        EXPECT_FLOAT_EQ(result[2].y, 36);
        EXPECT_FLOAT_EQ(result[2].z, 38);
        EXPECT_FLOAT_EQ(result[2].w, 40);
        EXPECT_FLOAT_EQ(result[3].x, 42);
        EXPECT_FLOAT_EQ(result[3].y, 44);
        EXPECT_FLOAT_EQ(result[3].z, 46);
        EXPECT_FLOAT_EQ(result[3].w, 48);
    }

    TEST_F(Matrix4Fixture, MatrixSubtraction)
    {
        matrix4<crackitos_physics::commons::fp> mat1{{17, 18, 19, 20}, {21, 22, 23, 24}, {25, 26, 27, 28}, {29, 30, 31, 32}};
        matrix4<crackitos_physics::commons::fp> mat2{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
        auto result = mat1 - mat2;

        EXPECT_FLOAT_EQ(result[0].x, 16);
        EXPECT_FLOAT_EQ(result[0].y, 16);
        EXPECT_FLOAT_EQ(result[0].z, 16);
        EXPECT_FLOAT_EQ(result[0].w, 16);
        EXPECT_FLOAT_EQ(result[1].x, 16);
        EXPECT_FLOAT_EQ(result[1].y, 16);
        EXPECT_FLOAT_EQ(result[1].z, 16);
        EXPECT_FLOAT_EQ(result[1].w, 16);
        EXPECT_FLOAT_EQ(result[2].x, 16);
        EXPECT_FLOAT_EQ(result[2].y, 16);
        EXPECT_FLOAT_EQ(result[2].z, 16);
        EXPECT_FLOAT_EQ(result[2].w, 16);
        EXPECT_FLOAT_EQ(result[3].x, 16);
        EXPECT_FLOAT_EQ(result[3].y, 16);
        EXPECT_FLOAT_EQ(result[3].z, 16);
        EXPECT_FLOAT_EQ(result[3].w, 16);
    }

    TEST_F(Matrix4Fixture, ScalarMultiplication)
    {
        matrix4<crackitos_physics::commons::fp> mat{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
        auto result = mat * 2.0f;

        EXPECT_FLOAT_EQ(result[0].x, 2);
        EXPECT_FLOAT_EQ(result[0].y, 4);
        EXPECT_FLOAT_EQ(result[0].z, 6);
        EXPECT_FLOAT_EQ(result[0].w, 8);
        EXPECT_FLOAT_EQ(result[1].x, 10);
        EXPECT_FLOAT_EQ(result[1].y, 12);
        EXPECT_FLOAT_EQ(result[1].z, 14);
        EXPECT_FLOAT_EQ(result[1].w, 16);
        EXPECT_FLOAT_EQ(result[2].x, 18);
        EXPECT_FLOAT_EQ(result[2].y, 20);
        EXPECT_FLOAT_EQ(result[2].z, 22);
        EXPECT_FLOAT_EQ(result[2].w, 24);
        EXPECT_FLOAT_EQ(result[3].x, 26);
        EXPECT_FLOAT_EQ(result[3].y, 28);
        EXPECT_FLOAT_EQ(result[3].z, 30);
        EXPECT_FLOAT_EQ(result[3].w, 32);
    }

    TEST_F(Matrix4Fixture, MultiplyByVec4)
    {
        matrix4<crackitos_physics::commons::fp> mat{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
        Vec4<crackitos_physics::commons::fp> vec{1, 1, 1, 1};
        auto result = mat * vec;

        EXPECT_FLOAT_EQ(result.x, 10);
        EXPECT_FLOAT_EQ(result.y, 26);
        EXPECT_FLOAT_EQ(result.z, 42);
        EXPECT_FLOAT_EQ(result.w, 58);
    }

    TEST_F(Matrix4Fixture, DeterminantLaplace)
    {
        matrix4<crackitos_physics::commons::fp> mat{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        auto det = mat.determinant();
        EXPECT_FLOAT_EQ(det, 1);
    }

    TEST_F(Matrix4Fixture, DeterminantSarrus)
    {
        matrix4<crackitos_physics::commons::fp> mat{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        auto det = mat.determinantSarrus();
        EXPECT_FLOAT_EQ(det, 1);
    }

    TEST_F(Matrix4Fixture, Transpose)
    {
        matrix4<crackitos_physics::commons::fp> mat{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
        auto transposed = mat.transpose();

        EXPECT_FLOAT_EQ(transposed[0].x, 1);
        EXPECT_FLOAT_EQ(transposed[0].y, 5);
        EXPECT_FLOAT_EQ(transposed[0].z, 9);
        EXPECT_FLOAT_EQ(transposed[0].w, 13);
        EXPECT_FLOAT_EQ(transposed[1].x, 2);
        EXPECT_FLOAT_EQ(transposed[1].y, 6);
        EXPECT_FLOAT_EQ(transposed[1].z, 10);
        EXPECT_FLOAT_EQ(transposed[1].w, 14);
        EXPECT_FLOAT_EQ(transposed[2].x, 3);
        EXPECT_FLOAT_EQ(transposed[2].y, 7);
        EXPECT_FLOAT_EQ(transposed[2].z, 11);
        EXPECT_FLOAT_EQ(transposed[2].w, 15);
        EXPECT_FLOAT_EQ(transposed[3].x, 4);
        EXPECT_FLOAT_EQ(transposed[3].y, 8);
        EXPECT_FLOAT_EQ(transposed[3].z, 12);
        EXPECT_FLOAT_EQ(transposed[3].w, 16);
    }
}