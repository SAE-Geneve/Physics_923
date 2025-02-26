#include <gtest/gtest.h>

#include <numbers>
#include "commons.h"
#include "matrix3.h"
namespace physics923::math
{
    struct Matrix3Fixture : public testing::Test
    {
    };

    TEST_F(Matrix3Fixture, MatrixAddition)
    {
        matrix3<physics923::commons::fp> mat1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        matrix3<physics923::commons::fp> mat2{{10, 11, 12}, {13, 14, 15}, {16, 17, 18}};
        auto result = mat1 + mat2;

        EXPECT_FLOAT_EQ(result[0].x, 11);
        EXPECT_FLOAT_EQ(result[0].y, 13);
        EXPECT_FLOAT_EQ(result[0].z, 15);
        EXPECT_FLOAT_EQ(result[1].x, 17);
        EXPECT_FLOAT_EQ(result[1].y, 19);
        EXPECT_FLOAT_EQ(result[1].z, 21);
        EXPECT_FLOAT_EQ(result[2].x, 23);
        EXPECT_FLOAT_EQ(result[2].y, 25);
        EXPECT_FLOAT_EQ(result[2].z, 27);
    }

    TEST_F(Matrix3Fixture, MatrixSubtraction)
    {
        matrix3<physics923::commons::fp> mat1{{10, 11, 12}, {13, 14, 15}, {16, 17, 18}};
        matrix3<physics923::commons::fp> mat2{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        auto result = mat1 - mat2;

        EXPECT_FLOAT_EQ(result[0].x, 9);
        EXPECT_FLOAT_EQ(result[0].y, 9);
        EXPECT_FLOAT_EQ(result[0].z, 9);
        EXPECT_FLOAT_EQ(result[1].x, 9);
        EXPECT_FLOAT_EQ(result[1].y, 9);
        EXPECT_FLOAT_EQ(result[1].z, 9);
        EXPECT_FLOAT_EQ(result[2].x, 9);
        EXPECT_FLOAT_EQ(result[2].y, 9);
        EXPECT_FLOAT_EQ(result[2].z, 9);
    }

    TEST_F(Matrix3Fixture, ScalarMultiplication)
    {
        matrix3<physics923::commons::fp> mat{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        auto result = mat * 2.0f;

        EXPECT_FLOAT_EQ(result[0].x, 2);
        EXPECT_FLOAT_EQ(result[0].y, 4);
        EXPECT_FLOAT_EQ(result[0].z, 6);
        EXPECT_FLOAT_EQ(result[1].x, 8);
        EXPECT_FLOAT_EQ(result[1].y, 10);
        EXPECT_FLOAT_EQ(result[1].z, 12);
        EXPECT_FLOAT_EQ(result[2].x, 14);
        EXPECT_FLOAT_EQ(result[2].y, 16);
        EXPECT_FLOAT_EQ(result[2].z, 18);
    }

    TEST_F(Matrix3Fixture, MultiplyByVec3)
    {
        matrix3<physics923::commons::fp> mat{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        Vec3<physics923::commons::fp> vec{1, 1, 1};
        auto result = mat * vec;

        EXPECT_FLOAT_EQ(result.x, 6);
        EXPECT_FLOAT_EQ(result.y, 15);
        EXPECT_FLOAT_EQ(result.z, 24);
    }

    TEST_F(Matrix3Fixture, DeterminantLaplace)
    {
        matrix3<physics923::commons::fp> mat{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        auto det = mat.determinant();
        EXPECT_FLOAT_EQ(det, 0);
    }

    TEST_F(Matrix3Fixture, DeterminantSarrus)
    {
        matrix3<physics923::commons::fp> mat{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        auto det = mat.determinantSarrus();
        EXPECT_FLOAT_EQ(det, 0);
    }

    TEST_F(Matrix3Fixture, Transpose)
    {
        matrix3<physics923::commons::fp> mat{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        auto transposed = mat.transpose();

        EXPECT_FLOAT_EQ(transposed[0].x, 1);
        EXPECT_FLOAT_EQ(transposed[0].y, 4);
        EXPECT_FLOAT_EQ(transposed[0].z, 7);
        EXPECT_FLOAT_EQ(transposed[1].x, 2);
        EXPECT_FLOAT_EQ(transposed[1].y, 5);
        EXPECT_FLOAT_EQ(transposed[1].z, 8);
        EXPECT_FLOAT_EQ(transposed[2].x, 3);
        EXPECT_FLOAT_EQ(transposed[2].y, 6);
        EXPECT_FLOAT_EQ(transposed[2].z, 9);
    }

    TEST_F(Matrix3Fixture, Inverse)
    {
        matrix3<physics923::commons::fp> mat{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        auto inv = mat.inverse();

        EXPECT_FLOAT_EQ(inv[0].x, 1);
        EXPECT_FLOAT_EQ(inv[0].y, 0);
        EXPECT_FLOAT_EQ(inv[0].z, 0);
        EXPECT_FLOAT_EQ(inv[1].x, 0);
        EXPECT_FLOAT_EQ(inv[1].y, 1);
        EXPECT_FLOAT_EQ(inv[1].z, 0);
        EXPECT_FLOAT_EQ(inv[2].x, 0);
        EXPECT_FLOAT_EQ(inv[2].y, 0);
        EXPECT_FLOAT_EQ(inv[2].z, 1);
    }

    TEST_F(Matrix3Fixture, Rotation)
    {
        matrix3<physics923::commons::fp> mat{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        auto rot = mat.rotation(std::numbers::pi_v<physics923::commons::fp> / 2);

        EXPECT_NEAR(rot[0].x, 0, 1e-6);
        EXPECT_NEAR(rot[0].y, -1, 1e-6);
        EXPECT_NEAR(rot[0].z, 0, 1e-6);
        EXPECT_NEAR(rot[1].x, 1, 1e-6);
        EXPECT_NEAR(rot[1].y, 0, 1e-6);
        EXPECT_NEAR(rot[1].z, 0, 1e-6);
        EXPECT_NEAR(rot[2].x, 0, 1e-6);
        EXPECT_NEAR(rot[2].y, 0, 1e-6);
        EXPECT_NEAR(rot[2].z, 1, 1e-6);
    }
}