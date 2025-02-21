#include <gtest/gtest.h>

#include "four_vec3.h"

namespace physics923::math
{
    TEST(FourVec3Test, Addition)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{1.0f, 2.0f, 3.0f}, Vec3<float>{4.0f, 5.0f, 6.0f}, Vec3<float>{7.0f, 8.0f, 9.0f},
            Vec3<float>{10.0f, 11.0f, 12.0f}
        };
        std::array<Vec3<float>, 4> b_arr = {
            Vec3<float>{13.0f, 14.0f, 15.0f}, Vec3<float>{16.0f, 17.0f, 18.0f}, Vec3<float>{19.0f, 20.0f, 21.0f},
            Vec3<float>{22.0f, 23.0f, 24.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> b(b_arr);
        FourVec3<float> result = a + b;

        EXPECT_EQ(result.x[0], 14.0f);
        EXPECT_EQ(result.x[1], 20.0f);
        EXPECT_EQ(result.x[2], 26.0f);
        EXPECT_EQ(result.x[3], 32.0f);

        EXPECT_EQ(result.y[0], 16.0f);
        EXPECT_EQ(result.y[1], 22.0f);
        EXPECT_EQ(result.y[2], 28.0f);
        EXPECT_EQ(result.y[3], 34.0f);

        EXPECT_EQ(result.z[0], 18.0f);
        EXPECT_EQ(result.z[1], 24.0f);
        EXPECT_EQ(result.z[2], 30.0f);
        EXPECT_EQ(result.z[3], 36.0f);
    }

    TEST(FourVec3Test, Subtraction)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{10.0f, 20.0f, 30.0f}, Vec3<float>{40.0f, 50.0f, 60.0f}, Vec3<float>{70.0f, 80.0f, 90.0f},
            Vec3<float>{100.0f, 110.0f, 120.0f}
        };
        std::array<Vec3<float>, 4> b_arr = {
            Vec3<float>{1.0f, 2.0f, 3.0f}, Vec3<float>{4.0f, 5.0f, 6.0f}, Vec3<float>{7.0f, 8.0f, 9.0f},
            Vec3<float>{10.0f, 11.0f, 12.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> b(b_arr);
        FourVec3<float> result = a - b;

        EXPECT_EQ(result.x[0], 9.0f);
        EXPECT_EQ(result.x[1], 36.0f);
        EXPECT_EQ(result.x[2], 63.0f);
        EXPECT_EQ(result.x[3], 90.0f);

        EXPECT_EQ(result.y[0], 18.0f);
        EXPECT_EQ(result.y[1], 45.0f);
        EXPECT_EQ(result.y[2], 72.0f);
        EXPECT_EQ(result.y[3], 99.0f);

        EXPECT_EQ(result.z[0], 27.0f);
        EXPECT_EQ(result.z[1], 54.0f);
        EXPECT_EQ(result.z[2], 81.0f);
        EXPECT_EQ(result.z[3], 108.0f);
    }

    TEST(FourVec3Test, Negation)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{1.0f, -2.0f, 3.0f}, Vec3<float>{4.0f, -5.0f, 6.0f}, Vec3<float>{7.0f, -8.0f, 9.0f},
            Vec3<float>{10.0f, -11.0f, 12.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> result = -a;

        EXPECT_EQ(result.x[0], -1.0f);
        EXPECT_EQ(result.x[1], -4.0f);
        EXPECT_EQ(result.x[2], -7.0f);
        EXPECT_EQ(result.x[3], -10.0f);

        EXPECT_EQ(result.y[0], 2.0f);
        EXPECT_EQ(result.y[1], 5.0f);
        EXPECT_EQ(result.y[2], 8.0f);
        EXPECT_EQ(result.y[3], 11.0f);

        EXPECT_EQ(result.z[0], -3.0f);
        EXPECT_EQ(result.z[1], -6.0f);
        EXPECT_EQ(result.z[2], -9.0f);
        EXPECT_EQ(result.z[3], -12.0f);
    }

    TEST(FourVec3Test, ScalarMultiplication)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{1.0f, 2.0f, 3.0f}, Vec3<float>{4.0f, 5.0f, 6.0f}, Vec3<float>{7.0f, 8.0f, 9.0f},
            Vec3<float>{10.0f, 11.0f, 12.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> result = a * 2.0f;

        EXPECT_EQ(result.x[0], 2.0f);
        EXPECT_EQ(result.x[1], 8.0f);
        EXPECT_EQ(result.x[2], 14.0f);
        EXPECT_EQ(result.x[3], 20.0f);

        EXPECT_EQ(result.y[0], 4.0f);
        EXPECT_EQ(result.y[1], 10.0f);
        EXPECT_EQ(result.y[2], 16.0f);
        EXPECT_EQ(result.y[3], 22.0f);

        EXPECT_EQ(result.z[0], 6.0f);
        EXPECT_EQ(result.z[1], 12.0f);
        EXPECT_EQ(result.z[2], 18.0f);
        EXPECT_EQ(result.z[3], 24.0f);
    }

    TEST(FourVec3Test, ScalarDivision)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{10.0f, 20.0f, 30.0f}, Vec3<float>{40.0f, 50.0f, 60.0f}, Vec3<float>{70.0f, 80.0f, 90.0f},
            Vec3<float>{100.0f, 110.0f, 120.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> result = a / 10.0f;

        EXPECT_EQ(result.x[0], 1.0f);
        EXPECT_EQ(result.x[1], 4.0f);
        EXPECT_EQ(result.x[2], 7.0f);
        EXPECT_EQ(result.x[3], 10.0f);

        EXPECT_EQ(result.y[0], 2.0f);
        EXPECT_EQ(result.y[1], 5.0f);
        EXPECT_EQ(result.y[2], 8.0f);
        EXPECT_EQ(result.y[3], 11.0f);

        EXPECT_EQ(result.z[0], 3.0f);
        EXPECT_EQ(result.z[1], 6.0f);
        EXPECT_EQ(result.z[2], 9.0f);
        EXPECT_EQ(result.z[3], 12.0f);
    }

    TEST(FourVec3Test, DotProduct)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{1.0f, 2.0f, 3.0f}, Vec3<float>{4.0f, 5.0f, 6.0f}, Vec3<float>{7.0f, 8.0f, 9.0f},
            Vec3<float>{10.0f, 11.0f, 12.0f}
        };
        std::array<Vec3<float>, 4> b_arr = {
            Vec3<float>{13.0f, 14.0f, 15.0f}, Vec3<float>{16.0f, 17.0f, 18.0f}, Vec3<float>{19.0f, 20.0f, 21.0f},
            Vec3<float>{22.0f, 23.0f, 24.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> b(b_arr);
        std::array<float, 4> result = a.Dot(b);

        EXPECT_FLOAT_EQ(result[0], 86.0f);
        EXPECT_FLOAT_EQ(result[1], 257.0f);
        EXPECT_FLOAT_EQ(result[2], 482.0f);
        EXPECT_FLOAT_EQ(result[3], 761.0f);
    }

    TEST(FourVec3Test, SquareMagnitude)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{1.0f, 2.0f, 3.0f},
            Vec3<float>{4.0f, 5.0f, 6.0f},
            Vec3<float>{7.0f, 8.0f, 9.0f},
            Vec3<float>{10.0f, 11.0f, 12.0f}
        };

        FourVec3<float> a(a_arr);
        std::array<float, 4> result = a.SquareMagnitude();

        EXPECT_FLOAT_EQ(result[0], 14.0f); // 1^2 + 2^2 + 3^2 = 14
        EXPECT_FLOAT_EQ(result[1], 77.0f); // 4^2 + 5^2 + 6^2 = 77
        EXPECT_FLOAT_EQ(result[2], 194.0f); // 7^2 + 8^2 + 9^2 = 194
        EXPECT_FLOAT_EQ(result[3], 365.0f); // 10^2 + 11^2 + 12^2 = 365
    }

    TEST(FourVec3Test, Magnitude)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{1.0f, 2.0f, 3.0f},
            Vec3<float>{4.0f, 5.0f, 6.0f},
            Vec3<float>{7.0f, 8.0f, 9.0f},
            Vec3<float>{10.0f, 11.0f, 12.0f}
        };

        FourVec3<float> a(a_arr);
        std::array<float, 4> result = a.Magnitude();

        EXPECT_NEAR(result[0], 3.741657f, 1e-5f); // sqrt(14)
        EXPECT_NEAR(result[1], 8.774964f, 1e-5f); // sqrt(77)
        EXPECT_NEAR(result[2], 13.928388f, 1e-5f); // sqrt(194)
        EXPECT_NEAR(result[3], 19.104973f, 1e-5f); // sqrt(365)
    }

    TEST(FourVec3Test, Normalize)
    {
        std::array<Vec3<float>, 4> a_arr = {
            Vec3<float>{3.0f, 0.0f, 0.0f},
            Vec3<float>{0.0f, 4.0f, 0.0f},
            Vec3<float>{0.0f, 0.0f, 5.0f},
            Vec3<float>{1.0f, 2.0f, 2.0f}
        };

        FourVec3<float> a(a_arr);
        FourVec3<float> result = a.Normalize();

        EXPECT_NEAR(result.x[0], 1.0f, 1e-5f); // Normalized (3,0,0)
        EXPECT_NEAR(result.y[1], 1.0f, 1e-5f); // Normalized (0,4,0)
        EXPECT_NEAR(result.z[2], 1.0f, 1e-5f); // Normalized (0,0,5)
        EXPECT_NEAR(result.x[3], 0.333333f, 1e-5f); // Normalized (1,2,2)
        EXPECT_NEAR(result.y[3], 0.666666f, 1e-5f);
        EXPECT_NEAR(result.z[3], 0.666666f, 1e-5f);
    }
}
