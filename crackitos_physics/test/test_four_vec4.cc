#include <gtest/gtest.h>

#include "four_vec4.h"
#include "commons.h"

namespace crackitos_physics::math
{
    TEST(FourVec4Test, Addition)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f}, Vec4<crackitos_physics::commons::fp>{5.0f, 6.0f, 7.0f, 8.0f},
            Vec4<crackitos_physics::commons::fp>{9.0f, 10.0f, 11.0f, 12.0f}, Vec4<crackitos_physics::commons::fp>{13.0f, 14.0f, 15.0f, 16.0f}
        };
        std::array<Vec4<crackitos_physics::commons::fp>, 4> b_arr = {
            Vec4<crackitos_physics::commons::fp>{17.0f, 18.0f, 19.0f, 20.0f}, Vec4<crackitos_physics::commons::fp>{21.0f, 22.0f, 23.0f, 24.0f},
            Vec4<crackitos_physics::commons::fp>{25.0f, 26.0f, 27.0f, 28.0f}, Vec4<crackitos_physics::commons::fp>{29.0f, 30.0f, 31.0f, 32.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        FourVec4<crackitos_physics::commons::fp> b(b_arr);
        FourVec4<crackitos_physics::commons::fp> result = a + b;

        EXPECT_EQ(result.x[0], 18.0f);
        EXPECT_EQ(result.x[1], 26.0f);
        EXPECT_EQ(result.x[2], 34.0f);
        EXPECT_EQ(result.x[3], 42.0f);

        EXPECT_EQ(result.y[0], 20.0f);
        EXPECT_EQ(result.y[1], 28.0f);
        EXPECT_EQ(result.y[2], 36.0f);
        EXPECT_EQ(result.y[3], 44.0f);

        EXPECT_EQ(result.z[0], 22.0f);
        EXPECT_EQ(result.z[1], 30.0f);
        EXPECT_EQ(result.z[2], 38.0f);
        EXPECT_EQ(result.z[3], 46.0f);

        EXPECT_EQ(result.w[0], 24.0f);
        EXPECT_EQ(result.w[1], 32.0f);
        EXPECT_EQ(result.w[2], 40.0f);
        EXPECT_EQ(result.w[3], 48.0f);
    }

    TEST(FourVec4Test, Subtraction)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{29.0f, 30.0f, 31.0f, 32.0f}, Vec4<crackitos_physics::commons::fp>{33.0f, 34.0f, 35.0f, 36.0f},
            Vec4<crackitos_physics::commons::fp>{37.0f, 38.0f, 39.0f, 40.0f}, Vec4<crackitos_physics::commons::fp>{41.0f, 42.0f, 43.0f, 44.0f}
        };
        std::array<Vec4<crackitos_physics::commons::fp>, 4> b_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f}, Vec4<crackitos_physics::commons::fp>{5.0f, 6.0f, 7.0f, 8.0f},
            Vec4<crackitos_physics::commons::fp>{9.0f, 10.0f, 11.0f, 12.0f}, Vec4<crackitos_physics::commons::fp>{13.0f, 14.0f, 15.0f, 16.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        FourVec4<crackitos_physics::commons::fp> b(b_arr);
        FourVec4<crackitos_physics::commons::fp> result = a - b;

        EXPECT_EQ(result.x[0], 28.0f);
        EXPECT_EQ(result.x[1], 28.0f);
        EXPECT_EQ(result.x[2], 28.0f);
        EXPECT_EQ(result.x[3], 28.0f);

        EXPECT_EQ(result.y[0], 28.0f);
        EXPECT_EQ(result.y[1], 28.0f);
        EXPECT_EQ(result.y[2], 28.0f);
        EXPECT_EQ(result.y[3], 28.0f);

        EXPECT_EQ(result.z[0], 28.0f);
        EXPECT_EQ(result.z[1], 28.0f);
        EXPECT_EQ(result.z[2], 28.0f);
        EXPECT_EQ(result.z[3], 28.0f);

        EXPECT_EQ(result.w[0], 28.0f);
        EXPECT_EQ(result.w[1], 28.0f);
        EXPECT_EQ(result.w[2], 28.0f);
        EXPECT_EQ(result.w[3], 28.0f);
    }

    TEST(FourVec4Test, Negation)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, -2.0f, 3.0f, -4.0f}, Vec4<crackitos_physics::commons::fp>{5.0f, -6.0f, 7.0f, -8.0f},
            Vec4<crackitos_physics::commons::fp>{9.0f, -10.0f, 11.0f, -12.0f}, Vec4<crackitos_physics::commons::fp>{13.0f, -14.0f, 15.0f, -16.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        FourVec4<crackitos_physics::commons::fp> result = -a;

        EXPECT_EQ(result.x[0], -1.0f);
        EXPECT_EQ(result.x[1], -5.0f);
        EXPECT_EQ(result.x[2], -9.0f);
        EXPECT_EQ(result.x[3], -13.0f);

        EXPECT_EQ(result.y[0], 2.0f);
        EXPECT_EQ(result.y[1], 6.0f);
        EXPECT_EQ(result.y[2], 10.0f);
        EXPECT_EQ(result.y[3], 14.0f);

        EXPECT_EQ(result.z[0], -3.0f);
        EXPECT_EQ(result.z[1], -7.0f);
        EXPECT_EQ(result.z[2], -11.0f);
        EXPECT_EQ(result.z[3], -15.0f);

        EXPECT_EQ(result.w[0], 4.0f);
        EXPECT_EQ(result.w[1], 8.0f);
        EXPECT_EQ(result.w[2], 12.0f);
        EXPECT_EQ(result.w[3], 16.0f);
    }

    TEST(FourVec4Test, ScalarMultiplication)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f}, Vec4<crackitos_physics::commons::fp>{5.0f, 6.0f, 7.0f, 8.0f},
            Vec4<crackitos_physics::commons::fp>{9.0f, 10.0f, 11.0f, 12.0f}, Vec4<crackitos_physics::commons::fp>{13.0f, 14.0f, 15.0f, 16.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        FourVec4<crackitos_physics::commons::fp> result = a * 2.0f;

        EXPECT_EQ(result.x[0], 2.0f);
        EXPECT_EQ(result.x[1], 10.0f);
        EXPECT_EQ(result.x[2], 18.0f);
        EXPECT_EQ(result.x[3], 26.0f);

        EXPECT_EQ(result.y[0], 4.0f);
        EXPECT_EQ(result.y[1], 12.0f);
        EXPECT_EQ(result.y[2], 20.0f);
        EXPECT_EQ(result.y[3], 28.0f);

        EXPECT_EQ(result.z[0], 6.0f);
        EXPECT_EQ(result.z[1], 14.0f);
        EXPECT_EQ(result.z[2], 22.0f);
        EXPECT_EQ(result.z[3], 30.0f);

        EXPECT_EQ(result.w[0], 8.0f);
        EXPECT_EQ(result.w[1], 16.0f);
        EXPECT_EQ(result.w[2], 24.0f);
        EXPECT_EQ(result.w[3], 32.0f);
    }

    TEST(FourVec4Test, ScalarDivision)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{4.0f, 8.0f, 12.0f, 16.0f},
            Vec4<crackitos_physics::commons::fp>{2.0f, 4.0f, 6.0f, 8.0f},
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f},
            Vec4<crackitos_physics::commons::fp>{0.0f, 1.0f, 2.0f, 3.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        crackitos_physics::commons::fp scalar = 2.0f;
        FourVec4<crackitos_physics::commons::fp> result = a / scalar;

        EXPECT_FLOAT_EQ(result.x[0], 2.0f); // 4.0f / 2.0f = 2.0f
        EXPECT_FLOAT_EQ(result.y[0], 4.0f); // 8.0f / 2.0f = 4.0f
        EXPECT_FLOAT_EQ(result.z[0], 6.0f); // 12.0f / 2.0f = 6.0f
        EXPECT_FLOAT_EQ(result.w[0], 8.0f); // 16.0f / 2.0f = 8.0f

        EXPECT_FLOAT_EQ(result.x[1], 1.0f); // 2.0f / 2.0f = 1.0f
        EXPECT_FLOAT_EQ(result.y[1], 2.0f); // 4.0f / 2.0f = 2.0f
        EXPECT_FLOAT_EQ(result.z[1], 3.0f); // 6.0f / 2.0f = 3.0f
        EXPECT_FLOAT_EQ(result.w[1], 4.0f); // 8.0f / 2.0f = 4.0f

        EXPECT_FLOAT_EQ(result.x[2], 0.5f); // 1.0f / 2.0f = 0.5f
        EXPECT_FLOAT_EQ(result.y[2], 1.0f); // 2.0f / 2.0f = 1.0f
        EXPECT_FLOAT_EQ(result.z[2], 1.5f); // 3.0f / 2.0f = 1.5f
        EXPECT_FLOAT_EQ(result.w[2], 2.0f); // 4.0f / 2.0f = 2.0f

        EXPECT_FLOAT_EQ(result.x[3], 0.0f); // 0.0f / 2.0f = 0.0f
        EXPECT_FLOAT_EQ(result.y[3], 0.5f); // 1.0f / 2.0f = 0.5f
        EXPECT_FLOAT_EQ(result.z[3], 1.0f); // 2.0f / 2.0f = 1.0f
        EXPECT_FLOAT_EQ(result.w[3], 1.5f); // 3.0f / 2.0f = 1.5f
    }

    TEST(FourVec4Test, DotProduct)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f},
            Vec4<crackitos_physics::commons::fp>{2.0f, 3.0f, 4.0f, 5.0f},
            Vec4<crackitos_physics::commons::fp>{3.0f, 4.0f, 5.0f, 6.0f},
            Vec4<crackitos_physics::commons::fp>{4.0f, 5.0f, 6.0f, 7.0f}
        };

        std::array<Vec4<crackitos_physics::commons::fp>, 4> b_arr = {
            Vec4<crackitos_physics::commons::fp>{5.0f, 6.0f, 7.0f, 8.0f},
            Vec4<crackitos_physics::commons::fp>{6.0f, 7.0f, 8.0f, 9.0f},
            Vec4<crackitos_physics::commons::fp>{7.0f, 8.0f, 9.0f, 10.0f},
            Vec4<crackitos_physics::commons::fp>{8.0f, 9.0f, 10.0f, 11.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        FourVec4<crackitos_physics::commons::fp> b(b_arr);
        std::array<crackitos_physics::commons::fp, 4> result = a.Dot(b);

        EXPECT_FLOAT_EQ(result[0], 70.0f); // 1*5 + 2*6 + 3*7 + 4*8 = 70
        EXPECT_FLOAT_EQ(result[1], 110.0f); // 2*6 + 3*7 + 4*8 + 5*9 = 110
        EXPECT_FLOAT_EQ(result[2], 158.0f); // 3*7 + 4*8 + 5*9 + 6*10 = 158
        EXPECT_FLOAT_EQ(result[3], 214.0f); // 4*8 + 5*9 + 6*10 + 7*11 = 214
    }

    TEST(FourVec4Test, SquareMagnitude)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f},
            Vec4<crackitos_physics::commons::fp>{4.0f, 5.0f, 6.0f, 7.0f},
            Vec4<crackitos_physics::commons::fp>{7.0f, 8.0f, 9.0f, 10.0f},
            Vec4<crackitos_physics::commons::fp>{10.0f, 11.0f, 12.0f, 13.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        std::array<crackitos_physics::commons::fp, 4> result = a.SquareMagnitude();

        EXPECT_FLOAT_EQ(result[0], 30.0f); // 1^2 + 2^2 + 3^2 + 4^2 = 30
        EXPECT_FLOAT_EQ(result[1], 126.0f); // 4^2 + 5^2 + 6^2 + 7^2 = 126
        EXPECT_FLOAT_EQ(result[2], 294.0f); // 7^2 + 8^2 + 9^2 + 10^2 = 294
        EXPECT_FLOAT_EQ(result[3], 534.0f); // 10^2 + 11^2 + 12^2 + 13^2 = 534
    }

    TEST(FourVec4Test, Magnitude)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 3.0f, 4.0f},
            Vec4<crackitos_physics::commons::fp>{4.0f, 5.0f, 6.0f, 7.0f},
            Vec4<crackitos_physics::commons::fp>{7.0f, 8.0f, 9.0f, 10.0f},
            Vec4<crackitos_physics::commons::fp>{10.0f, 11.0f, 12.0f, 13.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        std::array<crackitos_physics::commons::fp, 4> result = a.Magnitude();

        EXPECT_NEAR(result[0], 5.477225f, 1e-5f); // sqrt(30)
        EXPECT_NEAR(result[1], 11.224972f, 1e-5f); // sqrt(126)
        EXPECT_NEAR(result[2], 17.146428f, 1e-5f); // sqrt(294)
        EXPECT_NEAR(result[3], 23.108440f, 1e-5f); // sqrt(534)
    }

    TEST(FourVec4Test, Normalize)
    {
        std::array<Vec4<crackitos_physics::commons::fp>, 4> a_arr = {
            Vec4<crackitos_physics::commons::fp>{3.0f, 0.0f, 0.0f, 0.0f},
            Vec4<crackitos_physics::commons::fp>{0.0f, 4.0f, 0.0f, 0.0f},
            Vec4<crackitos_physics::commons::fp>{0.0f, 0.0f, 5.0f, 0.0f},
            Vec4<crackitos_physics::commons::fp>{1.0f, 2.0f, 2.0f, 2.0f}
        };

        FourVec4<crackitos_physics::commons::fp> a(a_arr);
        FourVec4<crackitos_physics::commons::fp> result = a.Normalize();

        EXPECT_NEAR(result.x[0], 1.0f, 1e-5f); // Normalized vector of {3,0,0,0}
        EXPECT_NEAR(result.y[0], 0.0f, 1e-5f);
        EXPECT_NEAR(result.z[0], 0.0f, 1e-5f);
        EXPECT_NEAR(result.w[0], 0.0f, 1e-5f);

        EXPECT_NEAR(result.x[1], 0.0f, 1e-5f); // Normalized vector of {0,4,0,0}
        EXPECT_NEAR(result.y[1], 1.0f, 1e-5f);
        EXPECT_NEAR(result.z[1], 0.0f, 1e-5f);
        EXPECT_NEAR(result.w[1], 0.0f, 1e-5f);

        EXPECT_NEAR(result.x[2], 0.0f, 1e-5f); // Normalized vector of {0,0,5,0}
        EXPECT_NEAR(result.y[2], 0.0f, 1e-5f);
        EXPECT_NEAR(result.z[2], 1.0f, 1e-5f);
        EXPECT_NEAR(result.w[2], 0.0f, 1e-5f);

        crackitos_physics::commons::fp magnitude = sqrt(1.0f + 4.0f + 4.0f + 4.0f); // {1,2,2,2}
        EXPECT_NEAR(result.x[3], 1.0f / magnitude, 1e-5f);
        EXPECT_NEAR(result.y[3], 2.0f / magnitude, 1e-5f);
        EXPECT_NEAR(result.z[3], 2.0f / magnitude, 1e-5f);
        EXPECT_NEAR(result.w[3], 2.0f / magnitude, 1e-5f);
    }
}
