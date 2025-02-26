#include <gtest/gtest.h>

#include "four_vec2.h"
#include "commons.h"

namespace physics923::math
{
    TEST(FourVec2Test, Addition)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{1.0f, 2.0f}, Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 6.0f}, Vec2<physics923::commons::fp>{7.0f, 8.0f}
        };
        std::array<Vec2<physics923::commons::fp>, 4> b_arr = {
            Vec2<physics923::commons::fp>{9.0f, 10.0f}, Vec2<physics923::commons::fp>{11.0f, 12.0f}, Vec2<physics923::commons::fp>{13.0f, 14.0f}, Vec2<physics923::commons::fp>{15.0f, 16.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> b(b_arr);
        FourVec2<physics923::commons::fp> result = a + b;

        EXPECT_EQ(result.x[0], 10.0f);
        EXPECT_EQ(result.x[1], 14.0f);
        EXPECT_EQ(result.x[2], 18.0f);
        EXPECT_EQ(result.x[3], 22.0f);

        EXPECT_EQ(result.y[0], 12.0f);
        EXPECT_EQ(result.y[1], 16.0f);
        EXPECT_EQ(result.y[2], 20.0f);
        EXPECT_EQ(result.y[3], 24.0f);
    }

    TEST(FourVec2Test, Subtraction)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{10.0f, 20.0f}, Vec2<physics923::commons::fp>{30.0f, 40.0f}, Vec2<physics923::commons::fp>{50.0f, 60.0f}, Vec2<physics923::commons::fp>{70.0f, 80.0f}
        };
        std::array<Vec2<physics923::commons::fp>, 4> b_arr = {
            Vec2<physics923::commons::fp>{1.0f, 2.0f}, Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 6.0f}, Vec2<physics923::commons::fp>{7.0f, 8.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> b(b_arr);
        FourVec2<physics923::commons::fp> result = a - b;

        EXPECT_EQ(result.x[0], 9.0f);
        EXPECT_EQ(result.x[1], 27.0f);
        EXPECT_EQ(result.x[2], 45.0f);
        EXPECT_EQ(result.x[3], 63.0f);

        EXPECT_EQ(result.y[0], 18.0f);
        EXPECT_EQ(result.y[1], 36.0f);
        EXPECT_EQ(result.y[2], 54.0f);
        EXPECT_EQ(result.y[3], 72.0f);
    }

    // Test for negation operator (-)
    TEST(FourVec2Test, Negation)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{1.0f, -2.0f}, Vec2<physics923::commons::fp>{3.0f, -4.0f}, Vec2<physics923::commons::fp>{5.0f, -6.0f}, Vec2<physics923::commons::fp>{7.0f, -8.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> result = -a;

        EXPECT_EQ(result.x[0], -1.0f);
        EXPECT_EQ(result.x[1], -3.0f);
        EXPECT_EQ(result.x[2], -5.0f);
        EXPECT_EQ(result.x[3], -7.0f);

        EXPECT_EQ(result.y[0], 2.0f);
        EXPECT_EQ(result.y[1], 4.0f);
        EXPECT_EQ(result.y[2], 6.0f);
        EXPECT_EQ(result.y[3], 8.0f);
    }

    // Test for scalar multiplication operator (*)
    TEST(FourVec2Test, ScalarMultiplication)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{1.0f, 2.0f}, Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 6.0f}, Vec2<physics923::commons::fp>{7.0f, 8.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> result = a * 2.0f;

        EXPECT_EQ(result.x[0], 2.0f);
        EXPECT_EQ(result.x[1], 6.0f);
        EXPECT_EQ(result.x[2], 10.0f);
        EXPECT_EQ(result.x[3], 14.0f);

        EXPECT_EQ(result.y[0], 4.0f);
        EXPECT_EQ(result.y[1], 8.0f);
        EXPECT_EQ(result.y[2], 12.0f);
        EXPECT_EQ(result.y[3], 16.0f);
    }

    // Test for multiplication by an array of scalars
    TEST(FourVec2Test, ElementWiseMultiplication)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{1.0f, 2.0f}, Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 6.0f}, Vec2<physics923::commons::fp>{7.0f, 8.0f}
        };
        std::array<physics923::commons::fp, 4> scalars = {2.0f, 3.0f, 4.0f, 5.0f};

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> result = a * scalars;

        // Check x components after element-wise multiplication
        EXPECT_EQ(result.x[0], 1.0f * 2.0f);
        EXPECT_EQ(result.x[1], 3.0f * 3.0f);
        EXPECT_EQ(result.x[2], 5.0f * 4.0f);
        EXPECT_EQ(result.x[3], 7.0f * 5.0f);

        // Check y components after element-wise multiplication
        EXPECT_EQ(result.y[0], 2.0f * 2.0f);
        EXPECT_EQ(result.y[1], 4.0f * 3.0f);
        EXPECT_EQ(result.y[2], 6.0f * 4.0f);
        EXPECT_EQ(result.y[3], 8.0f * 5.0f);
    }

    // Test for scalar division operator (/)
    TEST(FourVec2Test, ScalarDivision)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{10.0f, 20.0f}, Vec2<physics923::commons::fp>{30.0f, 40.0f}, Vec2<physics923::commons::fp>{50.0f, 60.0f}, Vec2<physics923::commons::fp>{70.0f, 80.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> result = a / 10.0f;

        EXPECT_EQ(result.x[0], 1.0f);
        EXPECT_EQ(result.x[1], 3.0f);
        EXPECT_EQ(result.x[2], 5.0f);
        EXPECT_EQ(result.x[3], 7.0f);

        EXPECT_EQ(result.y[0], 2.0f);
        EXPECT_EQ(result.y[1], 4.0f);
        EXPECT_EQ(result.y[2], 6.0f);
        EXPECT_EQ(result.y[3], 8.0f);
    }

    // Test for Dot product
    TEST(FourVec2Test, DotProduct)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{1.0f, 2.0f}, Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 6.0f}, Vec2<physics923::commons::fp>{7.0f, 8.0f}
        };
        std::array<Vec2<physics923::commons::fp>, 4> b_arr = {
            Vec2<physics923::commons::fp>{9.0f, 10.0f}, Vec2<physics923::commons::fp>{11.0f, 12.0f}, Vec2<physics923::commons::fp>{13.0f, 14.0f}, Vec2<physics923::commons::fp>{15.0f, 16.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> b(b_arr);
        std::array<physics923::commons::fp, 4> result = a.Dot(b);

        EXPECT_EQ(result[0], 29.0f); // (1*9 + 2*10)
        EXPECT_EQ(result[1], 81.0f); // (3*11 + 4*12)
        EXPECT_EQ(result[2], 149.0f); // (5*13 + 6*14)
        EXPECT_EQ(result[3], 233.0f); // (7*15 + 8*16)
    }

    // Test for Square Magnitude
    TEST(FourVec2Test, SquareMagnitude)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{1.0f, 2.0f}, Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 6.0f}, Vec2<physics923::commons::fp>{7.0f, 8.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        std::array<physics923::commons::fp, 4> result = a.SquareMagnitude();

        EXPECT_EQ(result[0], 5.0f); // 1^2 + 2^2
        EXPECT_EQ(result[1], 25.0f); // 3^2 + 4^2
        EXPECT_EQ(result[2], 61.0f); // 5^2 + 6^2
        EXPECT_EQ(result[3], 113.0f); // 7^2 + 8^2
    }

    // Test for Magnitude
    TEST(FourVec2Test, Magnitude)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 12.0f}, Vec2<physics923::commons::fp>{8.0f, 15.0f}, Vec2<physics923::commons::fp>{7.0f, 24.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        std::array<physics923::commons::fp, 4> result = a.Magnitude();

        EXPECT_FLOAT_EQ(result[0], 5.0f); // sqrt(3^2 + 4^2)
        EXPECT_FLOAT_EQ(result[1], 13.0f); // sqrt(5^2 + 12^2)
        EXPECT_FLOAT_EQ(result[2], 17.0f); // sqrt(8^2 + 15^2)
        EXPECT_FLOAT_EQ(result[3], 25.0f); // sqrt(7^2 + 24^2)
    }

    // Test for Normalize
    TEST(FourVec2Test, Normalize)
    {
        std::array<Vec2<physics923::commons::fp>, 4> a_arr = {
            Vec2<physics923::commons::fp>{3.0f, 4.0f}, Vec2<physics923::commons::fp>{5.0f, 12.0f}, Vec2<physics923::commons::fp>{8.0f, 15.0f}, Vec2<physics923::commons::fp>{7.0f, 24.0f}
        };

        FourVec2<physics923::commons::fp> a(a_arr);
        FourVec2<physics923::commons::fp> result = a.Normalize();

        // Normalized vector for {3.0f, 4.0f} should be {0.6, 0.8}
        EXPECT_FLOAT_EQ(result.x[0], 0.6f);
        EXPECT_FLOAT_EQ(result.y[0], 0.8f);

        // Normalized vector for {5.0f, 12.0f} should be {5/13, 12/13}
        EXPECT_FLOAT_EQ(result.x[1], 5.0f / 13.0f);
        EXPECT_FLOAT_EQ(result.y[1], 12.0f / 13.0f);

        // Normalized vector for {8.0f, 15.0f} should be {8/17, 15/17}
        EXPECT_FLOAT_EQ(result.x[2], 8.0f / 17.0f);
        EXPECT_FLOAT_EQ(result.y[2], 15.0f / 17.0f);

        // Normalized vector for {7.0f, 24.0f} should be {7/25, 24/25}
        EXPECT_FLOAT_EQ(result.x[3], 7.0f / 25.0f);
        EXPECT_FLOAT_EQ(result.y[3], 24.0f / 25.0f);
    }
}
