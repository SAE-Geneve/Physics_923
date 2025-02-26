#include <gtest/gtest.h>

#include "vec4.h"
#include "commons.h"
/**
 * \brief Tests for Vec4 with ints
 */
namespace physics923::math
{
    struct Vec4iOperatorFixture :
        public testing::TestWithParam<std::pair<Vec4<int>, Vec4<int>>>
    {
    };

    TEST_P(Vec4iOperatorFixture, Add)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 + v2;
        EXPECT_EQ(result.x, v1.x + v2.x);
        EXPECT_EQ(result.y, v1.y + v2.y);
        EXPECT_EQ(result.z, v1.z + v2.z);
        EXPECT_EQ(result.w, v1.w + v2.w);
    }

    TEST_P(Vec4iOperatorFixture, Sub)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 - v2;
        EXPECT_EQ(result.x, v1.x - v2.x);
        EXPECT_EQ(result.y, v1.y - v2.y);
        EXPECT_EQ(result.z, v1.z - v2.z);
        EXPECT_EQ(result.w, v1.w - v2.w);

        const auto neg1 = -v1;
        const auto neg2 = -v2;
        EXPECT_EQ(neg1.x, -v1.x);
        EXPECT_EQ(neg1.y, -v1.y);
        EXPECT_EQ(neg1.z, -v1.z);
        EXPECT_EQ(neg1.w, -v1.w);
        EXPECT_EQ(neg2.x, -v2.x);
        EXPECT_EQ(neg2.y, -v2.y);
        EXPECT_EQ(neg2.z, -v2.z);
        EXPECT_EQ(neg2.w, -v2.w);
    }

    TEST_P(Vec4iOperatorFixture, Dot)
    {
        auto [v1, v2] = GetParam();
        const auto result = Vec4<int>::Dot(v1, v2);
        EXPECT_EQ(result, v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w);
    }

    TEST_P(Vec4iOperatorFixture, MultiplyByScalar)
    {
        auto [v1, v2] = GetParam();
        const auto result1 = v1 * v2.x;
        const auto result2 = v1.x * v2;
        EXPECT_EQ(result1.x, v1.x * v2.x);
        EXPECT_EQ(result1.y, v1.y * v2.x);
        EXPECT_EQ(result1.z, v1.z * v2.x);
        EXPECT_EQ(result1.w, v1.w * v2.x);
        EXPECT_EQ(result2.x, v1.x * v2.x);
        EXPECT_EQ(result2.y, v1.x * v2.y);
        EXPECT_EQ(result2.z, v1.x * v2.z);
        EXPECT_EQ(result2.w, v1.x * v2.w);
    }

    TEST_P(Vec4iOperatorFixture, DivideByScalar)
    {
        auto [v1, v2] = GetParam();
        if (v2.x != 0)
        {
            const auto result1 = v1 / v2.x;
            EXPECT_EQ(result1.x, v1.x / v2.x);
            EXPECT_EQ(result1.y, v1.y / v2.x);
            EXPECT_EQ(result1.z, v1.z / v2.x);
            EXPECT_EQ(result1.w, v1.w / v2.x);
        }

        if (v2.y != 0)
        {
            const auto result2 = v1 / v2.y;
            EXPECT_EQ(result2.x, v1.x / v2.y);
            EXPECT_EQ(result2.y, v1.y / v2.y);
            EXPECT_EQ(result2.z, v1.z / v2.y);
            EXPECT_EQ(result2.w, v1.w / v2.y);
        }
    }

    TEST_P(Vec4iOperatorFixture, VectorLengths)
    {
        auto [v1, v2] = GetParam();
        const auto squareMagnitude1 = v1.SquareMagnitude();
        const auto squareMagnitude2 = v2.SquareMagnitude();
        EXPECT_EQ(squareMagnitude1, v1.x * v1.x + v1.y * v1.y + v1.z * v1.z + v1.w * v1.w);
        EXPECT_EQ(squareMagnitude2, v2.x * v2.x + v2.y * v2.y + v2.z * v2.z + v2.w * v2.w);
    }

    // Example Vectors
    INSTANTIATE_TEST_SUITE_P(AllNumbers, Vec4iOperatorFixture,
                             testing::Values(
                                 std::pair{ math::Vec4{-4, 3, 2, 1}, math::Vec4{2, 4, -1, 0} },
                                 std::pair{ math::Vec4{-10, -15, 5, 3}, math::Vec4{-25, 35, 0, 7} },
                                 std::pair{ math::Vec4{1, 3, 1, 2}, math::Vec4{2, 4, 0, 1} },
                                 std::pair{ math::Vec4{10, 15, 10, 5}, math::Vec4{25, 35, 5, 3} },
                                 std::pair{ math::Vec4<int>{}, math::Vec4<int>{} },
                                 std::pair{ math::Vec4{1, 5, 2, 1}, math::Vec4<int>{} },
                                 std::pair{ math::Vec4<int>{}, math::Vec4{1, 7, 3, 1} }
                             )
    );

    /**
     * \brief Tests for Vec4 with physics923::commons::fps
     */
    struct Vec4fOperatorFixture :
        public testing::TestWithParam<std::pair<Vec4<physics923::commons::fp>, Vec4<physics923::commons::fp>>>
    {
    };

    TEST_P(Vec4fOperatorFixture, Add)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 + v2;
        EXPECT_FLOAT_EQ(result.x, v1.x + v2.x);
        EXPECT_FLOAT_EQ(result.y, v1.y + v2.y);
        EXPECT_FLOAT_EQ(result.z, v1.z + v2.z);
        EXPECT_FLOAT_EQ(result.w, v1.w + v2.w);
    }

    TEST_P(Vec4fOperatorFixture, Sub)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 - v2;
        EXPECT_FLOAT_EQ(result.x, v1.x - v2.x);
        EXPECT_FLOAT_EQ(result.y, v1.y - v2.y);
        EXPECT_FLOAT_EQ(result.z, v1.z - v2.z);
        EXPECT_FLOAT_EQ(result.w, v1.w - v2.w);

        const auto neg1 = -v1;
        const auto neg2 = -v2;
        EXPECT_FLOAT_EQ(neg1.x, -v1.x);
        EXPECT_FLOAT_EQ(neg1.y, -v1.y);
        EXPECT_FLOAT_EQ(neg1.z, -v1.z);
        EXPECT_FLOAT_EQ(neg1.w, -v1.w);
        EXPECT_FLOAT_EQ(neg2.x, -v2.x);
        EXPECT_FLOAT_EQ(neg2.y, -v2.y);
        EXPECT_FLOAT_EQ(neg2.z, -v2.z);
        EXPECT_FLOAT_EQ(neg2.w, -v2.w);
    }

    TEST_P(Vec4fOperatorFixture, Dot)
    {
        auto [v1, v2] = GetParam();
        const auto result = Vec4<physics923::commons::fp>::Dot(v1, v2);
        EXPECT_FLOAT_EQ(result, v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w);
    }

    TEST_P(Vec4fOperatorFixture, MultiplyByScalar)
    {
        auto [v1, v2] = GetParam();
        const auto result1 = v1 * v2.x;
        const auto result2 = v1.x * v2;
        EXPECT_FLOAT_EQ(result1.x, v1.x * v2.x);
        EXPECT_FLOAT_EQ(result1.y, v1.y * v2.x);
        EXPECT_FLOAT_EQ(result1.z, v1.z * v2.x);
        EXPECT_FLOAT_EQ(result1.w, v1.w * v2.x);
        EXPECT_FLOAT_EQ(result2.x, v1.x * v2.x);
        EXPECT_FLOAT_EQ(result2.y, v1.x * v2.y);
        EXPECT_FLOAT_EQ(result2.z, v1.x * v2.z);
        EXPECT_FLOAT_EQ(result2.w, v1.x * v2.w);
    }

    TEST_P(Vec4fOperatorFixture, DivideByScalar)
    {
        auto [v1, v2] = GetParam();
        if (v2.x != 0)
        {
            const auto result1 = v1 / v2.x;
            EXPECT_FLOAT_EQ(result1.x, v1.x / v2.x);
            EXPECT_FLOAT_EQ(result1.y, v1.y / v2.x);
            EXPECT_FLOAT_EQ(result1.z, v1.z / v2.x);
            EXPECT_FLOAT_EQ(result1.w, v1.w / v2.x);
        }

        if (v2.y != 0)
        {
            const auto result2 = v1 / v2.y;
            EXPECT_FLOAT_EQ(result2.x, v1.x / v2.y);
            EXPECT_FLOAT_EQ(result2.y, v1.y / v2.y);
            EXPECT_FLOAT_EQ(result2.z, v1.z / v2.y);
            EXPECT_FLOAT_EQ(result2.w, v1.w / v2.y);
        }
    }

    TEST_P(Vec4fOperatorFixture, VectorLengths)
    {
        auto [v1, v2] = GetParam();
        const auto squareMagnitude1 = v1.SquareMagnitude();
        const auto squareMagnitude2 = v2.SquareMagnitude();
        EXPECT_FLOAT_EQ(squareMagnitude1, v1.x * v1.x + v1.y * v1.y + v1.z * v1.z + v1.w * v1.w);
        EXPECT_FLOAT_EQ(squareMagnitude2, v2.x * v2.x + v2.y * v2.y + v2.z * v2.z + v2.w * v2.w);
    }

    // Example Float Vectors
    INSTANTIATE_TEST_SUITE_P(AllNumbers, Vec4fOperatorFixture,
                             testing::Values(
                                 std::pair{ math::Vec4{-4.1f, 3.2f, 2.5f, 1.0f}, math::Vec4{2.3f, 4.5f, -1.1f, 0.0f} },
                                 std::pair{ math::Vec4{-10.5f, -15.3f, 5.0f, 3.8f}, math::Vec4{-25.8f, 35.2f, 0.0f, 7.2f} },
                                 std::pair{ math::Vec4{1.0f, 3.0f, 1.5f, 2.2f}, math::Vec4{2.2f, 4.1f, 0.8f, 1.1f} },
                                 std::pair{ math::Vec4{10.7f, 15.8f, 10.1f, 5.5f}, math::Vec4{25.3f, 35.4f, 5.5f, 3.3f} },
                                 std::pair{ math::Vec4<physics923::commons::fp>{}, math::Vec4<physics923::commons::fp>{} },
                                 std::pair{ math::Vec4{1.5f, 5.1f, 2.8f, 1.2f}, math::Vec4<physics923::commons::fp>{} },
                                 std::pair{ math::Vec4<physics923::commons::fp>{}, math::Vec4{1.9f, 7.6f, 3.3f, 1.4f} }
                             )
    );
}