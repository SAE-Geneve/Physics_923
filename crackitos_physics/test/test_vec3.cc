#include <gtest/gtest.h>

#include "vec3.h"
#include "commons.h"

/**
 * \brief Tests for Vec3 with ints
 */
namespace physics923::math
{
    struct Vec3iOperatorFixture :
        public testing::TestWithParam<std::pair<Vec3<int>, Vec3<int>>>
    {
    };

    TEST_P(Vec3iOperatorFixture, Add)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 + v2;
        EXPECT_EQ(result.x, v1.x + v2.x);
        EXPECT_EQ(result.y, v1.y + v2.y);
        EXPECT_EQ(result.z, v1.z + v2.z);
    }

    TEST_P(Vec3iOperatorFixture, Sub)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 - v2;
        EXPECT_EQ(result.x, v1.x - v2.x);
        EXPECT_EQ(result.y, v1.y - v2.y);
        EXPECT_EQ(result.z, v1.z - v2.z);

        const auto neg1 = -v1;
        const auto neg2 = -v2;
        EXPECT_EQ(neg1.x, -v1.x);
        EXPECT_EQ(neg1.y, -v1.y);
        EXPECT_EQ(neg1.z, -v1.z);
        EXPECT_EQ(neg2.x, -v2.x);
        EXPECT_EQ(neg2.y, -v2.y);
        EXPECT_EQ(neg2.z, -v2.z);
    }

    TEST_P(Vec3iOperatorFixture, Dot)
    {
        auto [v1, v2] = GetParam();
        const auto result = Vec3<int>::Dot(v1, v2);
        EXPECT_EQ(result, v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
    }

    TEST_P(Vec3iOperatorFixture, Cross)
    {
        auto [v1, v2] = GetParam();
        const auto result = Vec3<int>::Cross(v1, v2);
        EXPECT_EQ(result.x, v1.y * v2.z - v1.z * v2.y);
        EXPECT_EQ(result.y, v1.z * v2.x - v1.x * v2.z);
        EXPECT_EQ(result.z, v1.x * v2.y - v1.y * v2.x);
    }

    TEST_P(Vec3iOperatorFixture, MultiplyByScalar)
    {
        auto [v1, v2] = GetParam();
        const auto result1 = v1 * v2.x;
        const auto result2 = v1.x * v2;
        EXPECT_EQ(result1.x, v1.x * v2.x);
        EXPECT_EQ(result1.y, v1.y * v2.x);
        EXPECT_EQ(result1.z, v1.z * v2.x);
        EXPECT_EQ(result2.x, v1.x * v2.x);
        EXPECT_EQ(result2.y, v1.x * v2.y);
        EXPECT_EQ(result2.z, v1.x * v2.z);
    }

    TEST_P(Vec3iOperatorFixture, DivideByScalar)
    {
        auto [v1, v2] = GetParam();
        if (v2.x != 0)
        {
            const auto result1 = v1 / v2.x;
            EXPECT_EQ(result1.x, v1.x / v2.x);
            EXPECT_EQ(result1.y, v1.y / v2.x);
            EXPECT_EQ(result1.z, v1.z / v2.x);
        }

        if (v2.y != 0)
        {
            const auto result2 = v1 / v2.y;
            EXPECT_EQ(result2.x, v1.x / v2.y);
            EXPECT_EQ(result2.y, v1.y / v2.y);
            EXPECT_EQ(result2.z, v1.z / v2.y);
        }
    }

    TEST_P(Vec3iOperatorFixture, VectorLengths)
    {
        auto [v1, v2] = GetParam();
        const auto squareMagnitude1 = v1.SquareMagnitude();
        const auto squareMagnitude2 = v2.SquareMagnitude();
        EXPECT_EQ(squareMagnitude1, v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        EXPECT_EQ(squareMagnitude2, v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    }

    // Example Vectors
    INSTANTIATE_TEST_SUITE_P(AllNumbers, Vec3iOperatorFixture,
                             testing::Values(
                                 std::pair{ math::Vec3{-4, 3, 2}, math::Vec3{2, 4, -1} },
                                 std::pair{ math::Vec3{-10, -15, 5}, math::Vec3{-25, 35, 0} },
                                 std::pair{ math::Vec3{1, 3, 1}, math::Vec3{2, 4, 0} },
                                 std::pair{ math::Vec3{10, 15, 10}, math::Vec3{25, 35, 5} },
                                 std::pair{ math::Vec3<int>{}, math::Vec3<int>{} },
                                 std::pair{ math::Vec3{1, 5, 2}, math::Vec3<int>{} },
                                 std::pair{ math::Vec3<int>{}, math::Vec3{1, 7, 3} }
                             )
    );

    /**
     * \brief Tests for Vec3 with physics923::commons::fps
     */
    struct Vec3fOperatorFixture :
        public testing::TestWithParam<std::pair<Vec3<physics923::commons::fp>, Vec3<physics923::commons::fp>>>
    {
    };

    TEST_P(Vec3fOperatorFixture, Add)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 + v2;
        EXPECT_FLOAT_EQ(result.x, v1.x + v2.x);
        EXPECT_FLOAT_EQ(result.y, v1.y + v2.y);
        EXPECT_FLOAT_EQ(result.z, v1.z + v2.z);
    }

    TEST_P(Vec3fOperatorFixture, Sub)
    {
        auto [v1, v2] = GetParam();
        const auto result = v1 - v2;
        EXPECT_FLOAT_EQ(result.x, v1.x - v2.x);
        EXPECT_FLOAT_EQ(result.y, v1.y - v2.y);
        EXPECT_FLOAT_EQ(result.z, v1.z - v2.z);

        const auto neg1 = -v1;
        const auto neg2 = -v2;
        EXPECT_FLOAT_EQ(neg1.x, -v1.x);
        EXPECT_FLOAT_EQ(neg1.y, -v1.y);
        EXPECT_FLOAT_EQ(neg1.z, -v1.z);
        EXPECT_FLOAT_EQ(neg2.x, -v2.x);
        EXPECT_FLOAT_EQ(neg2.y, -v2.y);
        EXPECT_FLOAT_EQ(neg2.z, -v2.z);
    }

    TEST_P(Vec3fOperatorFixture, Dot)
    {
        auto [v1, v2] = GetParam();
        const auto result = Vec3<physics923::commons::fp>::Dot(v1, v2);
        EXPECT_FLOAT_EQ(result, v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
    }

    TEST_P(Vec3fOperatorFixture, Cross)
    {
        auto [v1, v2] = GetParam();
        const auto result = Vec3<physics923::commons::fp>::Cross(v1, v2);
        EXPECT_FLOAT_EQ(result.x, v1.y * v2.z - v1.z * v2.y);
        EXPECT_FLOAT_EQ(result.y, v1.z * v2.x - v1.x * v2.z);
        EXPECT_FLOAT_EQ(result.z, v1.x * v2.y - v1.y * v2.x);
    }

    TEST_P(Vec3fOperatorFixture, MultiplyByScalar)
    {
        auto [v1, v2] = GetParam();
        const auto result1 = v1 * v2.x;
        const auto result2 = v1.x * v2;
        EXPECT_FLOAT_EQ(result1.x, v1.x * v2.x);
        EXPECT_FLOAT_EQ(result1.y, v1.y * v2.x);
        EXPECT_FLOAT_EQ(result1.z, v1.z * v2.x);
        EXPECT_FLOAT_EQ(result2.x, v1.x * v2.x);
        EXPECT_FLOAT_EQ(result2.y, v1.x * v2.y);
        EXPECT_FLOAT_EQ(result2.z, v1.x * v2.z);
    }

    TEST_P(Vec3fOperatorFixture, DivideByScalar)
    {
        auto [v1, v2] = GetParam();
        if (v2.x != 0)
        {
            const auto result1 = v1 / v2.x;
            EXPECT_FLOAT_EQ(result1.x, v1.x / v2.x);
            EXPECT_FLOAT_EQ(result1.y, v1.y / v2.x);
            EXPECT_FLOAT_EQ(result1.z, v1.z / v2.x);
        }

        if (v2.y != 0)
        {
            const auto result2 = v1 / v2.y;
            EXPECT_FLOAT_EQ(result2.x, v1.x / v2.y);
            EXPECT_FLOAT_EQ(result2.y, v1.y / v2.y);
            EXPECT_FLOAT_EQ(result2.z, v1.z / v2.y);
        }
    }

    TEST_P(Vec3fOperatorFixture, VectorLengths)
    {
        auto [v1, v2] = GetParam();
        const auto squareMagnitude1 = v1.SquareMagnitude();
        const auto squareMagnitude2 = v2.SquareMagnitude();
        EXPECT_FLOAT_EQ(squareMagnitude1, v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        EXPECT_FLOAT_EQ(squareMagnitude2, v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    }


    // Example Float Vectors
    INSTANTIATE_TEST_SUITE_P(AllNumbers, Vec3fOperatorFixture,
                             testing::Values(
                                 std::pair{ math::Vec3{-4.1f, 3.2f, 2.5f}, math::Vec3{2.3f, 4.5f, -1.1f} },
                                 std::pair{ math::Vec3{-10.5f, -15.3f, 5.0f}, math::Vec3{-25.8f, 35.2f, 0.0f} },
                                 std::pair{ math::Vec3{1.0f, 3.0f, 1.5f}, math::Vec3{2.2f, 4.1f, 0.8f} },
                                 std::pair{ math::Vec3{10.7f, 15.8f, 10.1f}, math::Vec3{25.3f, 35.4f, 5.5f} },
                                 std::pair{ math::Vec3<physics923::commons::fp>{}, math::Vec3<physics923::commons::fp>{} },
                                 std::pair{ math::Vec3{1.5f, 5.1f, 2.8f}, math::Vec3<physics923::commons::fp>{} },
                                 std::pair{ math::Vec3<physics923::commons::fp>{}, math::Vec3{1.9f, 7.6f, 3.3f} }
                             )

    );
}