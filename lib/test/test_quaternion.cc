#include <gtest/gtest.h>

#include <numbers>

#include "quaternion.h"
namespace physics923::math
{
    struct QuaternionFixture : public testing::Test
    {
    };

    TEST_F(QuaternionFixture, DefaultIdentityQuaternion)
    {
        Quaternion<float> quat;
        EXPECT_FLOAT_EQ(quat.W, 1);
        EXPECT_FLOAT_EQ(quat.V.x, 0);
        EXPECT_FLOAT_EQ(quat.V.y, 0);
        EXPECT_FLOAT_EQ(quat.V.z, 0);
    }

    TEST_F(QuaternionFixture, QuaternionMultiplication)
    {
        Quaternion<float> quat1{1, 2, 3, 4};
        Quaternion<float> quat2{5, 6, 7, 8};

        auto result = quat1 * quat2;

        EXPECT_FLOAT_EQ(result.W, -60);
        EXPECT_FLOAT_EQ(result.V.x, 12);
        EXPECT_FLOAT_EQ(result.V.y, 30);
        EXPECT_FLOAT_EQ(result.V.z, 24);
    }

    TEST_F(QuaternionFixture, QuaternionConjugate)
    {
        Quaternion<float> quat{1, 2, 3, 4};

        auto conjugate = quat.Conjugate();

        EXPECT_FLOAT_EQ(conjugate.W, 1);
        EXPECT_FLOAT_EQ(conjugate.V.x, -2);
        EXPECT_FLOAT_EQ(conjugate.V.y, -3);
        EXPECT_FLOAT_EQ(conjugate.V.z, -4);
    }

    TEST_F(QuaternionFixture, IdentityQuaternion)
    {
        auto identity = Quaternion<float>::Identity();

        EXPECT_FLOAT_EQ(identity.W, 1);
        EXPECT_FLOAT_EQ(identity.V.x, 0);
        EXPECT_FLOAT_EQ(identity.V.y, 0);
        EXPECT_FLOAT_EQ(identity.V.z, 0);
    }

    TEST_F(QuaternionFixture, EulerToQuaternionConversion)
    {
        // 90-degree rotation around the Z axis
        Quaternion<float> quat;
        auto result = quat.EulerToQuaternion(0, 0, static_cast<float>(std::numbers::pi_v<float> / 2));

        EXPECT_NEAR(result.W, std::cos(static_cast<float>(std::numbers::pi_v<float> / 4)), 1e-6);
        EXPECT_NEAR(result.V.x, 0, 1e-6);
        EXPECT_NEAR(result.V.y, 0, 1e-6);
        EXPECT_NEAR(result.V.z, std::sin(static_cast<float>(std::numbers::pi_v<float> / 4)), 1e-6);
    }
}