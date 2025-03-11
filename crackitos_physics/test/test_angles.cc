//
// Created by Mat on 3/11/2025.
//

#include <gtest/gtest.h>

#include "angle.h"
#include "commons.h"

namespace crackitos_physics::angles {
TEST(Deg, constructor_and_convert) {

  const auto test_value = 12;

  auto e = crackitos_physics::angles::Deg(test_value);

  const auto result = Convert<Deg, Rad>(e);
  constexpr double compare = test_value * (commons::kPi / 180.0); //process to swap to rad

  EXPECT_EQ(e.value, test_value);
  EXPECT_DOUBLE_EQ(result.value, compare);
}

TEST(Rad, constructor_and_convert) {

  const auto test_value = 12;

  auto e = crackitos_physics::angles::Rad(test_value);

  const auto result = Convert<Rad, Deg>(e);
  constexpr double compare = test_value * (180.0 / commons::kPi); //process to swap to deg

  EXPECT_EQ(e.value, test_value);
  EXPECT_DOUBLE_EQ(result.value, compare);
}
}