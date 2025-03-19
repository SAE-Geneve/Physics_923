//
// Created by Mat on 3/11/2025.
//

#include <gtest/gtest.h>

#include "distance.h"

namespace crackitos_physics::angles {
TEST(Meter, constructor_and_convert) {

  const auto test_value = 12;
  const auto ratio = 20.0f;

  auto e = distance::Meter(test_value, ratio);

  const auto result = Convert<distance::Meter, distance::Pixel>(e);
  const double compare = test_value * ratio; //process to swap to pixels

  EXPECT_EQ(e.value, test_value);
  EXPECT_DOUBLE_EQ(result.value, compare);
}

TEST(Pixel, constructor_and_convert) {

  const auto test_value = 12;
  const auto ratio = 0.01f;

  auto e = distance::Pixel(test_value, ratio);

  const auto result = Convert<distance::Pixel, distance::Meter>(e);
  const double compare = test_value * ratio; //process to swap to meters

  EXPECT_EQ(e.value, test_value);
  EXPECT_DOUBLE_EQ(result.value, compare);
}
}