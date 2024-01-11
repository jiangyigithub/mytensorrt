#include <gtest/gtest.h>

#include "perception_kit_visu/utilities.h"

using namespace perception_kit_visu::internal;

TEST(Visu, normalize_class_name)
{
  ASSERT_EQ(normalizeClassName("vehicle"), "car");
  ASSERT_EQ(normalizeClassName("Car"), "car");
  ASSERT_EQ(normalizeClassName("traffic light"), "traffic_light");
  ASSERT_EQ(normalizeClassName("Traffic Sign"), "traffic_sign");
  ASSERT_EQ(normalizeClassName("ABC-123!?"), "abc_123__");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
