#include <gtest/gtest.h>
#include "perception_kit_object_transform/object_transform.h"
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double const epsilon{ 1e-10 };
rclcpp::Time const observation_time{ 1 };

#define DIAGONAL(x) x* perception_kit_msgs::msg::Object::NUM_COVARIANCE_ELEMENTS + x

geometry_msgs::msg::TransformStamped getTransformYawRotation(rclcpp::Time const& time, double const yaw_rotation,
                                                             std::string const& from_frame, std::string const& to_frame)
{
  geometry_msgs::msg::TransformStamped ret;
  ret.header.frame_id = from_frame;
  ret.header.stamp = time;
  ret.child_frame_id = to_frame;

  tf2::Quaternion nwu_to_enu;
  nwu_to_enu.setRPY(0, 0, yaw_rotation);
  tf2::convert(nwu_to_enu, ret.transform.rotation);
  return ret;
}

// Test 001: Object is driving in enu frame with 1m/s towards east. Transformation to nwu frame.
TEST(transformations, enu_to_nwu_no_egomotion)
{
  // Object is driving 1 m/s constant velocity towards east in coordinate system east north up
  perception_kit_msgs::msg::Object input_object;
  input_object.header.frame_id = "enu";
  input_object.header.stamp = observation_time;
  input_object.velocity.x = 1;

  // We want to convert it into an coordinate system north west up
  geometry_msgs::msg::TransformStamped const transform = getTransformYawRotation(observation_time, -M_PI_2, "nwu", "enu");
  auto const rotated_object = perception_kit::object_transform::transformObject(input_object, transform);

  // Now, from the perspective of the NWU coordinate system, this object
  // ... is in the nwu coordinate system
  ASSERT_EQ(rotated_object.header.frame_id, "nwu");

  // ... has a speed in west direction (y) of -1 m/s (as it is actually driving east)
  ASSERT_NEAR(rotated_object.velocity.y, -1, epsilon);

  // ... has a speed in x an z direction of zero
  ASSERT_NEAR(rotated_object.velocity.x, 0.0, epsilon);
  ASSERT_NEAR(rotated_object.velocity.z, 0.0, epsilon);

  // ... has the same timestamp than the original
  ASSERT_EQ(input_object.header.stamp, rotated_object.header.stamp);

  // .. has the same pose than the original object (as enu and nwu have the same origin)
  ASSERT_NEAR(rotated_object.position.x, 0.0, epsilon);
  ASSERT_NEAR(rotated_object.position.y, 0.0, epsilon);
  ASSERT_NEAR(rotated_object.position.z, 0.0, epsilon);
  ASSERT_NEAR(input_object.position.x, 0.0, epsilon);
  ASSERT_NEAR(input_object.position.y, 0.0, epsilon);
  ASSERT_NEAR(input_object.position.z, 0.0, epsilon);
}

// Test 002: Same as 001, but with variances
TEST(transformations, enu_to_nwu_no_egomotion_with_variances)
{
  // Object is driving 1 m/s constant velocity towards east in coordinate system east north up
  perception_kit_msgs::msg::Object input_object;
  input_object.header.frame_id = "enu";
  input_object.header.stamp = observation_time;
  input_object.velocity.x = 1;

  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_X)] = 1.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_Y)] = 2.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_Z)] = 3.0;

  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_X)] = 4.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Y)] = 5.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Z)] = 6.0;

  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_X)] = 7.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Y)] = 8.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Z)] = 9.0;

  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)] = 10.0;
  input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW_RATE)] = 11.0;

  // We want to convert it into an coordinate system north west up
  geometry_msgs::msg::TransformStamped const transform = getTransformYawRotation(observation_time, -M_PI_2, "nwu", "enu");
  auto const rotated_object = perception_kit::object_transform::transformObject(input_object, transform);

  // Now, from the perspective of the NWU coordinate system, this object
  // ... has a velocity covariance in x direction of input object y and vice versa. z remains the same
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_X)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Y)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Y)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_X)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Z)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_VELOCITY_Z)], epsilon);

  // ... the same applies for the postion and the acceleration
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_X)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_Y)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_Y)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_X)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_Z)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_POSITION_Z)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_X)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Y)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Y)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_X)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Z)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_ACCELERATION_Z)], epsilon);

  // ... the yaw rate remains the same for the variance, as this object is simply rotated
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW)], epsilon);
  ASSERT_NEAR(input_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW_RATE)],
              rotated_object.covariance[DIAGONAL(perception_kit_msgs::msg::Object::ELEMENT_YAW_RATE)], epsilon);
}

// Test 003: Object is driving in enu with a velocity of 1 m/s over ground in east direction.
//           The observer (ego) is 2 meters behind, having the same velocity in x direction and no velocity towards
//           another direction. The observers coordinate system is the base_link frame.
//           This is a typical setup from a lidar for instance measuring an object in layered_map_enu over ground and
//           you want to have the object properties in base_link.
TEST(transformations, following_object)
{
  // Object is driving 1 m/s constant velocity towards east in coordinate system east north up at position (2,0)
  perception_kit_msgs::msg::Object input_object;
  input_object.header.frame_id = "enu";
  input_object.header.stamp = observation_time;
  input_object.velocity.x = 1.0;
  input_object.position.x = 2.0;

  // Ego is driving in "base_link". At this moment, the enu and the base_link match exactly
  auto const identity_transform = getTransformYawRotation(observation_time, 0, "enu", "base_link");

  // The ego object (the base link frame) has a speed of 1 m/s in x (east) direction
  geometry_msgs::msg::Vector3 base_link_velocity;
  base_link_velocity.x = 1.0;

  // Do the transformation to the moving base link frame
  auto transformed_object =
      perception_kit::object_transform::transformObject(input_object, identity_transform, base_link_velocity);

  // So from the ego perspective, this object

  // ... has a pose of 2,0
  ASSERT_NEAR(transformed_object.position.x, 2.0, epsilon);
  ASSERT_NEAR(transformed_object.position.y, 0.0, epsilon);
  ASSERT_NEAR(transformed_object.position.z, 0.0, epsilon);

  // ... has a velocity of zero (as it is driving exactly the same speed ahead of us than we do)
  ASSERT_NEAR(transformed_object.velocity.x, 0.0, epsilon);
  ASSERT_NEAR(transformed_object.velocity.y, 0.0, epsilon);
  ASSERT_NEAR(transformed_object.velocity.z, 0.0, epsilon);
}

// Test 004: Object is driving in enu with a velocity of 1 m/s over ground in east direction.
//           The observer (ego) is 2 south of the object, having a velocity of 2 m/s in north direction. The observers
//           coordinate system is the base_link frame, which is directed with x towards north at observation time.
//           This can be a typical setup approching a T-crossing.
TEST(transformations, T_crossing)
{
  // Object is driving 1 m/s constant velocity towards east in coordinate system east north up at position (2,0)
  perception_kit_msgs::msg::Object input_object;
  input_object.header.frame_id = "enu";
  input_object.header.stamp = observation_time;
  input_object.velocity.x = 1.0;
  input_object.position.y = 2.0;

  // Ego is driving in "base_link". At this moment, base link is directed towards with x north
  auto const transform = getTransformYawRotation(observation_time, -M_PI_2, "enu", "base_link");

  // The ego object (the base link frame) has a speed of 2 m/s in y (north) direction
  geometry_msgs::msg::Vector3 base_link_velocity;
  base_link_velocity.x = 2.0;

  // Do the transformation to the moving base link frame
  auto transformed_object =
      perception_kit::object_transform::transformObject(input_object, transform, base_link_velocity);

  // So from the ego perspective, this object

  // ... is 2 (north),0,0 in pose
  ASSERT_NEAR(transformed_object.position.x, 2.0, epsilon);
  ASSERT_NEAR(transformed_object.position.y, 0.0, epsilon);
  ASSERT_NEAR(transformed_object.position.z, 0.0, epsilon);

  // ... has a velocity in x direction of -2 m/s in x, as ego is driving 2m/s up north and object is driving east
  ASSERT_NEAR(transformed_object.velocity.x, -2.0, epsilon);

  // ... has a velocity in y direction of -1 m/s in y (pointing west), as ego is driving 0m/s east/west and object is
  //     driving 1m/s east
  ASSERT_NEAR(transformed_object.velocity.y, -1.0, epsilon);
  ASSERT_NEAR(transformed_object.velocity.z, 0.0, epsilon);
}

// Test 005: Test that all transform invariant values remain as they are
TEST(transformations, transform_invariant_values_remain)
{
  // Create input object and fill with random values
  perception_kit_msgs::msg::Object input_object;
  input_object.header.frame_id = "enu";
  input_object.header.stamp = observation_time;
  perception_kit_msgs::msg::Attribute attribute;
  attribute.name = "test";
  attribute.value.push_back(1.0f);
  attribute.value.push_back(1.1f);
  attribute.value.push_back(1.2f);
  input_object.attributes.push_back(attribute);
  perception_kit_msgs::msg::Classification classification;
  classification.obj_class = "test_class";
  classification.confidence = 0.34;
  input_object.classification.push_back(classification);
  input_object.existence_probability = 0.999;
  input_object.height = 1.2;
  input_object.height_variance = 1.2;
  input_object.id, input_object.id = 0.2;
  input_object.length, input_object.length = 0.7;
  input_object.length_variance = 0.4;
  input_object.width = 0.12212;
  input_object.width_variance = M_1_PI;
  input_object.x_offset = -10;
  input_object.yaw_rate = 11.1;

  auto const transform = getTransformYawRotation(observation_time, 0, "enu", "base_link");

  geometry_msgs::msg::Vector3 base_link_velocity;

  auto transformed_object =
      perception_kit::object_transform::transformObject(input_object, transform, base_link_velocity);

  for (int i = 0; i < transformed_object.attributes.size(); ++i)
  {
    ASSERT_EQ(transformed_object.attributes[i].name, input_object.attributes[i].name);
    for (int j = 0; j < transformed_object.attributes[i].value.size(); ++j)
    {
      ASSERT_EQ(transformed_object.attributes[i].value[j], input_object.attributes[i].value[j]);
    }
  }
  for (int i = 0; i < transformed_object.classification.size(); ++i)
  {
    ASSERT_EQ(std::string(transformed_object.classification[i].obj_class),
              std::string(input_object.classification[i].obj_class));
    ASSERT_EQ(transformed_object.classification[i].confidence, input_object.classification[i].confidence);
  }
  ASSERT_EQ(transformed_object.existence_probability, input_object.existence_probability);
  ASSERT_EQ(transformed_object.height, input_object.height);
  ASSERT_EQ(transformed_object.height_variance, input_object.height_variance);
  ASSERT_EQ(transformed_object.id, input_object.id);
  ASSERT_EQ(transformed_object.length, input_object.length);
  ASSERT_EQ(transformed_object.length_variance, input_object.length_variance);
  ASSERT_EQ(transformed_object.width, input_object.width);
  ASSERT_EQ(transformed_object.width_variance, input_object.width_variance);
  ASSERT_EQ(transformed_object.x_offset, input_object.x_offset);
  ASSERT_EQ(transformed_object.yaw_rate, input_object.yaw_rate);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ::testing::GTEST_FLAG(filter) = "*";

  return RUN_ALL_TESTS();
}