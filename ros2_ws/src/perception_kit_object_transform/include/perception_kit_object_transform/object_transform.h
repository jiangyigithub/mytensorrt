#pragma once

#include "perception_kit_msgs/msg/objects.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace perception_kit
{
namespace object_transform
{
perception_kit_msgs::msg::Object
transformObject(perception_kit_msgs::msg::Object const& input_object, geometry_msgs::msg::TransformStamped const& transform,
                geometry_msgs::msg::Vector3 target_frame_velocity = geometry_msgs::msg::Vector3(),
                geometry_msgs::msg::Vector3 target_frame_acceleration = geometry_msgs::msg::Vector3());
}
}