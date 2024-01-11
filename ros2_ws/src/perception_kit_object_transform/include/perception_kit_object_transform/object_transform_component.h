#pragma once

#include <perception_kit_msgs/msg/objects.hpp>
#include <perception_kit_msgs/msg/motion.hpp>
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <list>

// Added for Yaw transform
#include <tf2/utils.h>

namespace perception_kit
{
  namespace object_transform
  {
    class ObjectTransform : public rclcpp::Node
    {
    public:
      //virtual void onInit(rclcpp::NodeOptions options);
      explicit ObjectTransform(const rclcpp::NodeOptions &options);

    private:
      void transformObjectsToTargetFrame(const perception_kit_msgs::msg::Objects::SharedPtr msg) const;
      void queueMotion(const perception_kit_msgs::msg::Motion::SharedPtr motion);

      perception_kit_msgs::msg::Motion::SharedPtr lookupMotion(rclcpp::Time const &time) const;

      rclcpp::Subscription<perception_kit_msgs::msg::Objects>::SharedPtr objects_subscriber_;

      std::string input_frame_{};
      std::string target_frame_{};

      rclcpp::Subscription<perception_kit_msgs::msg::Motion>::SharedPtr subscriber_motion_;

      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_{tf_buffer_};

      std::list<perception_kit_msgs::msg::Motion::SharedPtr> target_frame_motion_history_;

      //mutable ros::Publisher transformed_object_publisher_;
      rclcpp::Publisher<perception_kit_msgs::msg::Objects>::SharedPtr transformed_object_publisher_;

      bool needs_motion_{false};

      /* Added for Yaw transform */
      mutable double yaw_buffer{0};
    };
  } // namespace object_transform
} // namespace perception_kit