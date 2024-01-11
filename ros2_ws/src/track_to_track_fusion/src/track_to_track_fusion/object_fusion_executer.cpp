#include "track_to_track_fusion/object_fusion_executer.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop
#include "rclcpp/rclcpp.hpp"
#define ROS_ERROR_STREAM_ONCE(log_message) \
  RCLCPP_ERROR_ONCE(rclcpp::get_logger("track_to_track_fusion.classification"), log_message)
#include <iostream>
#include "track_to_track_fusion/global.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("object_fusion_executer.cpp");

namespace track_to_track_fusion
{
  ObjectFusionExecuter::ObjectFusionExecuter(
      std::map<ObjectFusion::Feature, ObjectFusion::ConstPtr> &&feature_fusion_elements, Weights::ConstPtr weights,
      OperationToWeightsFrameTransformFunction operation_to_weights_frame)
      : feature_fusion_elements_(std::move(feature_fusion_elements)), weights_(std::move(weights)), operation_to_weights_frame_(operation_to_weights_frame)
  {
  }

  void ObjectFusionExecuter::fuse(CostCell const &cell, PerceptionKitObject &fused_object) const
  {
    auto const &object_a = cell.row_object().object();
    auto const &object_b = cell.col_object().object();
    auto const &trace_a = cell.row_object().trace();
    auto const &trace_b = cell.col_object().trace();

    std::map<std::string, int> ta = cell.row_object().trace();
    std::map<std::string, int> tb = cell.col_object().trace();

    auto const pos_a_in_weights_frame = operation_to_weights_frame_(object_a.position);
    auto const pos_b_in_weights_frame = operation_to_weights_frame_(object_b.position);

    for (auto const &feature : ObjectFusion::AllFeatures())
    {
      if (feature_fusion_elements_.count(feature))
      {
        auto const weight_a = getWeight(trace_a, feature, pos_a_in_weights_frame);
        auto const weight_b = getWeight(trace_b, feature, pos_b_in_weights_frame);

        if (DEBUG_MODE_)
        {
          RCLCPP_INFO_STREAM(LOGGER, "feature: [" << feature << "]");

          std::map<std::string, int>::iterator it_a, it_b;
          int ia = 0;
          int ib = 0;
          for (it_a = ta.begin(); it_a != ta.end(); it_a++)
          {
            ia++;
            RCLCPP_INFO_STREAM(LOGGER, "obj_a(" << ia << "/" << ta.size() << "):[" << it_a->first << ", " << it_a->second << "]"
                                                << ",  weight_a = " << weight_a);
            // ia++;
          }
          for (it_b = tb.begin(); it_b != tb.end(); it_b++)
          {
            ib++;
            RCLCPP_INFO_STREAM(LOGGER, "obj_b(" << ib << "/" << tb.size() << "):[" << it_b->first << ", " << it_b->second << "]"
                                                << ",  weight_b = " << weight_b);
            // ib++;
          }
        }

        feature_fusion_elements_.at(feature)->fuse(object_a, object_b, weight_a, weight_b, fused_object);

        if (DEBUG_MODE_)
        {
          RCLCPP_INFO_STREAM(LOGGER, "[obj_a,  obj_b,  obj_fused]: \n"
                                         << "id: [" << object_a.id << ", " << object_b.id << ", " << fused_object.id << "]\n"
                                         << "position.x: [" << object_a.position.x << ", " << object_b.position.x << ", " << fused_object.position.x << "]\n"
                                         << "position.y: [" << object_a.position.y << ", " << object_b.position.y << ", " << fused_object.position.y << "]\n"
                                         << "velocity.x: [" << object_a.velocity.x << ", " << object_b.velocity.x << ", " << fused_object.velocity.x << "]\n"
                                         << "velocity.y: [" << object_a.velocity.y << ", " << object_b.velocity.y << ", " << fused_object.velocity.y << "]\n"
                                         << "width: [" << object_a.width << ", " << object_b.width << ", " << fused_object.width << "]\n"
                                         << "length: [" << object_a.length << ", " << object_b.length << ", " << fused_object.length << "]\n"
                                         << "existence_probability: [" << object_a.existence_probability << ", " << object_b.existence_probability << ", " << fused_object.existence_probability << "]\n"
                                         << "orientation: [" << object_a.yaw << ", " << object_b.yaw << ", " << fused_object.yaw << "]\n"
                             //  << "classification: [" << object_a.classification[0].obj_class << "]\n"
          );
        }
      }
      else
      {
        ROS_ERROR_STREAM_ONCE("No fusion methon set for feature ");
      }
    }
  }

  float ObjectFusionExecuter::getWeight(ObjectWithTrace::Trace const &trace, ObjectFusion::Feature const &feature,
                                        PerceptionKitObject::_position_type const &position_in_weights_frame) const
  {
    return weights_ ? weights_->getWeight(trace, feature, position_in_weights_frame) : 1.0f;
  }
} // namespace track_to_track_fusion
