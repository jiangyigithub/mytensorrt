#include "objects_processors/map_filter.h"
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "track_to_track_fusion/global.h"
// #include "ros2/fusion_node.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("objects_processors/map_filter.cpp");

namespace objects_processors
{
  MapFilter::MapFilter(double const &x_max, double const &x_min, double const &y_max, double const &y_min)
  : x_max_(x_max), x_min_(x_min), y_max_(y_max), y_min_(y_min)
  // MapFilter::MapFilter()
  {
  }

  bool MapFilter::process(PerceptionKitObject &object, std::string const &sensor_modality) const
  {
    // RCLCPP_INFO_STREAM(LOGGER, "\n---------------- roi: ----------------\n"
    //                                << "y_max_[" << y_max_ << "]\n"
    //                                << "y_min_[" << y_min_ << "]\n"
    //                                << "x_max_[" << x_max_ << "]\n"
    //                                << "x_max_[" << x_min_ << "]\n");

    if (object.position.y > y_max_ || object.position.y < y_min_ || object.position.x > x_max_ || object.position.x < x_min_)
    {
      if (DEBUG_MODE_)
      {
        RCLCPP_WARN_STREAM(LOGGER, "\n-----------------------------------------  obj outside ROI [deleted]:  -----------------------------------------\n"
                                       << "id: [" << object.id << "]\n"
                                       << "header: [" << object.header.stamp.sec << "." << object.header.stamp.nanosec << "]\n"
                                       << "position.x: [" << object.position.x << "]\n"
                                       << "position.y: [" << object.position.y << "]\n"
                                       << "velocity.x: [" << object.velocity.x << "]\n"
                                       << "velocity.y: [" << object.velocity.y << "]\n"
                                       << "width: [" << object.width << "]\n"
                                       << "length: [" << object.length << "]\n"
                                       << "existence_probability: [" << object.existence_probability << "]\n"
                                       << "orientation: [" << object.yaw << "]\n"
                                       << "----------------------------------------------------------------------------------------\n");
      }

      return 0;
    }
    else
    {
      return 1;
    }
  }

} // namespace objects_processors