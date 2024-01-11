#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class show_map : public rclcpp::Node
{
public:
  show_map();

private:
  void timer_callback();
  
  //lnl, timer
  void utc_timer();
  rclcpp::TimerBase::SharedPtr timer_t;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};
