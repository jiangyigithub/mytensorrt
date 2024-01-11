#include "show_map.hpp"
#include <math.h>

show_map::show_map() : Node("show_map")
{

  pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&show_map::timer_callback, this));
  
  //lnl, timer
  timer_t = this->create_wall_timer(10ms, std::bind(&show_map::utc_timer, this));
}

void show_map::utc_timer()
{
  //////////lnl, timer////////////
  auto const timer = this->now();
  const time_t timer_linux = static_cast<time_t>(timer.seconds());
  // auto const timer_ms = int((timer.nanoseconds() - int(timer.seconds())*1.0e+09f));///1.0e+06f);
  const uint timer_sec = floor(timer.seconds());
  auto const timer_ms =int(floor((timer.nanoseconds() - timer_sec*1.0E+09)/1.0E+06)); 
  const tm* timer_gm = gmtime(&timer_linux);
  char buf[128]={0};
  strftime(buf, 64,"Y-%m-%d %H:%M:%S",timer_gm);
  // std::string buf_string;
  // buf_string<<buf;

  const std::string timer_show = "::" + std::to_string(timer_ms);
  RCLCPP_INFO_STREAM(this->get_logger(), "ros sys utc timer"<<buf<<timer_show);
  /////////////////////////////////
}

void show_map::timer_callback()
{
    visualization_msgs::msg::Marker cylinder, linelist;
    cylinder.header.frame_id = "reference_point";
    cylinder.header.stamp = get_clock()->now();
    cylinder.ns = "cylinder";
    cylinder.id = 0;
    cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    cylinder.pose.position.x = 0;
    cylinder.pose.position.y = 0;
    cylinder.pose.position.z = 1.5;
    cylinder.pose.orientation.x = 0.0;
    cylinder.pose.orientation.y = 0.0;
    cylinder.pose.orientation.z = 0.0;
    cylinder.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    cylinder.scale.x = 0.5;
    cylinder.scale.y = 0.5;
    cylinder.scale.z = 3.0;

    // Set the color -- be sure to set alpha to something non-zero!
    cylinder.color.r = 0.0f;
    cylinder.color.g = 1.0f;
    cylinder.color.b = 0.0f;
    cylinder.color.a = 1.0;

    // RCLCPP_INFO(this->get_logger(), "Pub marker !!!");
    // pub_marker_->publish(cylinder);

    double dt_length_marker = 20;
    double dt_width_marker = 7;

    // std::vector<geometry_msgs::msg::Point> p_list;
    // geometry_msgs::msg::Point p;
    // for(int id=1; id<9; id++)
    // {
    //   p.x = id;
    //   p.y = 0;
    //   p.z = 0;
    //   cylinder.id = id;
    //   cylinder.points.push_back(p);
    // }

    // pub_marker_->publish(cylinder);

    for (int id = 1; id < 9; id++)
    {
      cylinder.id = id;

      if (id == 1)
        ;
      else if (id % 2 == 0) //偶数
      {
        cylinder.pose.position.y += dt_width_marker;
      }
      else
      {
        cylinder.pose.position.y -= dt_width_marker;
        cylinder.pose.position.x += dt_length_marker;
      }

      pub_marker_->publish(cylinder);
      RCLCPP_INFO(this->get_logger(), "Pub cylinder at [%f, %f, %f]", cylinder.pose.position.x, cylinder.pose.position.y, cylinder.pose.position.z);
    }

    linelist.header.frame_id = "reference_point";
    linelist.header.stamp = get_clock()->now();
    linelist.ns = "line";
    linelist.id = 0;
    linelist.type = visualization_msgs::msg::Marker::LINE_LIST;
    linelist.action = visualization_msgs::msg::Marker::ADD;

    linelist.scale.x = 0.1; //line width
    // linelist.scale.y = 0.2;
    // linelist.scale.z = 1.0;

    linelist.color.r = 0.0f;
    linelist.color.g = 0.0f;
    linelist.color.b = 1.0f;
    linelist.color.a = 1.0;

    geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    linelist.points.push_back(p1);
    p2.x = 100;
    p2.y = 0;
    p2.z = 0;
    linelist.points.push_back(p2);
    p3.x = 0;
    p3.y = 7;
    p3.z = 0;
    linelist.points.push_back(p3);
    p4.x = 100;
    p4.y = 7;
    p4.z = 0;
    linelist.points.push_back(p4);
    p5.x = 0;
    p5.y = 3.5;
    p5.z = 0;
    linelist.points.push_back(p5);
    p6.x = 100;
    p6.y = 3.5;
    p6.z = 0;
    linelist.points.push_back(p6);

    pub_marker_->publish(linelist);
    RCLCPP_INFO(this->get_logger(), "Pub line1: [(%f, %f), (%f, %f)]", p1.x, p1.y, p2.x, p2.y);
  }


