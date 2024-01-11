#include "rclcpp/rclcpp.hpp"
#include "shuttle_to_matlab_tracking_bridge.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CShuttleToMatlabTrackingBridge>());
  rclcpp::shutdown();

  return 0;
}
