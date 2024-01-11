#include "adma_publisher.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "adma node starting");

  rclcpp::spin(std::make_shared<AdmaUdpDataPackage>());
  rclcpp::shutdown();
  return 0;
}