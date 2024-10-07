
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "modem.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Modem>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}