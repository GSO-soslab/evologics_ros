#include "rclcpp/rclcpp.hpp"

#include "message_converter.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<MessageConverter> node = std::make_shared<MessageConverter>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}