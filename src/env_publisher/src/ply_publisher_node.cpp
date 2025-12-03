#include "../include/ply_publisher.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PLYPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}