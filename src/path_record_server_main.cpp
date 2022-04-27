//
// Created by zxl on 2022/4/25.
//

#include "rclcpp/rclcpp.hpp"
#include "path_record_server.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("path_record_server");
  auto service_node = std::make_shared<PathRecordServer>();
  rclcpp::spin(service_node);
  rclcpp::shutdown();
  service_node = nullptr;
  return 0;
}