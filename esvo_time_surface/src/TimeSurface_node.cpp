#include "esvo_time_surface/TimeSurface.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto ts_node = std::make_shared<esvo_time_surface::TimeSurface>();

  rclcpp::spin(ts_node);

  rclcpp::shutdown();

  return 0;
}
