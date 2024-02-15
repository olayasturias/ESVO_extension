#ifndef ESVO_CORE_PARAMS_HELPER_H
#define ESVO_CORE_PARAMS_HELPER_H

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace esvo_core {
namespace tools {

template<typename T>
T param(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &defaultValue) {
  T value = defaultValue;
  if (node->get_parameter(name, value)) {
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(value).c_str());
  } else {
    RCLCPP_WARN(node->get_logger(), "Cannot find value for parameter: %s, assigning default: %s", name.c_str(), std::to_string(defaultValue).c_str());
  }
  return value;
}

} // namespace tools
} // namespace esvo_core

#endif // ESVO_CORE_PARAMS_HELPER_H
