#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "dvs_msgs/msg/event_array.hpp"
#include <chrono>

int newEvents;

template <typename T>
T param(const rclcpp::Node::SharedPtr &nh, const std::string &name, const T &defaultValue)
{
    if (nh->has_parameter(name))
    {
        T v;
        nh->get_parameter(name, v);
        RCLCPP_INFO_STREAM(nh->get_logger(), "Found parameter: " << name << ", value: " << v);
        return v;
    }
    RCLCPP_WARN_STREAM(nh->get_logger(), "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}

void eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{
    newEvents += msg->events.size();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("esvo_time_surface_global_timer");
    auto nh_private = std::make_shared<rclcpp::Node>("~");

    const int MINIMUM_EVENTS = param<int>(nh_private, "minimum_events", static_cast<int>(1000));
    const int FREQUENCY_TIMER = param<int>(nh_private, "frequency_timer", static_cast<int>(100));
    const int MINIMUM_FREQUENCY_TIMER = param<int>(nh_private, "minimum_frequency_timer", static_cast<int>(40));
    auto event_sub = nh->create_subscription<dvs_msgs::msg::EventArray>("events", 0, eventsCallback);
    auto global_time_pub = nh->create_publisher<builtin_interfaces::msg::Time>("/sync", 1);

    newEvents = 0;
    auto timestampLast = std::chrono::steady_clock::now();
    rclcpp::Rate rate(FREQUENCY_TIMER);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(nh);
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timestampLast).count();
#ifdef TIME_SURFACE_SYNC
        if (elapsedTime >= 1000 / FREQUENCY_TIMER)
        {
            auto msg = std::make_shared<builtin_interfaces::msg::Time>();
            msg->sec = nh->now().seconds();
            msg->nanosec = nh->now().nanoseconds();
            global_time_pub->publish(msg);
            timestampLast = currentTime;
            newEvents = 0;
        }
#else
        if ((elapsedTime >= 1000 / FREQUENCY_TIMER && newEvents >= MINIMUM_EVENTS) ||
            (elapsedTime >= 1000 / MINIMUM_FREQUENCY_TIMER))
        {
            auto msg = std::make_shared<builtin_interfaces::msg::Time>();
            msg->sec = nh->now().seconds();
            msg->nanosec = nh->now().nanoseconds();
            global_time_pub->publish(*msg);
            timestampLast = currentTime;
            newEvents = 0;
        }
#endif
        rate.sleep();
    }
    return 0;
}
