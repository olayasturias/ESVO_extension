#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <memory>

using namespace std::chrono_literals;

class EventMessageEditor : public rclcpp::Node
{
public:
    EventMessageEditor(double frequency, const std::string& messageTopic)
    : Node("event_message_editor"), bFirstMessage_(true)
    {
        eArray_ = dvs_msgs::msg::EventArray();
        duration_threshold_ = rclcpp::Duration(1s / frequency);
        message_topic_ = messageTopic;

        publisher_ = this->create_publisher<dvs_msgs::msg::EventArray>(messageTopic, 10);
    }

    void resetBuffer(rclcpp::Time startTimeStamp)
    {
        start_time_ = startTimeStamp;
        end_time_ = start_time_ + duration_threshold_;
        eArray_.events.clear();
        eArray_.header.stamp = end_time_;
    }

    void resetArraySize(size_t width, size_t height)
    {
        eArray_.width = width;
        eArray_.height = height;
    }

    void insertEvent(dvs_msgs::msg::Event& e)
    {
        if(bFirstMessage_)
        {
            resetBuffer(e.ts);
            bFirstMessage_ = false;
        }

        if(e.ts.sec >= end_time_.seconds())
        {
            publisher_->publish(eArray_);
            resetBuffer(end_time_);
        }
        eArray_.events.push_back(e);
    }

private:
    dvs_msgs::msg::EventArray eArray_;
    rclcpp::Duration duration_threshold_;
    rclcpp::Time start_time_, end_time_;
    bool bFirstMessage_;
    std::string message_topic_;
    rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto frequency = std::stod(argv[1]); // Assuming frequency is passed as the first argument
    auto messageTopic = std::string(argv[2]); // Assuming message topic is passed as the second argument

    auto node = std::make_shared<EventMessageEditor>(frequency, messageTopic);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
