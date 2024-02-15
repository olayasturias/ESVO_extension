#ifndef EVO_TIME_SURFACE_H_
#define EVO_TIME_SURFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <mutex>
#include <memory>
#include <deque>


namespace esvo_time_surface
{
  
#define NUM_THREAD_TS 1
using EventQueue = std::deque<dvs_msgs::msg::Event>;
using std::placeholders::_1; // For subscriber callbacks

class EventQueueMat 
{
public:
  EventQueueMat(int width, int height, int queueLen)
  {
    width_ = width;
    height_ = height;
    queueLen_ = queueLen;
    eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
  }

  void insertEvent(const dvs_msgs::msg::Event& e)
  {
    if(!insideImage(e.x, e.y))
      return;
    else
    {
      EventQueue& eq = getEventQueue(e.x, e.y);
      eq.push_back(e);
      while(eq.size() > queueLen_)
        eq.pop_front();
    }
  }

  bool getMostRecentEventBeforeT(
    const size_t x,
    const size_t y,
    const ros::Time& t,
    dvs_msgs::msg::Event* ev)
  {
    if(!insideImage(x, y))
      return false;

    EventQueue& eq = getEventQueue(x, y);
    if(eq.empty())
      return false;

    for(auto it = eq.rbegin(); it != eq.rend(); ++it)
    {
      const dvs_msgs::msg::Event& e = *it;
      if(e.ts < t)
      {
        *ev = *it;
        return true;
      }
    }
    return false;
  }

  void clear()
  {
    eqMat_.clear();
  }

  bool insideImage(const size_t x, const size_t y)
  {
    return !(x < 0 || x >= width_ || y < 0 || y >= height_);
  }

  inline EventQueue& getEventQueue(const size_t x, const size_t y)
  {
    return eqMat_[x + width_ * y];
  }

  size_t width_;
  size_t height_;
  size_t queueLen_;
  std::vector<EventQueue> eqMat_; // each pixel stores the accumulated events
};

class TimeSurface : public rclcpp::Node {
public:
  TimeSurface();

private:
  // Node initialization and parameters
  void initParameters();

  // Callbacks
  void eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void syncCallback(const builtin_interfaces::msg::Time::SharedPtr msg);

  // Utility functions
  void createTimeSurfaceAtTime(const rclcpp::Time& external_sync_time);
  void thread(Job& job);
  void clearEventQueue();

  // ROS2 communication interfaces
  rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sync_topic_sub_;
  image_transport::Publisher time_surface_pub_;
  image_transport::Publisher time_surface_negative_pub_;

  // Member variables for parameters and state
  bool bCamInfoAvailable_;
  bool bUse_Sim_Time_;
  cv::Size sensor_size_;
  rclcpp::Time sync_time_;
  bool bSensorInitialized_;
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  int max_event_queue_length_;
  TimeSurfaceMode time_surface_mode_;

  // Containers and mutexes
  EventQueue events_;
  std::shared_ptr<EventQueueMat> pEventQueueMat_;
  std::mutex data_mutex_;

  enum TimeSurfaceMode { BACKWARD, FORWARD };
};

} // namespace esvo_time_surface

#endif // EVO_TIME_SURFACE_H_