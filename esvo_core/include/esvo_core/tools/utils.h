#ifndef ESVO_CORE_TOOLS_UTILS_H
#define ESVO_CORE_TOOLS_UTILS_H

#include <Eigen/Eigen>
#include <iostream>

#include <cv_bridge/cv_bridge.h>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <esvo_core/container/SmartGrid.h>
#include <esvo_core/container/DepthPoint.h>
#include <esvo_core/tools/TicToc.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.hpp>

#include <map>
#include <deque>
#include <algorithm>
#include <cmath>

using namespace std;

namespace esvo_core
{
  namespace tools
  {
    // TUNE this according to your platform's computational capability.
#define NUM_THREAD_TRACKING 2
#define NUM_THREAD_MAPPING 4

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    using RefPointCloudMap = std::map<ros::Time, PointCloud::SharedPtr>;

    using Transformation = Eigen::Transform<double, 3, Eigen::Affine>;

    inline static std::vector<dvs_msgs::msg::Event *>::iterator EventVecPtr_lower_bound(
        std::vector<dvs_msgs::msg::Event *> &vEventPtr, rclcpp::Time &t)
    {
      return std::lower_bound(vEventPtr.begin(), vEventPtr.end(), t,
                              [](const dvs_msgs::msg::Event *e, const rclcpp::Time &t) { return e->ts < t; });
    }

    using EventQueue = std::deque<dvs_msgs::msg::Event>;
    inline static EventQueue::iterator EventBuffer_lower_bound(
        EventQueue &eb, rclcpp::Time &t)
    {
      return std::lower_bound(eb.begin(), eb.end(), t,
                              [](const dvs_msgs::msg::Event &e, const rclcpp::Time &t) { return e.ts < t; });
    }

    inline static EventQueue::iterator EventBuffer_upper_bound(
        EventQueue &eb, rclcpp::Time &t)
    {
      return std::upper_bound(eb.begin(), eb.end(), t,
                              [](const rclcpp::Time &t, const dvs_msgs::msg::Event &e) { return t < e.ts; });
    }

    using StampTransformationMap = std::map<rclcpp::Time, tools::Transformation>;
    inline static StampTransformationMap::iterator StampTransformationMap_lower_bound(
        StampTransformationMap &stm, rclcpp::Time &t)
    {
      return std::lower_bound(stm.begin(), stm.end(), t,
                              [](const std::pair<rclcpp::Time, tools::Transformation> &st, const rclcpp::Time &t) { return st.first < t; });
    }

    /******************* Used by Block Match ********************/
    static inline void meanStdDev(
        Eigen::MatrixXd &patch,
        double &mean, double &sigma)
    {
      size_t numElement = patch.rows() * patch.cols();
      mean = patch.array().sum() / numElement;
      Eigen::MatrixXd sub = patch.array() - mean;
      sigma = sqrt((sub.array() * sub.array()).sum() / numElement) + 1e-6;
    }

    static inline void normalizePatch(
        Eigen::MatrixXd &patch_src,
        Eigen::MatrixXd &patch_dst)
    {
      double mean = 0;
      double sigma = 0;
      meanStdDev(patch_src, mean, sigma);
      patch_dst = (patch_src.array() - mean) / sigma;
    }

    // recursively create a directory
    static inline void _mkdir(const char *dir)
    {
      char tmp[256];
      char *p = NULL;
      size_t len;

      snprintf(tmp, sizeof(tmp), "%s", dir);
      len = strlen(tmp);
      if (tmp[len - 1] == '/')
        tmp[len - 1] = 0;
      for (p = tmp + 1; *p; p++)
        if (*p == '/')
        {
          *p = 0;
          mkdir(tmp, S_IRWXU);
          *p = '/';
        }
      mkdir(tmp, S_IRWXU);
    }

  } // namespace tools
} // namespace esvo_core

#endif //ESVO_CORE_TOOLS_UTILS_H
