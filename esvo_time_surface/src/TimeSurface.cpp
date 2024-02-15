#include "esvo_time_surface/TimeSurface.h"
#include "esvo_time_surface/TicToc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <memory>
#include <vector>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp> // For logging and time

using namespace std::chrono_literals;

namespace esvo_time_surface
{
	TimeSurface::TimeSurface() : Node("time_surface") {
		initParameters();

		// Setup communication interfaces
		event_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>(
			"events", rclcpp::QoS(10), std::bind(&TimeSurface::eventsCallback, this, _1));
		camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			"camera_info", rclcpp::QoS(10), std::bind(&TimeSurface::cameraInfoCallback, this, _1));
		sync_topic_sub_ = this->create_subscription<builtin_interfaces::msg::Time>(
			"sync", rclcpp::QoS(10), std::bind(&TimeSurface::syncCallback, this, _1));

		// Initialize publishers using image_transport (needs to be adapted for ROS2)
		auto image_transport = std::make_shared<image_transport::ImageTransport>(this);
		time_surface_pub_ = image_transport->advertise("time_surface", 1);
		// Similar for time_surface_negative_pub_ if used
	}

	TimeSurface::~TimeSurface() {}

	void TimeSurface::initParameters() {
		this->declare_parameter<bool>("use_sim_time", true);
		this->get_parameter("use_sim_time", bUse_Sim_Time_);
		this->declare_parameter<bool>("ignore_polarity", true);
		this->get_parameter("ignore_polarity", ignore_polarity_);
		// Continue for other parameters
		}

	void TimeSurface::init(int width, int height) {
		sensor_size_ = cv::Size(width, height);
		bSensorInitialized_ = true;
		pEventQueueMat_ = std::make_shared<EventQueueMat>(width, height, max_event_queue_length_);
		RCLCPP_INFO(this->get_logger(), "Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
	}

	void TimeSurface::createTimeSurfaceAtTime(const rclcpp::Time &external_sync_time) {
		std::lock_guard<std::mutex> lock(data_mutex_);

		if (!bSensorInitialized_ || !bCamInfoAvailable_) return;

		// Create exponential-decayed Time Surface map.
		const double decay_sec = decay_ms_ / 1000.0;
		cv::Mat time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

		// Loop through all coordinates
		for (int y = 0; y < sensor_size_.height; ++y) {
			for (int x = 0; x < sensor_size_.width; ++x) {
				dvs_msgs::msg::Event most_recent_event_at_coordXY_before_T;
				if (pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_sync_time, &most_recent_event_at_coordXY_before_T)) {
					const rclcpp::Time most_recent_stamp_at_coordXY = rclcpp::Time(most_recent_event_at_coordXY_before_T.ts);
					if (most_recent_stamp_at_coordXY.seconds() > 0) { // Check if negative timestamp
						const double dt = (external_sync_time - most_recent_stamp_at_coordXY).seconds();
						double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
						double expVal = std::exp(-dt / decay_sec);
						if (!ignore_polarity_) expVal *= polarity;

						// Backward version (this paper)
						if (time_surface_mode_ == BACKWARD) {
							time_surface_map.at<double>(y, x) = expVal;
						}

						// Forward version
						if (time_surface_mode_ == FORWARD && bCamInfoAvailable_) {
							Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
							size_t u_i, v_i;
							if (uv_rect(0) >= 0 && uv_rect(1) >= 0) {
								u_i = std::floor(uv_rect(0));
								v_i = std::floor(uv_rect(1));

								if (u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height) {
									double fu = uv_rect(0) - u_i;
									double fv = uv_rect(1) - v_i;
									double fu1 = 1.0 - fu;
									double fv1 = 1.0 - fv;
									time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
									time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
									time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
									time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;
								}
							}
						} // Forward
					}
				}
			}
		}

		// Process the time surface map (polarity, median blur) as before

		// Publish event image using image_transport (ROS2 style)
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = time_surface_map;
		cv_image.header.stamp = external_sync_time;
		auto message = cv_image.toImageMsg();

		if ((time_surface_mode_ == FORWARD || (time_surface_mode_ == BACKWARD && bCamInfoAvailable_)) && time_surface_pub_.getNumSubscribers() > 0) {
			time_surface_pub_.publish(message);
		}
	}
	
	void TimeSurface::createTimeSurfaceAtTime_hyperthread(const rclcpp::Time &external_sync_time) {
		std::lock_guard<std::mutex> lock(data_mutex_);

		if (!bSensorInitialized_ || !bCamInfoAvailable_)
			return;

		// Create exponential-decayed Time Surface map.
		const double decay_sec = decay_ms_ / 1000.0;
		cv::Mat time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

		// Distribute jobs
		std::vector<Job> jobs(NUM_THREAD_TS);
		size_t num_col_per_thread = sensor_size_.width / NUM_THREAD_TS;
		size_t res_col = sensor_size_.width % NUM_THREAD_TS;
		for (size_t i = 0; i < NUM_THREAD_TS; i++) {
			jobs[i].i_thread_ = i;
			jobs[i].pEventQueueMat_ = pEventQueueMat_.get();
			jobs[i].pTimeSurface_ = &time_surface_map;
			jobs[i].start_col_ = num_col_per_thread * i;
			jobs[i].end_col_ = (i == NUM_THREAD_TS - 1) ? jobs[i].start_col_ + num_col_per_thread - 1 + res_col : jobs[i].start_col_ + num_col_per_thread - 1;
			jobs[i].start_row_ = 0;
			jobs[i].end_row_ = sensor_size_.height - 1;
			jobs[i].external_sync_time_ = external_sync_time;
			jobs[i].decay_sec_ = decay_sec;
		}

		// Hyperthread processing
		std::vector<std::thread> threads;
		threads.reserve(NUM_THREAD_TS);
		for (auto &job : jobs)
			threads.emplace_back([this, &job](){ this->thread(job); });
		for (auto &thread : threads)
			if (thread.joinable())
				thread.join();

		// Post-processing and publishing (similar to createTimeSurfaceAtTime)
		processTimeSurfaceMap(time_surface_map, external_sync_time);
	}

	void TimeSurface::processTimeSurfaceMap(cv::Mat &time_surface_map, const rclcpp::Time &external_sync_time) {
		// Polarity adjustment and median blur (same as in createTimeSurfaceAtTime)

		// Publish event image
		cv_bridge::CvImage cv_image;
		cv_image.encoding = sensor_msgs::image_encodings::MONO8;
		cv_image.image = time_surface_map;

		auto message = cv_image.toImageMsg();
		message->header.stamp = external_sync_time;

		if (time_surface_mode_ == FORWARD && time_surface_pub_.getNumSubscribers() > 0) {
			time_surface_pub_.publish(*message);
		}

		if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0) {
			// If image remapping is needed before publishing
			cv_bridge::CvImage cv_image2;
			cv_image2.encoding = cv_image.encoding;
			cv_image2.header.stamp = message->header.stamp;
			cv::remap(cv_image.image, cv_image2.image, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
			time_surface_pub_.publish(cv_image2.toImageMsg());
		}
	}

	void TimeSurface::thread(Job &job) {
		EventQueueMat &eqMat = *job.pEventQueueMat_;
		cv::Mat &time_surface_map = *job.pTimeSurface_;
		size_t start_col = job.start_col_;
		size_t end_col = job.end_col_;
		size_t start_row = job.start_row_;
		size_t end_row = job.end_row_;
		size_t i_thread = job.i_thread_;

		for (size_t y = start_row; y <= end_row; y++) {
			for (size_t x = start_col; x <= end_col; x++) {
				dvs_msgs::msg::Event most_recent_event_at_coordXY_before_T;
				if (pEventQueueMat_->getMostRecentEventBeforeT(x, y, job.external_sync_time_, &most_recent_event_at_coordXY_before_T)) {
					const rclcpp::Time most_recent_stamp_at_coordXY = rclcpp::Time(most_recent_event_at_coordXY_before_T.ts.sec, most_recent_event_at_coordXY_before_T.ts.nanosec);
					if (most_recent_stamp_at_coordXY.seconds() > 0) { // Check if negative timestamp
						const double dt = (job.external_sync_time_ - most_recent_stamp_at_coordXY).seconds();
						double polarity = most_recent_event_at_coordXY_before_T.polarity ? 1.0 : -1.0;
						double expVal = std::exp(-dt / job.decay_sec_);
						if (!ignore_polarity_) expVal *= polarity;

						// Backward version (this paper)
						if (time_surface_mode_ == BACKWARD) {
							time_surface_map.at<double>(y, x) = expVal;
						}

						// Forward version
						if (time_surface_mode_ == FORWARD && bCamInfoAvailable_) {
							Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
							size_t u_i, v_i;
							if (uv_rect(0) >= 0 && uv_rect(1) >= 0) {
								u_i = std::floor(uv_rect(0));
								v_i = std::floor(uv_rect(1));

								// Interpolation
								if (u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height) {
									double fu = uv_rect(0) - u_i;
									double fv = uv_rect(1) - v_i;
									double fu1 = 1.0 - fu;
									double fv1 = 1.0 - fv;
									time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
									time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
									time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
									time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;
								}
							}
						} // Forward
					}
				} // A most recent event is available
			}
		}
	}


	void TimeSurface::syncCallback(const builtin_interfaces::msg::Time::SharedPtr msg) {
		// Adjusting for ROS2 time handling
		if (bUse_Sim_Time_) {
			sync_time_ = this->get_clock()->now();
		} else {
			// Converting builtin_interfaces::msg::Time to rclcpp::Time
			sync_time_ = rclcpp::Time(msg->data.sec, msg->data.nanosec);
		}

		// Assuming TicToc has been adapted for ROS2 or is a custom implementation that's compatible
		TicToc tt;
		tt.tic();

		// Determine which method to use for creating the time surface based on the number of threads
		if (NUM_THREAD_TS == 1) {
			createTimeSurfaceAtTime(sync_time_);
		} else if (NUM_THREAD_TS > 1) {
			createTimeSurfaceAtTime_hyperthread(sync_time_);
		}

		// Logging the time taken to create the time surface map
		RCLCPP_INFO(this->get_logger(), "Time Surface map's creation takes: %f ms.", tt.toc());
	}

	void TimeSurface::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (bCamInfoAvailable_) {
        return;
    }

    cv::Size sensor_size(msg->width, msg->height);
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            camera_matrix_.at<double>(cv::Point(i, j)) = msg->k[i + j * 3];
        }
    }

    distortion_model_ = msg->distortion_model;
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); i++) {
        dist_coeffs_.at<double>(i) = msg->d[i];
    }

    rectification_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rectification_matrix_.at<double>(cv::Point(i, j)) = msg->r[i + j * 3];
        }
    }

    projection_matrix_ = cv::Mat(3, 4, CV_64F);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            projection_matrix_.at<double>(cv::Point(i, j)) = msg->p[i + j * 4];
        }
    }

    if (distortion_model_ == "equidistant") {
        cv::fisheye::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                             rectification_matrix_, projection_matrix_,
                                             sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
        bCamInfoAvailable_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
    } else if (distortion_model_ == "plumb_bob") {
        cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                    rectification_matrix_, projection_matrix_,
                                    sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
        bCamInfoAvailable_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
    } else {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Distortion model %s is not supported.", distortion_model_.c_str());
        bCamInfoAvailable_ = false;
        return;
    }

    /* Pre-compute the undistorted-rectified look-up table */
    precomputed_rectified_points_ = Eigen::Matrix2Xd(2, sensor_size.height * sensor_size.width);
    // Raw coordinates
    cv::Mat_<cv::Point2f> RawCoordinates(1, sensor_size.height * sensor_size.width);
    for (int y = 0; y < sensor_size.height; y++) {
        for (int x = 0; x < sensor_size.width; x++) {
            int index = y * sensor_size.width + x;
            RawCoordinates(index) = cv::Point2f((float)x, (float)y);
        }
    }

    // Undistorted-rectified coordinates
    cv::Mat_<cv::Point2f> RectCoordinates(1, sensor_size.height * sensor_size.width);
    if (distortion_model_ == "plumb_bob") {
        cv::undistortPoints(RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
                            rectification_matrix_, projection_matrix_);
        RCLCPP_INFO(this->get_logger(), "Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
    } else if (distortion_model_ == "equidistant") {
        cv::fisheye::undistortPoints(
            RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
            rectification_matrix_, projection_matrix_);
        RCLCPP_INFO(this->get_logger(), "Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown distortion model is provided.");
        exit(-1); // Exiting might not be ideal, consider alternative error handling
    }

    // Load the look-up table
    for (size_t i = 0; i < sensor_size.height * sensor_size.width; i++) {
        precomputed_rectified_points_.col(i) = Eigen::Matrix<double, 2, 1>(
            RectCoordinates(i).x, RectCoordinates(i).y);
    }

    RCLCPP_INFO(this->get_logger(), "Undistorted-Rectified Look-Up Table has been computed.");
}

	void TimeSurface::eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!bSensorInitialized_) {
        init(msg->width, msg->height);
    }

    for (const dvs_msgs::msg::Event &e : msg->events) {
        events_.push_back(e);
        int i = events_.size() - 2;
        while (i >= 0 && events_[i].ts > e.ts) {
            events_[i + 1] = events_[i];
            i--;
        }
        events_[i + 1] = e;

        const dvs_msgs::msg::Event &last_event = events_.back();
        pEventQueueMat_->insertEvent(last_event);
    }

    clearEventQueue();
}

	void TimeSurface::clearEventQueue()
	{
		constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
		if (events_.size() > MAX_EVENT_QUEUE_LENGTH)
		{
			size_t remove_events = events_.size() - MAX_EVENT_QUEUE_LENGTH;
			events_.erase(events_.begin(), events_.begin() + remove_events);
		}
	}


} // namespace esvo_time_surface