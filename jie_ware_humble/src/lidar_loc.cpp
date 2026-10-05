#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class LidarLocNode : public rclcpp::Node
{
public:
  LidarLocNode()
  : Node("lidar_loc"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    laser_frame_ = this->declare_parameter<std::string>("laser_frame", "laser");
    laser_topic_ = this->declare_parameter<std::string>("laser_topic", "scan");
    initial_pose_topic_ =
      this->declare_parameter<std::string>("initial_pose_topic", "initialpose");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "map");
    local_clear_service_ = this->declare_parameter<std::string>(
      "local_clear_service", "/local_costmap/clear_entirely_local_costmap");
    global_clear_service_ = this->declare_parameter<std::string>(
      "global_clear_service", "/global_costmap/clear_entirely_global_costmap");

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      map_qos,
      std::bind(&LidarLocNode::mapCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarLocNode::scanCallback, this, std::placeholders::_1));

    initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_,
      10,
      std::bind(&LidarLocNode::initialPoseCallback, this, std::placeholders::_1));

    local_clear_client_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(local_clear_service_);
    global_clear_client_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(global_clear_service_);

    pose_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&LidarLocNode::publishPoseTf, this));
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = *msg;
    cropMap();
    processMap();
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (map_msg_.info.resolution <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Map is not ready, ignore initial pose");
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    lidar_x_ =
      static_cast<float>(
      (msg->pose.pose.position.x - map_msg_.info.origin.position.x) /
      map_msg_.info.resolution - static_cast<double>(map_roi_info_.x_offset));
    lidar_y_ =
      static_cast<float>(
      (msg->pose.pose.position.y - map_msg_.info.origin.position.y) /
      map_msg_.info.resolution - static_cast<double>(map_roi_info_.y_offset));
    lidar_yaw_ = static_cast<float>(-yaw);

    clear_countdown_ = 30;
    checkConverged(0.0F, 0.0F, 0.0F);
  }

  void cropMap()
  {
    const auto & info = map_msg_.info;
    if (info.width == 0U || info.height == 0U) {
      return;
    }

    int x_max = static_cast<int>(info.width / 2U);
    int x_min = x_max;
    int y_max = static_cast<int>(info.height / 2U);
    int y_min = y_max;
    bool first_point = true;

    cv::Mat map_raw(
      static_cast<int>(info.height), static_cast<int>(info.width), CV_8UC1,
      cv::Scalar(128));

    for (unsigned int y = 0; y < info.height; ++y) {
      for (unsigned int x = 0; x < info.width; ++x) {
        const auto index = y * info.width + x;
        map_raw.at<uchar>(static_cast<int>(y), static_cast<int>(x)) =
          static_cast<uchar>(map_msg_.data[index]);

        if (map_msg_.data[index] == 100) {
          if (first_point) {
            x_max = x_min = static_cast<int>(x);
            y_max = y_min = static_cast<int>(y);
            first_point = false;
            continue;
          }

          x_min = std::min(x_min, static_cast<int>(x));
          x_max = std::max(x_max, static_cast<int>(x));
          y_min = std::min(y_min, static_cast<int>(y));
          y_max = std::max(y_max, static_cast<int>(y));
        }
      }
    }

    if (first_point) {
      x_min = 0;
      y_min = 0;
      x_max = static_cast<int>(info.width) - 1;
      y_max = static_cast<int>(info.height) - 1;
    }

    const int center_x = (x_min + x_max) / 2;
    const int center_y = (y_min + y_max) / 2;
    const int new_half_width = std::abs(x_max - x_min) / 2 + 50;
    const int new_half_height = std::abs(y_max - y_min) / 2 + 50;

    int origin_x = center_x - new_half_width;
    int origin_y = center_y - new_half_height;
    int width = new_half_width * 2;
    int height = new_half_height * 2;

    if (origin_x < 0) {
      origin_x = 0;
    }
    if (origin_y < 0) {
      origin_y = 0;
    }
    if (origin_x + width > static_cast<int>(info.width)) {
      width = static_cast<int>(info.width) - origin_x;
    }
    if (origin_y + height > static_cast<int>(info.height)) {
      height = static_cast<int>(info.height) - origin_y;
    }

    const cv::Rect roi(origin_x, origin_y, width, height);
    map_cropped_ = map_raw(roi).clone();

    map_roi_info_.x_offset = origin_x;
    map_roi_info_.y_offset = origin_y;
    map_roi_info_.width = width;
    map_roi_info_.height = height;

    auto init_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    init_pose->pose.pose.orientation.w = 1.0;
    initialPoseCallback(init_pose);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (map_msg_.info.resolution <= 0.0 || map_cropped_.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        base_frame_, laser_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Laser transform failed: %s", ex.what());
      return;
    }

    scan_points_.clear();
    scan_points_.reserve(msg->ranges.size());

    tf2::Quaternion q_lidar;
    tf2::fromMsg(transform_stamped.transform.rotation, q_lidar);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);

    const bool lidar_is_inverted =
      std::abs(std::abs(roll) - M_PI) < 0.1 &&
      !(std::abs(std::abs(pitch) - M_PI) < 0.1);

    double angle = msg->angle_min;
    for (const auto range : msg->ranges) {
      if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
        geometry_msgs::msg::PointStamped point_laser;
        point_laser.header.frame_id = laser_frame_;
        point_laser.header.stamp = msg->header.stamp;
        point_laser.point.x = range * std::cos(angle);
        point_laser.point.y = -range * std::sin(angle);
        point_laser.point.z = 0.0;

        geometry_msgs::msg::PointStamped point_base;
        tf2::doTransform(point_laser, point_base, transform_stamped);

        float x = static_cast<float>(point_base.point.x / map_msg_.info.resolution);
        float y = static_cast<float>(point_base.point.y / map_msg_.info.resolution);
        if (lidar_is_inverted) {
          x = -x;
          y = -y;
        }

        scan_points_.emplace_back(x, y);
      }
      angle += msg->angle_increment;
    }

    if (scan_count_ == 0) {
      ++scan_count_;
    }

    constexpr int max_iterations = 200;
    for (int iter = 0; iter < max_iterations; ++iter) {
      float best_dx = 0.0F;
      float best_dy = 0.0F;
      float best_dyaw = 0.0F;
      int max_sum = 0;

      std::vector<cv::Point2f> base_points;
      std::vector<cv::Point2f> clockwise_points;
      std::vector<cv::Point2f> counter_points;
      base_points.reserve(scan_points_.size());
      clockwise_points.reserve(scan_points_.size());
      counter_points.reserve(scan_points_.size());

      for (const auto & point : scan_points_) {
        transformScanPoint(point, lidar_yaw_, base_points);
        transformScanPoint(point, lidar_yaw_ + deg_to_rad_, clockwise_points);
        transformScanPoint(point, lidar_yaw_ - deg_to_rad_, counter_points);
      }

      const std::vector<cv::Point2f> offsets = {
        {0.0F, 0.0F}, {1.0F, 0.0F}, {-1.0F, 0.0F}, {0.0F, 1.0F}, {0.0F, -1.0F}};
      const std::vector<std::vector<cv::Point2f> *> point_sets = {
        &base_points, &clockwise_points, &counter_points};
      const std::vector<float> yaw_offsets = {0.0F, deg_to_rad_, -deg_to_rad_};

      for (std::size_t i = 0; i < offsets.size(); ++i) {
        for (std::size_t j = 0; j < point_sets.size(); ++j) {
          int sum = 0;
          for (const auto & point : *point_sets[j]) {
            const int px = static_cast<int>(std::lround(point.x + offsets[i].x));
            const int py = static_cast<int>(std::lround(point.y + offsets[i].y));
            if (px >= 0 && px < map_temp_.cols && py >= 0 && py < map_temp_.rows) {
              sum += static_cast<int>(map_temp_.at<uchar>(py, px));
            }
          }

          if (sum > max_sum) {
            max_sum = sum;
            best_dx = offsets[i].x;
            best_dy = offsets[i].y;
            best_dyaw = yaw_offsets[j];
          }
        }
      }

      lidar_x_ += best_dx;
      lidar_y_ += best_dy;
      lidar_yaw_ += best_dyaw;

      if (checkConverged(lidar_x_, lidar_y_, lidar_yaw_)) {
        break;
      }
    }

    if (clear_countdown_ > -1) {
      --clear_countdown_;
    }
    if (clear_countdown_ == 0) {
      clear_countdown_ = -1;
      clearCostmaps();
    }
  }

  void transformScanPoint(
    const cv::Point2f & point, float yaw, std::vector<cv::Point2f> & output) const
  {
    const float rotated_x = point.x * std::cos(yaw) - point.y * std::sin(yaw);
    const float rotated_y = point.x * std::sin(yaw) + point.y * std::cos(yaw);
    output.emplace_back(rotated_x + lidar_x_, lidar_y_ - rotated_y);
  }

  bool checkConverged(float x, float y, float yaw)
  {
    if (x == 0.0F && y == 0.0F && yaw == 0.0F) {
      data_queue_.clear();
      return true;
    }

    data_queue_.emplace_back(x, y, yaw);
    if (data_queue_.size() > max_queue_size_) {
      data_queue_.pop_front();
    }

    if (data_queue_.size() == max_queue_size_) {
      const auto & first = data_queue_.front();
      const auto & last = data_queue_.back();

      const float dx = std::abs(std::get<0>(last) - std::get<0>(first));
      const float dy = std::abs(std::get<1>(last) - std::get<1>(first));
      const float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

      if (dx < 5.0F && dy < 5.0F && dyaw < 5.0F * deg_to_rad_) {
        data_queue_.clear();
        return true;
      }
    }
    return false;
  }

  cv::Mat createGradientMask(int size) const
  {
    cv::Mat mask(size, size, CV_8UC1);
    const int center = size / 2;

    for (int y = 0; y < size; ++y) {
      for (int x = 0; x < size; ++x) {
        const double distance = std::hypot(x - center, y - center);
        const int value = cv::saturate_cast<uchar>(
          255.0 * std::max(0.0, 1.0 - distance / static_cast<double>(center)));
        mask.at<uchar>(y, x) = static_cast<uchar>(value);
      }
    }

    return mask;
  }

  void processMap()
  {
    if (map_cropped_.empty()) {
      return;
    }

    map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
    const cv::Mat gradient_mask = createGradientMask(101);

    for (int y = 0; y < map_cropped_.rows; ++y) {
      for (int x = 0; x < map_cropped_.cols; ++x) {
        if (map_cropped_.at<uchar>(y, x) != 100) {
          continue;
        }

        const int left = std::max(0, x - 50);
        const int top = std::max(0, y - 50);
        const int right = std::min(map_cropped_.cols - 1, x + 50);
        const int bottom = std::min(map_cropped_.rows - 1, y + 50);

        const cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
        cv::Mat region = map_temp_(roi);

        const int mask_left = 50 - (x - left);
        const int mask_top = 50 - (y - top);
        const cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
        const cv::Mat mask = gradient_mask(mask_roi);

        cv::max(region, mask, region);
      }
    }
  }

  void publishPoseTf()
  {
    if (scan_count_ == 0 || map_cropped_.empty() || map_msg_.info.resolution <= 0.0) {
      return;
    }

    const double full_map_pixel_x = lidar_x_ + static_cast<double>(map_roi_info_.x_offset);
    const double full_map_pixel_y = lidar_y_ + static_cast<double>(map_roi_info_.y_offset);

    const double x_in_map_frame =
      full_map_pixel_x * map_msg_.info.resolution + map_msg_.info.origin.position.x;
    const double y_in_map_frame =
      full_map_pixel_y * map_msg_.info.resolution + map_msg_.info.origin.position.y;
    const double yaw_in_map_frame = -lidar_yaw_;

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_in_map_frame);
    map_to_base.setRotation(q);

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    try {
      odom_to_base_msg = tf_buffer_.lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Odom transform failed: %s", ex.what());
      return;
    }

    tf2::Transform odom_to_base_tf;
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf);
    const tf2::Transform map_to_odom = map_to_base * odom_to_base_tf.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = this->now();
    map_to_odom_msg.header.frame_id =
      map_msg_.header.frame_id.empty() ? "map" : map_msg_.header.frame_id;
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_.sendTransform(map_to_odom_msg);
  }

  void clearCostmaps()
  {
    sendClearRequest(local_clear_client_, "local_costmap");
    sendClearRequest(global_clear_client_, "global_costmap");
  }

  void sendClearRequest(
    const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & client,
    const std::string & name)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "%s clear service is not ready", name.c_str());
      return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto future = client->async_send_request(
      request,
      [this, name](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture result) {
        if (result.valid()) {
          RCLCPP_INFO(this->get_logger(), "%s cleared after relocalization", name.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "%s clear request failed", name.c_str());
        }
      });

    (void)future;
  }

  std::string base_frame_;
  std::string odom_frame_;
  std::string laser_frame_;
  std::string laser_topic_;
  std::string initial_pose_topic_;
  std::string map_topic_;
  std::string local_clear_service_;
  std::string global_clear_service_;

  nav_msgs::msg::OccupancyGrid map_msg_;
  sensor_msgs::msg::RegionOfInterest map_roi_info_;
  cv::Mat map_cropped_;
  cv::Mat map_temp_;
  std::vector<cv::Point2f> scan_points_;
  std::deque<std::tuple<float, float, float>> data_queue_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  float lidar_x_{250.0F};
  float lidar_y_{250.0F};
  float lidar_yaw_{0.0F};
  const float deg_to_rad_{static_cast<float>(M_PI / 180.0)};
  int clear_countdown_{-1};
  int scan_count_{0};
  const std::size_t max_queue_size_{10};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLocNode>());
  rclcpp::shutdown();
  return 0;
}
