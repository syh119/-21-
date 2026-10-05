#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

double wrapAngleToScan(double angle, double angle_min, double angle_max) {
  const double full_span = 2.0 * M_PI;
  const double scan_span = angle_max - angle_min;

  if (scan_span >= full_span - 1e-3) {
    while (angle < angle_min) {
      angle += full_span;
    }
    while (angle >= angle_min + full_span) {
      angle -= full_span;
    }
  }
  return angle;
}

tf2::Quaternion yawOnlyQuaternion(const geometry_msgs::msg::Quaternion &msg) {
  tf2::Quaternion quat;
  tf2::fromMsg(msg, quat);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, yaw);
  yaw_quat.normalize();
  return yaw_quat;
}

tf2::Quaternion yawOnlyQuaternion(const tf2::Quaternion &quat) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, yaw);
  yaw_quat.normalize();
  return yaw_quat;
}

tf2::Quaternion yawQuaternionFromAngle(double yaw) {
  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, yaw);
  yaw_quat.normalize();
  return yaw_quat;
}

double yawAngleOfQuaternion(const tf2::Quaternion &quat) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

bool initializeQuaternionFromAccel(
    double ax, double ay, double az, tf2::Quaternion &quat_out) {
  const double accel_norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (!std::isfinite(accel_norm) || accel_norm < 1e-6) {
    return false;
  }

  const double roll = std::atan2(ay, az);
  const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
  quat_out.setRPY(roll, pitch, 0.0);
  quat_out.normalize();
  return true;
}

void madgwickUpdateImu(tf2::Quaternion &quat, double gx, double gy, double gz,
                       double ax, double ay, double az, double dt_sec,
                       double beta) {
  if (dt_sec <= 0.0) {
    return;
  }

  double q0 = quat.w();
  double q1 = quat.x();
  double q2 = quat.y();
  double q3 = quat.z();

  double q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
  double q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
  double q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
  double q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

  const double accel_norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (std::isfinite(accel_norm) && accel_norm > 1e-6) {
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;

    const double _2q0 = 2.0 * q0;
    const double _2q1 = 2.0 * q1;
    const double _2q2 = 2.0 * q2;
    const double _2q3 = 2.0 * q3;
    const double _4q0 = 4.0 * q0;
    const double _4q1 = 4.0 * q1;
    const double _4q2 = 4.0 * q2;
    const double _8q1 = 8.0 * q1;
    const double _8q2 = 8.0 * q2;
    const double q0q0 = q0 * q0;
    const double q1q1 = q1 * q1;
    const double q2q2 = q2 * q2;
    const double q3q3 = q3 * q3;

    double s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    double s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 +
                _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    double s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
                _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    double s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

    const double step_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (std::isfinite(step_norm) && step_norm > 1e-9) {
      s0 /= step_norm;
      s1 /= step_norm;
      s2 /= step_norm;
      s3 /= step_norm;

      q_dot0 -= beta * s0;
      q_dot1 -= beta * s1;
      q_dot2 -= beta * s2;
      q_dot3 -= beta * s3;
    }
  }

  q0 += q_dot0 * dt_sec;
  q1 += q_dot1 * dt_sec;
  q2 += q_dot2 * dt_sec;
  q3 += q_dot3 * dt_sec;

  quat = tf2::Quaternion(q1, q2, q3, q0);
  quat.normalize();
}

}  // namespace

class ScanUndistortionNode : public rclcpp::Node {
public:
  ScanUndistortionNode()
      : Node("scan_undistortion_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    output_topic_ =
        this->declare_parameter<std::string>("output_topic", "/scan_deskewed");
    output_cloud_topic_ = this->declare_parameter<std::string>(
        "output_cloud_topic", "/scan_deskewed_cloud");
    world_cloud_topic_ = this->declare_parameter<std::string>(
        "world_cloud_topic", "/scan_deskewed_world_cloud");
    world_cloud_frame_ =
        this->declare_parameter<std::string>("world_cloud_frame", "odom");
    raw_cloud_topic_ = this->declare_parameter<std::string>(
        "raw_cloud_topic", "/scan_origin_cloud");

    default_scan_rate_hz_ =
        this->declare_parameter<double>("default_scan_rate_hz", 10.0);
    lidar_msg_delay_ms_ =
        this->declare_parameter<double>("lidar_msg_delay_ms", 10.0);
    imu_buffer_sec_ = this->declare_parameter<double>("imu_buffer_sec", 3.0);
    odom_buffer_sec_ = this->declare_parameter<double>("odom_buffer_sec", 3.0);
    range_epsilon_ = this->declare_parameter<double>("range_epsilon", 1e-4);
    laser_offset_x_ = this->declare_parameter<double>("laser_offset_x", 0.0);
    laser_offset_y_ = this->declare_parameter<double>("laser_offset_y", 0.0);
    scan_direction_clockwise_ =
        this->declare_parameter<bool>("scan_direction_clockwise", false);
    publish_raw_cloud_ =
        this->declare_parameter<bool>("publish_raw_cloud", true);
    publish_world_cloud_ =
        this->declare_parameter<bool>("publish_world_cloud", true);
    invert_imu_yaw_ =
        this->declare_parameter<bool>("invert_imu_yaw", false);
    use_odom_translation_ =
        this->declare_parameter<bool>("use_odom_translation", true);
    estimate_orientation_from_raw_imu_ =
        this->declare_parameter<bool>("estimate_orientation_from_raw_imu", true);
    madgwick_beta_ =
        this->declare_parameter<double>("madgwick_beta", 0.05);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        output_topic_, rclcpp::SensorDataQoS());
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, rclcpp::SensorDataQoS());
    if (publish_world_cloud_) {
      world_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          world_cloud_topic_, rclcpp::SensorDataQoS());
    }
    if (publish_raw_cloud_) {
      raw_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          raw_cloud_topic_, rclcpp::SensorDataQoS());
    }

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanUndistortionNode::imuCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanUndistortionNode::odomCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanUndistortionNode::scanCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "scan_undistortion_2d ready. scan=%s imu=%s odom=%s output=%s cloud=%s"
                " world_cloud=%s->%s invert_imu_yaw=%s use_odom_translation=%s"
                " laser_offset=(%.3f, %.3f)"
                " estimate_orientation_from_raw_imu=%s beta=%.3f",
                scan_topic_.c_str(), imu_topic_.c_str(), odom_topic_.c_str(),
                output_topic_.c_str(), output_cloud_topic_.c_str(),
                publish_world_cloud_ ? world_cloud_topic_.c_str() : "disabled",
                publish_world_cloud_ ? world_cloud_frame_.c_str() : "disabled",
                invert_imu_yaw_ ? "true" : "false",
                use_odom_translation_ ? "true" : "false",
                laser_offset_x_, laser_offset_y_,
                estimate_orientation_from_raw_imu_ ? "true" : "false",
                madgwick_beta_);
  }

private:
  struct Translation2D {
    double x{0.0};
    double y{0.0};
  };

  struct ImuSample {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    tf2::Quaternion orientation{0.0, 0.0, 0.0, 1.0};
    bool has_orientation{false};
    double yaw_rate{0.0};
  };

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuSample sample;
    sample.stamp = rclcpp::Time(msg->header.stamp);
    sample.yaw_rate = yawRateOf(*msg);

    if (hasUsableOrientation(*msg)) {
      tf2::Quaternion full_quat;
      tf2::fromMsg(msg->orientation, full_quat);
      full_quat.normalize();
      estimated_orientation_ = full_quat;
      estimator_initialized_ = true;
      last_estimator_stamp_ = sample.stamp;

      sample.orientation = yawOnlyQuaternion(msg->orientation);
      if (invert_imu_yaw_) {
        sample.orientation =
            yawQuaternionFromAngle(-yawAngleOfQuaternion(sample.orientation));
      }
      sample.has_orientation = true;
    } else if (estimate_orientation_from_raw_imu_ &&
               updateEstimatedOrientation(*msg, sample.stamp)) {
      sample.orientation = yawOnlyQuaternion(estimated_orientation_);
      if (invert_imu_yaw_) {
        sample.orientation =
            yawQuaternionFromAngle(-yawAngleOfQuaternion(sample.orientation));
      }
      sample.has_orientation = true;
      if (!logged_internal_estimator_) {
        logged_internal_estimator_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "IMU orientation missing on %s. Using internal Madgwick-style estimator.",
            imu_topic_.c_str());
      }
    }

    imu_buffer_.push_back(sample);
    trimImuBuffer();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_buffer_.push_back(msg);
    trimOdomBuffer();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) {
      return;
    }

    if (std::abs(msg->angle_increment) < 1e-9) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping scan: angle_increment is zero.");
      return;
    }

    double scan_time = estimateScanTime(*msg);
    double time_increment = msg->time_increment;
    if (time_increment <= 0.0 && scan_time > 0.0 && msg->ranges.size() > 1) {
      time_increment = scan_time / static_cast<double>(msg->ranges.size() - 1);
    }
    if (time_increment <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping scan: unable to estimate time_increment.");
      return;
    }

    const rclcpp::Time scan_stamp(msg->header.stamp);
    const rclcpp::Duration delay =
        rclcpp::Duration::from_seconds(lidar_msg_delay_ms_ / 1000.0);
    const rclcpp::Duration frame_duration =
        rclcpp::Duration::from_seconds(scan_time);
    const rclcpp::Time scan_start_time = scan_stamp - frame_duration - delay;
    const rclcpp::Time reference_time =
        scan_start_time + rclcpp::Duration::from_seconds(
                              time_increment * (msg->ranges.size() - 1));
    const bool use_absolute_imu_yaw = canUseAbsoluteImuYaw(reference_time);
    const bool use_odom_translation_for_scan =
        use_odom_translation_ && use_absolute_imu_yaw;

    tf2::Quaternion reference_quat;
    if (!interpolateImuYaw(reference_time, reference_quat)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping scan: no usable IMU data yet.");
      return;
    }

    Translation2D reference_translation;
    if (use_odom_translation_ && !use_absolute_imu_yaw) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "IMU orientation unavailable; disabling odom translation for this scan.");
    }
    if (use_odom_translation_for_scan &&
        !interpolateOdomTranslation(reference_time, reference_translation)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping scan: no usable odom data yet.");
      return;
    }

    auto corrected = *msg;
    corrected.ranges.assign(msg->ranges.size(),
                            std::numeric_limits<float>::infinity());
    if (msg->intensities.empty()) {
      corrected.intensities.clear();
    } else {
      corrected.intensities.assign(msg->ranges.size(), 0.0f);
    }
    corrected.scan_time = static_cast<float>(scan_time);
    corrected.time_increment = static_cast<float>(time_increment);

    std::size_t corrected_points = 0;

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      const std::size_t beam_ordinal =
          scan_direction_clockwise_ ? (msg->ranges.size() - 1 - i) : i;
      const rclcpp::Time beam_time =
          scan_start_time +
          rclcpp::Duration::from_seconds(time_increment * beam_ordinal);

      tf2::Quaternion beam_quat;
      if (!interpolateImuYaw(beam_time, beam_quat)) {
        continue;
      }

      const tf2::Quaternion delta_quat = reference_quat.inverse() * beam_quat;
      Translation2D beam_translation;
      if (use_odom_translation_for_scan) {
        if (!interpolateOdomTranslation(beam_time, beam_translation)) {
          continue;
        }
      }

      const double angle =
          msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const tf2::Vector3 point_beam(range * std::cos(angle),
                                    range * std::sin(angle), 0.0);
      tf2::Vector3 point_ref = tf2::quatRotate(delta_quat, point_beam);

      const tf2::Vector3 laser_offset_base(laser_offset_x_, laser_offset_y_, 0.0);
      const tf2::Vector3 beam_laser_world =
          tf2::quatRotate(beam_quat, laser_offset_base);
      const tf2::Vector3 reference_laser_world =
          tf2::quatRotate(reference_quat, laser_offset_base);
      point_ref += tf2::quatRotate(
          reference_quat.inverse(), beam_laser_world - reference_laser_world);

      if (use_odom_translation_for_scan) {
        const tf2::Vector3 translation_world(
            beam_translation.x - reference_translation.x,
            beam_translation.y - reference_translation.y, 0.0);
        point_ref += tf2::quatRotate(reference_quat.inverse(), translation_world);
      }

      const double corrected_range = std::hypot(point_ref.x(), point_ref.y());
      if (corrected_range < msg->range_min || corrected_range > msg->range_max) {
        continue;
      }

      const double corrected_angle = wrapAngleToScan(
          std::atan2(point_ref.y(), point_ref.x()), msg->angle_min,
          msg->angle_max);
      const int corrected_index = static_cast<int>(std::lround(
          (corrected_angle - msg->angle_min) / msg->angle_increment));
      if (corrected_index < 0 ||
          corrected_index >= static_cast<int>(corrected.ranges.size())) {
        continue;
      }

      const float previous_range = corrected.ranges[corrected_index];
      if (!std::isfinite(previous_range) ||
          corrected_range + range_epsilon_ < previous_range) {
        corrected.ranges[corrected_index] = static_cast<float>(corrected_range);
        if (!corrected.intensities.empty() && i < msg->intensities.size()) {
          corrected.intensities[corrected_index] = msg->intensities[i];
        }
        ++corrected_points;
      }
    }

    if (corrected_points == 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Skipping scan: no beams could be corrected with current IMU buffer.");
      return;
    }

    last_scan_stamp_ = scan_stamp;
    scan_pub_->publish(corrected);
    auto corrected_cloud = buildPointCloud(corrected);
    cloud_pub_->publish(corrected_cloud);
    if (publish_world_cloud_ && world_cloud_pub_) {
      sensor_msgs::msg::PointCloud2 world_cloud;
      if (transformPointCloud(corrected_cloud, world_cloud_frame_,
                              reference_time, world_cloud)) {
        world_cloud_pub_->publish(world_cloud);
      }
    }
    if (publish_raw_cloud_ && raw_cloud_pub_) {
      raw_cloud_pub_->publish(buildPointCloud(*msg));
    }
  }

  double estimateScanTime(const sensor_msgs::msg::LaserScan &scan) const {
    if (scan.scan_time > 0.0) {
      return scan.scan_time;
    }

    if (last_scan_stamp_.nanoseconds() > 0) {
      const double estimated =
          (rclcpp::Time(scan.header.stamp) - last_scan_stamp_).seconds();
      if (estimated > 0.0 && estimated < 1.0) {
        return estimated;
      }
    }

    if (default_scan_rate_hz_ > 0.0) {
      return 1.0 / default_scan_rate_hz_;
    }
    return 0.0;
  }

  bool hasUsableOrientation(const sensor_msgs::msg::Imu &imu) const {
    if (imu.orientation_covariance[0] < 0.0) {
      return false;
    }

    const auto &q = imu.orientation;
    const double norm_sq =
        q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(norm_sq) && norm_sq > 1e-12;
  }

  double yawRateOf(const sensor_msgs::msg::Imu &imu) const {
    const double yaw_rate =
        std::isfinite(imu.angular_velocity.z) ? imu.angular_velocity.z : 0.0;
    return invert_imu_yaw_ ? -yaw_rate : yaw_rate;
  }

  bool updateEstimatedOrientation(const sensor_msgs::msg::Imu &imu,
                                  const rclcpp::Time &stamp) {
    const double ax = imu.linear_acceleration.x;
    const double ay = imu.linear_acceleration.y;
    const double az = imu.linear_acceleration.z;
    const double gx = std::isfinite(imu.angular_velocity.x) ? imu.angular_velocity.x : 0.0;
    const double gy = std::isfinite(imu.angular_velocity.y) ? imu.angular_velocity.y : 0.0;
    double gz = std::isfinite(imu.angular_velocity.z) ? imu.angular_velocity.z : 0.0;
    if (invert_imu_yaw_) {
      gz = -gz;
    }

    if (!estimator_initialized_) {
      if (!initializeQuaternionFromAccel(ax, ay, az, estimated_orientation_)) {
        return false;
      }
      estimator_initialized_ = true;
      last_estimator_stamp_ = stamp;
      return true;
    }

    double dt_sec = (stamp - last_estimator_stamp_).seconds();
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      if (!initializeQuaternionFromAccel(ax, ay, az, estimated_orientation_)) {
        last_estimator_stamp_ = stamp;
        return false;
      }
      last_estimator_stamp_ = stamp;
      return true;
    }

    if (dt_sec > 0.5) {
      if (initializeQuaternionFromAccel(ax, ay, az, estimated_orientation_)) {
        last_estimator_stamp_ = stamp;
        return true;
      }
    }

    madgwickUpdateImu(estimated_orientation_, gx, gy, gz, ax, ay, az, dt_sec,
                      madgwick_beta_);
    last_estimator_stamp_ = stamp;
    return true;
  }

  bool interpolateImuYawFromOrientation(const rclcpp::Time &stamp,
                                        tf2::Quaternion &quat_out) const {
    if (imu_buffer_.size() < 2) {
      return false;
    }

    const int64_t target_ns = stamp.nanoseconds();
    const int64_t first_ns = imu_buffer_.front().stamp.nanoseconds();
    const int64_t last_ns = imu_buffer_.back().stamp.nanoseconds();
    if (target_ns < first_ns || target_ns > last_ns) {
      return false;
    }

    auto it = std::lower_bound(
        imu_buffer_.begin(), imu_buffer_.end(), target_ns,
        [](const ImuSample &imu, int64_t ns) {
          return imu.stamp.nanoseconds() < ns;
        });

    if (it == imu_buffer_.begin()) {
      if (!it->has_orientation) {
        return false;
      }
      quat_out = it->orientation;
      return true;
    }
    if (it == imu_buffer_.end()) {
      if (!imu_buffer_.back().has_orientation) {
        return false;
      }
      quat_out = imu_buffer_.back().orientation;
      return true;
    }

    const auto &imu_high = *it;
    const auto &imu_low = *(it - 1);
    if (!imu_low.has_orientation || !imu_high.has_orientation) {
      return false;
    }

    const int64_t t_low = imu_low.stamp.nanoseconds();
    const int64_t t_high = imu_high.stamp.nanoseconds();

    if (t_high == t_low) {
      quat_out = imu_high.orientation;
      return true;
    }

    const double alpha = static_cast<double>(target_ns - t_low) /
                         static_cast<double>(t_high - t_low);
    quat_out = imu_low.orientation.slerp(imu_high.orientation, alpha);
    quat_out.normalize();
    return true;
  }

  bool interpolateImuYawFromGyro(const rclcpp::Time &stamp,
                                 tf2::Quaternion &quat_out) const {
    if (imu_buffer_.size() < 2) {
      return false;
    }

    const int64_t target_ns = stamp.nanoseconds();
    const int64_t first_ns = imu_buffer_.front().stamp.nanoseconds();
    const int64_t last_ns = imu_buffer_.back().stamp.nanoseconds();
    if (target_ns < first_ns || target_ns > last_ns) {
      return false;
    }

    double integrated_yaw = 0.0;
    auto prev = imu_buffer_.begin();
    int64_t prev_ns = prev->stamp.nanoseconds();
    if (target_ns == prev_ns) {
      quat_out = yawQuaternionFromAngle(0.0);
      return true;
    }

    for (auto it = std::next(imu_buffer_.begin()); it != imu_buffer_.end(); ++it) {
      const int64_t curr_ns = it->stamp.nanoseconds();
      if (curr_ns <= prev_ns) {
        prev = it;
        prev_ns = curr_ns;
        continue;
      }

      const double prev_yaw_rate = prev->yaw_rate;
      const double curr_yaw_rate = it->yaw_rate;
      if (target_ns <= curr_ns) {
        const double alpha = static_cast<double>(target_ns - prev_ns) /
                             static_cast<double>(curr_ns - prev_ns);
        const double target_yaw_rate =
            prev_yaw_rate + alpha * (curr_yaw_rate - prev_yaw_rate);
        const double dt = static_cast<double>(target_ns - prev_ns) * 1e-9;
        integrated_yaw += 0.5 * (prev_yaw_rate + target_yaw_rate) * dt;
        quat_out = yawQuaternionFromAngle(integrated_yaw);
        return true;
      }

      const double dt = static_cast<double>(curr_ns - prev_ns) * 1e-9;
      integrated_yaw += 0.5 * (prev_yaw_rate + curr_yaw_rate) * dt;
      prev = it;
      prev_ns = curr_ns;
    }

    return false;
  }

  bool interpolateImuYaw(const rclcpp::Time &stamp, tf2::Quaternion &quat_out) const {
    if (interpolateImuYawFromOrientation(stamp, quat_out)) {
      return true;
    }
    return interpolateImuYawFromGyro(stamp, quat_out);
  }

  bool canUseAbsoluteImuYaw(const rclcpp::Time &stamp) const {
    tf2::Quaternion unused;
    return interpolateImuYawFromOrientation(stamp, unused);
  }

  bool interpolateOdomTranslation(const rclcpp::Time &stamp,
                                  Translation2D &translation_out) const {
    if (odom_buffer_.size() < 2) {
      return false;
    }

    const int64_t target_ns = stamp.nanoseconds();
    const int64_t first_ns =
        rclcpp::Time(odom_buffer_.front()->header.stamp).nanoseconds();
    const int64_t last_ns =
        rclcpp::Time(odom_buffer_.back()->header.stamp).nanoseconds();
    if (target_ns < first_ns || target_ns > last_ns) {
      return false;
    }

    auto it = std::lower_bound(
        odom_buffer_.begin(), odom_buffer_.end(), target_ns,
        [](const nav_msgs::msg::Odometry::SharedPtr &odom, int64_t ns) {
          return rclcpp::Time(odom->header.stamp).nanoseconds() < ns;
        });

    if (it == odom_buffer_.begin()) {
      translation_out = odomTranslationOf(*(*it));
      return true;
    }
    if (it == odom_buffer_.end()) {
      translation_out = odomTranslationOf(*(odom_buffer_.back()));
      return true;
    }

    const auto &odom_high = *it;
    const auto &odom_low = *(it - 1);
    const int64_t t_low = rclcpp::Time(odom_low->header.stamp).nanoseconds();
    const int64_t t_high = rclcpp::Time(odom_high->header.stamp).nanoseconds();
    if (t_high == t_low) {
      translation_out = odomTranslationOf(*odom_high);
      return true;
    }

    const double alpha = static_cast<double>(target_ns - t_low) /
                         static_cast<double>(t_high - t_low);
    const Translation2D low = odomTranslationOf(*odom_low);
    const Translation2D high = odomTranslationOf(*odom_high);
    translation_out.x = low.x + alpha * (high.x - low.x);
    translation_out.y = low.y + alpha * (high.y - low.y);
    return true;
  }

  void trimImuBuffer() {
    const auto now = this->now();
    while (!imu_buffer_.empty()) {
      if ((now - imu_buffer_.front().stamp).seconds() <= imu_buffer_sec_) {
        break;
      }
      imu_buffer_.pop_front();
    }
  }

  void trimOdomBuffer() {
    const auto now = this->now();
    while (!odom_buffer_.empty()) {
      const rclcpp::Time stamp(odom_buffer_.front()->header.stamp);
      if ((now - stamp).seconds() <= odom_buffer_sec_) {
        break;
      }
      odom_buffer_.pop_front();
    }
  }

  Translation2D odomTranslationOf(const nav_msgs::msg::Odometry &odom) const {
    Translation2D translation;
    translation.x = odom.pose.pose.position.x;
    translation.y = odom.pose.pose.position.y;
    return translation;
  }

  sensor_msgs::msg::PointCloud2 buildPointCloud(
      const sensor_msgs::msg::LaserScan &scan) const {
    std::size_t valid_count = 0;
    for (const auto range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min &&
          range <= scan.range_max) {
        ++valid_count;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(valid_count);
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(valid_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min ||
          range > scan.range_max) {
        continue;
      }

      const double angle =
          scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      *iter_x = range * std::cos(angle);
      *iter_y = range * std::sin(angle);
      *iter_z = 0.0f;
      *iter_intensity = i < scan.intensities.size() ? scan.intensities[i] : 0.0f;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }

    return cloud;
  }

  bool transformPointCloud(const sensor_msgs::msg::PointCloud2 &input_cloud,
                           const std::string &target_frame,
                           const rclcpp::Time &target_stamp,
                           sensor_msgs::msg::PointCloud2 &output_cloud) {
    const std::string output_frame =
        target_frame.empty() ? input_cloud.header.frame_id : target_frame;
    output_cloud = input_cloud;
    output_cloud.header.frame_id = output_frame;
    const int64_t stamp_ns = target_stamp.nanoseconds();
    output_cloud.header.stamp.sec =
        static_cast<int32_t>(stamp_ns / 1000000000LL);
    output_cloud.header.stamp.nanosec =
        static_cast<uint32_t>(stamp_ns % 1000000000LL);

    if (input_cloud.header.frame_id == output_frame) {
      return true;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          output_frame, input_cloud.header.frame_id, target_stamp,
          rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Skipping world cloud publish: failed TF %s -> %s at %.3f: %s",
          input_cloud.header.frame_id.c_str(), output_frame.c_str(),
          target_stamp.seconds(), ex.what());
      return false;
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);

    sensor_msgs::PointCloud2Iterator<float> iter_x(output_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      tf2::Vector3 point_in(*iter_x, *iter_y, *iter_z);
      const tf2::Vector3 point_out = transform * point_in;
      *iter_x = static_cast<float>(point_out.x());
      *iter_y = static_cast<float>(point_out.y());
      *iter_z = static_cast<float>(point_out.z());
    }

    return true;
  }

  std::string scan_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string output_cloud_topic_;
  std::string world_cloud_topic_;
  std::string world_cloud_frame_;
  std::string raw_cloud_topic_;

  double default_scan_rate_hz_{10.0};
  double lidar_msg_delay_ms_{10.0};
  double imu_buffer_sec_{3.0};
  double odom_buffer_sec_{3.0};
  double range_epsilon_{1e-4};
  double laser_offset_x_{0.0};
  double laser_offset_y_{0.0};
  double madgwick_beta_{0.05};
  bool scan_direction_clockwise_{false};
  bool publish_raw_cloud_{true};
  bool publish_world_cloud_{true};
  bool invert_imu_yaw_{false};
  bool use_odom_translation_{true};
  bool estimate_orientation_from_raw_imu_{true};
  bool estimator_initialized_{false};
  bool logged_internal_estimator_{false};

  tf2::Quaternion estimated_orientation_{0.0, 0.0, 0.0, 1.0};
  rclcpp::Time last_estimator_stamp_{0, 0, RCL_ROS_TIME};
  std::deque<ImuSample> imu_buffer_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanUndistortionNode>());
  rclcpp::shutdown();
  return 0;
}
