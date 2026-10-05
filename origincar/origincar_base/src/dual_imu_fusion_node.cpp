#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{
double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double clamp_abs_to_zero(double value, double threshold)
{
  return std::fabs(value) < threshold ? 0.0 : value;
}
}  // namespace

class DualImuFusionNode : public rclcpp::Node
{
public:
  DualImuFusionNode()
  : Node("dual_imu_fusion_node")
  {
    primary_topic_ = declare_parameter<std::string>("primary_imu_topic", "/imu660ra/data_raw");
    secondary_topic_ = declare_parameter<std::string>("secondary_imu_topic", "/imu/data_raw");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/imu/fused");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "imu_link");

    primary_weight_ = declare_parameter<double>("primary_weight", 0.75);
    secondary_weight_ = declare_parameter<double>("secondary_weight", 0.25);
    max_secondary_age_sec_ = declare_parameter<double>("max_secondary_age_sec", 0.10);
    max_time_skew_sec_ = declare_parameter<double>("max_time_skew_sec", 0.08);

    stationary_linear_vel_threshold_ =
      declare_parameter<double>("stationary_linear_vel_threshold", 0.03);
    stationary_angular_vel_threshold_ =
      declare_parameter<double>("stationary_angular_vel_threshold", 0.05);
    bias_learning_rate_ = declare_parameter<double>("bias_learning_rate", 0.03);
    zero_rate_threshold_radps_ =
      declare_parameter<double>("zero_rate_threshold_radps", 0.015);

    primary_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      primary_topic_, 20,
      std::bind(&DualImuFusionNode::primary_callback, this, std::placeholders::_1));
    secondary_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      secondary_topic_, 20,
      std::bind(&DualImuFusionNode::secondary_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&DualImuFusionNode::odom_callback, this, std::placeholders::_1));

    fused_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 20);

    RCLCPP_INFO(
      get_logger(),
      "Dual IMU fusion started. primary=%s secondary=%s odom=%s output=%s",
      primary_topic_.c_str(), secondary_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & twist = msg->twist.twist;
    const double linear_norm = std::hypot(twist.linear.x, twist.linear.y);
    const double angular_z = std::fabs(twist.angular.z);
    stationary_ = linear_norm < stationary_linear_vel_threshold_ &&
      angular_z < stationary_angular_vel_threshold_;
  }

  void primary_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    primary_msg_ = msg;
    if (!primary_bias_initialized_) {
      primary_bias_ = {
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z};
      primary_bias_initialized_ = true;
    }

    if (stationary_) {
      learn_bias(*msg, primary_bias_);
    }

    publish_fused();
  }

  void secondary_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    secondary_msg_ = msg;
    if (!secondary_bias_initialized_) {
      secondary_bias_ = {
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z};
      secondary_bias_initialized_ = true;
    }

    if (stationary_) {
      learn_bias(*msg, secondary_bias_);
    }
  }

  void learn_bias(
    const sensor_msgs::msg::Imu & msg,
    std::array<double, 3> & bias)
  {
    const std::array<double, 3> gyro = {
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z};

    for (std::size_t axis = 0; axis < bias.size(); ++axis) {
      bias[axis] = (1.0 - bias_learning_rate_) * bias[axis] +
        bias_learning_rate_ * gyro[axis];
    }
  }

  bool secondary_available_for(const sensor_msgs::msg::Imu & primary) const
  {
    if (!secondary_msg_) {
      return false;
    }

    const double primary_sec = stamp_to_seconds(primary.header.stamp);
    const double secondary_sec = stamp_to_seconds(secondary_msg_->header.stamp);
    const double age = std::fabs(primary_sec - secondary_sec);
    return age <= max_secondary_age_sec_ && age <= max_time_skew_sec_;
  }

  void publish_fused()
  {
    if (!primary_msg_) {
      return;
    }

    sensor_msgs::msg::Imu fused = *primary_msg_;
    fused.header.frame_id = output_frame_id_.empty() ? primary_msg_->header.frame_id : output_frame_id_;

    fused.orientation_covariance[0] = -1.0;
    fused.orientation_covariance[4] = 0.0;
    fused.orientation_covariance[8] = 0.0;

    const bool use_secondary = secondary_available_for(*primary_msg_);
    double primary_weight = std::max(0.0, primary_weight_);
    double secondary_weight = use_secondary ? std::max(0.0, secondary_weight_) : 0.0;
    const double total_weight = primary_weight + secondary_weight;
    if (total_weight <= 0.0) {
      primary_weight = 1.0;
      secondary_weight = 0.0;
    } else {
      primary_weight /= total_weight;
      secondary_weight /= total_weight;
    }

    const std::array<double, 3> primary_gyro = {
      primary_msg_->angular_velocity.x - primary_bias_[0],
      primary_msg_->angular_velocity.y - primary_bias_[1],
      primary_msg_->angular_velocity.z - primary_bias_[2]};

    std::array<double, 3> secondary_gyro = {0.0, 0.0, 0.0};
    std::array<double, 3> secondary_accel = {0.0, 0.0, 0.0};
    if (use_secondary) {
      secondary_gyro = {
        secondary_msg_->angular_velocity.x - secondary_bias_[0],
        secondary_msg_->angular_velocity.y - secondary_bias_[1],
        secondary_msg_->angular_velocity.z - secondary_bias_[2]};
      secondary_accel = {
        secondary_msg_->linear_acceleration.x,
        secondary_msg_->linear_acceleration.y,
        secondary_msg_->linear_acceleration.z};
    }

    std::array<double, 3> fused_gyro = {
      primary_weight * primary_gyro[0] + secondary_weight * secondary_gyro[0],
      primary_weight * primary_gyro[1] + secondary_weight * secondary_gyro[1],
      primary_weight * primary_gyro[2] + secondary_weight * secondary_gyro[2]};

    if (stationary_) {
      fused_gyro[0] = clamp_abs_to_zero(fused_gyro[0], zero_rate_threshold_radps_);
      fused_gyro[1] = clamp_abs_to_zero(fused_gyro[1], zero_rate_threshold_radps_);
      fused_gyro[2] = clamp_abs_to_zero(fused_gyro[2], zero_rate_threshold_radps_);
    }

    fused.angular_velocity.x = fused_gyro[0];
    fused.angular_velocity.y = fused_gyro[1];
    fused.angular_velocity.z = fused_gyro[2];

    fused.linear_acceleration.x =
      primary_weight * primary_msg_->linear_acceleration.x +
      secondary_weight * secondary_accel[0];
    fused.linear_acceleration.y =
      primary_weight * primary_msg_->linear_acceleration.y +
      secondary_weight * secondary_accel[1];
    fused.linear_acceleration.z =
      primary_weight * primary_msg_->linear_acceleration.z +
      secondary_weight * secondary_accel[2];

    fused.angular_velocity_covariance[0] =
      std::max(primary_msg_->angular_velocity_covariance[0], 1e-3);
    fused.angular_velocity_covariance[4] =
      std::max(primary_msg_->angular_velocity_covariance[4], 1e-3);
    fused.angular_velocity_covariance[8] =
      std::max(primary_msg_->angular_velocity_covariance[8], 5e-4);

    fused_pub_->publish(fused);
  }

  std::string primary_topic_;
  std::string secondary_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  double primary_weight_{0.75};
  double secondary_weight_{0.25};
  double max_secondary_age_sec_{0.10};
  double max_time_skew_sec_{0.08};
  double stationary_linear_vel_threshold_{0.03};
  double stationary_angular_vel_threshold_{0.05};
  double bias_learning_rate_{0.03};
  double zero_rate_threshold_radps_{0.015};

  bool stationary_{false};
  bool primary_bias_initialized_{false};
  bool secondary_bias_initialized_{false};
  std::array<double, 3> primary_bias_{0.0, 0.0, 0.0};
  std::array<double, 3> secondary_bias_{0.0, 0.0, 0.0};

  sensor_msgs::msg::Imu::SharedPtr primary_msg_;
  sensor_msgs::msg::Imu::SharedPtr secondary_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr primary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr secondary_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fused_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualImuFusionNode>());
  rclcpp::shutdown();
  return 0;
}
