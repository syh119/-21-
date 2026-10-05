#include "imu660ra_ros2/imu660ra_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace imu660ra_ros2
{

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kGravity = 9.80665;

double compute_low_pass_alpha(double cutoff_hz, double dt_sec)
{
  if (cutoff_hz <= 0.0 || dt_sec <= 0.0) {
    return 1.0;
  }

  const double tau = 1.0 / (kTwoPi * cutoff_hz);
  return dt_sec / (tau + dt_sec);
}

void apply_low_pass(
  const std::array<double, 3> & input, std::array<double, 3> * state, bool * initialized,
  double cutoff_hz, double dt_sec)
{
  if (state == nullptr || initialized == nullptr) {
    return;
  }

  if (cutoff_hz <= 0.0 || !*initialized) {
    *state = input;
    *initialized = true;
    return;
  }

  const double alpha = compute_low_pass_alpha(cutoff_hz, dt_sec);
  for (std::size_t axis = 0; axis < input.size(); ++axis) {
    (*state)[axis] += alpha * (input[axis] - (*state)[axis]);
  }
}

double vector_norm(const std::array<double, 3> & value)
{
  return std::sqrt(
    value[0] * value[0] +
    value[1] * value[1] +
    value[2] * value[2]);
}

}  // namespace

class Imu660raNode : public rclcpp::Node
{
public:
  Imu660raNode()
  : Node("imu660ra_node")
  {
    const std::string default_config = resolve_default_config_file();

    Imu660raParams driver_params;
    driver_params.transport = declare_parameter<std::string>("transport", "i2c");
    driver_params.device = declare_parameter<std::string>("device", "/dev/i2c-1");
    driver_params.config_file = declare_parameter<std::string>("config_file", default_config);
    driver_params.spi_speed_hz = declare_parameter<int>("spi_speed_hz", 10000000);
    driver_params.i2c_address = static_cast<uint8_t>(declare_parameter<int>("i2c_address", 0x69));
    driver_params.acc_range_g = declare_parameter<int>("acc_range_g", 8);
    driver_params.gyro_range_dps = declare_parameter<int>("gyro_range_dps", 2000);
    driver_params.acc_conf = static_cast<uint8_t>(declare_parameter<int>("acc_conf", 0xA7));
    driver_params.gyr_conf = static_cast<uint8_t>(declare_parameter<int>("gyr_conf", 0xA9));
    const int config_chunk_size = declare_parameter<int>("config_chunk_size", 64);
    driver_params.config_chunk_size = static_cast<std::size_t>(
      config_chunk_size > 0 ? config_chunk_size : 64);

    const std::string topic_name =
      declare_parameter<std::string>("topic_name", "/imu660ra/data_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    angular_velocity_cov_diag_ =
      declare_parameter<double>("angular_velocity_covariance_diag", 0.0);
    linear_acceleration_cov_diag_ =
      declare_parameter<double>("linear_acceleration_covariance_diag", 0.0);
    calibrate_gyro_on_startup_ = declare_parameter<bool>("calibrate_gyro_on_startup", true);
    const double calibration_duration_sec =
      declare_parameter<double>("gyro_calibration_duration_sec", 2.0);
    const double calibration_settle_sec =
      declare_parameter<double>("gyro_calibration_settle_sec", 0.5);
    gyro_calibration_stddev_threshold_radps_ =
      declare_parameter<double>("gyro_calibration_stddev_threshold_radps", 0.05);
    manual_gyro_offset_radps_[0] = declare_parameter<double>("gyro_offset_x", 0.0);
    manual_gyro_offset_radps_[1] = declare_parameter<double>("gyro_offset_y", 0.0);
    manual_gyro_offset_radps_[2] = declare_parameter<double>("gyro_offset_z", 0.0);
    accel_offset_mps2_[0] = declare_parameter<double>("acc_offset_x", 0.0);
    accel_offset_mps2_[1] = declare_parameter<double>("acc_offset_y", 0.0);
    accel_offset_mps2_[2] = declare_parameter<double>("acc_offset_z", 0.0);
    enable_online_bias_calibration_ =
      declare_parameter<bool>("enable_online_bias_calibration", true);
    online_bias_time_constant_sec_ =
      declare_parameter<double>("online_bias_time_constant_sec", 8.0);
    stationary_accel_norm_threshold_mps2_ =
      declare_parameter<double>("stationary_accel_norm_threshold_mps2", 0.20);
    stationary_gyro_threshold_radps_ =
      declare_parameter<double>("stationary_gyro_threshold_radps", 0.08);
    gyro_zero_deadband_radps_ =
      declare_parameter<double>("gyro_zero_deadband_radps", 0.01);
    gyro_low_pass_cutoff_hz_ = declare_parameter<double>("gyro_low_pass_cutoff_hz", 25.0);
    accel_low_pass_cutoff_hz_ = declare_parameter<double>("accel_low_pass_cutoff_hz", 20.0);
    if (publish_rate_hz <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than 0.");
    }
    if (calibration_duration_sec < 0.0 || calibration_settle_sec < 0.0) {
      throw std::runtime_error("Calibration timing parameters must be non-negative.");
    }
    if (gyro_low_pass_cutoff_hz_ < 0.0 || accel_low_pass_cutoff_hz_ < 0.0) {
      throw std::runtime_error("Low-pass cutoff frequency parameters must be non-negative.");
    }
    if (online_bias_time_constant_sec_ < 0.0) {
      throw std::runtime_error("online_bias_time_constant_sec must be non-negative.");
    }
    publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz);
    gyro_calibration_samples_ = std::max(
      1, static_cast<int>(std::llround(calibration_duration_sec * publish_rate_hz)));
    gyro_calibration_settle_ = std::chrono::duration<double>(calibration_settle_sec);

    driver_ = std::make_unique<Imu660raDriver>(driver_params);

    std::string error_message;
    if (!driver_->initialize(&error_message)) {
      throw std::runtime_error("Failed to initialize IMU660RA: " + error_message);
    }

    if (calibrate_gyro_on_startup_) {
      calibrate_gyro_bias();
    } else {
      learned_gyro_bias_radps_.fill(0.0);
    }

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topic_name, 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period_),
      std::bind(&Imu660raNode::publish_sample, this));

    RCLCPP_INFO(
      get_logger(),
      "IMU660RA node started via %s on %s at %.1f Hz (i2c addr: 0x%02X), topic: %s, "
      "gyro LPF: %.1f Hz, accel LPF: %.1f Hz, online bias calibration: %s.",
      driver_params.transport.c_str(), driver_params.device.c_str(), publish_rate_hz,
      driver_params.i2c_address, topic_name.c_str(), gyro_low_pass_cutoff_hz_,
      accel_low_pass_cutoff_hz_, enable_online_bias_calibration_ ? "on" : "off");
  }

private:
  struct VectorStats
  {
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    std::array<double, 3> sum_sq{0.0, 0.0, 0.0};
  };

  static std::string resolve_default_config_file()
  {
    try {
      return ament_index_cpp::get_package_share_directory("imu660ra_ros2") +
             "/config/imu660ra_config_file.txt";
    } catch (const std::exception &) {
      return "config/imu660ra_config_file.txt";
    }
  }

  void calibrate_gyro_bias()
  {
    RCLCPP_INFO(
      get_logger(),
      "Starting gyro bias calibration, keep the robot still for %.2f seconds.",
      publish_period_.count() * static_cast<double>(gyro_calibration_samples_));

    if (gyro_calibration_settle_.count() > 0.0) {
      std::this_thread::sleep_for(gyro_calibration_settle_);
    }

    VectorStats stats;
    for (int i = 0; i < gyro_calibration_samples_; ++i) {
      Imu660raSample sample;
      std::string error_message;
      if (!driver_->read_sample(&sample, &error_message)) {
        throw std::runtime_error("Failed during gyro calibration: " + error_message);
      }

      const std::array<double, 3> gyro = {
        static_cast<double>(sample.gyro_x) * driver_->gyro_scale_radps(),
        static_cast<double>(sample.gyro_y) * driver_->gyro_scale_radps(),
        static_cast<double>(sample.gyro_z) * driver_->gyro_scale_radps()};

      for (std::size_t axis = 0; axis < gyro.size(); ++axis) {
        stats.sum[axis] += gyro[axis];
        stats.sum_sq[axis] += gyro[axis] * gyro[axis];
      }
      std::this_thread::sleep_for(publish_period_);
    }

    std::array<double, 3> stddev{0.0, 0.0, 0.0};
    for (std::size_t axis = 0; axis < learned_gyro_bias_radps_.size(); ++axis) {
      const double mean = stats.sum[axis] / static_cast<double>(gyro_calibration_samples_);
      const double second_moment =
        stats.sum_sq[axis] / static_cast<double>(gyro_calibration_samples_);
      const double variance = std::max(0.0, second_moment - mean * mean);
      stddev[axis] = std::sqrt(variance);
      learned_gyro_bias_radps_[axis] = mean;
    }

    const double max_stddev = std::max(std::max(stddev[0], stddev[1]), stddev[2]);
    if (max_stddev > gyro_calibration_stddev_threshold_radps_) {
      RCLCPP_WARN(
        get_logger(),
        "Gyro bias calibration finished but sensor moved during sampling "
        "(max stddev %.6f rad/s > %.6f rad/s). Bias may be inaccurate.",
        max_stddev, gyro_calibration_stddev_threshold_radps_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Gyro offsets(rad/s): [%.6f, %.6f, %.6f], accel offsets(m/s^2): [%.6f, %.6f, %.6f].",
      learned_gyro_bias_radps_[0] + manual_gyro_offset_radps_[0],
      learned_gyro_bias_radps_[1] + manual_gyro_offset_radps_[1],
      learned_gyro_bias_radps_[2] + manual_gyro_offset_radps_[2],
      accel_offset_mps2_[0], accel_offset_mps2_[1], accel_offset_mps2_[2]);
  }

  void update_online_gyro_bias(
    const std::array<double, 3> & raw_gyro,
    const std::array<double, 3> & accel,
    double dt_sec)
  {
    if (!enable_online_bias_calibration_ || dt_sec <= 0.0) {
      return;
    }

    const std::array<double, 3> corrected_gyro = {
      raw_gyro[0] - learned_gyro_bias_radps_[0] - manual_gyro_offset_radps_[0],
      raw_gyro[1] - learned_gyro_bias_radps_[1] - manual_gyro_offset_radps_[1],
      raw_gyro[2] - learned_gyro_bias_radps_[2] - manual_gyro_offset_radps_[2]};

    const double accel_norm_error = std::fabs(vector_norm(accel) - kGravity);
    const double gyro_norm = vector_norm(corrected_gyro);
    const bool stationary =
      accel_norm_error <= stationary_accel_norm_threshold_mps2_ &&
      gyro_norm <= stationary_gyro_threshold_radps_;

    if (!stationary) {
      return;
    }

    const double alpha = online_bias_time_constant_sec_ <= 0.0 ?
      1.0 : std::min(1.0, dt_sec / online_bias_time_constant_sec_);
    for (std::size_t axis = 0; axis < learned_gyro_bias_radps_.size(); ++axis) {
      learned_gyro_bias_radps_[axis] +=
        alpha * ((raw_gyro[axis] - manual_gyro_offset_radps_[axis]) - learned_gyro_bias_radps_[axis]);
    }
  }

  void publish_sample()
  {
    Imu660raSample sample;
    std::string error_message;
    if (!driver_->read_sample(&sample, &error_message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "IMU660RA read failed: %s", error_message.c_str());
      return;
    }

    const rclcpp::Time sample_time = now();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = sample_time;
    msg.header.frame_id = frame_id_;

    msg.orientation_covariance[0] = -1.0;

    const std::array<double, 3> raw_gyro = {
      static_cast<double>(sample.gyro_x) * driver_->gyro_scale_radps(),
      static_cast<double>(sample.gyro_y) * driver_->gyro_scale_radps(),
      static_cast<double>(sample.gyro_z) * driver_->gyro_scale_radps()};
    const std::array<double, 3> accel = {
      static_cast<double>(sample.acc_x) * driver_->accel_scale_mps2() - accel_offset_mps2_[0],
      static_cast<double>(sample.acc_y) * driver_->accel_scale_mps2() - accel_offset_mps2_[1],
      static_cast<double>(sample.acc_z) * driver_->accel_scale_mps2() - accel_offset_mps2_[2]};

    double dt_sec = publish_period_.count();
    if (last_sample_time_.nanoseconds() > 0) {
      dt_sec = std::max(1e-4, (sample_time - last_sample_time_).seconds());
    }
    last_sample_time_ = sample_time;

    update_online_gyro_bias(raw_gyro, accel, dt_sec);

    const std::array<double, 3> gyro = {
      raw_gyro[0] - learned_gyro_bias_radps_[0] - manual_gyro_offset_radps_[0],
      raw_gyro[1] - learned_gyro_bias_radps_[1] - manual_gyro_offset_radps_[1],
      raw_gyro[2] - learned_gyro_bias_radps_[2] - manual_gyro_offset_radps_[2]};

    apply_low_pass(
      gyro, &filtered_gyro_radps_, &gyro_filter_initialized_, gyro_low_pass_cutoff_hz_, dt_sec);
    apply_low_pass(
      accel, &filtered_accel_mps2_, &accel_filter_initialized_, accel_low_pass_cutoff_hz_, dt_sec);

    for (std::size_t axis = 0; axis < filtered_gyro_radps_.size(); ++axis) {
      if (std::fabs(filtered_gyro_radps_[axis]) < gyro_zero_deadband_radps_) {
        filtered_gyro_radps_[axis] = 0.0;
      }
    }

    msg.angular_velocity.x = filtered_gyro_radps_[0];
    msg.angular_velocity.y = filtered_gyro_radps_[1];
    msg.angular_velocity.z = filtered_gyro_radps_[2];

    msg.linear_acceleration.x = filtered_accel_mps2_[0];
    msg.linear_acceleration.y = filtered_accel_mps2_[1];
    msg.linear_acceleration.z = filtered_accel_mps2_[2];

    set_diag(msg.angular_velocity_covariance, angular_velocity_cov_diag_);
    set_diag(msg.linear_acceleration_covariance, linear_acceleration_cov_diag_);

    imu_pub_->publish(msg);
  }

  static void set_diag(std::array<double, 9> & covariance, double diagonal_value)
  {
    covariance.fill(0.0);
    covariance[0] = diagonal_value;
    covariance[4] = diagonal_value;
    covariance[8] = diagonal_value;
  }

  std::string frame_id_;
  double angular_velocity_cov_diag_{0.0};
  double linear_acceleration_cov_diag_{0.0};
  double gyro_low_pass_cutoff_hz_{25.0};
  double accel_low_pass_cutoff_hz_{20.0};
  bool enable_online_bias_calibration_{true};
  bool calibrate_gyro_on_startup_{true};
  bool gyro_filter_initialized_{false};
  bool accel_filter_initialized_{false};
  int gyro_calibration_samples_{100};
  double gyro_calibration_stddev_threshold_radps_{0.05};
  double online_bias_time_constant_sec_{8.0};
  double stationary_accel_norm_threshold_mps2_{0.20};
  double stationary_gyro_threshold_radps_{0.08};
  double gyro_zero_deadband_radps_{0.01};
  std::chrono::duration<double> publish_period_{0.02};
  std::chrono::duration<double> gyro_calibration_settle_{0.5};
  std::array<double, 3> manual_gyro_offset_radps_{0.0, 0.0, 0.0};
  std::array<double, 3> learned_gyro_bias_radps_{0.0, 0.0, 0.0};
  std::array<double, 3> accel_offset_mps2_{0.0, 0.0, 0.0};
  std::array<double, 3> filtered_gyro_radps_{0.0, 0.0, 0.0};
  std::array<double, 3> filtered_accel_mps2_{0.0, 0.0, 0.0};
  rclcpp::Time last_sample_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<Imu660raDriver> driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace imu660ra_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<imu660ra_ros2::Imu660raNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("imu660ra_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
