#ifndef IMU660RA_ROS2__IMU660RA_DRIVER_HPP_
#define IMU660RA_ROS2__IMU660RA_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace imu660ra_ros2
{

struct Imu660raParams
{
  std::string transport = "i2c";
  std::string device = "/dev/i2c-1";
  std::string config_file;
  uint32_t spi_speed_hz = 10000000;
  uint8_t i2c_address = 0x69;
  int acc_range_g = 8;
  int gyro_range_dps = 2000;
  uint8_t acc_conf = 0xA7;
  uint8_t gyr_conf = 0xA9;
  std::size_t config_chunk_size = 64;
};

struct Imu660raSample
{
  int16_t acc_x = 0;
  int16_t acc_y = 0;
  int16_t acc_z = 0;
  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;
};

class Imu660raDriver
{
public:
  explicit Imu660raDriver(const Imu660raParams & params);
  ~Imu660raDriver();

  bool initialize(std::string * error_message = nullptr);
  bool read_sample(Imu660raSample * sample, std::string * error_message = nullptr);

  double accel_scale_mps2() const;
  double gyro_scale_radps() const;

private:
  bool open_device(std::string * error_message);
  void close_device();

  bool write_register(uint8_t reg, uint8_t value, std::string * error_message);
  bool write_registers(
    uint8_t reg, const uint8_t * data, std::size_t length, std::string * error_message);
  bool read_register(uint8_t reg, uint8_t * value, std::string * error_message);
  bool read_registers(
    uint8_t reg, uint8_t * data, std::size_t length, std::string * error_message);

  bool load_config_file(std::string * error_message);
  bool self_check(std::string * error_message);
  bool configure_ranges(std::string * error_message);
  bool use_spi() const;
  bool use_i2c() const;

  static double accel_lsb_per_g(int range_g);
  static double gyro_lsb_per_dps(int range_dps);

  Imu660raParams params_;
  int fd_{-1};
  double accel_lsb_per_g_{4096.0};
  double gyro_lsb_per_dps_{16.4};
};

}  // namespace imu660ra_ros2

#endif  // IMU660RA_ROS2__IMU660RA_DRIVER_HPP_
