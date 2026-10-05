#include "imu660ra_ros2/imu660ra_driver.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/spi/spidev.h>
#include <regex>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace imu660ra_ros2
{
namespace
{

constexpr uint8_t kSpiReadMask = 0x80;
constexpr uint8_t kChipIdReg = 0x00;
constexpr uint8_t kExpectedChipId = 0x24;
constexpr uint8_t kAccDataReg = 0x0C;
constexpr uint8_t kInternalStatusReg = 0x21;
constexpr uint8_t kAccConfReg = 0x40;
constexpr uint8_t kAccRangeReg = 0x41;
constexpr uint8_t kGyrConfReg = 0x42;
constexpr uint8_t kGyrRangeReg = 0x43;
constexpr uint8_t kInitCtrlReg = 0x59;
constexpr uint8_t kInitAddr0Reg = 0x5B;
constexpr uint8_t kInitAddr1Reg = 0x5C;
constexpr uint8_t kInitDataReg = 0x5E;
constexpr uint8_t kPwrConfReg = 0x7C;
constexpr uint8_t kPwrCtrlReg = 0x7D;
constexpr uint8_t kPwrCtrlPerfMode = 0x0E;

constexpr double kStandardGravity = 9.80665;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

void set_error(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

}  // namespace

Imu660raDriver::Imu660raDriver(const Imu660raParams & params)
: params_(params)
{
}

Imu660raDriver::~Imu660raDriver()
{
  close_device();
}

bool Imu660raDriver::initialize(std::string * error_message)
{
  if (!open_device(error_message)) {
    return false;
  }

  if (!self_check(error_message)) {
    return false;
  }

  if (!write_register(kPwrConfReg, 0x00, error_message)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (!write_register(kInitCtrlReg, 0x00, error_message)) {
    return false;
  }

  if (!load_config_file(error_message)) {
    return false;
  }

  if (!write_register(kInitCtrlReg, 0x01, error_message)) {
    return false;
  }

  for (int i = 0; i < 40; ++i) {
    uint8_t status = 0;
    if (!read_register(kInternalStatusReg, &status, error_message)) {
      return false;
    }
    if (status == 0x01) {
      break;
    }
    if (i == 39) {
      set_error(error_message, "BMI270 internal status did not reach init_ok.");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (!write_register(kPwrCtrlReg, kPwrCtrlPerfMode, error_message)) {
    return false;
  }
  if (!write_register(kAccConfReg, params_.acc_conf, error_message)) {
    return false;
  }
  if (!write_register(kGyrConfReg, params_.gyr_conf, error_message)) {
    return false;
  }

  return configure_ranges(error_message);
}

bool Imu660raDriver::read_sample(Imu660raSample * sample, std::string * error_message)
{
  if (sample == nullptr) {
    set_error(error_message, "Sample pointer is null.");
    return false;
  }

  uint8_t data[12] = {};
  if (!read_registers(kAccDataReg, data, sizeof(data), error_message)) {
    return false;
  }

  sample->acc_x = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  sample->acc_y = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  sample->acc_z = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  sample->gyro_x = static_cast<int16_t>((static_cast<uint16_t>(data[7]) << 8) | data[6]);
  sample->gyro_y = static_cast<int16_t>((static_cast<uint16_t>(data[9]) << 8) | data[8]);
  sample->gyro_z = static_cast<int16_t>((static_cast<uint16_t>(data[11]) << 8) | data[10]);
  return true;
}

double Imu660raDriver::accel_scale_mps2() const
{
  return kStandardGravity / accel_lsb_per_g_;
}

double Imu660raDriver::gyro_scale_radps() const
{
  return kDegToRad / gyro_lsb_per_dps_;
}

bool Imu660raDriver::open_device(std::string * error_message)
{
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(params_.device.c_str(), O_RDWR);
  if (fd_ < 0) {
    set_error(
      error_message,
      "Failed to open bus device " + params_.device + ": " + std::strerror(errno));
    return false;
  }

  if (use_spi()) {
    uint8_t mode = SPI_MODE_0;
    uint8_t bits_per_word = 8;
    uint32_t speed_hz = params_.spi_speed_hz;

    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd_, SPI_IOC_RD_MODE, &mode) < 0) {
      set_error(error_message, "Failed to configure SPI mode.");
      close_device();
      return false;
    }

    if (
      ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
      ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0)
    {
      set_error(error_message, "Failed to configure SPI bits per word.");
      close_device();
      return false;
    }

    if (
      ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
      ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0)
    {
      set_error(error_message, "Failed to configure SPI speed.");
      close_device();
      return false;
    }

    uint8_t ignored = 0;
    return read_register(kChipIdReg, &ignored, error_message);
  }

  if (use_i2c()) {
    const unsigned long address = static_cast<unsigned long>(params_.i2c_address);
    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
      set_error(error_message, "Failed to configure I2C slave address.");
      close_device();
      return false;
    }
    return true;
  }

  set_error(error_message, "Unsupported transport. Use 'i2c' or 'spi'.");
  close_device();
  return false;
}

void Imu660raDriver::close_device()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Imu660raDriver::write_register(uint8_t reg, uint8_t value, std::string * error_message)
{
  return write_registers(reg, &value, 1, error_message);
}

bool Imu660raDriver::write_registers(
  uint8_t reg, const uint8_t * data, std::size_t length, std::string * error_message)
{
  if (fd_ < 0) {
    set_error(error_message, "Bus device is not open.");
    return false;
  }

  std::vector<uint8_t> tx(length + 1, 0);
  tx[0] = reg;
  if (length > 0 && data != nullptr) {
    std::memcpy(tx.data() + 1, data, length);
  }

  if (use_spi()) {
    std::vector<uint8_t> rx(length + 1, 0);
    spi_ioc_transfer transfer = {};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    transfer.len = static_cast<uint32_t>(tx.size());
    transfer.speed_hz = params_.spi_speed_hz;
    transfer.bits_per_word = 8;

    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      set_error(error_message, "SPI write failed: " + std::string(std::strerror(errno)));
      return false;
    }
    return true;
  }

  if (use_i2c()) {
    const ssize_t written = ::write(fd_, tx.data(), tx.size());
    if (written != static_cast<ssize_t>(tx.size())) {
      set_error(error_message, "I2C write failed: " + std::string(std::strerror(errno)));
      return false;
    }
    return true;
  }

  set_error(error_message, "Unsupported transport. Use 'i2c' or 'spi'.");
  return false;
}

bool Imu660raDriver::read_register(uint8_t reg, uint8_t * value, std::string * error_message)
{
  return read_registers(reg, value, 1, error_message);
}

bool Imu660raDriver::read_registers(
  uint8_t reg, uint8_t * data, std::size_t length, std::string * error_message)
{
  if (fd_ < 0) {
    set_error(error_message, "Bus device is not open.");
    return false;
  }
  if (data == nullptr) {
    set_error(error_message, "Read buffer is null.");
    return false;
  }

  if (use_spi()) {
    std::vector<uint8_t> tx(length + 1, 0);
    std::vector<uint8_t> rx(length + 1, 0);
    tx[0] = static_cast<uint8_t>(reg | kSpiReadMask);

    spi_ioc_transfer transfer = {};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    transfer.len = static_cast<uint32_t>(tx.size());
    transfer.speed_hz = params_.spi_speed_hz;
    transfer.bits_per_word = 8;

    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      set_error(error_message, "SPI read failed: " + std::string(std::strerror(errno)));
      return false;
    }

    std::memcpy(data, rx.data() + 1, length);
    return true;
  }

  if (use_i2c()) {
    uint8_t reg_addr = reg;
    i2c_rdwr_ioctl_data ioctl_data = {};
    i2c_msg messages[2] = {};

    messages[0].addr = params_.i2c_address;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg_addr;

    messages[1].addr = params_.i2c_address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<decltype(messages[1].len)>(length);
    messages[1].buf = data;

    ioctl_data.msgs = messages;
    ioctl_data.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
      set_error(error_message, "I2C read failed: " + std::string(std::strerror(errno)));
      return false;
    }
    return true;
  }

  set_error(error_message, "Unsupported transport. Use 'i2c' or 'spi'.");
  return false;
}

bool Imu660raDriver::load_config_file(std::string * error_message)
{
  std::ifstream input(params_.config_file);
  if (!input.is_open()) {
    set_error(error_message, "Failed to open config file: " + params_.config_file);
    return false;
  }

  std::stringstream buffer;
  buffer << input.rdbuf();
  const std::string content = buffer.str();

  std::regex byte_regex("0x([0-9a-fA-F]{1,2})");
  std::sregex_iterator it(content.begin(), content.end(), byte_regex);
  std::sregex_iterator end;

  std::vector<uint8_t> config_bytes;
  config_bytes.reserve(8192);
  for (; it != end; ++it) {
    config_bytes.push_back(static_cast<uint8_t>(std::stoul((*it)[1].str(), nullptr, 16)));
  }

  if (config_bytes.size() != 8192) {
    set_error(
      error_message,
      "Unexpected config length in " + params_.config_file + ": " +
      std::to_string(config_bytes.size()) + " bytes.");
    return false;
  }

  std::size_t chunk_size = params_.config_chunk_size;
  if (chunk_size == 0) {
    chunk_size = 64;
  }
  if ((chunk_size % 2U) != 0U) {
    ++chunk_size;
  }

  for (std::size_t index = 0; index < config_bytes.size(); index += chunk_size) {
    const std::size_t remaining = config_bytes.size() - index;
    const std::size_t write_len = std::min(chunk_size, remaining);
    const uint16_t base_addr = static_cast<uint16_t>(index / 2U);
    const uint8_t addr0 = static_cast<uint8_t>(base_addr & 0x0F);
    const uint8_t addr1 = static_cast<uint8_t>((base_addr >> 4) & 0xFF);

    if (!write_register(kInitAddr0Reg, addr0, error_message)) {
      return false;
    }
    if (!write_register(kInitAddr1Reg, addr1, error_message)) {
      return false;
    }
    if (!write_registers(kInitDataReg, config_bytes.data() + index, write_len, error_message)) {
      return false;
    }
  }

  return true;
}

bool Imu660raDriver::self_check(std::string * error_message)
{
  for (int i = 0; i < 255; ++i) {
    uint8_t chip_id = 0;
    if (!read_register(kChipIdReg, &chip_id, error_message)) {
      return false;
    }
    if (chip_id == kExpectedChipId) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  set_error(error_message, "BMI270 chip id check failed, expected 0x24.");
  return false;
}

bool Imu660raDriver::configure_ranges(std::string * error_message)
{
  uint8_t acc_range_reg = 0;
  switch (params_.acc_range_g) {
    case 2:
      acc_range_reg = 0x00;
      break;
    case 4:
      acc_range_reg = 0x01;
      break;
    case 8:
      acc_range_reg = 0x02;
      break;
    case 16:
      acc_range_reg = 0x03;
      break;
    default:
      set_error(error_message, "Unsupported accelerometer range. Use 2, 4, 8, or 16 g.");
      return false;
  }

  uint8_t gyro_range_reg = 0;
  switch (params_.gyro_range_dps) {
    case 125:
      gyro_range_reg = 0x04;
      break;
    case 250:
      gyro_range_reg = 0x03;
      break;
    case 500:
      gyro_range_reg = 0x02;
      break;
    case 1000:
      gyro_range_reg = 0x01;
      break;
    case 2000:
      gyro_range_reg = 0x00;
      break;
    default:
      set_error(error_message, "Unsupported gyroscope range. Use 125, 250, 500, 1000, or 2000 dps.");
      return false;
  }

  if (!write_register(kAccRangeReg, acc_range_reg, error_message)) {
    return false;
  }
  if (!write_register(kGyrRangeReg, gyro_range_reg, error_message)) {
    return false;
  }

  accel_lsb_per_g_ = accel_lsb_per_g(params_.acc_range_g);
  gyro_lsb_per_dps_ = gyro_lsb_per_dps(params_.gyro_range_dps);
  return true;
}

double Imu660raDriver::accel_lsb_per_g(int range_g)
{
  switch (range_g) {
    case 2:
      return 16384.0;
    case 4:
      return 8192.0;
    case 8:
      return 4096.0;
    case 16:
      return 2048.0;
    default:
      return 4096.0;
  }
}

double Imu660raDriver::gyro_lsb_per_dps(int range_dps)
{
  switch (range_dps) {
    case 125:
      return 262.4;
    case 250:
      return 131.2;
    case 500:
      return 65.6;
    case 1000:
      return 32.8;
    case 2000:
      return 16.4;
    default:
      return 16.4;
  }
}

bool Imu660raDriver::use_spi() const
{
  std::string transport = params_.transport;
  std::transform(
    transport.begin(), transport.end(), transport.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return transport == "spi";
}

bool Imu660raDriver::use_i2c() const
{
  std::string transport = params_.transport;
  std::transform(
    transport.begin(), transport.end(), transport.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return transport == "i2c";
}

}  // namespace imu660ra_ros2
