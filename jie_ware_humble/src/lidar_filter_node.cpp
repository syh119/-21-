#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode()
  : Node("lidar_filter_node")
  {
    const auto source_topic =
      this->declare_parameter<std::string>("source_topic", "/scan");
    const auto pub_topic =
      this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
    outlier_threshold_ =
      this->declare_parameter<double>("outlier_threshold", 0.1);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic, 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      source_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::lidarCallback, this, std::placeholders::_1));
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
  {
    if (scan->ranges.size() < 3U) {
      scan_pub_->publish(*scan);
      return;
    }

    auto filtered_scan = *scan;

    for (std::size_t i = 1; i + 1 < filtered_scan.ranges.size(); ++i) {
      const auto prev_range = filtered_scan.ranges[i - 1];
      const auto current_range = filtered_scan.ranges[i];
      const auto next_range = filtered_scan.ranges[i + 1];

      const bool current_valid =
        std::isfinite(current_range) &&
        current_range >= filtered_scan.range_min &&
        current_range <= filtered_scan.range_max;

      if (!current_valid) {
        continue;
      }

      if (std::abs(current_range - prev_range) > outlier_threshold_ &&
        std::abs(current_range - next_range) > outlier_threshold_)
      {
        filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
        if (i < filtered_scan.intensities.size()) {
          filtered_scan.intensities[i] = 0.0F;
        }
      }
    }

    scan_pub_->publish(filtered_scan);
  }

  double outlier_threshold_{0.1};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}
