#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CostmapCleaner : public rclcpp::Node
{
public:
  CostmapCleaner()
  : Node("costmap_cleaner")
  {
    const auto initial_pose_topic =
      this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    const auto local_service =
      this->declare_parameter<std::string>(
      "local_clear_service",
      "/local_costmap/clear_entirely_local_costmap");
    const auto global_service =
      this->declare_parameter<std::string>(
      "global_clear_service",
      "/global_costmap/clear_entirely_global_costmap");

    local_client_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(local_service);
    global_client_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(global_service);

    initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic,
      10,
      std::bind(&CostmapCleaner::initialPoseCallback, this, std::placeholders::_1));
  }

private:
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
  {
    clearCostmap(local_client_, "local_costmap");
    clearCostmap(global_client_, "global_costmap");
  }

  void clearCostmap(
    const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & client,
    const std::string & name)
  {
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "%s clear service is not ready", name.c_str());
      return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto future = client->async_send_request(
      request,
      [this, name](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture result) {
        if (result.valid()) {
          RCLCPP_INFO(this->get_logger(), "%s cleared", name.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "%s clear request failed", name.c_str());
        }
      });

    (void)future;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapCleaner>());
  rclcpp::shutdown();
  return 0;
}
