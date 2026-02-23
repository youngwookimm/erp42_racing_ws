#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "lla_utm_converter.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("lla_utm_node");

  double at  = 6378137.0; 
  double fla = 1.0/298.257223563; 
  double k0  = 0.9996;

  int zone = 0;
  std::string hemisphere;

  node->declare_parameter<int>("utm_zone", 0);
  node->declare_parameter<std::string>("hemisphere", "North");

  node->get_parameter("utm_zone", zone);
  node->get_parameter("hemisphere", hemisphere);

  if (zone == 0) {
    RCLCPP_ERROR(node->get_logger(), "Error! You must specify the utm zone.");
    rclcpp::shutdown();
    return 0;
  }
  RCLCPP_INFO(node->get_logger(), "UTM Zone: %d, Hemisphere: %s", zone, hemisphere.c_str());

  ULConverter con(hemisphere, zone, at, fla, k0);

  con.RegiHandle(node);

  auto sub_left = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/utm_pose/left", 10, std::bind(&ULConverter::LeftPoseCallback, &con, std::placeholders::_1));
  auto sub_right = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/utm_pose/right", 10, std::bind(&ULConverter::RightPoseCallback, &con, std::placeholders::_1));

  auto sub2 = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10, std::bind(&ULConverter::GPSCallback, &con, std::placeholders::_1));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}