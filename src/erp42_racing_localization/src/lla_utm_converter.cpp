#include "lla_utm_converter.hpp"

using namespace std;

ULConverter::ULConverter(string hemi, int zone, double at, double fla, double k0)
: tm_(at, fla, k0), zone_(zone), hemi_(hemi) {}

struct basic {
  double bx = 0.0; 
  double by = 0.0;
  double bz = 0.0; 
};

void ULConverter::RegiHandle(const rclcpp::Node::SharedPtr &n)
{
  left_gps_pub_  = n->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/left", 10);
  right_gps_pub_ = n->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/right", 10);
  xyz_pub_ = n->create_publisher<geometry_msgs::msg::PoseStamped>("/utm", 10);
  
  node_ = n; 
}

void ULConverter::LeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgs)
{ 
  basic bs;
  sensor_msgs::msg::NavSatFix gps;
  gps.header = msgs->header;

  double px = -bs.bx + msgs->pose.position.x;
  double py = -bs.by + msgs->pose.position.y;
  double pz = -bs.bz + msgs->pose.position.z;

  RCLCPP_INFO(node_->get_logger(), "Get data x: [%f]", px);
  RCLCPP_INFO(node_->get_logger(), "Get data y: [%f]", py);
  RCLCPP_INFO(node_->get_logger(), "Get data z: [%f]", pz);
  
  if(hemi_ == "North")
    UTMConvert2LLA(NorthH, zone_, px, py, pz);
  else if(hemi_ == "South")
    UTMConvert2LLA(SouthH, zone_, px, py, pz);
  else {
    RCLCPP_ERROR(node_->get_logger(), "only North or South Hemisphere. Are you on Mars???");
    rclcpp::shutdown();
  }
    
  gps.latitude  = lla_[0];
  gps.longitude = lla_[1];
  gps.altitude  = lla_[2];
  
  RCLCPP_INFO(node_->get_logger(), "Convert to latitude: [%f]", lla_[0]);
  RCLCPP_INFO(node_->get_logger(), "Convert to longitude: [%f]", lla_[1]);
  RCLCPP_INFO(node_->get_logger(), "Convert to altitude: [%f]", lla_[2]);
  
  left_gps_pub_->publish(gps);
  lla_.clear(); 
}
void ULConverter::RightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgs)
{ 
  basic bs;
  sensor_msgs::msg::NavSatFix gps;
  gps.header = msgs->header;

  double px = -bs.bx + msgs->pose.position.x;
  double py = -bs.by + msgs->pose.position.y;
  double pz = -bs.bz + msgs->pose.position.z;

  RCLCPP_INFO(node_->get_logger(), "Get data x: [%f]", px);
  RCLCPP_INFO(node_->get_logger(), "Get data y: [%f]", py);
  RCLCPP_INFO(node_->get_logger(), "Get data z: [%f]", pz);
  
  if(hemi_ == "North")
    UTMConvert2LLA(NorthH, zone_, px, py, pz);
  else if(hemi_ == "South")
    UTMConvert2LLA(SouthH, zone_, px, py, pz);
  else {
    RCLCPP_ERROR(node_->get_logger(), "only North or South Hemisphere. Are you on Mars???");
    rclcpp::shutdown();
  }
    
  gps.latitude  = lla_[0];
  gps.longitude = lla_[1];
  gps.altitude  = lla_[2];
  
  RCLCPP_INFO(node_->get_logger(), "Convert to latitude: [%f]", lla_[0]);
  RCLCPP_INFO(node_->get_logger(), "Convert to longitude: [%f]", lla_[1]);
  RCLCPP_INFO(node_->get_logger(), "Convert to altitude: [%f]", lla_[2]);
  
  right_gps_pub_->publish(gps);
  lla_.clear(); 
}

void ULConverter::GPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msgs)
{
  basic bs;
  RCLCPP_INFO(node_->get_logger(), "Get data latitude from GPS: [%f]", msgs->latitude);
  RCLCPP_INFO(node_->get_logger(), "Get data longitude from GPS: [%f]", msgs->longitude);
  RCLCPP_INFO(node_->get_logger(), "Get data altitude from GPS: [%f]", msgs->altitude);

  if(hemi_ == "North")
    LLAConvert2UTM(NorthH, zone_, msgs->latitude, msgs->longitude, msgs->altitude);
  else if(hemi_ == "South")
    LLAConvert2UTM(SouthH, zone_, msgs->latitude, msgs->longitude, msgs->altitude);
  else {
    RCLCPP_ERROR(node_->get_logger(), "only North or South Hemisphere. Are you on Mars?");
    rclcpp::shutdown();
  }

  geometry_msgs::msg::PoseStamped local_pose;

  local_pose.header = msgs->header;
  local_pose.pose.position.x = utm_[0] + bs.bx;
  local_pose.pose.position.y = utm_[1] + bs.by;
  local_pose.pose.position.z = utm_[2] + bs.bz;

  RCLCPP_INFO(node_->get_logger(), "Convert to x: [%f]", local_pose.pose.position.x);
  RCLCPP_INFO(node_->get_logger(), "Convert to y: [%f]", local_pose.pose.position.y);
  RCLCPP_INFO(node_->get_logger(), "Convert to z: [%f]", local_pose.pose.position.z);

  xyz_pub_->publish(local_pose);

  utm_.clear();
}

void ULConverter::UTMConvert2LLA(Hemi hemi, int zone, double east, double north, double height)
{
  double latitude;
  double longitude;

  int lon0 = zone * 6 - 183;
  east  -= kE0_;
  north -= (hemi == NorthH) ? kNN_ : kNS_; 

  tm_.Reverse(lon0, east, north, latitude, longitude); 

  lla_.clear();
  lla_.push_back(latitude); 
  lla_.push_back(longitude); 
  lla_.push_back(height);
}

void ULConverter::LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double height)
{
  double east, north;

  int lon0 = zone * 6 - 183;

  tm_.Forward(lon0, latitude, longitude, east, north);

  east += kE0_;
  north += (hemi == NorthH) ? kNN_ : kNS_;

  utm_.clear();
  utm_.push_back(east); 
  utm_.push_back(north); 
  utm_.push_back(height);
}

std::vector<double> ULConverter::get_lla()
{
  return lla_;
}