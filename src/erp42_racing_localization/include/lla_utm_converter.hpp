#ifndef GEO_TOLLA_HPP_
#define GEO_TOLLA_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <GeographicLib/TransverseMercator.hpp>

enum Hemi {NorthH, SouthH};

class ULConverter {
public:
    ULConverter(std::string hemi, int zone, double at, double fla, double k0); 
    
    void RegiHandle(const rclcpp::Node::SharedPtr &n);
    
    void UTMConvert2LLA(Hemi hemi, int zone, double east, double north, double height);
    void LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double altitude);
    
    void LeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgs);
    void RightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msgs);  
    void GPSCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msgs);
    
    std::vector<double> get_lla();

private:
    const double kNN_      = 0;
    const double kNS_      = 10000000.0;
    const double kE0_      = 500000.0;
    const double kPI_      = 3.14159265359;
    const double kDist_    = 1.2;

    rclcpp::Node::SharedPtr node_;

    std::vector<double> lla_;
    std::vector<double> utm_;
    
    GeographicLib::TransverseMercator tm_;  

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr left_gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr right_gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr xyz_pub_;


    size_t tmp_counter = 0;
    double tmp_x, tmp_y, tmp_z;
    std::vector<double> his_x, his_y, his_z;
    int zone_;
    std::string hemi_;
};

#endif