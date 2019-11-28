//
// Created by linsin on 29/09/2019.
//

#include "gps-to-enu/gps_init.h"
#include "util/util.hpp"

Eigen::Vector3d GpsInit::lla_origin_;
bool GpsInit::system_init_;
nav_msgs::Odometry GpsInit::GpsToOdom(const nav_msgs::Odometry& gps_data) {
  Eigen::Vector3d lla_matrix = GpsMsg2Eigen(gps_data);
  Eigen::Vector3d ecef_matrix = LLA2ECEF(lla_matrix);
  Eigen::Vector3d enu_matrix = ECEF2ENU(ecef_matrix);
  nav_msgs::Odometry res_odom;
  res_odom.header = gps_data.header;
  res_odom.pose.pose.position.x = enu_matrix(0, 0);
  res_odom.pose.pose.position.y = enu_matrix(1, 0);
  res_odom.pose.pose.position.z = enu_matrix(2, 0);
  res_odom.pose.pose.orientation = gps_data.pose.pose.orientation;
  res_odom.pose.pose.orientation.w = -res_odom.pose.pose.orientation.w;

  if (!system_init_) {
    lla_origin_ = lla_matrix;
    system_init_ = 1;
    std::cout << system_init_ << std::endl;
  }
  return res_odom;
}

Eigen::Vector3d GpsInit::LLA2ECEF(const Eigen::Vector3d& lla) {
  Eigen::Vector3d ecef;
  double lat = deg2rad(lla.x());
  double lon = deg2rad(lla.y());
  double alt = lla.z();
  double earth_r = pow(EARTH_MAJOR, 2) / sqrt(pow(EARTH_MAJOR * cos(lat), 2) +
                                              pow(EARTH_MINOR * sin(lat), 2));
  ecef.x() = (earth_r + alt) * cos(lat) * cos(lon);
  ecef.y() = (earth_r + alt) * cos(lat) * sin(lon);
  ecef.z() = (pow(EARTH_MINOR / EARTH_MAJOR, 2) * earth_r + alt) * sin(lat);
  return ecef;
}

Eigen::Vector3d GpsInit::ECEF2ENU(const Eigen::Vector3d& ecef) {
  double lat = deg2rad(lla_origin_.x());
  double lon = deg2rad(lla_origin_.y());
  Eigen::Vector3d t = -LLA2ECEF(lla_origin_);
  Eigen::Matrix3d r;
  r << -sin(lon), cos(lon), 0, -cos(lon) * sin(lat), -sin(lat) * sin(lon),
      cos(lat), cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat);

  Eigen::Vector3d enu;
  enu = ecef + t;
  enu = r * enu;
  return enu;
}

Eigen::Vector3d GpsInit::GpsMsg2Eigen(const nav_msgs::Odometry& gps_msgs) {
  // x is lat , y is long , z is alt
  Eigen::Vector3d lla(gps_msgs.pose.pose.position.x,
                      gps_msgs.pose.pose.position.y,
                      gps_msgs.pose.pose.position.z);
  return lla;
}
