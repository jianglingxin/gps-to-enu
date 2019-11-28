//
// Created by linsin on 29/09/2019.
//

#ifndef UNITY_BUILD_CAR_INCLUDE_UNITY_BUILD_CAR_GPS_INIT_H_
#define UNITY_BUILD_CAR_INCLUDE_UNITY_BUILD_CAR_GPS_INIT_H_

#include <nav_msgs/Odometry.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0            ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.3142        ///< WGS84 MINOR AXIS


class GpsInit {
 public:
  static nav_msgs::Odometry GpsToOdom(const nav_msgs::Odometry& gps_data);
 private:
  static Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla);
  static Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &ecef);
  static Eigen::Vector3d GpsMsg2Eigen(const nav_msgs::Odometry& gps_msgs);

  static Eigen::Vector3d lla_origin_;
  static bool system_init_;
};



#endif //UNITY_BUILD_CAR_INCLUDE_UNITY_BUILD_CAR_GPS_INIT_H_
