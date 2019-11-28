//
// Created by linsin on 30/09/2019.
//

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "gps-to-enu/gps_init.h"

ros::Publisher publish_odom;
void GpsToOdomCallBack(const nav_msgs::Odometry& gps_data) {
  nav_msgs::Odometry odom_send = GpsInit::GpsToOdom(gps_data);
  publish_odom.publish(odom_send);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_to_odom");
  ros::NodeHandle nh("~");
  ros::Subscriber gps_sub =
      nh.subscribe("/cpt/gps_odom", 10, GpsToOdomCallBack);
  publish_odom = nh.advertise<nav_msgs::Odometry>("/cpt/enu_odom", 10);
  ros::spin();
}
