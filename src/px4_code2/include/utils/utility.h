//
// Created by jbs on 21. 1. 22..
//

#ifndef PX4_CODE2_UTILITY_H
#define PX4_CODE2_UTILITY_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped convertTo(const nav_msgs::Odometry& odom);



#endif //PX4_CODE2_UTILITY_H
