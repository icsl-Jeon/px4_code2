//
// Created by jbs on 21. 1. 22..
//
#include <utils/utility.h>


geometry_msgs::PoseStamped convertTo(const nav_msgs::Odometry& odom){
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.position = odom.pose.pose.position;
    poseStamped.pose.orientation = odom.pose.pose.orientation;
    poseStamped.header = odom.header;
    return poseStamped;
}
