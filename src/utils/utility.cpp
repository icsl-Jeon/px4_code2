//
// Created by jbs on 21. 1. 22..
//
#include <utils/utility.h>

namespace px4_code2 {
    geometry_msgs::PoseStamped convertTo(const nav_msgs::Odometry &odom) {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position = odom.pose.pose.position;
        poseStamped.pose.orientation = odom.pose.pose.orientation;
        poseStamped.header = odom.header;
        return poseStamped;
    }

    Trajectory::Trajectory(TrajGenObj *trajPtr,double duration) {
        int N = 50; //TODO
        double dt = duration/N;

        for (int n = 0 ; n < N ; n++){
            double t = dt*n;
            TrajVector p = trajPtr->eval(t,0);
            ts.push_back(t);
            xs.push_back(p(0));
            ys.push_back(p(1));
            zs.push_back(p(2));
            yaws.push_back(0.0);
        }

    }

}