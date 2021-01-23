//
// Created by jbs on 21. 1. 22..
//

#ifndef PX4_CODE2_UTILITY_H
#define PX4_CODE2_UTILITY_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_gen/TrajGen.hpp>

const int TRAJGEN_DIM = 3;
typedef trajgen::FixPin<double,TRAJGEN_DIM> FixPin;
typedef trajgen::Vector<double,TRAJGEN_DIM> TrajVector;
typedef trajgen::PolyTrajGen<double,TRAJGEN_DIM> TrajGenObj;

geometry_msgs::PoseStamped convertTo(const nav_msgs::Odometry& odom);
namespace px4_code2 {

    struct Trajectory {
    private:
        std::vector<double> ts; // start from 0
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        std::vector<double> yaws; // TODO
    public:
        Trajectory() = default;
        Trajectory(TrajGenObj* trajPtr, double duration);

    };

    struct Mission {
        void loadTrajectory(const Trajectory& traj  );
        ros::Time triggerTime;



    };
}
#endif //PX4_CODE2_UTILITY_H
