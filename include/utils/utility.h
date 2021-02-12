//
// Created by jbs on 21. 1. 22..
//

#ifndef PX4_CODE2_UTILITY_H
#define PX4_CODE2_UTILITY_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_gen/TrajGen.hpp>
#include <utility>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Geometry>
#include <px4_code2/UploadTrajectory.h>
#include <iostream>

const int TRAJGEN_DIM = 3;
const int TRAJ_SAMPLE_PNT = 50;

typedef trajgen::Pin<double,TRAJGEN_DIM> Pin;
typedef trajgen::FixPin<double,TRAJGEN_DIM> FixPin;
typedef trajgen::LoosePin<double,TRAJGEN_DIM> LoosePin;
typedef trajgen::Vector<double,TRAJGEN_DIM> TrajVector;
typedef trajgen::PolyTrajGen<double,TRAJGEN_DIM> TrajGenObj;

namespace px4_code2 {


    enum Phase { NOT_INIT , STANDBY , TAKEOFF, GO_START, TRAJ_FOLLOWING, LAND };
    double interpolate(const vector<double>& xData,const vector<double>& yData, const double & x,bool extrapolate);
    double getYaw (const geometry_msgs::Quaternion& q);
    geometry_msgs::PoseStamped convertTo(const nav_msgs::Odometry& odom);
    nav_msgs::Path convertTo (const vector<geometry_msgs::Point>& points);
    double distance(const geometry_msgs::Point& pnt1, const geometry_msgs::Point& pnt2);

    struct Trajectory {
        std::vector<double> ts; // start from 0
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        std::vector<double> yaws;
        Trajectory() = default;
        Trajectory(TrajGenObj* trajPtr, double fixedYaw, double duration);
        Trajectory(TrajGenObj* trajPtr, double yaw0, double yawf, double duration);
        Trajectory(TrajGenObj* trajPtr, double duration);  // align yaw to tangential
        Trajectory(const px4_code2::UploadTrajectoryRequest& req);
        Trajectory(string fileDir,bool& isLoaded);
        nav_msgs::Path getPath (string frameId);
        void append (const Trajectory& otherTraj);
        bool writeTxt(string fileDir);

    };

    struct Mission {
    private:
        Trajectory missionTraj;
        ros::Time triggerTime;
        double emittedDuration = 0;
        double lastEmitTime = 0 ;
        nav_msgs::Path path;
        bool isActive = false;
        geometry_msgs::PoseStamped getCurDesPose(double t);
        Phase type;
        geometry_msgs::PoseStamped lastEmitPose;

    public:
        std::string frame;
        Mission() = default;
        Mission(Phase type_) : type(type_) {};
        Mission(std::string frameId,Phase type_) : frame(std::move(frameId)), type(type_) {};
        void loadTrajectory(const Trajectory& traj  );
        void trigger() {triggerTime = ros::Time::now(); isActive = true;};
        void stop() {isActive = false; emittedDuration = lastEmitTime;};
        bool isTriggered() const {return isActive;};
        Phase getType() {return type;}

        geometry_msgs::PoseStamped emitDesPose(ros::Time t ) ;
        nav_msgs::Path getCurPath () {return path; };

    };
}
#endif //PX4_CODE2_UTILITY_H
