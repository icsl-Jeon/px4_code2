//
// Created by jbs on 21. 1. 22..
//

#ifndef PX4_CODE2_SERVER_H
#define PX4_CODE2_SERVER_H

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <px4_code2_msgs/Takeoff.h>
#include <px4_code2_msgs/Lock.h>
#include <px4_code2_msgs/Land.h>
#include <px4_code2_msgs/Phase.h>

#include <string>
#include <utils/utility.h>

#define takeoffServiceName "/px4_code/takeoff"
#define landServiceName "/px4_code/land"
#define  lockServiceName "/px4_code/lock"
#define armServiceName "/mavros/cmd/arming"
#define modeSwitchName "/mavros/set_mode"
#define trajFollowServiceName "/px4_code/trajectory_follow"
#define smoothStartTriggerDist 0.1
#define smoothStartSpeed 0.1


#define  phaseTopicName "/px4_code/phase"
#define px4StateTopicName "/mavros/state"


using namespace std;

namespace px4_code2{

    std::string phaseToString(Phase phase);
    Phase stringToPhase(std::string phase_);
    class Server{
        struct Status{
            bool isPoseReceived = false; // odom from external ?
            bool isMavrosPoseReceived = false; // mavros fused pose ?
            bool isInit = false;
            bool isMissionReceived = false;
            Phase phase = NOT_INIT;
            Mission curMission;
        };

        struct State{
            geometry_msgs::PoseStamped curPose; // This pose is from odom
            geometry_msgs::PoseStamped curMavrosPose; // This pose is from EKF2
            geometry_msgs::PoseStamped curSetPose;
            geometry_msgs::PoseStamped lastMissionPose;
        };

        struct Param{
            string labelServer = "[[ px4_code/server of ";
            string droneName;
            string getLabel() {return labelServer + droneName + " ]] ";};
            string worldFrame = "/map";
            double yawFromPX4toSensor = 0;


        };

        struct SubscriberSet{
            ros::Subscriber subOdom;
            ros::Subscriber subMavrosState;
            ros::Subscriber subMavrosPose;

        };
        struct PublisherSet{
            ros::Publisher pubSetPose;
            ros::Publisher pubVisionPose;
            ros::Publisher pubVisionOdom; // odom_version of vision pose
            ros::Publisher pubMissionPath;
            ros::Publisher pubPhase;
            ros::Publisher pubLastMissionPose;
        };
        struct ServerSet{
            ros::ServiceServer takeoffServer;
            ros::ServiceServer lockServer;
            ros::ServiceServer landServer;
            ros::ServiceServer trajServer;
        };

    private:

        Status status;
        State state;
        Param param;

        tf::TransformListener tfListener;
        SubscriberSet subSet;
        PublisherSet pubSet;
        ServerSet servSet;
        ros::NodeHandle nh;

        void initToCurPose(){
                state.curSetPose = state.curMavrosPose;
                ROS_INFO_STREAM(param.droneName << " : setting planning pose to current mavros position [ " <<
                state.curSetPose.pose.position.x <<" , " <<
                state.curSetPose.pose.position.y <<" , " <<
                state.curSetPose.pose.position.z <<" ] "
                );

        }

        void callbackOdom(const nav_msgs::OdometryConstPtr & msgPtr);
        void callbackMavrosPose(const geometry_msgs::PoseStampedConstPtr& msgPtr);
        void callbackMavrosState(const mavros_msgs::StateConstPtr & msgPtr);

        bool callbackTakeoff(px4_code2_msgs::TakeoffRequest & req, px4_code2_msgs::TakeoffResponse & resp);
        bool callbackLand(px4_code2_msgs::LandRequest & req, px4_code2_msgs::LandResponse & resp);
        bool callbackLock(px4_code2_msgs::LockRequest& req, px4_code2_msgs::LockResponse& resp);
        bool callbackTraj(px4_code2_msgs::UploadTrajectoryRequest& req, px4_code2_msgs::UploadTrajectoryResponse& resp);

        void publish();

    public:
        Server();
        void run();


    };



}



#endif //PX4_CODE2_SERVER_CPP_H
