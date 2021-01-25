//
// Created by jbs on 21. 1. 22..
//

#ifndef PX4_CODE2_SERVER_H
#define PX4_CODE2_SERVER_H

#include <mavros_msgs/State.h>
#include <px4_code2/Takeoff.h>
#include <px4_code2/Lock.h>
#include <px4_code2/Land.h>
#include <px4_code2/phase.h>

#include <string>
#include <utils/utility.h>

#define takeoffServiceName "/px4_code/takeoff"
#define landServiceName "/px4_code/land"
#define  lockServiceName "/px4_code/lock"
#define  phaseTopicName "/px4_code/phase"

#define speedToAir 0.1
#define speedToLand 0.08
#define landHeight -0.6

using namespace std;

namespace px4_code2{

    std::string phaseToString(Phase phase);
    Phase stringToPhase(std::string phase_);
    class Server{
        struct Status{
            bool isPoseReceived = false;
            bool isInit = false;
            bool isMissionReceived = false;
            Phase phase = NOT_INIT;
            Mission curMission;
        };

        struct State{
            geometry_msgs::PoseStamped curPose; // This pose is from world frame
            geometry_msgs::PoseStamped curSetPose;
            geometry_msgs::PoseStamped lastMissionPose;
        };

        struct Param{
            string labelServer = "[[ px4_code/server of ";
            string droneName;
            string getLabel() {return labelServer + droneName + " ]] ";};
            string worldFrame = "/map";

        };

        struct SubscriberSet{
            ros::Subscriber subOdom;
            ros::Subscriber subMavrosState;

        };
        struct PublisherSet{
            ros::Publisher pubSetPose;
            ros::Publisher pubVisionPose;
            ros::Publisher pubMissionPath;
            ros::Publisher pubPhase;
            ros::Publisher pubLastMissionPose;
        };
        struct ServerSet{
            ros::ServiceServer takeoffServer;
            ros::ServiceServer lockServer;
            ros::ServiceServer landServer;
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
                state.curSetPose = state.curPose;
                ROS_INFO_STREAM(param.droneName << " : setting planning pose to current position [ " <<
                state.curSetPose.pose.position.x <<" , " <<
                state.curSetPose.pose.position.y <<" , " <<
                state.curSetPose.pose.position.z <<" ] "
                );

        }

        void callbackOdom(const nav_msgs::OdometryConstPtr & msgPtr);
        void callbackMavrosState(const mavros_msgs::StateConstPtr & msgPtr);

        bool callbackTakeoff(px4_code2::TakeoffRequest & req, px4_code2::TakeoffResponse & resp);
        bool callbackLand(px4_code2::LandRequest & req, px4_code2::LandResponse & resp);
        bool callbackLock(px4_code2::LockRequest& req, px4_code2::LockResponse& resp);

        void publish();

    public:
        Server();
        void run();


    };



}



#endif //PX4_CODE2_SERVER_CPP_H
