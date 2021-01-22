//
// Created by jbs on 21. 1. 22..
//

#ifndef PX4_CODE2_SERVER_H
#define PX4_CODE2_SERVER_H

#include <mavros_msgs/State.h>
#include <px4_code2/Takeoff.h>
#include <tf/transform_listener.h>
#include <string>
#include <utils/utility.h>

using namespace std;


namespace px4_code2{

    enum Phase { NOT_INIT , STANDBY , TAKEOFF, GO_START, TRAJ_FOLLOWING };

    class Server{
        struct Status{
            bool isPoseReceived = false;
            bool isInit = false;
            Phase phase;
        };

        struct State{
            geometry_msgs::PoseStamped curPose; // This pose is from world frame
            geometry_msgs::PoseStamped curSetPose;
        };

        struct Param{
            string droneName;
            string worldFrame = "/map";

        };

        struct Service{
            ros::ServiceServer takeoffServer;

        };
        struct SubscriberSet{
            ros::Subscriber subOdom;

        };
        struct PublisherSet{
            ros::Publisher pubSetPose;
            ros::Publisher pubVisionPose;
        };


    private:

        Status status;
        State state;
        Param param;

        tf::TransformListener tfListener;
        SubscriberSet subSet;
        PublisherSet pubSet;
        ros::NodeHandle nh;

        void initToCurPose(){
                state.curSetPose = state.curPose;
                ROS_INFO_STREAM(param.droneName << " : init to current position [ " <<
                state.curSetPose.pose.position.x <<" , " <<
                state.curSetPose.pose.position.y <<" , " <<
                state.curSetPose.pose.position.z <<" ] "
                );

        }

        void callbackOdom(const nav_msgs::OdometryConstPtr & msgPtr);
        void publish();

    public:
        Server();
        void run();


    };



}



#endif //PX4_CODE2_SERVER_CPP_H
