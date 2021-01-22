#include <px4_code2/server.h>

using namespace px4_code2;

void Server::callbackOdom(const nav_msgs::OdometryConstPtr& msgPtr) {
    try {
        status.isPoseReceived = true;
        // Transform
        geometry_msgs::PoseStamped pose = convertTo(*msgPtr);
        geometry_msgs::PoseStamped poseWorld;
        tfListener.transformPose(param.worldFrame, pose, state.curPose);

        if (status.phase == Phase::NOT_INIT) {
            status.isInit = true;
            status.phase = Phase::STANDBY;
            initToCurPose();
        }
    }
    catch (tf::TransformException ex){
        ROS_ERROR_STREAM( "[px4_code2 server] received odom but no tf to global frame " << param.worldFrame );
    }

}


Server::Server() : nh("~"){

    nh.param<string>("drone_name",param.droneName,"target1");
    pubSet.pubSetPose = nh.advertise<geometry_msgs::PoseStamped>("/"+param.droneName+"/mavros/setpoint_position/local",1);
    pubSet.pubVisionPose = nh.advertise<geometry_msgs::PoseStamped>("/"+param.droneName+"/mavros/vision_pose/pose",1);
    subSet.subOdom = nh.subscribe("/" + param.droneName + "/t265/odom/sample", 10, &Server::callbackOdom, this);




}

void Server::publish() {

   if (status.isPoseReceived)
       pubSet.pubVisionPose.publish(state.curPose);

    if (status.isInit)
        pubSet.pubSetPose.publish(state.curSetPose);

}


void Server::run() {

    while(ros::ok()){
        publish();
        ros::spinOnce();
        ros::Rate(50).sleep();
    }



}
