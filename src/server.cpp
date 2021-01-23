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
        ROS_ERROR_STREAM( param.getLabel() + ": received odom but no tf to global frame " << param.worldFrame );
    }

}

bool Server::callbackTakeoff(px4_code2::TakeoffRequest &req, px4_code2::TakeoffResponse &resp) {

    if (status.isInit) {

        // Takeoff parameters
        double height = req.height;
        double speedToAir = 0.1;
        double takeoffTime = height / speedToAir;
        trajgen::time_knots<double> ts{0, takeoffTime};

        // Initial translation
        FixPin x0(0.0, 0 ,TrajVector(state.curPose.pose.position.x,
                                  state.curPose.pose.position.y,
                                  state.curPose.pose.position.z
                                  ));
        FixPin xdot0(0.0, 1 ,TrajVector(0,0,0));
        FixPin xddot0(0.0, 2 ,TrajVector(0,0,0));

        // Final translation
        FixPin xf(takeoffTime, 0 ,TrajVector(state.curPose.pose.position.x,
                                     state.curPose.pose.position.y,
                                     state.curPose.pose.position.z
        ));
        FixPin xdotf(takeoffTime, 1 ,TrajVector(0,0,0));
        FixPin xddotf(takeoffTime, 2 ,TrajVector(0,0,0));

        // Solve trajectory
        std::vector<FixPin*> pinSet{&x0,&xdot0,&xddot0,&xf,&xdotf,&xddotf};
        int polyOrder = 5;
        trajgen::PolyParam pp(polyOrder,2,trajgen::ALGORITHM::POLY_COEFF);
        TrajGenObj trajGenObj(ts,pp); trajGenObj.solve(false);

        // Upload mission
        Trajectory takeoffTraj(&trajGenObj,takeoffTime);
        status.curMission.loadTrajectory(takeoffTraj);
        status.curMission.triggerTime = ros::Time::now();
        status.isMissionReceived = true;

        return true;
    }else{
        resp.is_success = false;
        return false;
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
