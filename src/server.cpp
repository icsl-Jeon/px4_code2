#include <px4_code2/server.h>


namespace px4_code2 {

    std::string phaseToString(px4_code2::Phase phase_) {
        if (phase_ == NOT_INIT)
            return "NOT_INIT";
        if (phase_ == STANDBY)
            return "STANDBY";
        if (phase_ == TAKEOFF)
            return "TAKEOFF";
        if (phase_ == GO_START)
            return "GO_START";
        if (phase_ == TRAJ_FOLLOWING)
            return "TRAJ_FOLLOWING";
        if (phase_ == LAND)
            return "LAND";

    }

    px4_code2::Phase stringToPhase(std::string phase_) {

        if (phase_ == "NOT_INIT")
            return NOT_INIT;
        if (phase_ == "STANDBY")
            return STANDBY;
        if (phase_ == "TAKEOFF")
            return TAKEOFF;
        if (phase_ == "GO_START")
            return GO_START;
        if (phase_ == "TRAJ_FOLLOWING")
            return TRAJ_FOLLOWING;
        if (phase_ == "LAND")
            return LAND;

    }


    void Server::callbackOdom(const nav_msgs::OdometryConstPtr &msgPtr) {
        try {
            status.isPoseReceived = true;
            // Transform
            geometry_msgs::PoseStamped pose = convertTo(*msgPtr);
            geometry_msgs::PoseStamped poseWorld;
            tfListener.transformPose(param.worldFrame, pose, state.curPose);

        }
        catch (tf::TransformException ex) {
            ROS_ERROR_STREAM(param.getLabel() + ": received odom but no tf to global frame " << param.worldFrame);
        }
    }
   void Server::callbackMavrosPose(const geometry_msgs::PoseStampedConstPtr *msgPtr) {
        status.isMavrosPoseReceived = true;

       if (status.phase == Phase::NOT_INIT and status.isPoseReceived) {
           ROS_INFO_STREAM_ONCE(param.labelServer + " : the fused pose from px4 have arrived.");
           initToCurPose();
           status.isInit = true;
           status.phase = Phase::STANDBY;
       }

   }




    void Server::callbackMavrosState(const mavros_msgs::StateConstPtr &msgPtr) {

        if (msgPtr->mode == "OFFBOARD") {
            ROS_INFO_STREAM_ONCE(param.getLabel() + " : Offboard confirmed. Ready to fly!");
        }

    }

    bool Server::callbackTakeoff(px4_code2::TakeoffRequest &req, px4_code2::TakeoffResponse &resp) {

        if (status.isInit) {

            // Takeoff parameters
            double height = req.height;
            double speedToAir = req.speed;
            double takeoffTime = height / speedToAir;
            trajgen::time_knots<double> ts{0, takeoffTime};
            double yaw = getYaw(state.curMavrosPose.pose.orientation);

            // Initial translation
            FixPin x0(0.0, 0, TrajVector(state.curMavrosPose.pose.position.x,
                                         state.curMavrosPose.pose.position.y,
                                         state.curMavrosPose.pose.position.z
            ));
            FixPin xdot0(0.0, 1, TrajVector(0, 0, 0));
            FixPin xddot0(0.0, 2, TrajVector(0, 0, 0));

            // Final translation
            FixPin xf(takeoffTime, 0, TrajVector(state.curMavrosPose.pose.position.x,
                                                 state.curMavrosPose.pose.position.y,
                                                 req.height
            ));
            FixPin xdotf(takeoffTime, 1, TrajVector(0, 0, 0));
            FixPin xddotf(takeoffTime, 2, TrajVector(0, 0, 0));

            // Solve trajectory
            std::vector<Pin *> pinSet{&x0, &xdot0, &xddot0, &xf, &xdotf, &xddotf};
            int polyOrder = 5;
            trajgen::PolyParam pp(polyOrder, 2, trajgen::ALGORITHM::POLY_COEFF);
            TrajGenObj trajGenObj(ts, pp);
            trajGenObj.setDerivativeObj(TrajVector(0, 1, 1));
            trajGenObj.addPinSet(pinSet);

            // Upload mission
            if (trajGenObj.solve(false)) {
                Trajectory takeoffTraj(&trajGenObj, yaw, takeoffTime);
                status.curMission = Mission(param.worldFrame,Phase::TAKEOFF);
                status.curMission.loadTrajectory(takeoffTraj);
                status.curMission.trigger();
                status.isMissionReceived = true;
                status.phase =TAKEOFF;

                ROS_INFO_STREAM(param.getLabel() + " : uploaded takeoff trajectory");
                resp.is_success = true;
                return true;
            } else {
                ROS_WARN_STREAM(param.getLabel() + " : failed to compute takeoff trajectory");
                resp.is_success = false;
                return false;
            }
        } else {
            ROS_WARN_STREAM(param.getLabel() + " : still the odom topic not received");
            resp.is_success = false;
            return false;
        }
    }

    bool Server::callbackLock(px4_code2::LockRequest &req, px4_code2::LockResponse &resp) {

        initToCurPose();

        // Drone is performing mission unless standby. We are interrupting
        if (status.phase != Phase::STANDBY) {
            status.curMission.stop();
            status.phase = Phase::STANDBY;
            resp.mission_remain = false; // STANDBY
        }else
         // In this case, already locked. As mission remains, we resume
        if (status.isMissionReceived) {
            status.curMission.trigger();
            status.phase = status.curMission.getType();
            resp.mission_remain = true; // RESUME
        }else{ // STANDBY and No mission.
            resp.mission_remain = false;
        }
        resp.is_success = true;

        return true;
    }
    bool Server::callbackLand (px4_code2::LandRequest &req, px4_code2::LandResponse &resp) {
        if (status.isInit){

            // compute trajectory to land ( z= 0 )
            double height = state.curMavrosPose.pose.position.z;
            double speedToLand = req.speed; double landHeight = req.ground;
            double landTime= (height - landHeight) / speedToLand;
            trajgen::time_knots<double> ts{0,landTime};
            double yaw = getYaw(state.curMavrosPose.pose.orientation);

            // Initial translation
            FixPin x0(0.0, 0, TrajVector(state.curMavrosPose.pose.position.x,
                                         state.curMavrosPose.pose.position.y,
                                         state.curMavrosPose.pose.position.z
            ));
            FixPin xdot0(0.0, 1, TrajVector(0, 0, 0));
            FixPin xddot0(0.0, 2, TrajVector(0, 0, 0));

            // Final translation
            FixPin xf(landTime, 0, TrajVector(state.curMavrosPose.pose.position.x,
                                                 state.curMavrosPose.pose.position.y,landHeight));
            FixPin xdotf(landTime, 1, TrajVector(0, 0, 0));
            FixPin xddotf(landTime, 2, TrajVector(0, 0, 0));

            // Solve trajectory
            std::vector<Pin *> pinSet{&x0, &xdot0, &xddot0, &xf, &xdotf, &xddotf};
            int polyOrder = 5;
            trajgen::PolyParam pp(polyOrder, 2, trajgen::ALGORITHM::POLY_COEFF);
            TrajGenObj trajGenObj(ts, pp);
            trajGenObj.setDerivativeObj(TrajVector(0, 1, 1));
            trajGenObj.addPinSet(pinSet);


            // Upload mission
            if (trajGenObj.solve(false)) {
                Trajectory landTraj(&trajGenObj, yaw,landTime);
                status.curMission = Mission(param.worldFrame,Phase::LAND);
                status.curMission.loadTrajectory(landTraj);
                status.curMission.trigger();
                status.isMissionReceived = true;
                status.phase = Phase::LAND;

                ROS_INFO_STREAM(param.getLabel() + " : uploaded landing trajectory");
                resp.is_success = true;
                return true;
            } else {
                ROS_WARN_STREAM(param.getLabel() + " : failed to compute landing trajectory");
                resp.is_success = false;
                return false;
            }


        }else{
            resp.is_success = false;
            return false;
        }


    }

    Server::Server() : nh("~") {
        // TOPIC
        nh.param<string>("drone_name", param.droneName, "target1");
        pubSet.pubSetPose = nh.advertise<geometry_msgs::PoseStamped>(
                "/" + param.droneName + "/mavros/setpoint_position/local", 1);
        pubSet.pubMissionPath = nh.advertise<nav_msgs::Path>("/" + param.droneName + "/px4_code/mission_path", 1);
        pubSet.pubVisionPose = nh.advertise<geometry_msgs::PoseStamped>(
                "/" + param.droneName + "/mavros/vision_pose/pose", 1);
        pubSet.pubPhase = nh.advertise<px4_code2::phase>("/" + param.droneName + "/px4_code/phase", 1);
        pubSet.pubLastMissionPose = nh.advertise<geometry_msgs::PoseStamped>("/"+param.droneName +"/px4_code/last_mission_pose",1);

        subSet.subOdom = nh.subscribe("/" + param.droneName + "/t265/odom/sample", 10, &Server::callbackOdom, this);
        subSet.subMavrosState = nh.subscribe("/" + param.droneName + "/mavros/state", 10, &Server::callbackMavrosState,
                                             this);


        // SERVICE
        servSet.takeoffServer = nh.advertiseService("/" + param.droneName + takeoffServiceName,
                                                    &Server::callbackTakeoff, this);
        servSet.lockServer = nh.advertiseService("/" + param.droneName + lockServiceName,&Server::callbackLock,this);
        servSet.landServer = nh.advertiseService("/"+param.droneName + landServiceName,&Server::callbackLand,this);

    }

    void Server::publish() {
        // Phase
        phase phaseMsg;
        phaseMsg.phase.data = phaseToString(status.phase);
        phaseMsg.isMissionExist = status.isMissionReceived;
        pubSet.pubPhase.publish(phaseMsg);

        // Republish odom
        if (status.isPoseReceived)
            pubSet.pubVisionPose.publish(state.curPose);


        if (status.isMissionReceived) {

            // show current mission
            pubSet.pubMissionPath.publish(status.curMission.getCurPath());

            // mission execution
            if (status.curMission.isTriggered()) {
                state.curSetPose = status.curMission.emitDesPose(ros::Time::now());
                state.lastMissionPose = state.curSetPose;
            }
            pubSet.pubLastMissionPose.publish(state.lastMissionPose);
        }

        if (status.isInit) {
            pubSet.pubSetPose.publish(state.curSetPose);
        }

    }


    void Server::run() {

        while (ros::ok()) {

            publish();
            ros::spinOnce();
            ros::Rate(50).sleep();
        }


    }
}