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
            // Transform from s1 to s2
            geometry_msgs::PoseStamped pose = convertTo(*msgPtr);

            // Tps
            Eigen::Affine3d Tps ; Tps.setIdentity();
            Eigen::Matrix3d Rps;
            double yaw = param.yawFromPX4toSensor;
            Rps << cos(yaw) , -sin(yaw) , 0 ,
                    sin(yaw), cos(yaw) ,0 ,
                    0,0,1;
            Tps.prerotate(Rps);

            // Ts1s2 (sensor odometry)
            Eigen::Quaterniond quat;
            Eigen::Vector3d transl;
            transl(0) = pose.pose.position.x;
            transl(1) = pose.pose.position.y;
            transl(2) = pose.pose.position.z;
            quat.x() =  pose.pose.orientation.x;
            quat.y() =  pose.pose.orientation.y;
            quat.z() =  pose.pose.orientation.z;
            quat.w() =  pose.pose.orientation.w;
            Eigen::Affine3d Ts1s2; Ts1s2.setIdentity(); Ts1s2.translate(transl); Ts1s2.rotate(quat);

            // Tp1p2 (px4 start to current != map frame)
            Eigen::Affine3d Tp1p2 = Tps*Ts1s2*Tps.inverse();

            geometry_msgs::PoseStamped pose_p1p2;
            Eigen::Quaterniond quatNew(Tp1p2.rotation()); quatNew.normalize();
            pose_p1p2.header.frame_id = msgPtr->header.frame_id;
            pose_p1p2.pose.position.x = Tp1p2.translation().x();
            pose_p1p2.pose.position.y = Tp1p2.translation().y();
            pose_p1p2.pose.position.z = Tp1p2.translation().z();
            pose_p1p2.pose.orientation.x = quatNew.x();
            pose_p1p2.pose.orientation.y = quatNew.y();
            pose_p1p2.pose.orientation.z = quatNew.z();
            pose_p1p2.pose.orientation.w = quatNew.w();

            // For convenience
            geometry_msgs::PoseStamped poseWorld;

            // To map frame
            tfListener.transformPose(param.worldFrame, pose_p1p2, state.curPose);


            nav_msgs::Odometry odomWorld;
            odomWorld.pose.pose = state.curPose.pose;
            odomWorld.header.frame_id = param.worldFrame;
            pubSet.pubVisionOdom.publish(odomWorld);


        }
        catch (tf::TransformException ex) {
            ROS_ERROR_STREAM(param.getLabel() + ": received odom but no tf to global frame " << param.worldFrame);
        }
    }
   void Server::callbackMavrosPose(const geometry_msgs::PoseStampedConstPtr & msgPtr) {
        status.isMavrosPoseReceived = true;
        state.curMavrosPose = *msgPtr;
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

    bool Server::callbackTakeoff(px4_code2_msgs::TakeoffRequest &req, px4_code2_msgs::TakeoffResponse &resp) {

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
            ROS_WARN_STREAM(param.getLabel() + " : still the  mavros pose topic not received");
            resp.is_success = false;
            return false;
        }
    }

    bool Server::callbackLock(px4_code2_msgs::LockRequest &req, px4_code2_msgs::LockResponse &resp) {

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
    bool Server::callbackLand (px4_code2_msgs::LandRequest &req, px4_code2_msgs::LandResponse &resp) {
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


    bool Server::callbackTraj(px4_code2_msgs::UploadTrajectoryRequest &req, px4_code2_msgs::UploadTrajectoryResponse &resp) {

        if (status.isInit) {
            // Check the distance to the start position of the traj
            geometry_msgs::Point startTraj;
            startTraj.x = req.xs.front();
            startTraj.y = req.ys.front();
            startTraj.z = req.zs.front();
            Trajectory uploadedTraj(req);

            // smooth start ...
            double distToStart = distance(state.curMavrosPose.pose.position, startTraj);
            double timeToStart = distToStart / smoothStartSpeed;
            trajgen::time_knots<double> ts{0, timeToStart};

            // Initial condition
            FixPin x0(0.0, 0, TrajVector(state.curMavrosPose.pose.position.x,
                                         state.curMavrosPose.pose.position.y,
                                         state.curMavrosPose.pose.position.z
            ));
            FixPin xdot0(0.0, 1, TrajVector(0, 0, 0));
            FixPin xddot0(0.0, 2, TrajVector(0, 0, 0));
            double yaw0 = getYaw(state.curMavrosPose.pose.orientation);

            // Final condition
            FixPin xf(timeToStart, 0, TrajVector(startTraj.x,
                                                 startTraj.y,
                                                 startTraj.z));
            FixPin xdotf(timeToStart, 1, TrajVector(0, 0, 0));
            FixPin xddotf(timeToStart, 2, TrajVector(0, 0, 0));
            double yawf = req.yaws.front();

            // Trajectory solving
            std::vector<Pin *> pinSet{&x0, &xdot0, &xddot0, &xf, &xdotf, &xddotf};
            int polyOrder = 5;
            trajgen::PolyParam pp(polyOrder, 2, trajgen::ALGORITHM::POLY_COEFF);
            TrajGenObj trajGenObj(ts, pp);
            trajGenObj.setDerivativeObj(TrajVector(0, 1, 1));
            trajGenObj.addPinSet(pinSet);

            // Upload mission
            if (trajGenObj.solve(false)){
                Trajectory smoothGo(&trajGenObj,yaw0,yawf,timeToStart);
                // concatenating smoothGo + missionTrajectory
                smoothGo.append(uploadedTraj);
                status.curMission = Mission(param.worldFrame,Phase::TRAJ_FOLLOWING);
                status.curMission.loadTrajectory(smoothGo);
                status.curMission.trigger();
                status.isMissionReceived = true;
                status.phase =TRAJ_FOLLOWING;

            }else{
                ROS_WARN_STREAM(param.getLabel() + " : failed to compute smooth go trajectory");
                return false;
            }


        }else{

            ROS_WARN_STREAM(param.getLabel() + " : still the  mavros pose topic not received");
            return false;

        }

    }

    Server::Server() : nh("~") {
        // TOPIC
        nh.param<string>("drone_name", param.droneName, "target1");
        nh.param("yaw_from_px4_to_sensor",param.yawFromPX4toSensor,0.0);
        pubSet.pubSetPose = nh.advertise<geometry_msgs::PoseStamped>(
                "/" + param.droneName + "/mavros/setpoint_position/local", 1);
        pubSet.pubMissionPath = nh.advertise<nav_msgs::Path>("/" + param.droneName + "/px4_code/mission_path", 1);
        pubSet.pubVisionPose = nh.advertise<geometry_msgs::PoseStamped>(
                "/" + param.droneName + "/mavros/vision_pose/pose", 1);


        pubSet.pubVisionOdom = nh.advertise<nav_msgs::Odometry>(
                "/" + param.droneName + "/px4_code/vision_odom", 1);

        pubSet.pubPhase = nh.advertise<px4_code2_msgs::Phase>("/" + param.droneName + "/px4_code/phase", 1);
        pubSet.pubLastMissionPose = nh.advertise<geometry_msgs::PoseStamped>("/"+param.droneName +"/px4_code/last_mission_pose",1);

        subSet.subOdom = nh.subscribe("/" + param.droneName + "/t265/odom/sample", 10, &Server::callbackOdom, this);
        subSet.subMavrosState = nh.subscribe("/" + param.droneName + "/mavros/state", 10, &Server::callbackMavrosState,
                                             this);
        subSet.subMavrosPose = nh.subscribe("/" + param.droneName + "/mavros/local_position/pose", 10, &Server::callbackMavrosPose,
                                            this);

        // SERVICE
        servSet.takeoffServer = nh.advertiseService("/" + param.droneName + takeoffServiceName,
                                                    &Server::callbackTakeoff, this);
        servSet.lockServer = nh.advertiseService("/" + param.droneName + lockServiceName,&Server::callbackLock,this);
        servSet.landServer = nh.advertiseService("/"+param.droneName + landServiceName,&Server::callbackLand,this);
        servSet.trajServer = nh.advertiseService("/" + param.droneName + trajFollowServiceName,&Server::callbackTraj,this);
    }

    void Server::publish() {
        // Phase
        px4_code2_msgs::Phase phaseMsg;
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