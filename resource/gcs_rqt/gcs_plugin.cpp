//
// Created by jbs on 21. 1. 23..
//

#include "gcs_plugin.h"
#include <pluginlib/class_list_macros.h>

namespace px4_code2{
    std::string labelClient = "[[ px4_code/client ]] ";

    GcsPlugin::GcsPlugin() : rqt_gui_cpp::Plugin() , widget(new Widget){

        setObjectName("FelipeSuite");
    }

    void GcsPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
        std::cout << "init plugin" << std::endl;
        QStringList argv = context.argv();
        context.addWidget(widget);

        // Signal - slot connect
        connect(widget,SIGNAL(callService(int,std::string,std::vector<double>,int *)),
                this,SLOT(callService(int, std::string,std::vector<double>,int *)));
        connect(this,SIGNAL(enablePushButton(bool)),widget,SLOT(enableButton(bool)));
        connect(this,SIGNAL(enablePushButtonPX4(int, bool)),widget,SLOT(enableButtonPX4(int, bool)));
        connect(this,SIGNAL(updateMissionStatus(bool,bool)),widget,SLOT(updateMissionStatus(bool,bool)));
        connect(this,SIGNAL(updatePX4State(int, bool)),widget,SLOT(updatePX4Status(int, bool)));

        connect (widget,SIGNAL(toggleWaypoints(int, bool)),this,SLOT(toggleWaypoints(int, bool)));
        connect (widget,SIGNAL(eraseWaypoitns(int)),this,SLOT(eraseWaypoints(int)));
        connect (widget,SIGNAL(generateTrajectory(int,int,double,double)),this,SLOT(generateTrajectory(int,int,double,double)));
        connect (widget,SIGNAL(saveTrajectory(int,std::string)),this,SLOT(saveTrajectoryTxt(int,std::string)));
        connect (widget,SIGNAL(loadTrajectory(int,std::string)),this,SLOT(loadTrajectoryTxt(int,std::string)));
        connect (widget,SIGNAL(doTest(bool)),this,SLOT(doTest(bool)));

        // ROS initialization
        timer = nh.createTimer(ros::Duration(0.01),&GcsPlugin::callbackTimer,this);

        // 0. Slot loading
        nh = getNodeHandle();
        nh.getParam("/qt_setting_dir",param.setting_file);
        nh.getParam("/world_frame_id",param.worldFrameId);
        nh.getParam("/print_ros_warning",param.printRosWarning);
        nh.getParam("/slider_max_height",param.maxHeightSlider);
//        cout << param.maxHeightSlider << endl;
        widget->setMaxSlideValue(param.maxHeightSlider);

//        cout << param.worldFrameId << endl;
        std::cout << labelClient + " : setting file : " << param.setting_file << std::endl;
        widget->readSettings(param.setting_file);

        // 1. Drone set
        std::vector<std::string> droneNameSet;
        nh.getParam("/drone_name_set",droneNameSet);
        waypointsSet = vector<vector<geometry_msgs::Point>>(droneNameSet.size());
        trajectorySet.resize(droneNameSet.size());
        missionSet.resize(droneNameSet.size());

         int idx = 0;
        std::cout << labelClient << ": mission drones are [ ";
        for (auto it = droneNameSet.begin() ; it < droneNameSet.end() ; it++){
            // Subscribing phase
            ros::Subscriber sub = nh.subscribe<px4_code2_msgs::Phase>("/" + *it + phaseTopicName,
                                           1,boost::bind(&GcsPlugin::callbackPhase,this,_1,idx));
            subPhaseSet.push_back(sub);

            // Subscribing px4 state
            sub = nh.subscribe<mavros_msgs::State>("/" + *it + px4StateTopicName,
                                                                 1,boost::bind(&GcsPlugin::callbackPX4State,this,_1,idx));
            subPX4Set.push_back(sub);

            // Phase init
            px4_code2_msgs::Phase phase_; phase_.isMissionExist = false;
            status.phaseSet.push_back(phase_);
            status.px4StateSet.push_back(mavros_msgs::State ());
            lastCommTimePX4.push_back(ros::Time(0));


            // Publish for waypoint selection and trajectory
            ros::Publisher pub = nh.advertise<nav_msgs::Path>("/" + *it + "/waypoints",1);
            pubWaypointsSet.push_back(pub);
            pub = nh.advertise<nav_msgs::Path>("/" + *it + "/trajectory",1);
            pubTrajSet.push_back(pub);
            trajectorySet[idx].first = false; // init with invalid trajectory
            missionSet[idx].first = false;

            // Publish for testing
            pub = nh.advertise<geometry_msgs::PoseStamped>("/"+*it + "/testing_pose",1);
            pubTestPoseSet.push_back(pub);

            // Printing
            std::cout << *it ;
            if (it != droneNameSet.end()-1)
                std::cout << ", ";
            idx++;
        }
        std::cout <<  " ]"<<std::endl;
        param.droneNameSet = droneNameSet;
        subWaypoints = nh.subscribe("waypoints",1,&GcsPlugin::callbackWaypoint,this);
        widget->initNames(droneNameSet);
    }


    void GcsPlugin::shutdownPlugin()
    {
        // Save slot values
        ROS_INFO_STREAM(labelClient + " : writing config file to " << param.setting_file );
        widget->writeSettings(param.setting_file);
        // unregister all publishers here
    }

    void GcsPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                      qt_gui_cpp::Settings& instance_settings) const
    {

        // instance_settings.setValue(k, v)
    }

    void GcsPlugin::restoreSettings(
            const qt_gui_cpp::Settings& plugin_settings,
            const qt_gui_cpp::Settings& instance_settings)
    {
        // v = instance_settings.value(k)
    }

    GcsPlugin::~GcsPlugin()  {};

    void GcsPlugin::callService(int droneId, std::string service,std::vector<double> args,int* out) {

        if (droneId < param.getNdrone()) {
            std::cout << labelClient << " : for drone " << param.droneNameSet[droneId] <<
                      ", service " << service << " is called." << std::endl;

            if (service == "takeoff") {
                px4_code2_msgs::Takeoff takeoffSrv;
                takeoffSrv.request.height = args[0];
                double takeoffSpeed = args[1];
                if (takeoffSpeed<= 0){
                    ROS_WARN_STREAM(labelClient << " : takeoff speed is below zero. resetting 0.05 [m/s]" );
                    takeoffSpeed = 0.05;
                }

                takeoffSrv.request.speed=takeoffSpeed;
                ros::service::call<px4_code2_msgs::Takeoff>("/" + param.droneNameSet[droneId] + takeoffServiceName,
                                                       takeoffSrv);
            }
            if (service == "lock") {
                px4_code2_msgs::Lock lockSrv;
                ros::service::call<px4_code2_msgs::Lock>("/" + param.droneNameSet[droneId] +lockServiceName,
                                                    lockSrv);
                // we should fill output
                if (out != NULL){
                    *out = lockSrv.response.mission_remain;
                }

            }
            if (service == "land") {
                px4_code2_msgs::Land landSrv;
                double landGround = args[0];  double landSpeed = args[1];
                if (landGround > 0)
                    ROS_WARN_STREAM(labelClient << " : landing ground is larger than zero." );
                if (landSpeed <= 0){
                    ROS_WARN_STREAM(labelClient << " : landing speed is below zero. resetting 0.05 [m/s]" );
                    landSpeed = 0.05;
                }

                landSrv.request.ground = landGround; landSrv.request.speed = landSpeed;
                ros::service::call<px4_code2_msgs::Land>("/" + param.droneNameSet[droneId] + landServiceName,
                                                    landSrv);
            }

            if (service == "arm"){
                mavros_msgs::CommandBool cmdSrv;
                if (args[0] == 0) { // not armed. This service is arm
                    cmdSrv.request.value = true;
                    ros::service::call<mavros_msgs::CommandBool>("/" + param.droneNameSet[droneId] + armServiceName,cmdSrv);
                }else{ // armed. This service is disarm
                    cmdSrv.request.value = false;
                    ros::service::call<mavros_msgs::CommandBool>("/" + param.droneNameSet[droneId] + armServiceName,cmdSrv);
                }
            }

            if (service == "offboard"){
                mavros_msgs::SetMode modeSrv;

                if (args[0] == 0) { // not offboard. This service is to offboard
                    modeSrv.request.custom_mode= "OFFBOARD";
                    ros::service::call<mavros_msgs::SetMode>("/" + param.droneNameSet[droneId] + modeSwitchName,modeSrv);
                }else{ // offboard. This service is to manual
                    modeSrv.request.custom_mode = "MANUAL";
                    ros::service::call<mavros_msgs::SetMode>("/" + param.droneNameSet[droneId] + modeSwitchName,modeSrv);
                }
            }

            if (service == "trajectory_follow"){
                px4_code2_msgs::UploadTrajectory trajSrv;
                bool isValid = trajectorySet[droneId].first;
                Trajectory traj = trajectorySet[droneId].second;

                if (isValid){
                    trajSrv.request.ts = traj.ts;
                    trajSrv.request.xs = traj.xs;
                    trajSrv.request.ys = traj.ys;
                    trajSrv.request.zs = traj.zs;
                    trajSrv.request.yaws = traj.yaws;
                    ros::service::call<px4_code2_msgs::UploadTrajectory>("/"+ param.droneNameSet[droneId] + trajFollowServiceName ,trajSrv);
                }else{
                    widget->writeMakise("No valid trajectory for service call");
                }



            }
        }
    }
    void GcsPlugin::toggleWaypoints(int droneId, bool startListen) {
        if (startListen) {
            status.isListenWaypoints = true;
            status.curWaypointTarget = droneId;
            waypointsSet[droneId].clear();
            ROS_INFO_STREAM(labelClient+ " : Start listening waypoints for drone " +
            to_string(droneId) + " . Initialize waypoints set");
        }else{
           status.isListenWaypoints = false;
            ROS_INFO_STREAM(labelClient+ " : Finish listening waypoints.");
        }
    }

    void GcsPlugin::callbackWaypoint(const geometry_msgs::PoseStampedPtr &msgPtr) {
        if (status.isListenWaypoints) {
            geometry_msgs::Point curWaypoint;
            curWaypoint.x = msgPtr->pose.position.x;
            curWaypoint.y = msgPtr->pose.position.y;
            curWaypoint.z = widget->getSliderValue() * param.maxHeightSlider;
            waypointsSet[status.curWaypointTarget].push_back(curWaypoint);
            string msg = "Received points [" + to_string(curWaypoint.x) + " , "+
                    to_string(curWaypoint.y) + " , "+ to_string(curWaypoint.z) + "]";
            widget->writeMakise(msg);
        }
    }
    void GcsPlugin::eraseWaypoints(int droneId) {
        if (not waypointsSet[droneId].empty()) {
            widget->writeMakise("Erased last waypoint.");
            waypointsSet[droneId].pop_back();
        }else{

            widget->writeMakise("Already empty.");
        }
    }
    void GcsPlugin::generateTrajectory(int droneId,int polyOrder, double tf_, double margin_) {

        // Read waypoints
        int N = waypointsSet[droneId].size() -1;
        if (N >= 0) {
            double t0 = 0, tf = tf_;
            double margin = margin_;
            // 1. Time knots proportion to interval distance
            trajgen::time_knots<double> ts(N + 1);
            ts[0] = t0;
            double lengthSum = 0;
            for (int n = 1; n <= N; n++) {
                double dist = distance(waypointsSet[droneId][n], waypointsSet[droneId][n - 1]);
                lengthSum += dist;
                ts[n] = lengthSum * tf;
            }
            for (int n = 1; n <= N; n++) {
                ts[n] /= lengthSum; // TODO check
            }

            // 2. Define pin
            vector<Pin *> pinSet(N + 1 + 2);  // 2 = initial vel, accel

            for (int n = 0; n <= N; n++) {
                double t = ts[n];
                auto waypoint = waypointsSet[droneId][n];
                TrajVector xl(waypoint.x - margin, waypoint.y - margin, waypoint.z - margin);
                TrajVector xu(waypoint.x + margin, waypoint.y + margin, waypoint.z + margin);
                pinSet[n] = new LoosePin(t, 0, xl, xu);
            }

            FixPin xdot0(0.0, 1, TrajVector(0, 0, 0));
            FixPin xddot0(0.0, 2, TrajVector(0, 0, 0));

            pinSet[N + 1] = &xdot0;
            pinSet[N + 2] = &xddot0;

            // TrajGen settings
            trajgen::PolyParam pp(polyOrder, 2, trajgen::ALGORITHM::POLY_COEFF);
            TrajGenObj trajGenObj(ts, pp);
            trajGenObj.setDerivativeObj(TrajVector(0, 1, 1));
            trajGenObj.addPinSet(pinSet);

            if (trajGenObj.solve(false)) {
                auto traj = make_pair<bool, Trajectory>(true, Trajectory(&trajGenObj, tf));
                trajectorySet[droneId] = traj;
                widget->writeMakise("Generation success!");
            } else {
                auto traj = make_pair<bool, Trajectory>(false, Trajectory());
                trajectorySet[droneId] = traj;
                widget->writeMakise("Trajectory generation failed.");
            }
        }else
            widget->writeMakise("No waypoints.");
    }

    void GcsPlugin::saveTrajectoryTxt(int droneId, std::string fileDir) {
        if (trajectorySet[droneId].first){
            if (trajectorySet[droneId].second.writeTxt(fileDir))
                widget->writeMakise("Trajectory saved to : "+ fileDir);
            else
                widget->writeMakise("File "+ fileDir + " was not opened");
        }else{
            widget->writeMakise("Still no trajectory to save");
        }
    }
    void GcsPlugin::loadTrajectoryTxt(int droneId, std::string fileName) {
        bool isLoaded;
        Trajectory traj (fileName,isLoaded);
        if (isLoaded){
            // upload trajectory
            trajectorySet[droneId].first = isLoaded;
            trajectorySet[droneId].second = traj;

            // upload mission from the trajectory
            missionSet[droneId].first = isLoaded;
            auto mission = Mission (param.worldFrameId,Phase::TRAJ_FOLLOWING);
            mission.loadTrajectory(traj);
            missionSet[droneId].second = mission;

            widget->writeMakise("Loaded txt. Updated trajectory.");
        }else{
            widget->writeMakise("Loading txt failed.");
        }

    }

    void GcsPlugin::doTest(bool doTest_) {

        // Mode switcher
        if (status.isDuringTest) { // switch to pause
            status.isDuringTest = false;

        } else { // switch to start testing
            status.isDuringTest = true;
        }

        for (int n =0 ; n < param.getNdrone() ; n++) {
            if (missionSet[n].first) {  // valid mission ?
                if (status.isDuringTest)
                    missionSet[n].second.trigger();
                else
                    missionSet[n].second.stop();
            }
        }

    }

    void GcsPlugin::callbackPhase(const px4_code2_msgs::PhaseConstPtr &msgPtr,int droneId) {
        status.phaseSet[droneId] =*(msgPtr);
        lastCommTime = ros::Time::now();
    }

    void GcsPlugin::callbackPX4State(const mavros_msgs::StateConstPtr &msgPtr, int droneId) {
        status.px4StateSet[droneId] = *msgPtr;
        lastCommTimePX4[droneId] = ros::Time::now();
    }

    void GcsPlugin::callbackTimer(const ros::TimerEvent &event) {

        // 1. Phase update and q_emit (From px4_code2 server)
        if ((ros::Time::now() - lastCommTime).toSec() > 3){
            if (param.printRosWarning)
                ROS_WARN_STREAM_THROTTLE(3,labelClient + " : Communication with drones are missing.");
            Q_EMIT enablePushButton(false);
        }else{

            bool isAllInit = true;
            bool isAnyDuringMission = false;
            for (auto phase_ : status.phaseSet){
                isAllInit = isAllInit and (phase_.phase.data != "NOT_INIT");
            }
            if (isAllInit){
                ROS_INFO_STREAM_ONCE(labelClient + " : All drones are ready");
                for (auto phase_: status.phaseSet){
                    isAnyDuringMission = isAnyDuringMission or (phase_.phase.data != "STANDBY");
                    Q_EMIT updateMissionStatus(isAnyDuringMission,phase_.isMissionExist);
                }
            }

            Q_EMIT enablePushButton(true);
        }
        for (int m = 0 ; m < param.getNdrone() ; m++) {
            // Update mavros topics
            if ((ros::Time::now() - lastCommTimePX4[m]).toSec() < 3) {
                //  PX4 update and q_emit (From mavros)
                bool isOffboard = status.px4StateSet[m].mode == "OFFBOARD";

                Q_EMIT enablePushButtonPX4(m,true);
                Q_EMIT updatePX4State(m,isOffboard);
            } else {
                if (param.printRosWarning)
                ROS_WARN_STREAM_THROTTLE(3, labelClient + " : Communication with mavros of drones are missing.");
                Q_EMIT enablePushButtonPX4(m,false);
            }
        }

        // publish
        for (int m = 0 ; m < param.getNdrone() ; m++){
            nav_msgs::Path path = convertTo(waypointsSet[m]);
            path.header.frame_id = param.worldFrameId;
            pubWaypointsSet[m].publish(path);

            if (trajectorySet[m].first) { // if it is valid trajectory,
                // publish the entire path
                pubTrajSet[m].publish(trajectorySet[m].second.getPath(param.worldFrameId));

                // is during test
                if (missionSet[m].second.isTriggered()){
                    pubTestPoseSet[m].publish(missionSet[m].second.emitDesPose(ros::Time::now()));
                }

            }



        }





    }

}



PLUGINLIB_EXPORT_CLASS(px4_code2::GcsPlugin, rqt_gui_cpp::Plugin)


