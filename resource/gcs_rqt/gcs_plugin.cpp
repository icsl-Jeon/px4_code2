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
        // ROS initialization
        timer = nh.createTimer(ros::Duration(0.01),&GcsPlugin::callbackTimer,this);

        // 0. Slot loading
        nh = getNodeHandle();
        nh.getParam("/qt_setting_dir",param.setting_file);
        std::cout << labelClient + " : setting file : " << param.setting_file << std::endl;
        widget->readSettings(param.setting_file);

        // 1. Drone set
        std::vector<std::string> droneNameSet;
        nh.getParam("/drone_name_set",droneNameSet);
         int idx = 0;
        std::cout << labelClient << ": mission drones are [ ";
        for (auto it = droneNameSet.begin() ; it < droneNameSet.end() ; it++){
            // Subscribing phase
            ros::Subscriber sub = nh.subscribe<px4_code2::phase>("/" + *it + phaseTopicName,
                                           1,boost::bind(&GcsPlugin::callbackPhase,this,_1,idx));
            subPhaseSet.push_back(sub);

            // Subscribing px4 state
            sub = nh.subscribe<mavros_msgs::State>("/" + *it + px4StateTopicName,
                                                                 1,boost::bind(&GcsPlugin::callbackPX4State,this,_1,idx));
            subPX4Set.push_back(sub);
            // Phase init
            phase phase_; phase_.isMissionExist = false;
            status.phaseSet.push_back(phase_);
            status.px4StateSet.push_back(mavros_msgs::State ());
            lastCommTimePX4.push_back(ros::Time(0));

            std::cout << *it ;
            if (it != droneNameSet.end()-1)
                std::cout << ", ";
            idx++;
        }
        std::cout <<  " ]"<<std::endl;
        param.droneNameSet = droneNameSet;
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
                px4_code2::Takeoff takeoffSrv;
                takeoffSrv.request.height = args[0];
                double takeoffSpeed = args[1];
                if (takeoffSpeed<= 0){
                    ROS_WARN_STREAM(labelClient << " : takeoff speed is below zero. resetting 0.05 [m/s]" );
                    takeoffSpeed = 0.05;
                }

                takeoffSrv.request.speed=takeoffSpeed;
                ros::service::call<px4_code2::Takeoff>("/" + param.droneNameSet[droneId] + takeoffServiceName,
                                                       takeoffSrv);
            }
            if (service == "lock") {
                px4_code2::Lock lockSrv;
                ros::service::call<px4_code2::Lock>("/" + param.droneNameSet[droneId] +lockServiceName,
                                                    lockSrv);
                // we should fill output
                if (out != NULL){
                    *out = lockSrv.response.mission_remain;
                }

            }
            if (service == "land") {
                px4_code2::Land landSrv;
                double landGround = args[0];  double landSpeed = args[1];
                if (landGround > 0)
                    ROS_WARN_STREAM(labelClient << " : landing ground is larger than zero." );
                if (landSpeed <= 0){
                    ROS_WARN_STREAM(labelClient << " : landing speed is below zero. resetting 0.05 [m/s]" );
                    landSpeed = 0.05;
                }

                landSrv.request.ground = landGround; landSrv.request.speed = landSpeed;
                ros::service::call<px4_code2::Land>("/" + param.droneNameSet[droneId] + landServiceName,
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
        }
    }

    void GcsPlugin::callbackPhase(const px4_code2::phaseConstPtr &msgPtr,int droneId) {
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
                ROS_WARN_STREAM_THROTTLE(3, labelClient + " : Communication with mavros of drones are missing.");
                Q_EMIT enablePushButtonPX4(m,false);
            }
        }


    }

}



PLUGINLIB_EXPORT_CLASS(px4_code2::GcsPlugin, rqt_gui_cpp::Plugin)


