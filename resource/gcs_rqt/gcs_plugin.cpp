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
        connect(this,SIGNAL(updateMissionStatus(bool,bool)),widget,SLOT(updateMissionStatus(bool,bool)));
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
        std::string nameForUI[3] = {"","",""}; int idx = 0;
        std::cout << labelClient << ": mission drones are [ ";
        for (auto it = droneNameSet.begin() ; it < droneNameSet.end() ; it++){

            ros::Subscriber sub = nh.subscribe<px4_code2::phase>("/" + *it + phaseTopicName,
                                           1,boost::bind(&GcsPlugin::callbackPhase,this,_1,idx));
            subPhaseSet.push_back(sub);

            // Phase init
            phase phase_; phase_.isMissionExist = false;
            status.phaseSet.push_back(phase_);

            std::cout << *it ;
            nameForUI[idx] = *it;
            if (it != droneNameSet.end()-1)
                std::cout << ", ";
            idx++;
        }
        std::cout <<  " ]"<<std::endl;
        param.droneNameSet = droneNameSet;
        widget->initNames(nameForUI[0],nameForUI[1],nameForUI[2]);
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
                ros::service::call<px4_code2::Land>("/" + param.droneNameSet[droneId] + landServiceName,
                                                    landSrv);

            }
        }
    }

    void GcsPlugin::callbackPhase(const px4_code2::phaseConstPtr &msgPtr,int droneId) {
        status.phaseSet[droneId] =*(msgPtr);
        lastCommTime = ros::Time::now();
    }

    void GcsPlugin::callbackTimer(const ros::TimerEvent &event) {

        if ((ros::Time::now() - lastCommTime).toSec() > 3){
            ROS_WARN_STREAM_THROTTLE(3,labelClient + " : Communication with drones are missing.");
            Q_EMIT enablePushButton(false);
        }else{
            // Phase update and q_emit
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


    }

}



PLUGINLIB_EXPORT_CLASS(px4_code2::GcsPlugin, rqt_gui_cpp::Plugin)


