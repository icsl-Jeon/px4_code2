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
        connect(widget,SIGNAL(callService(int,std::string)),this,SLOT(callService(int, std::string)));

        // ROS initialization
        // 1. Drone set
        std::vector<std::string> droneNameSet;
        nh = getPrivateNodeHandle();
        nh.getParam("/drone_name_set",droneNameSet);
        std::cout << labelClient << ": mission drones are [ ";
        for (auto it = droneNameSet.begin() ; it < droneNameSet.end() ; it++){
            std::cout << *it ;
            if (it != droneNameSet.end()-1)
                std::cout << ", ";
        }

        std::cout <<  " ]"<<std::endl;
        param.droneNameSet = droneNameSet;

    }


    void GcsPlugin::shutdownPlugin()
    {
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

    void GcsPlugin::callService(int droneId, std::string service) {
        if (droneId < param.getNdrone())
            std::cout << labelClient << " : for drone " << param.droneNameSet[droneId] <<
            ", service " << service << " is called." << std::endl;

    }

}


PLUGINLIB_EXPORT_CLASS(px4_code2::GcsPlugin, rqt_gui_cpp::Plugin)


