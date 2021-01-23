//
// Created by jbs on 21. 1. 23..
//

#ifndef PX4_CODE2_GCS_PLUGIN_H
#define PX4_CODE2_GCS_PLUGIN_H

#include <gcs_widget/widget.h>
#include <rqt_gui_cpp/plugin.h>

// Service List
#include <px4_code2/Takeoff.h>

// ROS
#include <ros/ros.h>

namespace px4_code2{
    class GcsPlugin :  public rqt_gui_cpp::Plugin{

        struct Param{
            std::vector<std::string> droneNameSet;
            int getNdrone() {return droneNameSet.size();}

        };

        std::string takeoffServiceName = "";


        Q_OBJECT // NOTE : the class with this macros should be included in moc
    public:
        GcsPlugin();
        virtual ~GcsPlugin();

        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                  qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                     const qt_gui_cpp::Settings& instance_settings);

    private:
        Widget* widget;
        ros::NodeHandle nh;
        Param param;

        // ROS
    private slots:
        void callService(int droneId, std::string service);



    };



}


#endif //PX4_CODE2_GCS_PLUGIN_H
