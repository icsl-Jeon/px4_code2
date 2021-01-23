//
// Created by jbs on 21. 1. 23..
//

#include "gcs_plugin.h"
#include <pluginlib/class_list_macros.h>

namespace px4_code2{
    GcsPlugin::GcsPlugin() : rqt_gui_cpp::Plugin() , widget(new Widget){
    }

    void GcsPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
        std::cout << "init plugin" << std::endl;
        QStringList argv = context.argv();
        context.addWidget(widget);
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

}


PLUGINLIB_EXPORT_CLASS(px4_code2::GcsPlugin, rqt_gui_cpp::Plugin)


