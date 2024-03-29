//
// Created by jbs on 21. 1. 23..
//

#ifndef PX4_CODE2_GCS_PLUGIN_H
#define PX4_CODE2_GCS_PLUGIN_H

#include <gcs_widget/widget.h>
#include <rqt_gui_cpp/plugin.h>
// Service List
#include <px4_code2/server.h>
#include <px4_code2_msgs/Takeoff.h>
// ROS
#include <ros/ros.h>

namespace px4_code2{

    class GcsPlugin :  public rqt_gui_cpp::Plugin{

        struct Param{
            std::vector<std::string> droneNameSet;
            int getNdrone() {return droneNameSet.size();}
            std::string setting_file;
            std::string worldFrameId ;
            bool printRosWarning = true; //! in case of simple traj gen mode, warning is not required
            float maxHeightSlider = 3; //! max slider will be mapped to this height when selecting z value of waypoints
        };
        struct Status{
            std::vector<px4_code2_msgs::Phase> phaseSet;
            std::vector<mavros_msgs::State> px4StateSet;
            bool isListenWaypoints = false;
            int curWaypointTarget = -1;
            bool isDuringTest = false;
        };


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

    signals:
        void enablePushButton(bool);
        void enablePushButtonPX4(int,bool);
        void updateMissionStatus(bool isDuring,bool isExist);
        void updatePX4State(int,   bool isOffboard);

    private:
        Status status;
        Widget* widget;
        ros::NodeHandle nh;

        // Communication
        ros::Timer timer;
        ros::Time lastCommTime;
        vector<ros::Time> lastCommTimePX4;
        vector<ros::Subscriber> subPhaseSet;
        vector<ros::Subscriber> subPX4Set;

        // Trajectory
        ros::Subscriber subWaypoints;
        vector<vector<geometry_msgs::Point>> waypointsSet;
        vector<ros::Publisher> pubWaypointsSet;
        vector<pair<bool,Trajectory>> trajectorySet; // bool = is uploaded
        vector<pair<bool,Mission>> missionSet; // bool = is uploaded
        vector<ros::Publisher> pubTrajSet;
        vector<ros::Publisher> pubTestPoseSet;
        Param param;

        void callbackTimer(const ros::TimerEvent& event);
        void callbackPhase( const px4_code2_msgs::PhaseConstPtr & msgPtr,int droneId);
        void callbackPX4State( const mavros_msgs::StateConstPtr & msgPtr,int droneId);
        void callbackWaypoint (const geometry_msgs::PoseStampedPtr& msgPtr);
        // ROS
    private slots:
        void callService(int droneId, std::string service,std::vector<double>,int * out);
        void toggleWaypoints(int droneId, bool startListen);
        void eraseWaypoints(int droneId);
        void generateTrajectory(int droneId, int polyOrder, double tf, double margin);
        void saveTrajectoryTxt(int droneId,std::string);
        void loadTrajectoryTxt(int droneId, std::string);
        void doTest(bool doTest_);
    };



}


#endif //PX4_CODE2_GCS_PLUGIN_H
