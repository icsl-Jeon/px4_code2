#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <vector>
#include <QSettings>
#include <QFile>
#include <iostream>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT


public:
    explicit Widget(QWidget *parent = 0);
    void initNames(std::string name1,std::string name2, std::string name3);

    void writeSettings(std::string settingFile);
    void readSettings(std::string settingFile);
    ~Widget();
signals:
    void callService(int droneId, std::string serviceName,
                    std::vector<double> argDouble,int * returnPhase);
public slots:
    void enableButton(bool enable); // enable buttons regarding px4_code at the start communication
    void enableButtonPX4(bool enable); // enable buttons regarding mavros at the start communication
    void updateMissionStatus(bool duringMission_,bool isMissionUpload_ ) ;
    void updatePX4Status(bool isArmed_, bool isOffBorad_);

private slots:
    // From UI
    void on_pushButton_takeoff_clicked();
    void on_pushButton_lock_clicked();
    void on_pushButton_land_clicked();

    void on_pushButton_arm_clicked();

    void on_pushButton_mode_switcher_clicked();

private:
    bool isLockPushLock =true;
    Ui::Widget *ui;

    // mission state
    bool duringMission = false;
    bool isMissionUpload = false;
    bool isArmed = false;
    bool isOffBorad = false;
};

#endif // WIDGET_H
