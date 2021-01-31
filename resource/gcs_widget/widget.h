#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <vector>
#include <QSettings>
#include <QFile>
#include <iostream>
#include <QObject>
#include <QCheckBox>
#include <QPushButton>
#include <stdio.h>
#include <stdlib.h>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT


public:
    explicit Widget(QWidget *parent = 0);
    void writeMakise(std::string words);
    void initNames(std::vector<std::string> nameSet);
    void writeSettings(std::string settingFile);
    void readSettings(std::string settingFile);
    double getSliderValue();
    ~Widget();
signals:
    void callService(int droneId, std::string serviceName,
                    std::vector<double> argDouble,int * returnPhase);
    void toggleWaypoints(int, bool );
    void eraseWaypoitns(int);
    void generateTrajectory(int droneId,int polyOrder, double tf , double margin );

public slots :  // communicate with outer loop
    void enableButton(bool enable); // enable buttons regarding px4_code at the start communication
    void enableButtonPX4(int droneIdx,bool enable); // enable buttons regarding mavros at the start communication
    void updateMissionStatus(bool duringMission_,bool isMissionUpload_ ) ;
    void updatePX4Status(int droneIdx, bool isOffborad_);

private slots:
    // From UI
    void on_pushButton_takeoff_p_clicked();
    void on_pushButton_lock_p_clicked();
    void on_pushButton_land_p_clicked();

    void on_pushButton_arm_m_clicked();
    void on_pushButton_disarm_m_clicked();

    void on_pushButton_mode_switcher1_om_clicked();
    void on_pushButton_mode_switcher2_om_clicked();
    void on_pushButton_mode_switcher3_om_clicked();

    void on_pushButton_listenxy_clicked(bool checked);

    void on_pushButton_erase_clicked();

    void on_pushButton_trajgen_generate_clicked();

private:
    bool isLockPushLock =true;
    Ui::Widget *ui;

    int Ndrone ;
    // mission state
    bool isButtonEnabled = false;
    bool isPX4buttonEnabled= false;
    bool duringMission = false;
    bool isMissionUpload = false;
    bool isOffborad[3] = {false,false,false};
    bool droneExist [3] = {false,false,false};
    std::string droneNames[3];
    QList<QCheckBox* > checkBoxAll[3]  ;
    QList<QPushButton* > pushButtonMavrosOffboard[3] ;
    QList<QPushButton* > pushButtonMavrosArmDisarm;
    QList<QPushButton* > pushButtonPx4code ;



};

#endif // WIDGET_H
