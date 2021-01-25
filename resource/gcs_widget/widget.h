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
    void enableButton(bool enable);
    void updateMissionStatus(bool duringMission_,bool isMissionUpload_ ) ;

private slots:
    // From UI
    void on_pushButton_takeoff_clicked();
    void on_pushButton_lock_clicked();
    void on_pushButton_land_clicked();

private:
    bool isLockPushLock =true;
    Ui::Widget *ui;

    // mission state
    bool duringMission = false;
    bool isMissionUpload = false;
};

#endif // WIDGET_H
