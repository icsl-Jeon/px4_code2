// From qtcreator default application
#include "widget.h"
#include "ui_widget.h"



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowTitle("FelipeSuite");

    ui->lineEdit_Drone1Name->setReadOnly(true);
    ui->lineEdit_Drone2Name->setReadOnly(true);
    ui->lineEdit_Drone3Name->setReadOnly(true);

    ui->checkBox_4->setEnabled(false);
    ui->checkBox_5->setEnabled(false);
    ui->checkBox_6->setEnabled(false);

    ui->pushButton_lock->setEnabled(false);
    ui->pushButton_takeoff->setEnabled(false);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::initNames(std::string name1, std::string name2, std::string name3){

    ui->lineEdit_Drone1Name->setText( QString::fromStdString( name1));
    ui->lineEdit_Drone2Name->setText(QString::fromStdString(name2));
    ui->lineEdit_Drone3Name->setText(QString::fromStdString(name3));


    if (name1 == ""){
        ui->checkBox_1->setEnabled(false);
        ui->checkBox_7->setEnabled(false);
    }else{
        ui->checkBox_1->setChecked(true);
        ui->checkBox_7 ->setChecked(true);
    }
    if (name2 == ""){
        ui->checkBox_2->setEnabled(false);
        ui->checkBox_8->setEnabled(false);
    }else{
        ui->checkBox_2->setChecked(true);
        ui->checkBox_8->setChecked(true);
    }

    if (name3 == ""){
        ui->checkBox_3->setEnabled(false);
        ui->checkBox_9->setEnabled(false);
    }else{
        ui->checkBox_3->setChecked(true);
        ui->checkBox_9->setChecked(true);
    }

}

void Widget::on_pushButton_takeoff_clicked()
{


    isMissionUpload = true;
    bool checkBoxes[3] = {(ui->checkBox_1->isChecked()) ,
                          ( ui->checkBox_2->isChecked()),
                          (ui->checkBox_3->isChecked())} ;

    for (int m = 0 ; m < 3 ; m++){
        if (checkBoxes[m]){
            QString q( ui->lineEdit->text());
            double height = q.toDouble();

            std::vector<double> args(1); args[0] = height;
            Q_EMIT callService(m,"takeoff",args,NULL);
        }
    }

}

void Widget::writeSettings(std::string settingFile){

   QSettings settings(QString::fromStdString(settingFile),QSettings::IniFormat) ;

    settings.setValue("takeoff1",ui->lineEdit->text());
    settings.setValue("takeoff2",ui->lineEdit_2->text());
    settings.setValue("takeoff3",ui->lineEdit_3->text());


}

void Widget::readSettings(std::string settingFile){

    if (!QFile::exists(QString::fromStdString(settingFile))) return;


   QSettings settings(QString::fromStdString(settingFile),QSettings::IniFormat) ;

   ui->lineEdit->setText(settings.value("takeoff1").toString());
   ui->lineEdit_2->setText(settings.value("takeoff2").toString());
   ui->lineEdit_3->setText(settings.value("takeoff3").toString());





}

void Widget::updateMissionStatus(bool duringMission_, bool isMissionUpload_){
    duringMission  =   duringMission_;
    isMissionUpload = isMissionUpload_;


    // update lock button configs

    if (duringMission and isMissionUpload){
        if (not isLockPushLock){
            ui->pushButton_lock->setText("Lock");
            ui->pushButton_lock->setStyleSheet("background-color: red");
            isLockPushLock = true;
        }

    }else if ((not duringMission) and isMissionUpload){
        if (isLockPushLock){

            ui->pushButton_lock->setText("Resume");
            ui->pushButton_lock->setStyleSheet("background-color: green");
            isLockPushLock = false;

        }

    }else{

        if (not isLockPushLock){
            ui->pushButton_lock->setText("Lock");
            ui->pushButton_lock->setStyleSheet("background-color: red");
            isLockPushLock = true;
        }


    }



}




void Widget::enableButton(bool enable){

//    std::cout << enable << std::endl;
    ui->pushButton_lock->setEnabled(enable);
    if (enable)
//    ui->pushButton_lock->setStyleSheet("background-color: red");


    ui->pushButton_takeoff->setEnabled(enable);


}


void Widget::on_pushButton_lock_clicked()
{

    std::vector<double> args(1) ; int returnPhase;


    Q_EMIT callService(0,"lock",args,&returnPhase);
    Q_EMIT callService(1,"lock",args,&returnPhase);
    Q_EMIT callService(2,"lock",args,&returnPhase);


//    if (! isLockPushLock){ //when  this button = resume
//            args[0] = 1;
//            isLockPushLock = true; // switch to lock button
//            if (isMissionUpload)
//                duringMission = true; // after pushing, drone will be performing mission

//            ui->pushButton_lock->setText("Lock");
//            ui->pushButton_lock->setStyleSheet("background-color: red");

//    }else{ // button  = lock
//             args[0] = 0;

//             if (isMissionUpload){
//                duringMission = false;

//                ui->pushButton_lock->setText("Resume");
//                ui->pushButton_lock->setStyleSheet("background-color: green");
//                 isLockPushLock = false;

//              }
//             if (not isMissionUpload){
//                 // nothing changes;
//                 isLockPushLock = true;

//             }
//    }
}

void Widget::on_pushButton_land_clicked()
{

    isMissionUpload = true;
    bool checkBoxes[3] = {(ui->checkBox_7->isChecked()) ,
                          ( ui->checkBox_8->isChecked()),
                          (ui->checkBox_9->isChecked())} ;

    for (int m = 0 ; m < 3 ; m++){
        if (checkBoxes[m]){
            std::vector<double> args(1);
            Q_EMIT callService(m,"land",args,NULL);
        }
    }


}
