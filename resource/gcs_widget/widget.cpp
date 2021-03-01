// From qtcreator default application
#include "widget.h"
#include "ui_widget.h"


Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    // ui image
    QPixmap pixmap;
    pixmap.load( ":/makise.png" );
    int w = ui->label_makise->width();
    int h = ui->label_makise->height();
    ui->label_makise->setPixmap(pixmap.scaled(w,h,Qt::KeepAspectRatio));
    this->setWindowTitle("FelipeSuite");
    ui->pushButton_listenxy->setCheckable(true);
    // line edit read only
    ui->lineEdit_Drone1Name->setReadOnly(true);
    ui->lineEdit_Drone2Name->setReadOnly(true);
    ui->lineEdit_Drone3Name->setReadOnly(true);
    ui->lineEdit_height->setReadOnly(true);



    ui->textEdit_makise->setReadOnly(true);
    ui->textEdit_makise->append("Welcome pilot. Have a safe flight. \n");
    QList<QCheckBox *> l_checkboxes =findChildren<QCheckBox *>();
    QList<QPushButton *> l_pushbuttons =findChildren<QPushButton *>();

    ui->TrajGenheightSlider->setMinimum(0);
    ui->TrajGenheightSlider->setMaximum(10);
    ui->TrajGenheightSlider->setTickPosition(QSlider::TicksBelow);
    ui->TrajGenheightSlider->setTickInterval(1);

    for(auto checkbox : l_checkboxes){
        checkbox->setEnabled(false);
        char idxChar = checkbox->objectName().toStdString().back();
        char idxStr[2];
        idxStr[0] = idxChar;
        idxStr[1] = '\0';
        int idx = atoi(idxStr)-1;
//       std::cout << "to target "<< idx <<checkbox->objectName().toStdString() << std::endl;
        checkBoxAll[idx].push_back(checkbox);
    }
    for (int m =0 ; m < 3 ; m++)
    for (auto checkbox : checkBoxAll[m])
        checkbox->setEnabled(false);

    for (auto pushbutton : l_pushbuttons){

        // identify type of this button
        auto itBack =  pushbutton->objectName().toStdString().end();
        char typeChar = pushbutton->objectName().toStdString().back();
        char offboardIdx = *(itBack-2);

        if (typeChar == 'm'){ // mavros

            pushbutton->setEnabled(false);

            if (offboardIdx == 'o'){ // buttons with individual drones
                char id = *(itBack-4);

            std::cout << pushbutton->objectName().toStdString() << offboardIdx << "/" << id << std::endl;

                char idxStr[2];
                idxStr[0] = id;
                idxStr[1] = '\0';
                int idx = atoi(idxStr) - 1;
                pushButtonMavrosOffboard[idx] .push_back(pushbutton);
            }
            else  { // buttons with all drons

                pushButtonMavrosArmDisarm.push_back(pushbutton);
            }

        }else if (typeChar == 'p'){ // px4_code
            pushbutton->setEnabled(false);
            pushButtonPx4code.push_back(pushbutton);
        }else{ // trajgen or other buttons
            pushbutton->setEnabled(true);
        }
    }
}

Widget::~Widget()
{
    delete ui;
}
void Widget::initNames(std::vector<std::string> nameSet){


    QLineEdit* set [3] = {ui->lineEdit_Drone1Name, ui->lineEdit_Drone2Name, ui->lineEdit_Drone3Name};
    Ndrone = nameSet.size();



    for (int m =0 ; m < Ndrone ; m ++){
        droneNames[m] = nameSet[m];
        ui->comboBox_drone_selector->addItem(QString::fromStdString(nameSet[m]));
        set[m]->setText(QString::fromStdString(nameSet[m]));
            // push button
            for(auto checkbox : checkBoxAll[m]){
                checkbox->setEnabled(true);
                checkbox->setChecked(true);
            }


    }
}

void Widget::on_pushButton_takeoff_p_clicked()
{


    isMissionUpload = true;
    bool checkBoxes[3] = {(ui->checkBox_takeoff1->isChecked()) ,
                          ( ui->checkBox_takeoff2->isChecked()),
                          (ui->checkBox_takeoff3->isChecked())} ;


    QLineEdit* argTakeoffSpeed[3] = {ui->lineEditSpeed1, ui->lineEditSpeed2, ui->lineEditSpeed3};
    QLineEdit* argTakeoffHeight[3] = {ui->lineEdit_takeoff_height1,
                                      ui->lineEdit_takeoff_height2, ui->lineEdit_takeoff_height3};

    for (int m = 0 ; m < 3 ; m++){
        if (checkBoxes[m]){
            QString q( argTakeoffHeight[m]->text());
            double height = q.toDouble();
            double speed = argTakeoffSpeed[m]->text().toDouble();

            std::vector<double> args(2); args[0] = height;  args[1] = speed;
            Q_EMIT callService(m,"takeoff",args,NULL);
        }
    }

    ui->textEdit_makise->append("Takeoff requested for all drones\n");

}

void Widget::writeSettings(std::string settingFile){

   QSettings settings(QString::fromStdString(settingFile),QSettings::IniFormat) ;
    QList<QLineEdit*> l_lineEdit =findChildren<QLineEdit *>();
    for(auto lineEdit : l_lineEdit )
        settings.setValue(lineEdit->objectName(),lineEdit->text());



}

void Widget::readSettings(std::string settingFile){

    if (!QFile::exists(QString::fromStdString(settingFile))) return;


   QSettings settings(QString::fromStdString(settingFile),QSettings::IniFormat) ;

    QList<QLineEdit*> l_lineEdit =findChildren<QLineEdit *>();
    for(auto lineEdit : l_lineEdit ){
        lineEdit->setText(settings.value(lineEdit->objectName()).toString());
    }


    // MISC

    double height = MAX_HEIGHT*getSliderValue();
    ui->lineEdit_height->setText(QString::number(height));


}

void Widget::updateMissionStatus(bool duringMission_, bool isMissionUpload_){
    duringMission  =   duringMission_;
    isMissionUpload = isMissionUpload_;


    // update lock button configs

    if (duringMission and isMissionUpload){
        if (not isLockPushLock){
            ui->pushButton_lock_p->setText("Lock");
            ui->pushButton_lock_p->setStyleSheet("background-color: red");
            isLockPushLock = true;
        }

    }else if ((not duringMission) and isMissionUpload){
        if (isLockPushLock){

            ui->pushButton_lock_p->setText("Resume");
            ui->pushButton_lock_p->setStyleSheet("background-color: green");
            isLockPushLock = false;

        }

    }else{

        if (not isLockPushLock){
            ui->pushButton_lock_p->setText("Lock");
            ui->pushButton_lock_p->setStyleSheet("background-color: red");
            isLockPushLock = true;
        }


    }
}

void Widget::updatePX4Status(int droneIdx, bool isOffborad_){


   isOffborad[droneIdx] = isOffborad_; // is all ?

    if (isOffborad[droneIdx]){
        if (pushButtonMavrosOffboard[droneIdx][0]->isEnabled())
            pushButtonMavrosOffboard[droneIdx][0]->setText("Manual");
    }
    else{
        if (pushButtonMavrosOffboard[droneIdx][0]->isEnabled())
            pushButtonMavrosOffboard[droneIdx][0]->setText("Offboard");
    }




}


// enable button if one of drone px4_code2 updated
void Widget::enableButton(bool enable){

//    std::cout << enable << std::endl;
    ui->pushButton_lock_p->setEnabled(enable);
    ui->pushButton_takeoff_p->setEnabled(enable);
    ui->pushButton_land_p->setEnabled(enable);




    if (isButtonEnabled == false){
        if (enable) {
            ui->textEdit_makise->append("px4_code connected. \n");
            isButtonEnabled = enable;
        }
    }else{
        if (not enable){
            ui->textEdit_makise->append("px4_code disconnected. \n");
            isButtonEnabled = enable;
        }
    }
}

void Widget::enableButtonPX4(int droneIdx, bool enable){

    for (auto button : pushButtonMavrosArmDisarm)
        button->setEnabled(enable);
//    ui->pushButton_lock->setStyleSheet("background-color: red");
    pushButtonMavrosOffboard[droneIdx][0]->setEnabled(enable);
    for (auto checkbox : checkBoxAll[droneIdx] )
        checkbox->setEnabled(enable);

    if (isPX4buttonEnabled == false ){
        if (enable){
        ui->textEdit_makise->append("mavros connected. \n");
        isPX4buttonEnabled = enable;
        }
    }else{
        if(not enable){
        ui->textEdit_makise->append("mavros disconnected. \n");
        isPX4buttonEnabled = enable;
        }
    }

}

double Widget::getSliderValue(){

    return double(ui->TrajGenheightSlider->value())/ ui->TrajGenheightSlider->maximum();

}





void Widget::on_pushButton_lock_p_clicked()
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

void Widget::on_pushButton_land_p_clicked()
{

    isMissionUpload = true;
    bool checkBoxes[3] = {(ui->checkBox_land1->isChecked()) ,
                          ( ui->checkBox_land2->isChecked()),
                          (ui->checkBox_land3->isChecked())} ;

    QLineEdit* argLandGround[3] = {ui->lineEditLandGround1, ui->lineEditLandGround2, ui->lineEditLandGround3};
    QLineEdit* argLandSpeed[3] = {ui->lineEditSpeed1, ui->lineEditSpeed2, ui->lineEditSpeed3};

    for (int m = 0 ; m < 3 ; m++){
        if (checkBoxes[m]){
            std::vector<double> args(2);
            args[0] = argLandGround[m]->text().toDouble();
            args[1] = argLandSpeed[m]->text().toDouble();
            Q_EMIT callService(m,"land",args,NULL);
        }
    }


}

void Widget::on_pushButton_arm_m_clicked()
{

    bool checkBoxes[3] = {(ui->checkBox_arm1->isChecked()) ,
                          ( ui->checkBox_arm2->isChecked()),
                          (ui->checkBox_arm3->isChecked())} ;

    for (int m = 0 ; m < 3 ; m++){
        if (checkBoxes[m]){
            std::vector<double> args(1); args[0] =false ;
                Q_EMIT callService(m,"arm",args,NULL);
        }
    }

        ui->textEdit_makise->append("Requested arming for all drones . \n");

}

void Widget::on_pushButton_disarm_m_clicked()
{

    bool checkBoxes[3] = {(ui->checkBox_disarm1->isChecked()) ,
                          ( ui->checkBox_disarm2->isChecked()),
                          (ui->checkBox_disarm3->isChecked())} ;

    for (int m = 0 ; m < 3 ; m++){
        if (checkBoxes[m]){
            std::vector<double> args(1); args[0] = true;
                Q_EMIT callService(m,"arm",args,NULL);
        }
    }
        ui->textEdit_makise->append("Requested disarming for all drones . \n");
}


void Widget::on_pushButton_mode_switcher1_om_clicked()
{
        if (isOffborad[0])
            ui->textEdit_makise->append(QString::fromStdString("requested manual mode for" + droneNames[0] ));
        else
            ui->textEdit_makise->append(QString::fromStdString("requested offboard mode for" + droneNames[0] ));

        std::vector<double> args(1); args[0] = isOffborad[0];
        Q_EMIT callService(0,"offboard",args,NULL);
}

void Widget::on_pushButton_mode_switcher2_om_clicked()
{

        if (isOffborad[1])
            ui->textEdit_makise->append(QString::fromStdString("requested manual mode for" + droneNames[1] ));
        else
            ui->textEdit_makise->append(QString::fromStdString("requested offboard mode for" + droneNames[1] ));

            std::vector<double> args(1); args[0] = isOffborad[1];
                Q_EMIT callService(1,"offboard",args,NULL);
}

void Widget::on_pushButton_mode_switcher3_om_clicked()
{
        if (isOffborad[2])
            ui->textEdit_makise->append(QString::fromStdString("requested manual mode for" + droneNames[2] ));
        else
            ui->textEdit_makise->append(QString::fromStdString("requested offboard mode for" + droneNames[2] ));

            std::vector<double> args(1); args[0] = isOffborad[2];
                Q_EMIT callService(2,"offboard",args,NULL);
}

void Widget::on_pushButton_listenxy_clicked(bool checked)
{
        int idx = ui->comboBox_drone_selector->currentIndex() ;
    if (checked){
        Q_EMIT toggleWaypoints(idx,true);
        std::string targetString =  "For "+ droneNames[idx]+" ";
        writeMakise(targetString  + ", I am Listening waypoints");
    }
    else{
        Q_EMIT toggleWaypoints(idx,false);
        writeMakise("Finishing waypoints");
    }
}

void Widget::writeMakise(std::string words){

    ui->textEdit_makise->append(QString::fromStdString(words));

}




void Widget::on_pushButton_erase_clicked()
{
        int idx = ui->comboBox_drone_selector->currentIndex() ;
        Q_EMIT eraseWaypoitns(idx);

}

void Widget::on_pushButton_trajgen_generate_clicked()
{

        int idx = ui->comboBox_drone_selector->currentIndex() ;
        int order = ui->lineEdit_trajgen_order->text().toInt();
        double tf = ui->lineEdit_trajgen_duration->text().toDouble();
        double margin = ui->lineEdit_trajgen_waypoint->text().toDouble();

        Q_EMIT generateTrajectory(idx,order,tf,margin);

}

void Widget::on_pushButton_trajgen_save_clicked()
{

        int idx = ui->comboBox_drone_selector->currentIndex() ;
        std::string fileDir = ui->lineEdit_trajgen_save_dir->text().toStdString();

        Q_EMIT saveTrajectory(idx,fileDir);

}

void Widget::on_pushButton_trajgen_load_clicked()
{

        int idx = ui->comboBox_drone_selector->currentIndex() ;
        std::string fileDir = ui->lineEdit_trajgen_load_dir->text().toStdString();
        Q_EMIT loadTrajectory(idx,fileDir);

}


void Widget::on_pushButton_traj_upload_clicked()
{
    isTestActive = false;
    ui->pushButton_trajgen_test->setText(QString("Test start"));
    ui->pushButton_trajgen_test->setStyleSheet("background-color: rgb(250,250,250)");

    QLineEdit* dirs[3] = {ui->lineEdit_traj_load_dir1, ui->lineEdit_traj_load_dir2,ui->lineEdit_traj_load_dir3};
    for (int m = 0 ; m < Ndrone; m++){
        QString dir = dirs[m] ->text();
        if (not dir.isEmpty()){
            Q_EMIT loadTrajectory(m,dir.toStdString());
            writeMakise("For " + droneNames[m] + " , upload trajectory in client.");
        }
    }
}

void Widget::on_pushButton_traj_start_clicked()
{

    QLineEdit* dirs[3] = {ui->lineEdit_traj_load_dir1, ui->lineEdit_traj_load_dir2,ui->lineEdit_traj_load_dir3};
    writeMakise("Mission  requested for servers. Good luck :)");
    for (int m = 0 ; m < Ndrone ; m++){
        QString dir = dirs[m]->text();
        if (not dir.isEmpty()){
             std::vector<double> args(1); int output;
            Q_EMIT callService(m,"trajectory_follow",args,&output);
        }
    }

}

void Widget::on_pushButton_trajgen_test_clicked()
{

    Q_EMIT doTest(true);
    if (not isTestActive){
        ui->pushButton_trajgen_test->setText(QString("Pause"));
        ui->pushButton_trajgen_test->setStyleSheet("background-color: red");
        isTestActive = true;
    }else{
        ui->pushButton_trajgen_test->setText(QString("Resume"));
        ui->pushButton_trajgen_test->setStyleSheet("background-color: green");
        isTestActive = false;
    }

}

void Widget::on_TrajGenheightSlider_sliderMoved(int position)
{
//    writeMakise("Slider moved to "+ std::to_string(position));
    double height = MAX_HEIGHT*getSliderValue();
    ui->lineEdit_height->setText(QString::number(height));
}

void Widget::on_label_height_linkActivated(const QString &link)
{


}
