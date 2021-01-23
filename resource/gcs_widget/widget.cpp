// From qtcreator default application
#include "widget.h"
#include "ui_widget.h"




Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowTitle("FelipeSuite");
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_pushButton_takeoff_clicked()
{

    if (ui->checkBox_1->isChecked())
        Q_EMIT callService(0,"takeoff");

    if (ui->checkBox_2->isChecked())
        Q_EMIT callService(1,"takeoff");

    if (ui->checkBox_3->isChecked())
        Q_EMIT callService(2,"takeoff");

}
