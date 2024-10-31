#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  initUi();

  QMetaObject::connectSlotsByName(this);

  commNode.set_image_label(ui->image_label);
  commNode.set_current_mode_label(ui->current_mode_label);

  commNode.start();
}

void MainWindow::initUi()
{
  QIcon icon("://ros-icon.png");

  if (icon.isNull()) {
      qDebug() << "Failed to load the icon.";
  } else {
      qDebug() << "The icon was loaded successfully.";
  }

  this->setWindowIcon(icon);

  setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
  
  setStatusBar(nullptr);
}

void MainWindow::on_Init_System_clicked()
{
  ui->current_mode_label->setText("Current Mode:\nInit System");
  commNode.mode_info.mode = 0;
}

void MainWindow::on_Measure_Offset_clicked()
{
  ui->current_mode_label->setText("Current Mode:\nMeasure Offset");
  commNode.mode_info.mode = 1;
}

void MainWindow::on_Stay_clicked()
{
  ui->current_mode_label->setText("Current Mode:\nStay");
  commNode.mode_info.mode = 3;
}

void MainWindow::on_Push_Pull_clicked()
{
  ui->current_mode_label->setText("Current Mode:\nPush/Pull");
  commNode.mode_info.mode = 4;
}

void MainWindow::on_Human_Following_clicked()
{
  ui->current_mode_label->setText("Current Mode:\nHuman Following");
  commNode.mode_info.mode = 5;
}

void MainWindow::on_Eight_Direction_clicked()
{
  ui->current_mode_label->setText("Current Mode:\nEight Direction");
  commNode.mode_info.mode = 6;
}

void MainWindow::on_Quit_clicked()
{
  this->close();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  rclcpp::shutdown();
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() { delete ui;}
