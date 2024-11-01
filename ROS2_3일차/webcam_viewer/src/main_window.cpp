/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/webcam_viewer/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  ui->label->setFixedSize(640, 480);

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(imageReceived(QImage)), this, SLOT(updateImage(QImage)));  // 이미지 수신 시 연결

  qnode->start();
}

void MainWindow::updateImage(const QImage &image)
{
    if (QThread::currentThread() == this->thread())
    {
        // 이미지의 해상도비를 유지하면서 QLabel의 크기에 맞게 조정
        QPixmap pixmap = QPixmap::fromImage(image).scaled(ui->label->size(), Qt::KeepAspectRatio);
        ui->label->setPixmap(pixmap);
    }
    else
    {
        // UI 스레드에서 안전하게 실행되도록 보장
        QMetaObject::invokeMethod(this, [this, image]()
                                  {
                                      QPixmap pixmap = QPixmap::fromImage(image).scaled(ui->label->size(), Qt::KeepAspectRatio);
                                      ui->label->setPixmap(pixmap);
                                  }, Qt::QueuedConnection);
    }
}




void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}
