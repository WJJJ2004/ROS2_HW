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

#include "../include/talker_listener_qt/main_window.hpp"
#include "../include/talker_listener_qt/talker.hpp"
#include "../include/talker_listener_qt/listener.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    QIcon icon("://ros-icon.png");
    this->setWindowIcon(icon);

    // Talker와 Listener 노드 생성
    talkerNode = std::make_shared<TalkerNode>();
    listenerNode = std::make_shared<ListenerNode>();

    // 종료 시그널 연결
    QObject::connect(listenerNode.get(), SIGNAL(rosShutDown()), this, SLOT(close()));

    // 발행 버튼 클릭 시 publishMessage 호출
    QObject::connect(ui->publishButton, &QPushButton::clicked, this, &MainWindow::on_publish_button_clicked);

    // ListenerNode에서 메시지 수신 시 라벨 업데이트
    QObject::connect(listenerNode.get(), &ListenerNode::messageReceived, this, &MainWindow::updateLabel);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_publish_button_clicked()
{
    // QLineEdit에서 메시지 가져오기 (단일 줄 입력)
    QString message = ui->lineEdit->text();
    talkerNode->publishMessage(message.toStdString());
}

void MainWindow::updateLabel(const QString &message)
{
    // 수신된 메시지를 라벨에 표시
    ui->label->setText(message);
}
