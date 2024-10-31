#include "../include/my_turtle_gui/main_window.hpp"
#include "../include/my_turtle_gui/qnode.hpp"  // QNode 헤더 추가

// main_window.cpp 수정
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    qnode = new QNode();

    // ROS 노드 종료 시 창 닫기
    connect(qnode, &QNode::rosShutDown, this, &MainWindow::close);

    // 로그 메시지를 UI로 전달하는 신호 연결
    connect(qnode, &QNode::messageToUI, this, &MainWindow::logMessage);

    // // logTextEdit를 UI의 특정 위치에 추가 (예: 수직 레이아웃에 추가)
    // logTextEdit = new QTextEdit(this);
    // logTextEdit->setReadOnly(true);

    ui->logTextEdit->setReadOnly(true);

    qnode->start();
}


void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete qnode;  // 동적 할당 해제
    delete ui;
}

void MainWindow::on_key_a_clicked()
{
    qnode->publishTwist(0.0, 1.0);  // 좌회전
}

void MainWindow::on_key_w_clicked()
{
    qnode->publishTwist(1.0, 0.0);  // 전진
}

void MainWindow::on_key_s_clicked()
{
    qnode->publishTwist(-1.0, 0.0);  // 후진
}

void MainWindow::on_key_d_clicked()
{
    qnode->publishTwist(0.0, -1.0);  // 우회전
}

void MainWindow::on_Draw_Circle_clicked()
{
    qnode->drawCircle();  // 원 그리기
}

void MainWindow::on_Draw_Square_clicked()
{
    qnode->drawSquare();  // 사각형 그리기
}

void MainWindow::on_Draw_Triangle_clicked()
{
    qnode->drawTriangle();  // 삼각형 그리기
}

void MainWindow::on_Set_Pen_clicked()
{
    qnode->setPen();  // 펜 설정
}

void MainWindow::on_Set_Background_clicked()
{
    qnode->setBackground();  // 배경색 설정
}

void MainWindow::on_Set_Trutle_type_clicked()
{
    qnode->setTurtleType();  // 거북이 모양 설정
}

// QNode로부터 메시지를 받아 logTextEdit에 출력
void MainWindow::logMessage(const QString &message)
{
    logTextEdit->append(message);  // 메시지를 추가
}
