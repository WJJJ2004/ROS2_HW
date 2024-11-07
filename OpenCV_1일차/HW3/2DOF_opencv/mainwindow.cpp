#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath>
#include <QPainter>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , targetX(150)  // 초기 목표 좌표 설정
    , targetY(100)
{
    ui->setupUi(this);

    // 이미지 초기화 및 타이머 설정
    img = cv::Mat::zeros(400, 400, CV_8UC3);
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateImage);
    timer->start(30);  // 30ms마다 updateImage 호출
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    QImage qimg(img.data, img.cols, img.rows, img.step, QImage::Format_BGR888);
    painter.drawImage(0, 0, qimg);
}

void MainWindow::updateImage() {
    img = cv::Mat::zeros(400, 400, CV_8UC3);  // 이미지 초기화

    // 목표 좌표값에 따라 각도 계산
    double theta1, theta2;
    calculateAngles(targetX, targetY, theta1, theta2);

    // 로봇 팔 그리기
    drawRobotArm(img, theta1, theta2);

    update();  // paintEvent를 호출하여 화면 갱신
}

void MainWindow::calculateAngles(double x, double y, double &theta1, double &theta2) {
    double distance = sqrt(x * x + y * y);

    // 주어진 좌표가 로봇 팔이 도달 가능한지 확인
    if (distance > L1 + L2 || distance < abs(L1 - L2)) {
        theta1 = 0;
        theta2 = 0;
        return;
    }

    // 각도 계산
    theta2 = acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2));
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
}

void MainWindow::drawRobotArm(cv::Mat &img, double theta1, double theta2) {
    cv::Point base(200, 200);  // 기준점 설정
    cv::Point joint1(base.x + L1 * cos(theta1), base.y + L1 * sin(theta1));
    cv::Point endEffector(joint1.x + L2 * cos(theta1 + theta2), joint1.y + L2 * sin(theta1 + theta2));

    // 팔 그리기
    cv::line(img, base, joint1, cv::Scalar(255, 0, 0), 5);  // 첫 번째 팔
    cv::line(img, joint1, endEffector, cv::Scalar(0, 255, 0), 5);  // 두 번째 팔

    // 관절과 끝 포인트 표시
    cv::circle(img, base, 5, cv::Scalar(0, 0, 255), -1);        // 기준점
    cv::circle(img, joint1, 5, cv::Scalar(0, 0, 255), -1);      // 첫 번째 관절
    cv::circle(img, endEffector, 5, cv::Scalar(0, 0, 255), -1); // 끝 포인트
}

void MainWindow::on_Target_X_valueChanged(int value)
{
    targetX = value;  // 슬라이더 값을 targetX에 저장
    ui->indicate_x->setText(QString::number(value));  // 현재 X 값을 라벨에 표시
}

// Target_Y 슬라이더의 값이 변경될 때 호출
void MainWindow::on_Target_Y_valueChanged(int value)
{
    targetY = value;  // 슬라이더 값을 targetY에 저장
    ui->indicate_y->setText(QString::number(value));  // 현재 Y 값을 라벨에 표시
}

// Target 버튼을 눌렀을 때 호출
void MainWindow::on_push_Target_button_pressed()
{
    // 목표 좌표에 따라 로봇 팔을 이동시키는 함수 호출
    double theta1, theta2;
    calculateAngles(targetX, targetY, theta1, theta2);

    // 로봇 팔을 새로운 목표 좌표로 그리기
    img = cv::Mat::zeros(400, 400, CV_8UC3);  // 이미지를 초기화
    drawRobotArm(img, theta1, theta2);         // 로봇 팔 그리기

    update();  // 화면을 갱신하여 새로운 좌표를 반영
}

