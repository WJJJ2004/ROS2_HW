#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include <QWidget>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void updateImage();  // 타이머에 연결되어 주기적으로 화면을 갱신

    void on_Target_X_valueChanged(int value);

    void on_Target_Y_valueChanged(int value);

    void on_push_Target_button_pressed();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    cv::Mat img;

    // 로봇 팔 관련 변수
    const double L1 = 100;  // 첫 번째 팔의 길이
    const double L2 = 100;  // 두 번째 팔의 길이
    double targetX;         // 목표 x 좌표
    double targetY;         // 목표 y 좌표

    void calculateAngles(double x, double y, double &theta1, double &theta2);  // 각도 계산 함수
    void drawRobotArm(cv::Mat &img, double theta1, double theta2);  // 로봇 팔 그리기 함수
};

#endif // MAINWINDOW_H
