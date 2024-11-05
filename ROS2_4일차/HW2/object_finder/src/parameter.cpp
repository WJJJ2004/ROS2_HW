#include "../include/object_finder/parameter.hpp"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    QIcon icon(":/images/icon.png");
    this->setWindowIcon(icon);

    // ParameterImgNode 초기화 및 연결 설정
    parameterImgNode = new ParameterImgNode(rclcpp::Node::make_shared("parameter_client_node"), this);
    parameterImgNode->start();

    // ROS 종료 시그널 처리
    connect(parameterImgNode, &ParameterImgNode::finished, this, &MainWindow::close);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

// ROI 설정 슬롯
void MainWindow::on_ROI_col_left_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_x_min", value);
}

void MainWindow::on_ROI_col_right_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_x_max", value);
}

void MainWindow::on_ROI_row_left_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_y_min", value);
}

void MainWindow::on_ROI_row_right_valueChanged(int value)
{
    parameterImgNode->setParameter("roi_y_max", value);
}

// Erode 및 Dilate 반복 횟수 설정 슬롯
void MainWindow::on_errode_times_valueChanged(int arg1)
{
    parameterImgNode->setParameter("erode_iterations", arg1);
}

void MainWindow::on_dilate_times_valueChanged(int arg1)
{
    parameterImgNode->setParameter("dilate_iterations", arg1);
}

// Canny 엣지 검출 임계값 설정 슬롯
void MainWindow::on_Canny_upper_threshold_valueChanged(int value)
{
    parameterImgNode->setParameter("canny_threshold_max", value);
}

void MainWindow::on_Canny_lower_threshold_valueChanged(int value)
{
    parameterImgNode->setParameter("canny_threshold_min", value);
}

// 흰색 HSV 범위 설정 슬롯
void MainWindow::on_detect_white_h_high_valueChanged(int value)
{
    parameterImgNode->setParameter("white_h_max", value);
}

void MainWindow::on_detect_white_h_low_valueChanged(int value)
{
    parameterImgNode->setParameter("white_h_min", value);
}

void MainWindow::on_detect_white_w_high_valueChanged(int value)
{
    parameterImgNode->setParameter("white_v_max", value);
}

void MainWindow::on_detect_white_w_low_valueChanged(int value)
{
    parameterImgNode->setParameter("white_v_min", value);
}

void MainWindow::on_detect_white_s_high_valueChanged(int value)
{
    parameterImgNode->setParameter("white_s_max", value);
}

void MainWindow::on_detect_white_s_low_valueChanged(int value)
{
    parameterImgNode->setParameter("white_s_min", value);
}

// 주황색 HSV 범위 설정 슬롯
void MainWindow::on_detect_orange_h_high_valueChanged(int value)
{
    parameterImgNode->setParameter("orange_h_max", value);
}

void MainWindow::on_detect_orange_h_low_valueChanged(int value)
{
    parameterImgNode->setParameter("orange_h_min", value);
}

void MainWindow::on_detect_orange_w_high_2_valueChanged(int value)
{
    parameterImgNode->setParameter("orange_v_max", value);
}

void MainWindow::on_detect_orange_w_low_valueChanged(int value)
{
    parameterImgNode->setParameter("orange_v_min", value);
}

void MainWindow::on_detect_orange_s_high_valueChanged(int value)
{
    parameterImgNode->setParameter("orange_s_max", value);
}

void MainWindow::on_detect_orange_s_low_valueChanged(int value)
{
    parameterImgNode->setParameter("orange_s_min", value);
}

// 초록색 HSV 범위 설정 슬롯
void MainWindow::on_detect_lime_h_high_valueChanged(int value)
{
    parameterImgNode->setParameter("green_h_max", value);
}

void MainWindow::on_detect_lime_h_low_valueChanged(int value)
{
    parameterImgNode->setParameter("green_h_min", value);
}

void MainWindow::on_detect_lime_w_high_valueChanged(int value)
{
    parameterImgNode->setParameter("green_v_max", value);
}

void MainWindow::on_detect_lime_w_low_valueChanged(int value)
{
    parameterImgNode->setParameter("green_v_min", value);
}

void MainWindow::on_detect_lime_s_high_valueChanged(int value)
{
    parameterImgNode->setParameter("green_s_max", value);
}

void MainWindow::on_detect_lime_s_low_valueChanged(int value)
{
    parameterImgNode->setParameter("green_s_min", value);
}
