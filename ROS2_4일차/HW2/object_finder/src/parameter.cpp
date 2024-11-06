#include <QTimer>
#include <QMap>
#include "../include/object_finder/parameter.hpp"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    QIcon icon(":/images/icon.png");
    this->setWindowIcon(icon);

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);  // 노드 초기화가 안 된 경우에만 초기화
    }

    // ParameterImgNode 초기화 및 연결 설정
    parameterImgNode = new ParameterImgNode(rclcpp::Node::make_shared("parameter_client_node"), this);
    parameterImgNode->start();

    // ROS 종료 시그널 처리
    connect(parameterImgNode, &ParameterImgNode::finished, this, &MainWindow::close);
}

// Destructor
MainWindow::~MainWindow()
{
    delete ui;

    // 생성된 모든 타이머 삭제하여 메모리 누수 방지
    for (auto timer : debounceTimers) {
        delete timer;
    }
}

// 파라미터 디바운싱을 위한 타이머 맵
QMap<QString, QTimer*> debounceTimers;
QMap<QString, int> pendingParameterValues;

void MainWindow::updateParameterWithDebounce(const QString &paramName, int value)
{
    // 기존 타이머가 있는 경우 정지 후 재설정, 없을 경우 타이머 생성
    if (!debounceTimers.contains(paramName)) {
        debounceTimers[paramName] = new QTimer(this);
        debounceTimers[paramName]->setSingleShot(true);
        connect(debounceTimers[paramName], &QTimer::timeout, this, [this, paramName]() {
            parameterImgNode->setParameter(paramName.toStdString(), pendingParameterValues[paramName]);
        });
    }

    // 최신 값을 저장하고 타이머 시작 (100ms 딜레이)
    pendingParameterValues[paramName] = value;
    debounceTimers[paramName]->start(100); // 100ms 딜레이로 디바운싱 처리
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    delete ui;
    if (parameterImgNode) {
        parameterImgNode->quit();
        parameterImgNode->wait(); // thread 종료까지 대기
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();  // 전체 ROS 노드를 종료
    }
}

// ROI 설정 슬롯
void MainWindow::on_ROI_col_left_valueChanged(int value)
{
    updateParameterWithDebounce("roi_x_min", value);
}

void MainWindow::on_ROI_col_right_valueChanged(int value)
{
    updateParameterWithDebounce("roi_x_max", value);
}

void MainWindow::on_ROI_row_left_valueChanged(int value)
{
    updateParameterWithDebounce("roi_y_min", value);
}

void MainWindow::on_ROI_row_right_valueChanged(int value)
{
    updateParameterWithDebounce("roi_y_max", value);
}

// Erode 및 Dilate 반복 횟수 설정 슬롯
void MainWindow::on_errode_times_valueChanged(int arg1)
{
    updateParameterWithDebounce("erode_iterations", arg1);
}

void MainWindow::on_dilate_times_valueChanged(int arg1)
{
    updateParameterWithDebounce("dilate_iterations", arg1);
}

// Canny 엣지 검출 임계값 설정 슬롯
void MainWindow::on_Canny_upper_threshold_valueChanged(int value)
{
    updateParameterWithDebounce("canny_threshold_max", value);
}

void MainWindow::on_Canny_lower_threshold_valueChanged(int value)
{
    updateParameterWithDebounce("canny_threshold_min", value);
}

// 흰색 HSV 범위 설정 슬롯
void MainWindow::on_detect_white_h_high_valueChanged(int value)
{
    updateParameterWithDebounce("white_h_max", value);
}

void MainWindow::on_detect_white_h_low_valueChanged(int value)
{
    updateParameterWithDebounce("white_h_min", value);
}

void MainWindow::on_detect_white_w_high_valueChanged(int value)
{
    updateParameterWithDebounce("white_v_max", value);
}

void MainWindow::on_detect_white_w_low_valueChanged(int value)
{
    updateParameterWithDebounce("white_v_min", value);
}

void MainWindow::on_detect_white_s_high_valueChanged(int value)
{
    updateParameterWithDebounce("white_s_max", value);
}

void MainWindow::on_detect_white_s_low_valueChanged(int value)
{
    updateParameterWithDebounce("white_s_min", value);
}

// 주황색 HSV 범위 설정 슬롯
void MainWindow::on_detect_orange_h_high_valueChanged(int value)
{
    updateParameterWithDebounce("orange_h_max", value);
}

void MainWindow::on_detect_orange_h_low_valueChanged(int value)
{
    updateParameterWithDebounce("orange_h_min", value);
}

void MainWindow::on_detect_orange_w_high_2_valueChanged(int value)
{
    updateParameterWithDebounce("orange_v_max", value);
}

void MainWindow::on_detect_orange_w_low_valueChanged(int value)
{
    updateParameterWithDebounce("orange_v_min", value);
}

void MainWindow::on_detect_orange_s_high_valueChanged(int value)
{
    updateParameterWithDebounce("orange_s_max", value);
}

void MainWindow::on_detect_orange_s_low_valueChanged(int value)
{
    updateParameterWithDebounce("orange_s_min", value);
}

// 초록색 HSV 범위 설정 슬롯
void MainWindow::on_detect_lime_h_high_valueChanged(int value)
{
    updateParameterWithDebounce("green_h_max", value);
}

void MainWindow::on_detect_lime_h_low_valueChanged(int value)
{
    updateParameterWithDebounce("green_h_min", value);
}

void MainWindow::on_detect_lime_w_high_valueChanged(int value)
{
    updateParameterWithDebounce("green_v_max", value);
}

void MainWindow::on_detect_lime_w_low_valueChanged(int value)
{
    updateParameterWithDebounce("green_v_min", value);
}

void MainWindow::on_detect_lime_s_high_valueChanged(int value)
{
    updateParameterWithDebounce("green_s_max", value);
}

void MainWindow::on_detect_lime_s_low_valueChanged(int value)
{
    updateParameterWithDebounce("green_s_min", value);
}
