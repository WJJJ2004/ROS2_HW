#include "../include/object_finder/image.hpp"

SecondWindow::SecondWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::SecondWindowDesign)
{
    ui->setupUi(this);

    QIcon icon(":/images/icon.png");
    this->setWindowIcon(icon);

    // ProcessImgNode 초기화 및 시그널-슬롯 연결
    processImgNode = new ProcessImgNode(rclcpp::Node::make_shared("image_processing_node"), this);

    // ProcessImgNode의 시그널을 SecondWindow의 슬롯에 연결
    connect(processImgNode, &ProcessImgNode::updateLabel, this, &SecondWindow::updateLabel);

    processImgNode->start();  // QThread 실행

    // UI 라벨 초기화
    initializeLabels();
}

void SecondWindow::initializeLabels()
{
    // 각 라벨에 초기 이미지를 설정
    QPixmap placeholder(640, 480);  // 예시 크기
    placeholder.fill(Qt::gray);  // 회색 배경 설정

    ui->USB_CAM->setPixmap(placeholder);
    ui->Find_object->setPixmap(placeholder);
    ui->ROI->setPixmap(placeholder);
    ui->Binary_white->setPixmap(placeholder);
    ui->Binary_orange->setPixmap(placeholder);
    ui->Binary_Lime->setPixmap(placeholder);
}

void SecondWindow::updateLabel(const QString &labelName, const QImage &image)
{
    QPixmap pixmap = QPixmap::fromImage(image);

    // 전달된 라벨 이름에 따라 특정 라벨에 이미지 설정
    if (labelName == "USB_CAM")
        ui->USB_CAM->setPixmap(pixmap);
    else if (labelName == "Find_object")
        ui->Find_object->setPixmap(pixmap);
    else if (labelName == "ROI")
        ui->ROI->setPixmap(pixmap);
    else if (labelName == "Binary_white")
        ui->Binary_white->setPixmap(pixmap);
    else if (labelName == "Binary_orange")
        ui->Binary_orange->setPixmap(pixmap);
    else if (labelName == "Binary_Lime")
        ui->Binary_Lime->setPixmap(pixmap);
}

void SecondWindow::closeEvent(QCloseEvent* event)
{
    if (processImgNode)
    {
        processImgNode->quit();
        processImgNode->wait();
        delete processImgNode;
    }
    QMainWindow::closeEvent(event);
}

SecondWindow::~SecondWindow()
{
    delete ui;
}
