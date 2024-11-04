#include "../include/image_analysis/parameter.hpp"

parameter::parameter(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::SecondWindowDesign)
{
    ui->setupUi(this);

    QIcon icon(":/images/icon.png");
    this->setWindowIcon(icon);

    // ImageNode 초기화 및 시작
    Image_Node = new Image_Node(rclcpp::Node::make_shared("image_processing_node"), this);
    Image_Node->start();  // QThread 실행

    // 이미지 처리 결과를 UI의 라벨에 업데이트하는 연결
    connect(Image_Node, &Image_Node::imageProcessed, this, [this](const QImage& image) {
        ui->frame_org->setPixmap(QPixmap::fromImage(image));
    });
}

parameter::~parameter()
{
    if (Image_Node) {
        Image_Node->quit();  // QThread 종료 요청
        Image_Node->wait();  // 쓰레드가 종료될 때까지 대기
        delete Image_Node;   // 메모리 해제
    }
    delete ui;
}

void parameter::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}
