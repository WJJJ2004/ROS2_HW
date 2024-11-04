#include "../include/image_analysis/main_window.hpp"
#include <QImage>
#include <QPixmap>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
    ui->setupUi(this);

    QIcon icon("://ros-icon.png");
    this->setWindowIcon(icon);

    qnode = new QNode();

    // QNode가 종료될 때 창을 닫습니다.
    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
    QObject::connect(qnode, &QNode::imageReceived, this, &MainWindow::updateImageOnLabel);

    // 라벨 포인터 초기화
    usb_cam_label = findChild<QLabel*>("usb_cam");
    find_object_label = findChild<QLabel*>("find_object");
    roi_label = findChild<QLabel*>("roi");
    bw_label = findChild<QLabel*>("bw");
    bl_label = findChild<QLabel*>("bl");
    bo_label = findChild<QLabel*>("bo");
}

void MainWindow::updateImageOnLabel(const QImage &image, const QString &label_name)
{
    QLabel *target_label = nullptr;

    if (label_name == "usb_cam")
    {
        target_label = usb_cam_label;
    }
    else if (label_name == "find_object")
    {
        target_label = find_object_label;
    }
    else if (label_name == "roi")
    {
        target_label = roi_label;
    }
    else if (label_name == "bw")
    {
        target_label = bw_label;
    }
    else if (label_name == "bl")
    {
        target_label = bl_label;
    }
    else if (label_name == "bo")
    {
        target_label = bo_label;
    }

    if (target_label != nullptr)
    {
        target_label->setPixmap(QPixmap::fromImage(image).scaled(target_label->size(), Qt::KeepAspectRatio));
    }
    else
    {
        std::cout << "Received an empty image!" << std::endl;
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
