/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the Qt GUI.
 *
 * @date August 2024
 **/

#include "../include/parameter_gui/main_window.hpp"
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
    ui->setupUi(this);
    QIcon icon("://ros-icon.png");
    this->setWindowIcon(icon);

    qnode = new QNode();

    // QNode가 종료될 때 창을 닫습니다.
    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));

    // UI 레이아웃 설정
    QVBoxLayout *main_layout = new QVBoxLayout;

    // 파라미터 설정 UI 생성 및 추가
    main_layout->addLayout(createControlGroup("Hue Lower", "h_lower", 0, 180));
    main_layout->addLayout(createControlGroup("Hue Upper", "h_upper", 0, 180));
    main_layout->addLayout(createControlGroup("Saturation Lower", "s_lower", 0, 255));
    main_layout->addLayout(createControlGroup("Saturation Upper", "s_upper", 0, 255));
    main_layout->addLayout(createControlGroup("Value Lower", "v_lower", 0, 255));
    main_layout->addLayout(createControlGroup("Value Upper", "v_upper", 0, 255));

    // 이미 존재하는 중앙 위젯의 레이아웃에 main_layout 추가
    if (ui->centralwidget->layout() == nullptr) {
        ui->centralwidget->setLayout(main_layout);
    } else {
        ui->centralwidget->layout()->addItem(main_layout);
    }
}

QHBoxLayout* MainWindow::createControlGroup(const QString &label_text, const std::string &param_name, int min, int max) {
    QHBoxLayout *layout = new QHBoxLayout;
    QLabel *label = new QLabel(label_text);
    QSlider *slider = new QSlider(Qt::Horizontal);
    QSpinBox *spin_box = new QSpinBox;

    slider->setRange(min, max);
    spin_box->setRange(min, max);

    layout->addWidget(label);
    layout->addWidget(slider);
    layout->addWidget(spin_box);

    connect(slider, &QSlider::valueChanged, spin_box, &QSpinBox::setValue);
    connect(spin_box, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);

    connect(slider, &QSlider::valueChanged, this, [=](int value) {
        qnode->setParameter(param_name, value);
    });

    return layout;
}

void MainWindow::closeEvent(QCloseEvent* event) {
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() {
    delete ui;
}

