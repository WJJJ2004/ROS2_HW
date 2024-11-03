#include "../include/chatter_gui/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  talker_node = std::make_shared<Talker>();
  listener_node = std::make_shared<Listener>();

  QObject::connect(ui->publish_button, &QPushButton::clicked, this, &MainWindow::on_publish_button_clicked);
  QObject::connect(listener_node.get(), &Listener::messageReceived, this, &MainWindow::initlabel);

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
    QString message = ui->lineEdit->text();
    talker_node->publishMessage(message.toStdString());
}

void MainWindow::initlabel(const QString &message)
{
    ui->label->setText(message);
}
