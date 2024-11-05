#include <QApplication>
#include <iostream>

#include "../include/object_finder/parameter.hpp"
#include "../include/object_finder/image.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  QApplication a(argc, argv);

  MainWindow w;
  w.show();

  SecondWindow sw;
  sw.move(w.x() + w.width() + 120, w.y());
  sw.show();

  int result = a.exec();

  rclcpp::shutdown();

  return result;
}


