#include <QApplication>
#include <iostream>

#include "../include/image_analysis/image.hpp"
#include "../include/image_analysis/parameter.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);

    image w;
    w.show();

    parameter sw;
    sw.move(w.x() + w.width() + 120, w.y());
    sw.show();

    int result = a.exec();

    rclcpp::shutdown();

    return result;
}
