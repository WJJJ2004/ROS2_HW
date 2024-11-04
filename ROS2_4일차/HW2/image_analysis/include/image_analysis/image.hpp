#ifndef IMAGE_ANALYSIS_IMAGE_H
#define IMAGE_ANALYSIS_IMAGE_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QLabel>
#include "image_node.hpp"
#include "ui_image.h"

class image : public QMainWindow {
    Q_OBJECT

public:
    image(QWidget* parent = nullptr);
    ~image();

private slots:
    // 이미지 업데이트 슬롯
    void updateImageOnLabel(const QImage &image, const QString &label_name);


private:
    Ui::MainWindowDesign* ui;
    QNode* qnode;

    // 각 라벨 포인터 선언
    QLabel *usb_cam_label;
    QLabel *find_object_label;
    QLabel *roi_label;
    QLabel *bw_label;
    QLabel *bl_label;
    QLabel *bo_label;

    void closeEvent(QCloseEvent* event);
};

#endif
