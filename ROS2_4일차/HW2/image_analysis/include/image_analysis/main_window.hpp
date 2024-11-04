#ifndef IMAGE_ANALYSIS_MAIN_WINDOW_H
#define IMAGE_ANALYSIS_MAIN_WINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QLabel>
#include "qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

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

#endif  // IMAGE_ANALYSIS_MAIN_WINDOW_H
