#ifndef OBJECT_FINDER_IMAGE_H
#define OBJECT_FINDER_IMAGE_H

#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include "ui_secondwindow.h"
#include "parameter_node.hpp"
#include "image_node.hpp"

class ProcessImgNode;

class SecondWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit SecondWindow(QWidget* parent = nullptr);
    ~SecondWindow();

private slots:
    void updateLabel(const QString &labelName, const QImage &image);  // 라벨 업데이트 슬롯

private:
    Ui::SecondWindowDesign* ui;
    ProcessImgNode* processImgNode;
    void closeEvent(QCloseEvent* event) override;

    void initializeLabels();  // UI 라벨 초기화 함수
};

#endif
