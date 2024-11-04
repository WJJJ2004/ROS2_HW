#ifndef IMAGE_ANALYSIS_PARAMETER_H
#define IMAGE_ANALYSIS_PARAMETER_H

#include <QMainWindow>
#include <QThread>
#include "ui_parameter.h"
#include "image_node.hpp"

class ImageNode;  // 전방 선언으로 process_img_node 사용

class parameter : public QMainWindow
{
    Q_OBJECT

public:
    explicit parameter(QWidget* parent = nullptr);
    ~parameter();

private:
    Ui::SecondWindowDesign* ui;
    ImageNode* Image_Node;  // 이미지 처리 노드 객체
    void closeEvent(QCloseEvent* event);
};

#endif  // CAM_PARAMS_PKG_SECOND_WINDOW_H
