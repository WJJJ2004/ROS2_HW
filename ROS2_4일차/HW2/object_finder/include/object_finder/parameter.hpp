#ifndef OBJECT_FINDER_MAIN_WINDOW_H
#define OBJECT_FINDER_MAIN_WINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <QMap>
#include "ui_mainwindow.h"
#include "parameter_node.hpp"

class ParameterImgNode;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void on_ROI_col_left_valueChanged(int value);

    void on_ROI_col_right_valueChanged(int value);

    void on_ROI_row_left_valueChanged(int value);

    void on_ROI_row_right_valueChanged(int value);

    void on_errode_times_valueChanged(int arg1);

    void on_dilate_times_valueChanged(int arg1);

    void on_Canny_upper_threshold_valueChanged(int value);

    void on_Canny_lower_threshold_valueChanged(int value);

    void on_detect_white_h_high_valueChanged(int value);

    void on_detect_white_h_low_valueChanged(int value);

    void on_detect_white_w_high_valueChanged(int value);

    void on_detect_white_w_low_valueChanged(int value);

    void on_detect_white_s_high_valueChanged(int value);

    void on_detect_white_s_low_valueChanged(int value);

    void on_detect_orange_h_high_valueChanged(int value);

    void on_detect_orange_h_low_valueChanged(int value);

    void on_detect_orange_w_high_2_valueChanged(int value);

    void on_detect_orange_w_low_valueChanged(int value);

    void on_detect_orange_s_high_valueChanged(int value);

    void on_detect_orange_s_low_valueChanged(int value);

    void on_detect_lime_h_high_valueChanged(int value);

    void on_detect_lime_h_low_valueChanged(int value);

    void on_detect_lime_w_high_valueChanged(int value);

    void on_detect_lime_w_low_valueChanged(int value);

    void on_detect_lime_s_high_valueChanged(int value);

    void on_detect_lime_s_low_valueChanged(int value);

private:
    Ui::MainWindowDesign* ui;
    ParameterImgNode* parameterImgNode;
    void closeEvent(QCloseEvent* event);
    // 디바운싱을 위한 QTimer 맵
    QMap<QString, QTimer*> debounceTimers;

    // 디바운싱 적용 메서드
    void updateParameterWithDebounce(const QString &paramName, int value);

};

#endif  // OBJECT_FINDER_MAIN_WINDOW_H
