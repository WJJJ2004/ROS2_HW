/**
 * @file /include/parameter_gui/main_window.hpp
 *
 * @brief Qt based GUI for parameter_gui.
 *
 * @date August 2024
 **/

#ifndef PARAMETER_GUI_MAIN_WINDOW_H
#define PARAMETER_GUI_MAIN_WINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <QHBoxLayout>
#include <QLabel>
/**
 * @brief MainWindow class for the parameter GUI.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    Ui::MainWindowDesign* ui;
    QNode* qnode;

    // 파라미터 설정 UI 생성 함수
    QHBoxLayout* createControlGroup(const QString &label_text, const std::string &param_name, int min, int max);
};

#endif  // PARAMETER_GUI_MAIN_WINDOW_H
