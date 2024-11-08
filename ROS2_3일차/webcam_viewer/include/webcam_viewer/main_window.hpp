/**
 * @file /include/webcam_viewer/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef webcam_viewer_MAIN_WINDOW_H
#define webcam_viewer_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);

private Q_SLOTS:
  void updateImage(const QImage &image);
};

#endif  // webcam_viewer_MAIN_WINDOW_H
