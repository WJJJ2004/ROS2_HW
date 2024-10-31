/**
 * @file /include/my_turtle_gui/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef my_turtle_gui_MAIN_WINDOW_H
#define my_turtle_gui_MAIN_WINDOW_H

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

  private slots:
  void on_key_a_clicked();

  void on_key_w_clicked();

  void on_key_s_clicked();

  void on_key_d_clicked();

  void on_Draw_Circle_clicked();

  void on_Draw_Square_clicked();

  void on_Draw_Triangle_clicked();

  void on_Set_Pen_clicked();

  void on_Set_Background_clicked();

  void on_Set_Trutle_type_clicked();

  // void logMessage(const QString &message);

  private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
  QTextEdit *logTextEdit;


  void logMessage(const QString &message);

};

#endif  // my_turtle_gui_MAIN_WINDOW_H
