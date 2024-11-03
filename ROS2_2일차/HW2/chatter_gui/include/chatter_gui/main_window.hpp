#ifndef chatter_gui_MAIN_WINDOW_H
#define chatter_gui_MAIN_WINDOW_H

#include "chatter_gui/talker_node.hpp"
#include "chatter_gui/listener_node.hpp"
#include <QMainWindow>
#include "QIcon"
#include <QString>
#include <QLabel>
#include <QPushButton>

#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  std::shared_ptr<Talker> talker_node;
  std::shared_ptr<Listener> listener_node;

  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private slots:
  void on_publish_button_clicked();
  void initlabel(const QString& msg);

private:
  Ui::MainWindowDesign* ui;

  void closeEvent(QCloseEvent* event);
};

#endif  // chatter_gui_MAIN_WINDOW_H
