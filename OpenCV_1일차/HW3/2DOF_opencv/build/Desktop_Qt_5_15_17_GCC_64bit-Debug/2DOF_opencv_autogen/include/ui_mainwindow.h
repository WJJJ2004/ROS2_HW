/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.17
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QSlider *Target_Y;
    QSlider *Target_X;
    QLabel *indicate_x;
    QLabel *indicate_y;
    QPushButton *push_Target_button;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        MainWindow->setStyleSheet(QString::fromUtf8(""));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayoutWidget = new QWidget(centralwidget);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(550, 80, 231, 301));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        Target_Y = new QSlider(gridLayoutWidget);
        Target_Y->setObjectName(QString::fromUtf8("Target_Y"));
        Target_Y->setMaximum(200);
        Target_Y->setOrientation(Qt::Orientation::Horizontal);

        gridLayout->addWidget(Target_Y, 1, 1, 1, 1);

        Target_X = new QSlider(gridLayoutWidget);
        Target_X->setObjectName(QString::fromUtf8("Target_X"));
        Target_X->setMaximum(200);
        Target_X->setOrientation(Qt::Orientation::Horizontal);

        gridLayout->addWidget(Target_X, 0, 1, 1, 1);

        indicate_x = new QLabel(gridLayoutWidget);
        indicate_x->setObjectName(QString::fromUtf8("indicate_x"));

        gridLayout->addWidget(indicate_x, 0, 2, 1, 1);

        indicate_y = new QLabel(gridLayoutWidget);
        indicate_y->setObjectName(QString::fromUtf8("indicate_y"));

        gridLayout->addWidget(indicate_y, 1, 2, 1, 1);

        push_Target_button = new QPushButton(gridLayoutWidget);
        push_Target_button->setObjectName(QString::fromUtf8("push_Target_button"));

        gridLayout->addWidget(push_Target_button, 2, 1, 1, 2);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Target_X", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Target_Y", nullptr));
        label_3->setText(QString());
        indicate_x->setText(QString());
        indicate_y->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        push_Target_button->setText(QCoreApplication::translate("MainWindow", "Push", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
