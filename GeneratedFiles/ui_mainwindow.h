/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QLabel *label;
    QSpinBox *setMotor;
    QLabel *label_2;
    QSpinBox *setVelocity;
    QLabel *label_3;
    QSpinBox *setPosition;
    QTextBrowser *state;
    QLabel *motornumber;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *init;
    QPushButton *start;
    QPushButton *rotate;
    QPushButton *stop;
    QPushButton *findslave;
    QPushButton *reset;
    QWidget *layoutWidget2;
    QGridLayout *gridLayout_3;
    QPushButton *PID;
    QTabWidget *tabWidget;
    QWidget *single_control;
    QWidget *multi_control;
    QWidget *layoutWidget3;
    QVBoxLayout *verticalLayout_2;
    QPushButton *ALL_UP;
    QPushButton *ALL_STOP;
    QPushButton *ALL_Mid;
    QPushButton *ALL_ZERO;
    QPushButton *ALL_Trans;
    QWidget *layoutWidget4;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_4;
    QPushButton *X_0;
    QPushButton *Y_0;
    QPushButton *Pitch_0;
    QPushButton *Roll_0;
    QPushButton *Yaw_0;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QSlider *X;
    QHBoxLayout *horizontalLayout_2;
    QSlider *Y;
    QHBoxLayout *horizontalLayout_3;
    QSlider *Pitch;
    QHBoxLayout *horizontalLayout_4;
    QSlider *Roll;
    QHBoxLayout *horizontalLayout_5;
    QSlider *Yaw;
    QWidget *layoutWidget5;
    QVBoxLayout *verticalLayout_5;
    QTextBrowser *X_output;
    QTextBrowser *Y_output;
    QTextBrowser *Pitch_output;
    QTextBrowser *Roll_output;
    QTextBrowser *Yaw_output;
    QSplitter *splitter;
    QSlider *Z;
    QPushButton *Z_0;
    QTextBrowser *Z_output;
    QPushButton *Motion_Test;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *showChart;
    QPushButton *btnShow;
    QPushButton *btnStop;
    QWidget *layoutWidget6;
    QHBoxLayout *horizontalLayout_7;
    QVBoxLayout *verticalLayout_6;
    QPushButton *UP_Mid;
    QPushButton *UP_STOP;
    QPushButton *UP_ZERO;
    QPushButton *UP_Trans;
    QVBoxLayout *verticalLayout_7;
    QPushButton *DOWN_Mid;
    QPushButton *DOWN_STOP;
    QPushButton *DOWN_ZERO;
    QPushButton *DOWN_Trans;
    QWidget *layoutWidget7;
    QVBoxLayout *verticalLayout_8;
    QPushButton *UP_Start;
    QPushButton *DOWN_Start;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(967, 877);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(112, 12, 258, 344));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        setMotor = new QSpinBox(layoutWidget);
        setMotor->setObjectName(QStringLiteral("setMotor"));

        gridLayout->addWidget(setMotor, 1, 0, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        setVelocity = new QSpinBox(layoutWidget);
        setVelocity->setObjectName(QStringLiteral("setVelocity"));
        setVelocity->setMinimum(-1000);
        setVelocity->setMaximum(1000);

        gridLayout->addWidget(setVelocity, 3, 0, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 4, 0, 1, 1);

        setPosition = new QSpinBox(layoutWidget);
        setPosition->setObjectName(QStringLiteral("setPosition"));
        setPosition->setMinimum(-1000);
        setPosition->setMaximum(1000);

        gridLayout->addWidget(setPosition, 5, 0, 1, 1);

        state = new QTextBrowser(layoutWidget);
        state->setObjectName(QStringLiteral("state"));

        gridLayout->addWidget(state, 7, 0, 1, 1);

        motornumber = new QLabel(layoutWidget);
        motornumber->setObjectName(QStringLiteral("motornumber"));

        gridLayout->addWidget(motornumber, 6, 0, 1, 1);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(20, 11, 79, 172));
        gridLayout_2 = new QGridLayout(layoutWidget1);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        init = new QPushButton(layoutWidget1);
        init->setObjectName(QStringLiteral("init"));

        verticalLayout->addWidget(init);

        start = new QPushButton(layoutWidget1);
        start->setObjectName(QStringLiteral("start"));

        verticalLayout->addWidget(start);

        rotate = new QPushButton(layoutWidget1);
        rotate->setObjectName(QStringLiteral("rotate"));

        verticalLayout->addWidget(rotate);

        stop = new QPushButton(layoutWidget1);
        stop->setObjectName(QStringLiteral("stop"));

        verticalLayout->addWidget(stop);

        findslave = new QPushButton(layoutWidget1);
        findslave->setObjectName(QStringLiteral("findslave"));

        verticalLayout->addWidget(findslave);


        gridLayout_2->addLayout(verticalLayout, 0, 0, 1, 1);

        reset = new QPushButton(layoutWidget1);
        reset->setObjectName(QStringLiteral("reset"));

        gridLayout_2->addWidget(reset, 1, 0, 1, 1);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(20, 190, 77, 112));
        gridLayout_3 = new QGridLayout(layoutWidget2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        PID = new QPushButton(layoutWidget2);
        PID->setObjectName(QStringLiteral("PID"));

        gridLayout_3->addWidget(PID, 0, 0, 1, 1);

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(390, 10, 561, 351));
        single_control = new QWidget();
        single_control->setObjectName(QStringLiteral("single_control"));
        tabWidget->addTab(single_control, QString());
        multi_control = new QWidget();
        multi_control->setObjectName(QStringLiteral("multi_control"));
        layoutWidget3 = new QWidget(multi_control);
        layoutWidget3->setObjectName(QStringLiteral("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(30, 60, 77, 141));
        verticalLayout_2 = new QVBoxLayout(layoutWidget3);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        ALL_UP = new QPushButton(layoutWidget3);
        ALL_UP->setObjectName(QStringLiteral("ALL_UP"));

        verticalLayout_2->addWidget(ALL_UP);

        ALL_STOP = new QPushButton(layoutWidget3);
        ALL_STOP->setObjectName(QStringLiteral("ALL_STOP"));

        verticalLayout_2->addWidget(ALL_STOP);

        ALL_Mid = new QPushButton(layoutWidget3);
        ALL_Mid->setObjectName(QStringLiteral("ALL_Mid"));

        verticalLayout_2->addWidget(ALL_Mid);

        ALL_ZERO = new QPushButton(layoutWidget3);
        ALL_ZERO->setObjectName(QStringLiteral("ALL_ZERO"));

        verticalLayout_2->addWidget(ALL_ZERO);

        ALL_Trans = new QPushButton(layoutWidget3);
        ALL_Trans->setObjectName(QStringLiteral("ALL_Trans"));

        verticalLayout_2->addWidget(ALL_Trans);

        layoutWidget4 = new QWidget(multi_control);
        layoutWidget4->setObjectName(QStringLiteral("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(120, 50, 221, 161));
        horizontalLayout_6 = new QHBoxLayout(layoutWidget4);
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        X_0 = new QPushButton(layoutWidget4);
        X_0->setObjectName(QStringLiteral("X_0"));

        verticalLayout_4->addWidget(X_0);

        Y_0 = new QPushButton(layoutWidget4);
        Y_0->setObjectName(QStringLiteral("Y_0"));

        verticalLayout_4->addWidget(Y_0);

        Pitch_0 = new QPushButton(layoutWidget4);
        Pitch_0->setObjectName(QStringLiteral("Pitch_0"));

        verticalLayout_4->addWidget(Pitch_0);

        Roll_0 = new QPushButton(layoutWidget4);
        Roll_0->setObjectName(QStringLiteral("Roll_0"));

        verticalLayout_4->addWidget(Roll_0);

        Yaw_0 = new QPushButton(layoutWidget4);
        Yaw_0->setObjectName(QStringLiteral("Yaw_0"));

        verticalLayout_4->addWidget(Yaw_0);


        horizontalLayout_6->addLayout(verticalLayout_4);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        X = new QSlider(layoutWidget4);
        X->setObjectName(QStringLiteral("X"));
        X->setMinimum(-200);
        X->setMaximum(200);
        X->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(X);


        verticalLayout_3->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        Y = new QSlider(layoutWidget4);
        Y->setObjectName(QStringLiteral("Y"));
        Y->setMinimum(-200);
        Y->setMaximum(200);
        Y->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(Y);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        Pitch = new QSlider(layoutWidget4);
        Pitch->setObjectName(QStringLiteral("Pitch"));
        Pitch->setMinimum(-5);
        Pitch->setMaximum(5);
        Pitch->setValue(0);
        Pitch->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(Pitch);


        verticalLayout_3->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        Roll = new QSlider(layoutWidget4);
        Roll->setObjectName(QStringLiteral("Roll"));
        Roll->setMinimum(-5);
        Roll->setMaximum(5);
        Roll->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(Roll);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        Yaw = new QSlider(layoutWidget4);
        Yaw->setObjectName(QStringLiteral("Yaw"));
        Yaw->setMinimum(-8);
        Yaw->setMaximum(8);
        Yaw->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(Yaw);


        verticalLayout_3->addLayout(horizontalLayout_5);


        horizontalLayout_6->addLayout(verticalLayout_3);

        layoutWidget5 = new QWidget(multi_control);
        layoutWidget5->setObjectName(QStringLiteral("layoutWidget5"));
        layoutWidget5->setGeometry(QRect(350, 50, 73, 161));
        verticalLayout_5 = new QVBoxLayout(layoutWidget5);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        X_output = new QTextBrowser(layoutWidget5);
        X_output->setObjectName(QStringLiteral("X_output"));

        verticalLayout_5->addWidget(X_output);

        Y_output = new QTextBrowser(layoutWidget5);
        Y_output->setObjectName(QStringLiteral("Y_output"));

        verticalLayout_5->addWidget(Y_output);

        Pitch_output = new QTextBrowser(layoutWidget5);
        Pitch_output->setObjectName(QStringLiteral("Pitch_output"));

        verticalLayout_5->addWidget(Pitch_output);

        Roll_output = new QTextBrowser(layoutWidget5);
        Roll_output->setObjectName(QStringLiteral("Roll_output"));

        verticalLayout_5->addWidget(Roll_output);

        Yaw_output = new QTextBrowser(layoutWidget5);
        Yaw_output->setObjectName(QStringLiteral("Yaw_output"));

        verticalLayout_5->addWidget(Yaw_output);

        splitter = new QSplitter(multi_control);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setGeometry(QRect(480, 40, 31, 161));
        splitter->setOrientation(Qt::Vertical);
        Z = new QSlider(splitter);
        Z->setObjectName(QStringLiteral("Z"));
        Z->setMinimum(-300);
        Z->setMaximum(300);
        Z->setOrientation(Qt::Vertical);
        splitter->addWidget(Z);
        Z_0 = new QPushButton(splitter);
        Z_0->setObjectName(QStringLiteral("Z_0"));
        splitter->addWidget(Z_0);
        Z_output = new QTextBrowser(multi_control);
        Z_output->setObjectName(QStringLiteral("Z_output"));
        Z_output->setGeometry(QRect(470, 210, 51, 31));
        Motion_Test = new QPushButton(multi_control);
        Motion_Test->setObjectName(QStringLiteral("Motion_Test"));
        Motion_Test->setGeometry(QRect(50, 240, 91, 23));
        tabWidget->addTab(multi_control, QString());
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(270, 380, 681, 451));
        showChart = new QVBoxLayout(verticalLayoutWidget);
        showChart->setSpacing(6);
        showChart->setContentsMargins(11, 11, 11, 11);
        showChart->setObjectName(QStringLiteral("showChart"));
        showChart->setContentsMargins(0, 0, 0, 0);
        btnShow = new QPushButton(centralWidget);
        btnShow->setObjectName(QStringLiteral("btnShow"));
        btnShow->setGeometry(QRect(90, 500, 75, 23));
        btnStop = new QPushButton(centralWidget);
        btnStop->setObjectName(QStringLiteral("btnStop"));
        btnStop->setGeometry(QRect(180, 500, 75, 23));
        layoutWidget6 = new QWidget(centralWidget);
        layoutWidget6->setObjectName(QStringLiteral("layoutWidget6"));
        layoutWidget6->setGeometry(QRect(100, 380, 162, 114));
        horizontalLayout_7 = new QHBoxLayout(layoutWidget6);
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        UP_Mid = new QPushButton(layoutWidget6);
        UP_Mid->setObjectName(QStringLiteral("UP_Mid"));

        verticalLayout_6->addWidget(UP_Mid);

        UP_STOP = new QPushButton(layoutWidget6);
        UP_STOP->setObjectName(QStringLiteral("UP_STOP"));

        verticalLayout_6->addWidget(UP_STOP);

        UP_ZERO = new QPushButton(layoutWidget6);
        UP_ZERO->setObjectName(QStringLiteral("UP_ZERO"));

        verticalLayout_6->addWidget(UP_ZERO);

        UP_Trans = new QPushButton(layoutWidget6);
        UP_Trans->setObjectName(QStringLiteral("UP_Trans"));
        UP_Trans->setCheckable(true);

        verticalLayout_6->addWidget(UP_Trans);


        horizontalLayout_7->addLayout(verticalLayout_6);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        DOWN_Mid = new QPushButton(layoutWidget6);
        DOWN_Mid->setObjectName(QStringLiteral("DOWN_Mid"));

        verticalLayout_7->addWidget(DOWN_Mid);

        DOWN_STOP = new QPushButton(layoutWidget6);
        DOWN_STOP->setObjectName(QStringLiteral("DOWN_STOP"));

        verticalLayout_7->addWidget(DOWN_STOP);

        DOWN_ZERO = new QPushButton(layoutWidget6);
        DOWN_ZERO->setObjectName(QStringLiteral("DOWN_ZERO"));

        verticalLayout_7->addWidget(DOWN_ZERO);

        DOWN_Trans = new QPushButton(layoutWidget6);
        DOWN_Trans->setObjectName(QStringLiteral("DOWN_Trans"));
        DOWN_Trans->setCheckable(true);

        verticalLayout_7->addWidget(DOWN_Trans);


        horizontalLayout_7->addLayout(verticalLayout_7);

        layoutWidget7 = new QWidget(centralWidget);
        layoutWidget7->setObjectName(QStringLiteral("layoutWidget7"));
        layoutWidget7->setGeometry(QRect(10, 400, 77, 54));
        verticalLayout_8 = new QVBoxLayout(layoutWidget7);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        UP_Start = new QPushButton(layoutWidget7);
        UP_Start->setObjectName(QStringLiteral("UP_Start"));
        UP_Start->setCheckable(true);

        verticalLayout_8->addWidget(UP_Start);

        DOWN_Start = new QPushButton(layoutWidget7);
        DOWN_Start->setObjectName(QStringLiteral("DOWN_Start"));
        DOWN_Start->setCheckable(true);

        verticalLayout_8->addWidget(DOWN_Start);

        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        start->setDefault(false);
        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\345\215\225\350\275\264\350\260\203\350\257\225", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251\347\224\265\346\234\272", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\351\200\237\345\272\246", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "\344\275\215\347\275\256\350\256\276\347\275\256", Q_NULLPTR));
        motornumber->setText(QApplication::translate("MainWindow", "\345\275\223\345\211\215\344\275\215\347\275\256", Q_NULLPTR));
        init->setText(QApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226", Q_NULLPTR));
        start->setText(QApplication::translate("MainWindow", "\344\274\272\346\234\215\345\220\257\345\212\250", Q_NULLPTR));
        rotate->setText(QApplication::translate("MainWindow", "\347\247\273\345\212\250", Q_NULLPTR));
        stop->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242", Q_NULLPTR));
        findslave->setText(QApplication::translate("MainWindow", "\345\215\225\347\274\270\345\233\236\351\233\266", Q_NULLPTR));
        reset->setText(QApplication::translate("MainWindow", "\346\270\205\351\233\266", Q_NULLPTR));
        PID->setText(QApplication::translate("MainWindow", "Single_PID", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(single_control), QApplication::translate("MainWindow", "\345\215\225\350\275\264\346\216\247\345\210\266", Q_NULLPTR));
        ALL_UP->setText(QApplication::translate("MainWindow", "ALL_UP", Q_NULLPTR));
        ALL_STOP->setText(QApplication::translate("MainWindow", "ALL_STOP", Q_NULLPTR));
        ALL_Mid->setText(QApplication::translate("MainWindow", "ALL_Mid", Q_NULLPTR));
        ALL_ZERO->setText(QApplication::translate("MainWindow", "ALL_ZERO", Q_NULLPTR));
        ALL_Trans->setText(QApplication::translate("MainWindow", "ALL_Trans", Q_NULLPTR));
        X_0->setText(QApplication::translate("MainWindow", "X_0", Q_NULLPTR));
        Y_0->setText(QApplication::translate("MainWindow", "Y_0", Q_NULLPTR));
        Pitch_0->setText(QApplication::translate("MainWindow", "Pitch_0", Q_NULLPTR));
        Roll_0->setText(QApplication::translate("MainWindow", "Roll_0", Q_NULLPTR));
        Yaw_0->setText(QApplication::translate("MainWindow", "Yaw_0", Q_NULLPTR));
        Z_0->setText(QApplication::translate("MainWindow", "Z_0", Q_NULLPTR));
        Motion_Test->setText(QApplication::translate("MainWindow", "Motion_Test", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(multi_control), QApplication::translate("MainWindow", "\345\244\232\350\275\264\346\216\247\345\210\266", Q_NULLPTR));
        btnShow->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\346\233\262\347\272\277", Q_NULLPTR));
        btnStop->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242\346\230\276\347\244\272", Q_NULLPTR));
        UP_Mid->setText(QApplication::translate("MainWindow", "UP_Mid", Q_NULLPTR));
        UP_STOP->setText(QApplication::translate("MainWindow", "UP_STOP", Q_NULLPTR));
        UP_ZERO->setText(QApplication::translate("MainWindow", "UP_ZERO", Q_NULLPTR));
        UP_Trans->setText(QApplication::translate("MainWindow", "UP_Trans", Q_NULLPTR));
        DOWN_Mid->setText(QApplication::translate("MainWindow", "DOWN_Mid", Q_NULLPTR));
        DOWN_STOP->setText(QApplication::translate("MainWindow", "DOWN_STOP", Q_NULLPTR));
        DOWN_ZERO->setText(QApplication::translate("MainWindow", "DOWN_ZERO", Q_NULLPTR));
        DOWN_Trans->setText(QApplication::translate("MainWindow", "DOWN_Trans", Q_NULLPTR));
        UP_Start->setText(QApplication::translate("MainWindow", "\344\274\272\346\234\215\345\220\257\345\212\250-\344\270\212", Q_NULLPTR));
        DOWN_Start->setText(QApplication::translate("MainWindow", "\344\274\272\346\234\215\345\220\257\345\212\250-\344\270\213", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
