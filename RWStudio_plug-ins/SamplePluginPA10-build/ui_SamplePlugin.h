/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *_btn0;
    QPushButton *_btn1;
    QPushButton *_btn2;
    QPushButton *_btn3;
    QCheckBox *_checkBox;
    QSpinBox *_spinBox;
    QSlider *_slider;
    QLabel *_winTitle;
    QLabel *_label;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QStringLiteral("SamplePlugin"));
        SamplePlugin->resize(428, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QStringLiteral("_btn0"));

        verticalLayout->addWidget(_btn0);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QStringLiteral("_btn1"));

        verticalLayout->addWidget(_btn1);

        _btn2 = new QPushButton(dockWidgetContents);
        _btn2->setObjectName(QStringLiteral("_btn2"));

        verticalLayout->addWidget(_btn2);

        _btn3 = new QPushButton(dockWidgetContents);
        _btn3->setObjectName(QStringLiteral("_btn3"));

        verticalLayout->addWidget(_btn3);

        _checkBox = new QCheckBox(dockWidgetContents);
        _checkBox->setObjectName(QStringLiteral("_checkBox"));

        verticalLayout->addWidget(_checkBox);

        _spinBox = new QSpinBox(dockWidgetContents);
        _spinBox->setObjectName(QStringLiteral("_spinBox"));

        verticalLayout->addWidget(_spinBox);

        _slider = new QSlider(dockWidgetContents);
        _slider->setObjectName(QStringLiteral("_slider"));
        _slider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(_slider);

        _winTitle = new QLabel(dockWidgetContents);
        _winTitle->setObjectName(QStringLiteral("_winTitle"));
        _winTitle->setMaximumSize(QSize(16777215, 23));
        _winTitle->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(_winTitle);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QStringLiteral("_label"));

        verticalLayout->addWidget(_label);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "ROVI 1 Final Project (SDU) - Visual Servoing PA10 ", 0));
        _btn0->setText(QApplication::translate("SamplePlugin", "PB0 - Setup", 0));
        _btn1->setText(QApplication::translate("SamplePlugin", "PB1 - Start/Stop 1 Point", 0));
        _btn2->setText(QApplication::translate("SamplePlugin", "PB2 - Start/Stop 3 Points", 0));
        _btn3->setText(QApplication::translate("SamplePlugin", "PB3 - Restart Simulation", 0));
        _checkBox->setText(QApplication::translate("SamplePlugin", "CheckBox", 0));
        _winTitle->setText(QApplication::translate("SamplePlugin", "CAMERA VIEW:", 0));
        _label->setText(QApplication::translate("SamplePlugin", "Label", 0));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
