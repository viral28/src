/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <sawIntuitiveResearchKit/mtsTeleOperationPSMQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSMQtWidget, mtsComponent, std::string);

mtsTeleOperationPSMQtWidget::mtsTeleOperationPSMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timers are in milliseconds
    LogEnabled(false)
{
    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("TeleOperation");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetScale", TeleOperation.SetScale);
        interfaceRequired->AddFunction("LockRotation", TeleOperation.LockRotation);
        interfaceRequired->AddFunction("LockTranslation", TeleOperation.LockTranslation);
        interfaceRequired->AddFunction("GetPositionCartesianMTM", TeleOperation.GetPositionCartesianMTM);
        interfaceRequired->AddFunction("GetPositionCartesianPSM", TeleOperation.GetPositionCartesianPSM);
        interfaceRequired->AddFunction("GetRegistrationRotation", TeleOperation.GetRegistrationRotation);
        interfaceRequired->AddFunction("GetPeriodStatistics", TeleOperation.GetPeriodStatistics);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::DesiredStateEventHandler,
                                                this, "DesiredState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::CurrentStateEventHandler,
                                                this, "CurrentState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::ScaleEventHandler,
                                                this, "Scale");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::RotationLockedEventHandler,
                                                this, "RotationLocked");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::TranslationLockedEventHandler,
                                                this, "TranslationLocked");
        // messages
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::ErrorEventHandler,
                                                this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::WarningEventHandler,
                                                this, "Warning");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::StatusEventHandler,
                                                this, "Status");
    }
}

void mtsTeleOperationPSMQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationPSMQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
    if (!LogEnabled) {
        QTEMessages->hide();
    }
    if (!parent()) {
        show();
    }
}

void mtsTeleOperationPSMQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationPSMQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsTeleOperationPSMQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsTeleOperationPSMQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // retrieve transformations
    TeleOperation.GetPositionCartesianMTM(PositionMTM);
    TeleOperation.GetPositionCartesianPSM(PositionPSM);
    TeleOperation.GetRegistrationRotation(RegistrationRotation);

    // apply registration orientation
    vctFrm3 registeredPSM;
    RegistrationRotation.ApplyInverseTo(PositionPSM.Position().Rotation(), registeredPSM.Rotation());
    RegistrationRotation.ApplyInverseTo(PositionPSM.Position().Translation(), registeredPSM.Translation());

    // update display
    QFRPositionMTMWidget->SetValue(PositionMTM.Position());
    QFRPositionPSMWidget->SetValue(registeredPSM);

    TeleOperation.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsTeleOperationPSMQtWidget::SlotSetScale(double scale)
{
    TeleOperation.SetScale(scale);
}

void mtsTeleOperationPSMQtWidget::SlotLockRotation(bool lock)
{
    TeleOperation.LockRotation(lock);
}

void mtsTeleOperationPSMQtWidget::SlotLockTranslation(bool lock)
{
    TeleOperation.LockTranslation(lock);
}


void mtsTeleOperationPSMQtWidget::SlotDesiredStateEventHandler(QString state)
{
    QLEDesiredState->setText(state);
}

void mtsTeleOperationPSMQtWidget::SlotCurrentStateEventHandler(QString state)
{
    QLECurrentState->setText(state);
}

void mtsTeleOperationPSMQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsTeleOperationPSMQtWidget::SlotRotationLockedEventHandler(bool lock)
{
    QCBLockRotation->setChecked(lock);
}

void mtsTeleOperationPSMQtWidget::SlotTranslationLockedEventHandler(bool lock)
{
    QCBLockTranslation->setChecked(lock);
}

void mtsTeleOperationPSMQtWidget::setupUi(void)
{
    // 3D frames
    QGridLayout * frameLayout = new QGridLayout;
    QLabel * masterLabel = new QLabel("<b>MTM</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, 0);
    QFRPositionMTMWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMTMWidget, 1, 0);
    QLabel * slaveLabel = new QLabel("<b>PSM</b>");
    slaveLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(slaveLabel, 2, 0);
    QFRPositionPSMWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionPSMWidget, 3, 0);

    // right side
    QVBoxLayout * controlLayout = new QVBoxLayout;

    QLabel * instructionsLabel = new QLabel("To start tele-operation you must first insert the tool past the cannula tip (push tool clutch button and manually insert tool).\nOperator must be present to operate (sometime using COAG pedal).\nYou can use the clutch pedal to re-position your masters.");
    instructionsLabel->setWordWrap(true);
    controlLayout->addWidget(instructionsLabel);

    // scale/lock/unlock/messages
    QHBoxLayout * buttonsLayout = new QHBoxLayout;
    controlLayout->addLayout(buttonsLayout);

    // scale
    QSBScale = new QDoubleSpinBox();
    QSBScale->setRange(0.1, 1.0);
    QSBScale->setSingleStep(0.1);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(0.2);
    buttonsLayout->addWidget(QSBScale);

    // enable/disable rotation/translation
    QCBLockRotation = new QCheckBox("Lock Rotation");
    buttonsLayout->addWidget(QCBLockRotation);

    QCBLockTranslation = new QCheckBox("Lock Translation");
    buttonsLayout->addWidget(QCBLockTranslation);

    // messages on/off
    QPBLog = new QPushButton("Messages");
    QPBLog->setCheckable(true);
    buttonsLayout->addWidget(QPBLog);

    // state info
    QHBoxLayout * stateLayout = new QHBoxLayout;
    controlLayout->addLayout(stateLayout);

    QLabel * label = new QLabel("Desired");
    stateLayout->addWidget(label);
    QLEDesiredState = new QLineEdit("");
    QLEDesiredState->setReadOnly(true);
    stateLayout->addWidget(QLEDesiredState);

    label = new QLabel("Current");
    stateLayout->addWidget(label);
    QLECurrentState = new QLineEdit("");
    QLECurrentState->setReadOnly(true);
    stateLayout->addWidget(QLECurrentState);

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    controlLayout->addWidget(QMIntervalStatistics);

    // messages
    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    controlLayout->addWidget(QTEMessages);

    // add stretch
    controlLayout->addStretch();

    QWidget * leftWidget = new QWidget();
    leftWidget->setLayout(frameLayout);
    addWidget(leftWidget);

    QWidget * rightWidget = new QWidget();
    rightWidget->setLayout(controlLayout);
    addWidget(rightWidget);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());

    // setup Qt Connection
    connect(this, SIGNAL(SignalDesiredState(QString)),
            this, SLOT(SlotDesiredStateEventHandler(QString)));
    connect(this, SIGNAL(SignalCurrentState(QString)),
            this, SLOT(SlotCurrentStateEventHandler(QString)));

    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetScale(double)));
    connect(this, SIGNAL(SignalScale(double)),
            this, SLOT(SlotScaleEventHandler(double)));

    connect(QCBLockRotation, SIGNAL(clicked(bool)),
            this, SLOT(SlotLockRotation(bool)));
    connect(this, SIGNAL(SignalRotationLocked(bool)),
            this, SLOT(SlotRotationLockedEventHandler(bool)));

    connect(QCBLockTranslation, SIGNAL(clicked(bool)),
            this, SLOT(SlotLockTranslation(bool)));
    connect(this, SIGNAL(SignalTranslationLocked(bool)),
            this, SLOT(SlotTranslationLockedEventHandler(bool)));

    // messages
    connect(QPBLog, SIGNAL(clicked()),
            this, SLOT(SlotLogEnabled()));
    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));
}

void mtsTeleOperationPSMQtWidget::DesiredStateEventHandler(const std::string & state)
{
    emit SignalDesiredState(QString(state.c_str()));
}

void mtsTeleOperationPSMQtWidget::CurrentStateEventHandler(const std::string & state)
{
    emit SignalCurrentState(QString(state.c_str()));
}

void mtsTeleOperationPSMQtWidget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}

void mtsTeleOperationPSMQtWidget::RotationLockedEventHandler(const bool & lock)
{
    emit SignalRotationLocked(lock);
}

void mtsTeleOperationPSMQtWidget::TranslationLockedEventHandler(const bool & lock)
{
    emit SignalTranslationLocked(lock);
}

void mtsTeleOperationPSMQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsTeleOperationPSMQtWidget::SlotLogEnabled(void)
{
    LogEnabled = QPBLog->isChecked();
    if (LogEnabled) {
        QTEMessages->show();
    } else {
        QTEMessages->hide();
    }
}

void mtsTeleOperationPSMQtWidget::ErrorEventHandler(const std::string & message)
{
    if (LogEnabled) {
        emit SignalSetColor(QColor("red"));
        emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
    }
}

void mtsTeleOperationPSMQtWidget::WarningEventHandler(const std::string & message)
{
    if (LogEnabled) {
        emit SignalSetColor(QColor("darkRed"));
        emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Warning: ") + QString(message.c_str()));
    }
}

void mtsTeleOperationPSMQtWidget::StatusEventHandler(const std::string & message)
{
    if (LogEnabled) {
        emit SignalSetColor(QColor("black"));
        emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
    }
}
