/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-10-29

  (C) Copyright 2009-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsNDISerialControllerQtComponent_h
#define _mtsNDISerialControllerQtComponent_h

#include <QTimer>
#include <QWidget>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstMultiTask/mtsFunctionVoid.h>
#include <sawNDITracker/sawNDITrackerExportQt.h>  // always include last

namespace Ui {
    class mtsNDISerialControllerQtWidget;
}

class CISST_EXPORT mtsNDISerialControllerQtComponent : public QObject, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsNDISerialControllerQtComponent(const std::string & taskName);
    ~mtsNDISerialControllerQtComponent(void) {};

    void Configure(const std::string & CMN_UNUSED(filename) = "") {};

    void AddTool(QObject * toolQtComponent, QWidget * toolQtWidget);

    QWidget * GetWidget(void) {
        return &CentralWidget;
    }

 protected:
    Ui::mtsNDISerialControllerQtWidget * ControllerWidget;
    QWidget CentralWidget;
    QTimer * Timer;

    struct {
        mtsFunctionWrite Beep;
        mtsFunctionVoid Initialize;
        mtsFunctionVoid Query;
        mtsFunctionVoid Enable;
        mtsFunctionWrite CalibratePivot;
        mtsFunctionWrite Track;
        mtsFunctionVoid ReportStrayMarkers;
    } NDI;

    struct {
        mtsFunctionVoid Start;
        mtsFunctionVoid Stop;
    } Collector;

 public slots:
    void NDIBeepQSlot(void);
    void NDIInitializeQSlot(void);
    void NDICalibratePivotQSlot(void);
    void NDITrackQSlot(bool toggled);
    void NDIReportStrayMarkersQSlot(void);
    void RecordQSlot(bool toggled);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsNDISerialControllerQtComponent);

#endif  // _mtsNDISerialControllerQtComponent_h
