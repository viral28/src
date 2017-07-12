/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-10-27

  (C) Copyright 2009-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsMicronTrackerToolQtComponent_h
#define _mtsMicronTrackerToolQtComponent_h

#include <QObject>
#include <QWidget>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawClaronMicronTracker/sawClaronMicronTrackerExportQt.h>  // always include last

namespace Ui {
    class mtsMicronTrackerToolQtWidget;
}

class CISST_EXPORT mtsMicronTrackerToolQtComponent : public QObject, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsMicronTrackerToolQtComponent(const std::string & taskName);
    ~mtsMicronTrackerToolQtComponent(void) {};

    void Configure(const std::string & CMN_UNUSED(filename) = "") {};

    QWidget * GetWidget(void) {
        return &CentralWidget;
    }
    QPoint * GetMarkerProjectionLeft(void) {
        return &MarkerProjectionLeft;
    }
    QPoint * GetMarkerProjectionRight(void) {
        return &MarkerProjectionRight;
    }

 protected:
    Ui::mtsMicronTrackerToolQtWidget * ToolWidget;
    QWidget CentralWidget;

    struct {
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionRead GetMarkerProjectionLeft;
        mtsFunctionRead GetMarkerProjectionRight;
        mtsFunctionRead GetMarkerTemplateTrackingPositions;
        
        prmPositionCartesianGet PositionCartesian;
        mtsDoubleVec MarkerProjectionLeft;
        mtsDoubleVec MarkerProjectionRight;
        std::vector<vct3> MarkerTemplateTrackingPositions;
        
    } MTC;

    QPoint MarkerProjectionLeft;
    QPoint MarkerProjectionRight;


 public slots:
    void UpdatePositionCartesian();
    void RecordQSlot(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsMicronTrackerToolQtComponent);

#endif  // _mtsMicronTrackerToolQtComponent_h
