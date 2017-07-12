/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-10-27

  (C) Copyright 2009-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsNDISerialToolQtComponent_h
#define _mtsNDISerialToolQtComponent_h

#include <QObject>
#include <QWidget>

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsFunctionRead.h>
#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawNDITracker/sawNDITrackerExportQt.h>  // always include last

namespace Ui {
    class mtsNDISerialToolQtWidget;
}

class CISST_EXPORT mtsNDISerialToolQtComponent : public QObject, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsNDISerialToolQtComponent(const std::string & taskName);
    ~mtsNDISerialToolQtComponent(void) {};

    void Configure(const std::string & CMN_UNUSED(filename) = "") {};

    QWidget * GetWidget(void) {
        return &CentralWidget;
    }

 protected:
    Ui::mtsNDISerialToolQtWidget * ToolWidget;
    QWidget CentralWidget;

    struct {
        mtsFunctionRead GetPositionCartesian;
        prmPositionCartesianGet PositionCartesian;
    } NDI;

 public slots:
    void UpdatePositionCartesian(void);
    void RecordQSlot(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsNDISerialToolQtComponent);

#endif  // _mtsNDISerialToolQtComponent_h
