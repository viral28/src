/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s): Marcin Balicki
  Created on: 2011-02-10

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _sdpPlayerPlot2D_h
#define _sdpPlayerPlot2D_h

#include <QObject>
#include <QtGui/QCloseEvent>

#include "ui_sdpPlayerWidget.h"
#include "sdpPlayerBase.h"
#include "sdpPlayerParseStateTableData.h"
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstOSAbstraction/osaCriticalSection.h>

// Always include last
#include "sdpExport.h"

class CISST_EXPORT sdpPlayerPlot2D: public sdpPlayerBase
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    sdpPlayerPlot2D(const std::string & name, double period);
    ~sdpPlayerPlot2D();

    //call this after tasks connections are made
    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void) {};
    void SetSynced(bool isSynced);



private:
    Ui::sdpPlayerWidget ExWidget;
    QWidget * mainWidget;
    QGridLayout * CentralLayout ;
    QDoubleSpinBox * ScaleZoom;
    QLabel * ZoomInOut;
    vctPlot2DOpenGLQtWidget * Plot;
    vctPlot2DBase::Signal * SignalPointer;
    vctPlot2DBase::VerticalLine * VerticalLinePointer;

    std::vector <double> *TimeStamps;
    std::vector <double> *Data;

    double TopBoundary, LowBoundary;

    QDoubleSpinBox *UpperYSpinBox;
    QDoubleSpinBox *LowerYSpinBox;


    // PoolPoint is pointing to ping pong buffer
    mtsInt PoolPoint;
    // VectorIndex is point to the data we are playing
    mtsInt VectorIndex;
    double ZoomScaleValue;
    mtsDouble LastTime;
    mtsDouble Plot2DTime;
    double TimeBoundary;

    struct{
        mtsFunctionRead GetVectorIndex;
        mtsFunctionRead GetZoomScale;
        mtsFunctionWrite WriteVectorIndex;
    }Plot2DAccess;

    // Parser for data file
    mtsTaskManager * taskManager;
    sdpPlayerParseStateTableData Parser;
    osaCriticalSection CS;

    void SetVectorIndex(const mtsInt & index){ VectorIndex = index; };
    
    void MakeQTConnections(void);
    //by calling "emit QSignalQTUpdate" this function will be called.
    //used this to udpate qt widgets in a thread safe way.
    void UpdateQT(void);
    void UpdatePlot(void);

    void Stop(const mtsDouble & time);
    void Seek(const mtsDouble & time);
    void Play(const mtsDouble & time);
    void Save(const sdpSaveParameters & saveParameters);
    void Quit(void);
    void LoadData(void);
    void UpdateLimits(void);

    void OpenFile(void);
    bool ExtractDataFromStateTableCSVFile(std::string & path);
    void ResetPlayer(void);

    std::string FileName;

private slots:
    void QSlotSeekSliderMoved(int c);
    void QSlotSyncCheck(bool checked);
    void QSlotPlayClicked(void);
    void QSlotStopClicked(void);
    void QSlotSetSaveStartClicked(void);
    void QSlotSetSaveEndClicked(void);
    void QSlotOpenFileClicked(void);
    void QSlotSpinBoxValueChanged(double);
    void QSlotUpperYRangeSpinChanged(double);
    void QSlotLowerYRangeSpinChanged(double);
};

CMN_DECLARE_SERVICES_INSTANTIATION(sdpPlayerPlot2D);

#endif // _sdpPlayerPlot2D_h
