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

#ifndef _sdpPlayerAudio_h
#define _sdpPlayerAudio_h

#include <QObject>
#include <QtGui/QCloseEvent>
#include "ui_sdpPlayerWidget.h"
#include <sawOpenAL/osaOpenAL.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <QProgressBar>
#include "sdpPlayerBase.h"


//! todo Think about the thread safety issue. (At the moment, mtscommands and qslots can manipulte same data at the same time)
//! If the data processing is not heavy we can use the main QT thread via QTimer and mtsTaskFromCallback, or add more mtsFunction calls.
//! for example if we load audio file and presss play at the same time, then not all the data could be laoded which
//! can cause a segfault.

// Always include last
#include "sdpExport.h"

class CISST_EXPORT sdpPlayerAudio: public sdpPlayerBase
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    sdpPlayerAudio(const std::string & name, double period);
    ~sdpPlayerAudio();

    //call this after tasks connections are made
    void Configure(const std::string & filename = "");

    void Startup(void);
    void Run(void);
    void Cleanup(void) {};

    //If not synced then the player needs to go through Manager in order to play.
    //void SetSync(bool isSynced);
    void SetSynced(bool isSynced);


private:

    Ui::sdpPlayerWidget ExWidget;
    void MakeQTConnections(void);
    //by calling "emit QSignalQTUpdate" this function will be called.
    //used this to udpate qt widgets in a thread safe way.
    void UpdateQT(void);

    void Stop(const mtsDouble & time);
    void Seek(const mtsDouble & time);
    void Play(const mtsDouble & time);
    void Save(const sdpSaveParameters & saveParameters);
    void Quit(void);


    //not thread safe, use baseaccess to execute in mts thread.
    void LoadData(void);
    void UpdateLimits(void);


    QWidget                     MainWindow;
    vctPlot2DOpenGLQtWidget     * Plot;
    vctPlot2DBase::Signal       * DataTrace;

    QHBoxLayout                 *HorizontalLayout;
    QVBoxLayout                 *SliderVLayout;
    QProgressBar                *VolumeProgressBar;

    QLabel                      *label;
    QSlider                     *VolumeSlider;

    osaOpenAL                   Audio;
    std::string                 FileName;

private slots:

    void QSlotSeekSliderMoved(int c);
    void QSlotSyncCheck(bool checked);
    void QSlotPlayClicked(void);
    void QSlotStopClicked(void);
    void QSlotSetSaveStartClicked(void);
    void QSlotSetSaveEndClicked(void);
    void QSlotOpenFileClicked(void);
    void QSlotVolumeSliderMoved(int v);
    void QSlotSetRangeClicked(void);

signals:
   void QSignalUpdateRange(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(sdpPlayerAudio);

#endif // _sdpPlayerAudio_h
