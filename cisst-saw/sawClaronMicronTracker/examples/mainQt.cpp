/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-11-06

  (C) Copyright 2009-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief An example interface for Claron Micron Tracker.
  \ingroup devicesTutorial
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaThreadedLogFile.h>
#include <cisstMultiTask/mtsCollectorState.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawClaronMicronTracker/mtsMicronTracker.h>
#include <sawClaronMicronTracker/mtsMicronTrackerControllerQtComponent.h>
#include <sawClaronMicronTracker/mtsMicronTrackerToolQtComponent.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsMicronTracker", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create a Qt user interface
    QApplication application(argc, argv);

    // create the components
    mtsMicronTracker * componentMicronTracker = new mtsMicronTracker("componentMicronTracker", 50.0 * cmn_ms);
    mtsMicronTrackerControllerQtComponent * componentControllerQtComponent = new mtsMicronTrackerControllerQtComponent("componentControllerQtComponent");

    // configure the components
    cmnPath searchPath;
    searchPath.Add(cmnPath::GetWorkingDirectory());
    std::string configPath = searchPath.Find("configClaronMicronTracker.xml");
    if (configPath.empty()) {
        std::cerr << "Failed to find configuration: " << configPath << std::endl;
        return 1;
    }
    componentMicronTracker->Configure(configPath);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(componentMicronTracker);
    componentManager->AddComponent(componentControllerQtComponent);

    // connect the components, e.g. RequiredInterface -> ProvidedInterface
    componentManager->Connect(componentControllerQtComponent->GetName(), "Controller",
                              componentMicronTracker->GetName(), "Controller");

    // add data collection for mtsMicronTracker state table
    mtsCollectorState * componentCollector =
        new mtsCollectorState(componentMicronTracker->GetName(),
                              componentMicronTracker->GetDefaultStateTableName(),
                              mtsCollectorBase::COLLECTOR_FILE_FORMAT_CSV);

    // add interfaces for tools and populate controller widget with tool widgets
    for (unsigned int i = 0; i < componentMicronTracker->GetNumberOfTools(); i++) {
        std::string toolName = componentMicronTracker->GetToolName(i);
        mtsMicronTrackerToolQtComponent * componentToolQtComponent = new mtsMicronTrackerToolQtComponent(toolName);
        componentControllerQtComponent->AddTool(componentToolQtComponent,
                                                componentToolQtComponent->GetWidget(),
                                                componentToolQtComponent->GetMarkerProjectionLeft(),
                                                componentToolQtComponent->GetMarkerProjectionRight());
        componentManager->AddComponent(componentToolQtComponent);
        componentManager->Connect(toolName, toolName,
                                  componentMicronTracker->GetName(), toolName);

        componentCollector->AddSignal(toolName + "Position");
    }
    componentManager->AddComponent(componentCollector);
    componentCollector->Connect();
    componentManager->Connect(componentControllerQtComponent->GetName(), "DataCollector",
                              componentCollector->GetName(), "Control");

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // create a main window to hold QWidgets
    QMainWindow * mainWindow = new QMainWindow();
    mainWindow->setCentralWidget(componentControllerQtComponent->GetWidget());
    mainWindow->setWindowTitle("MicronTracker Controller");
    mainWindow->resize(0,0);
    mainWindow->show();

    // run Qt user interface
    application.exec();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
