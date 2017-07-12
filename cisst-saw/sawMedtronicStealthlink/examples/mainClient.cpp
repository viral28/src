/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Peter Kazanzides, Anton Deguet
  Created on: 2011-07-14

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstCommon/cmnGetChar.h>
#include <sawMedtronicStealthlink/mtsMedtronicStealthlink.h>
#include "mtsMedtronicStealthlinkExampleComponent.h"

int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // set the log level of detail on select components
    cmnLogger::SetMaskClassMatching("mts", CMN_LOG_ALLOW_ALL);

    // enable system-wide thread-safe logging
#ifdef MTS_LOGGING
    mtsManagerLocal::SetLogForwarding(true);
#endif

    std::string globalComponentManagerIP;
    int stateCollectionFlag = 0;

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " [GlobalManagerIP] [flagForDataCollection]" << std::endl;
        std::cerr << "       GlobalManagerIP is set as 127.0.0.1 by default" << std::endl;
    }

    // Set global component manager's ip and argument type
    // If flag is not specified, or not 1, then use generic type (mtsDouble)
    if (argc == 1) {
        globalComponentManagerIP = "localhost";
    } else if (argc == 2) {
        globalComponentManagerIP = argv[1];
    }else if (argc == 3) {
        globalComponentManagerIP = argv[1];
        stateCollectionFlag = atoi(argv[2]);
    } else {
        exit(-1);
    }

    std::cout << "Starting server, IP = " << globalComponentManagerIP << " 'q' to stop" << std::endl;
    if(stateCollectionFlag){
        std::cout << "        with data collection. 's' to start/stop data collection" << std::endl;
    }

    // Get the TaskManager instance and set operation mode
    mtsManagerLocal * componentManager;
    try {
        componentManager = mtsManagerLocal::GetInstance(globalComponentManagerIP, "ProcessClient");
    } catch (...) {
        CMN_LOG_INIT_ERROR << "Failed to initialize local component manager" << std::endl;
        return 1;
    }

    // client
    mtsMedtronicStealthlinkExampleComponent * componentExample;
    if(stateCollectionFlag)
        componentExample= new mtsMedtronicStealthlinkExampleComponent("Example", 50 * cmn_ms, true);
    else
        componentExample = new mtsMedtronicStealthlinkExampleComponent("Example", 50 * cmn_ms);

    // add the components to the component manager
    componentManager->AddComponent(componentExample);

    // Connect the test component to the Stealthlink component
    if (!componentManager->Connect("ProcessClient", componentExample->GetName(), "Stealthlink",
                                   "ProcessServer", "Stealthlink", "Controller")) {
        CMN_LOG_INIT_ERROR << "Could not connect test component to Stealthlink component." << std::endl;
        return 0;
    }

    // Now, connect to the tools (we assume these are pre-defined in the XML file)
    if (!componentManager->Connect("ProcessClient", componentExample->GetName(), "Pointer",
                                   "ProcessServer", "Stealthlink", "Pointer")) {
        CMN_LOG_INIT_WARNING << "Could not connect test component to Pointer tool." << std::endl;
    }
    if (!componentManager->Connect("ProcessClient", componentExample->GetName(), "Frame",
                                   "ProcessServer", "Stealthlink",  "Frame")) {
        CMN_LOG_INIT_WARNING << "Could not connect test component to Frame tool." << std::endl;
    }

    // Connect the registration interface
    if (!componentManager->Connect("ProcessClient", componentExample->GetName(), "Registration",
                                   "ProcessServer", "Stealthlink",  "Registration")) {
        CMN_LOG_INIT_ERROR << "Could not connect test component to Registration interface." << std::endl;
        return 0;
    }

    // Connect the exam information interface
    if (!componentManager->Connect("ProcessClient", componentExample->GetName(), "ExamInformation",
                                   "ProcessServer", "Stealthlink", "ExamInformation")) {
        CMN_LOG_INIT_ERROR << "Could not connect test component to ExamInformation interface." << std::endl;
        return 0;
    }
    
    if(stateCollectionFlag)
    {
        if (!componentManager->Connect("ProcessClient", componentExample->GetName(), "CollectorState",
                                       "ProcessServer", "StealthlinkStateCollector", "Control")) {
            CMN_LOG_INIT_ERROR << "Could not connect test component to Control interface." << std::endl;
            return 0;
        }
    }

    // create the tasks, i.e. find the commands
    componentManager->CreateAll();
    componentManager->WaitForStateAll(mtsComponentState::READY);

    // start the periodic Run
    componentManager->StartAll();
    componentManager->WaitForStateAll(mtsComponentState::ACTIVE);

    // wait for keyboard input in command window
    int ch;
    // execution result used by all functions
    mtsExecutionResult executionResult;

    bool GCMActive = true;
    bool started = false;
    while (GCMActive && ch != 'q') {
        osaSleep(5.0 * cmn_ms);
        GCMActive = componentManager->IsGCMActive();

        ch = cmnGetChar();

        switch (ch) {
        case 's':
            if(started)
            {
                if(componentExample->CollectorState.StopCollection.IsValid())
                {
                    executionResult = componentExample->CollectorState.StopCollection();
                    osaSleep(0.1 * cmn_s);
                    std::cout << "Stop data collection" << std::endl;
                    started = false;
                }
            }
            else
            {
                if(componentExample->CollectorState.StartCollection.IsValid())
                {
                    executionResult = componentExample->CollectorState.StartCollection();
                    osaSleep(0.1 * cmn_s);
                    if(executionResult.IsOK())
                    {
                        std::cout << "Start data collection" << std::endl;
                        started = true;
                    }
                    else
                        std::cout << "StartCollection failed to execute." << std::endl;

                }
            }
            break;

        default:
            break;
        }

    }

    if (!GCMActive) {
        CMN_LOG_RUN_ERROR << "Global Component Manager is disconnected" << std::endl;
    }

    // cleanup
    componentManager->KillAll();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 20.0 * cmn_s);

    // delete components
    delete componentExample;

    componentManager->Cleanup();

    return 0;
}
