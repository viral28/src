/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Author(s):  Anton Deguet
  Created on: 2008

  (C) Copyright 2008-2011 Johns Hopkins University (JHU), All Rights
  Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---

*/

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawNovintFalcon/mtsNovintHDL.h>

#include "displayTask.h"
#include "displayUI.h"

int main(void)
{
    // log configuration
	cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
	cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    // specify a higher, more verbose log level for these classes
	cmnClassRegister::SetLogMaskClassMatching("mts", CMN_LOG_ALLOW_ALL);

    // create our two tasks
    const double PeriodDisplay = 10.0; // in milliseconds
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    displayTask * displayTaskObject =
        new displayTask("DISP", PeriodDisplay * cmn_ms);
    displayTaskObject->Configure();
    componentManager->AddComponent(displayTaskObject);

    // name as defined in Sensable configuration
    mtsNovintHDL * robotObject = new mtsNovintHDL("Novint", "Novint");
	componentManager->AddComponent(robotObject);

    // connect the tasks
    componentManager->Connect("DISP", "Robot", "Novint", "Novint");
    componentManager->Connect("DISP", "Button1", "Novint", "NovintButton1");
    componentManager->Connect("DISP", "Button2", "Novint", "NovintButton2");

    // create the tasks, i.e. find the commands
    componentManager->CreateAll();
	componentManager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    componentManager->StartAll();
	componentManager->WaitForStateAll(mtsComponentState::ACTIVE, 2.0 * cmn_s);

    // wait until the close button of the UI is pressed
    while (true) {
        osaSleep(10.0 * cmn_ms); // sleep to save CPU
        if (displayTaskObject->GetExitFlag()) {
            break;
        }
    }

    // cleanup
    componentManager->KillAll();
	componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

	componentManager->Cleanup();
    cmnLogger::Kill();

    return 0;
}
