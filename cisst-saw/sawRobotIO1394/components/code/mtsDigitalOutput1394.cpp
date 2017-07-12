/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-11-06

  (C) Copyright 2014-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstParameterTypes/prmEventButton.h>

#include "mtsDigitalOutput1394.h"

using namespace sawRobotIO1394;


mtsDigitalOutput1394::mtsDigitalOutput1394(const cmnGenericObject & owner,
                                           const osaDigitalOutput1394Configuration & config):
    osaDigitalOutput1394(config),
    OwnerServices(owner.Services())
{
}

void mtsDigitalOutput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mValue, mName + "Value");
}

void mtsDigitalOutput1394::SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable)
{
    osaDigitalOutput1394 * thisBase = dynamic_cast<osaDigitalOutput1394 *>(this);
    CMN_ASSERT(thisBase);

    interfaceProvided->AddCommandReadState(stateTable, this->mValue, "GetValue");
    interfaceProvided->AddCommandWrite(&osaDigitalOutput1394::SetValue, thisBase,
                                       "SetValue");
    if (mConfiguration.IsPWM) {
        interfaceProvided->AddCommandWrite(&osaDigitalOutput1394::SetPWMDutyCycle, thisBase,
                                           "SetPWMDutyCycle");
    }
}

void mtsDigitalOutput1394::CheckState(void)
{
    std::cerr << CMN_LOG_DETAILS
              << " --- nothing here?   Can we have outputs changed on us for no reason and we should emit event" << std::endl;
}
