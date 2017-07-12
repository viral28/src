/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsStateTable.h>
#include <cisstParameterTypes/prmEventButton.h>

#include "mtsDigitalInput1394.h"

using namespace sawRobotIO1394;


mtsDigitalInput1394::mtsDigitalInput1394(const cmnGenericObject & owner,
                                         const osaDigitalInput1394Configuration & config):
    osaDigitalInput1394(config),
    OwnerServices(owner.Services())
{
}

void mtsDigitalInput1394::SetupStateTable(mtsStateTable & stateTable)
{
    stateTable.AddData(mValue, mName + "Value");
}

void mtsDigitalInput1394::SetupProvidedInterface(mtsInterfaceProvided * prov, mtsStateTable & stateTable)
{
    prov->AddCommandReadState(stateTable, this->mValue, "GetButton");
    prov->AddEventWrite(this->Button, "Button", prmEventButton());
}

void mtsDigitalInput1394::CheckState(void)
{
    static const prmEventButton
        pressed = prmEventButton::PRESSED,
        released = prmEventButton::RELEASED;

    // Send appropriate events if the value changed in the last update

    // Check if value has changed
    if (mValue != mPreviousValue) {
        // Check if the value is equal to the value when the digital input is considered pressed
        if (mValue) {
            // Emit a press event
            if (mTriggerPress) {
                Button(pressed);
            }
        } else {
            // Emit a release event
            if (mTriggerRelease) {
                Button(released);
            }
        }
    }
}
