/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDigitalInput1394_h
#define _mtsDigitalInput1394_h

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/osaDigitalInput1394.h>

namespace sawRobotIO1394 {

    class mtsDigitalInput1394: public osaDigitalInput1394 {
    public:
        /*! Pointer on existing services.  This allows to use the class
          name and level of detail of another class, e.g. the class that
          owns this map.  To set the "Owner", use the method SetOwner
          after the cmnNamedMap is constructed. */
        const cmnClassServicesBase * OwnerServices;

        /*! Method used to emulate the cmnGenericObject interface used by
          CMN_LOG_CLASS macros. */
        //@{
        inline const cmnClassServicesBase * Services(void) const {
            return this->OwnerServices;
        }

        inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
            return cmnLogger::GetMultiplexer();
        }
        //@}

        mtsDigitalInput1394(const cmnGenericObject & owner,
                            const osaDigitalInput1394Configuration & config);

        void SetupStateTable(mtsStateTable & stateTable);
        void SetupProvidedInterface(mtsInterfaceProvided * interfaceProvided, mtsStateTable & stateTable);

        /*! Check state and trigger events as needed. */
        void CheckState(void);

    protected:
        mtsFunctionWrite Button;    // The event function for button, will return prmEventButton
    };

} // namespace sawRobotIO1394

#endif // _mtsDigitalInput1394_h
