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

#ifndef _osaDigitalOutput1394_h
#define _osaDigitalOutput1394_h

#include <sawRobotIO1394/osaConfiguration1394.h>
#include "AmpIO.h"
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class CISST_EXPORT osaDigitalOutput1394 {
    public:
        osaDigitalOutput1394(const osaDigitalOutput1394Configuration & config);

        void Configure(const osaDigitalOutput1394Configuration & config);
        void SetBoard(AmpIO * board);

        void PollState(void);

        const osaDigitalOutput1394Configuration & Configuration(void) const;
        const std::string & Name(void) const;
        const bool & Value(void) const;
        void SetValue(const bool & newValue);
        void SetPWMDutyCycle(const double & dutyCycle);

    protected:
        AmpIO * mBoard;              // Board Assignment

        osaDigitalOutput1394Configuration mConfiguration;
        std::string mName;
        int mBitID;                  // Board assigned bitID for this Digital Output
        AmpIO_UInt32 mBitMask;       // BitMask for this input. From DigitalOutput Stream.

        // State data
        AmpIO_UInt32 mDigitalOutputBits; // BitMask for this output. From DigitalOutput Stream.
        bool mValue;                     // Current read value
    };

}

#endif // _osaDigitalOutput1394_h
