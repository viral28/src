/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaPort1394_h
#define _osaPort1394_h

#include <vector>
#include <map>

#ifdef SAW_ROBOT_IO_1394_WO_CISST
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "EigenWrapper.h"
#include "MinimalPrm.h"
#endif

#include <sawRobotIO1394/sawRobotIO1394ForwardDeclarations.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaRobot1394.h>
#include <sawRobotIO1394/osaDigitalInput1394.h>
#include <sawRobotIO1394/osaDigitalOutput1394.h>
#include <sawRobotIO1394/sawRobotIO1394Export.h>

namespace sawRobotIO1394 {

    class CISST_EXPORT osaPort1394 {
        /**
         * IO1394 Port Abstraction Layer
         * This class handles allocation, interfacing, and power-control for the QLA
         * robot control architecture. It is also responsible for some low-level error
         * handling.
         */

    public:

        osaPort1394(int portNumber, std::ostream & messageStream = std::cerr);
        ~osaPort1394();

        //! Set Firewire protocol
        void SetProtocol(const sawRobotIO1394::ProtocolType & protocol);

        //! Configure the port using the specified configuration file. This method is not
        //  used by the mtsRobotIO1394 SAW component.
        void Configure(const osaPort1394Configuration & config);

        //! Add a robot to this port. This method is used by the mtsRobotIO1394 SAW component,
        // which provides an instance of an mtsRobot1394 component (derived from osaRobot1394).
        // This class takes "ownership" of the pointer and deletes it in the destructor (it
        // assumes the object was dynamically created).
        void AddRobot(osaRobot1394 * Robot);

        //! Add a digital input to this port. This method is used by the mtsRobotIO1394 SAW component,
        // which provides an instance of an mtsDigitalInput1394 component (derived from osaDigitalInput1394).
        // This class takes "ownership" of the pointer and deletes it in the destructor (it
        // assumes the object was dynamically created).
        void AddDigitalInput(osaDigitalInput1394 * digitalInput);
        void AddDigitalOutput(osaDigitalOutput1394 * digitalInput);

        //! Robot Accessors
        osaRobot1394 * Robot(const std::string & name);
        const osaRobot1394 * Robot(const std::string & name) const;

        osaRobot1394 * Robot(const int index);
        const osaRobot1394 * Robot(const int index) const;

        void GetRobotNames(std::vector<std::string> & names) const;
        void GetDigitalInputNames(std::vector<std::string> & names) const;
        void GetDigitalOutputNames(std::vector<std::string> & names) const;

        //! Input/Ouput
        void Read(void);
        void Write(void);

        int NumberOfBoards(void) const;
        int NumberOfRobots(void) const;
        int NumberOfDigitalInputs(void) const;
        int NumberOfDigitalOutputs(void) const;

    protected:

        //! Board Objects
        FirewirePort * mPort;

        std::map<int, AmpIO*> mBoards;
        typedef std::map<int, AmpIO*>::iterator board_iterator;
        typedef std::map<int, AmpIO*>::const_iterator board_const_iterator;

        //! Robot Objects
        std::vector<osaRobot1394*> mRobots;
        std::map<std::string, osaRobot1394*> mRobotsByName;
        typedef std::vector<osaRobot1394*>::iterator robot_iterator;
        typedef std::vector<osaRobot1394*>::const_iterator robot_const_iterator;
        typedef std::map<std::string, osaRobot1394*>::iterator robotByName_iterator;
        typedef std::map<std::string, osaRobot1394*>::const_iterator robotByName_const_iterator;

        std::vector<osaDigitalInput1394*> mDigitalInputs;
        std::map<std::string, osaDigitalInput1394*> mDigitalInputsByName;
        typedef std::vector<osaDigitalInput1394*>::iterator digital_input_iterator;
        typedef std::vector<osaDigitalInput1394*>::const_iterator digital_input_const_iterator;

        std::vector<osaDigitalOutput1394*> mDigitalOutputs;
        std::map<std::string, osaDigitalOutput1394*> mDigitalOutputsByName;
        typedef std::vector<osaDigitalOutput1394*>::iterator digital_output_iterator;
        typedef std::vector<osaDigitalOutput1394*>::const_iterator digital_output_const_iterator;
    };

} // namespace sawRobotIO1394

#endif // _osaPort1394_h
