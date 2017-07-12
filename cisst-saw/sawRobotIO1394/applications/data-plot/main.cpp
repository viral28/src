/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-01-09

  (C) Copyright 2014-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaXML1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>

// plot object
#include "plotObject.h"

// Qt
#include <QApplication>

int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    int portNumber = 0;
    int actuatorIndex = 0;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("a", "actuator",
                              "actuator index",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &actuatorIndex);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portNumber);
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"." << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << portNumber << std::endl;

    std::cout << "Loading config file ..." << std::endl;
    sawRobotIO1394::osaPort1394Configuration config;
    sawRobotIO1394::osaXML1394ConfigurePort(configFile, config);

    std::cout << "Creating robot ..." << std::endl;
    if (config.Robots.size() == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (config.Robots.size() != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    sawRobotIO1394::osaRobot1394 * robot = new sawRobotIO1394::osaRobot1394(config.Robots[0]);

    std::cout << "Creating port ..." << std::endl;
    sawRobotIO1394::osaPort1394 * port = new sawRobotIO1394::osaPort1394(portNumber);
    port->AddRobot(robot);

    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
    // preload encoders
    robot->CalibrateEncoderOffsetsFromPots();

    QApplication application(argc, argv);
    plotObject * plot = new plotObject(port, robot, actuatorIndex);

    application.exec();

    delete plot;
    delete port;
    return 0;
}
