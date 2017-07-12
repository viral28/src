/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Author(s):  Zihan Chen
  Created on: 2013-02-07

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsQtWidgetComponent.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>

//#define WITH_ROS
#ifdef WITH_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sawROS/mtsROSBridge.h>
#endif

// Qt includes
#include <QApplication>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create a Qt application
    QApplication application(argc, argv);

    // parse options
    cmnCommandLineOptions options;
    int port = 0;
    std::string configFile;
    std::string robotName = "Robot";
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);
    options.AddOptionOneValue("n", "name",
                              "robot name",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &robotName);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"" << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << port << std::endl;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // RobotIO
    mtsRobotIO1394 * robotIO = new mtsRobotIO1394("robotIO", 1 * cmn_ms, port);
    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");

    componentManager->AddComponent(robotIO);
    componentManager->AddComponent(robotWidgetFactory);

    robotIO->Configure(configFile);

    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "robotIO", "Configuration");
    robotWidgetFactory->Configure();

#ifdef WITH_ROS
    // ros wrapper
    mtsROSBridge robotBridge("RobotBridge", 1 * cmn_ms);
    robotBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>
        (robotName, "GetPositionJoint", "/" + robotName + "/joint_position");
    robotBridge.AddPublisherFromReadCommand<vctDoubleVec, sawROS::vctDoubleVec>
        (robotName, "GetVelocity", "/" + robotName + "/joint_velocity");

    robotBridge.AddPublisherFromReadCommand<vctDoubleVec, sawROS::vctDoubleVec>
        (robotName, "GetAnalogInputPosSI", "/" + robotName + "/pot_position");
    robotBridge.AddPublisherFromReadCommand<vctDoubleVec, sawROS::vctDoubleVec>
        (robotName, "GetAnalogInputVelSI", "/" + robotName + "/pot_velocity");

    componentManager->AddComponent(&robotBridge);
    componentManager->Connect(robotBridge.GetName(), robotName,
                              "robotIO", robotName);
#endif

    // create the components
    componentManager->CreateAllAndWait(2.0 * cmn_s);

    // start the periodic Run
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // run Qt app
    application.exec();

    // cleanup
    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete robotWidgetFactory;
    delete robotIO;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
