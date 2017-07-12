/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-12-20

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaXML1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>

using namespace sawRobotIO1394;
const double WatchdogPeriod = 100.0 * cmn_ms;

int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    int portNumber = 0;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portNumber);
    options.AddOptionNoValue("b", "brakes",
                             "calibrate current feedback on brakes instead of actuators",
                              cmnCommandLineOptions::OPTIONAL_OPTION);

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

    std::cout << "Make sure:" << std::endl
              << " - your computer is connected to the firewire controller." << std::endl
              << " - the arm corresponding to the configuration file \"" << configFile << "\" is connected to the controller." << std::endl
              << " - the E-Stop is closed, i.e. will let the controller power on." << std::endl
              << " - you have no other device connected to the firewire chain." << std::endl
              << " - you have no other program trying to communicate with the controller." << std::endl
              << std::endl
              << "Press any key to start." << std::endl;
    cmnGetChar();

    std::cout << "Loading config file ..." << std::endl;
    osaPort1394Configuration config;
    osaXML1394ConfigurePort(configFile, config);

    std::cout << "Creating robot ..." << std::endl;
    if (config.Robots.size() == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (config.Robots.size() != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    osaRobot1394 * robot = new osaRobot1394(config.Robots[0]);
    size_t numberOfAxis = 0;
    bool brakes = false;
    if (options.IsSet("brakes")) {
        brakes = true;
        numberOfAxis = robot->NumberOfBrakes();
    } else {
        brakes = false;
        numberOfAxis = robot->NumberOfActuators();
    }
    vctDoubleVec zeros;
    zeros.SetSize(numberOfAxis);
    zeros.SetAll(0.0);

    std::cout << "Creating port ..." << std::endl;
    osaPort1394 * port = new osaPort1394(portNumber);
    port->AddRobot(robot);

    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
    // preload encoders
    robot->CalibrateEncoderOffsetsFromPots();

    std::cout << std::endl
              << "Ready to power?  Press any key to start." << std::endl;
    cmnGetChar();

    std::cout << "Enabling power ..." << std::endl;
    robot->SetWatchdogPeriod(300.0 * cmn_ms);
    if (brakes) {
        robot->SetBrakeCurrent(zeros);
    } else {
        robot->SetActuatorCurrent(zeros);
    }
    robot->EnablePower();
    port->Write();

    // wait a bit to make sure current stabilizes, 500 * 10 ms = 5 seconds
    for (size_t i = 0; i < 500; ++i) {
        if (brakes) {
            robot->SetBrakeCurrent(zeros);
        } else {
            robot->SetActuatorCurrent(zeros);
        }
        osaSleep(10.0 * cmn_ms);
        port->Write();
    }

    // check that power is on
    port->Read();
    if (!robot->PowerStatus()) {
        std::cerr << "Error: unable to power on controllers, make sure E-Stop is ok." << std::endl;
        delete port;
        return -1;
    }
    if (brakes) {
        if (!robot->BrakePowerStatus().All()) {
            std::cerr << "Error: failed to turn on brake power: " << robot->BrakePowerStatus() << std::endl;
            delete port;
            return -1;
        }
    } else {
        if (!robot->ActuatorPowerStatus().All()) {
            std::cerr << "Error: failed to turn on actuator power: " << robot->ActuatorPowerStatus() << std::endl;
            delete port;
            return -1;
        }
    }
    std::cout << "Status: power seems fine." << std::endl
              << "Starting calibration ..." << std::endl;

    // collect samples
    const size_t totalSamples = 50000;
    std::vector<vctDoubleVec> samples;
    samples.resize(totalSamples);
    vctDoubleVec sumSamples, averageAllSamples;
    sumSamples.SetSize(numberOfAxis);
    averageAllSamples.SetSize(numberOfAxis);
    sumSamples.SetAll(0.0);
    for (size_t index = 0; index < totalSamples; ++index) {
        // write to make sure watchdog is not tripped
        if (brakes) {
            robot->SetBrakeCurrent(zeros);
        } else {
            robot->SetActuatorCurrent(zeros);
        }
        port->Write();
        port->Read();
        if (brakes) {
            samples[index].ForceAssign(robot->BrakeCurrentFeedback());
        } else {
            samples[index].ForceAssign(robot->ActuatorCurrentFeedback());
        }
        samples[index].Multiply(1000.0); // convert all values to mA to be easier to read
        sumSamples.Add(samples[index]);
    }

    // disable power
    robot->DisablePower();
    port->Write();

    // compute simple average
    averageAllSamples.Assign(sumSamples);
    averageAllSamples.Divide(totalSamples);

    // compute standard deviation
    vctDoubleVec sumDifferencesSquared(numberOfAxis);
    vctDoubleVec difference(numberOfAxis);
    for (size_t index = 0; index < totalSamples; ++index) {
        difference.DifferenceOf(samples[index], averageAllSamples);
        sumDifferencesSquared.AddElementwiseProductOf(difference, difference);
    }
    vctDoubleVec stdDeviation(sumDifferencesSquared);
    stdDeviation.Divide(totalSamples);
    for (size_t index = 0; index < stdDeviation.size(); ++index) {
        stdDeviation[index] = sqrt(stdDeviation[index]);
    }

    // eliminate outliers
    vctDoubleVec lower(numberOfAxis);
    lower.DifferenceOf(averageAllSamples, stdDeviation);
    vctDoubleVec upper(numberOfAxis);
    upper.SumOf(averageAllSamples, stdDeviation);
    size_t validSamples = 0;
    vctDoubleVec averageValidSamples(numberOfAxis);
    sumSamples.SetAll(0.0);
    for (size_t index = 0; index < totalSamples; ++index) {
        if (samples[index].ElementwiseLesserOrEqual(upper).All()
            && samples[index].ElementwiseGreaterOrEqual(lower).All()) {
            sumSamples.Add(samples[index]);
            validSamples++;
        }
    }
    averageValidSamples.Assign(sumSamples);
    averageValidSamples.Divide(validSamples);

    // display results
    std::cout << "Status: average current feedback in mA: " << averageAllSamples << std::endl
              << "Status: standard deviation in mA:       " << stdDeviation << std::endl
              << "Status: kept " << validSamples << " samples out of " << totalSamples << std::endl
              << "Status: new average in mA:              " << averageValidSamples << std::endl
              << std::endl
              << "Do you want to update the config file with these values? [Y/y]" << std::endl;

    // save if needed
    char key;
    key = cmnGetChar();
    if ((key == 'y') || (key == 'Y')) {
        cmnXMLPath xmlConfig;
        xmlConfig.SetInputSource(configFile);
        std::string xmlQueryOffset, xmlQueryScale;
        if (brakes) {
            xmlQueryOffset = "Robot[1]/Actuator[%d]/AnalogBrake/AmpsToBits/@Offset";
            xmlQueryScale  = "Robot[1]/Actuator[%d]/AnalogBrake/AmpsToBits/@Scale";
        } else {
            xmlQueryOffset = "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Offset";
            xmlQueryScale  = "Robot[1]/Actuator[%d]/Drive/AmpsToBits/@Scale";
        }
        // query previous current offset and scales
        vctDoubleVec previousOffsets(numberOfAxis, 0.0);
        vctDoubleVec previousScales(numberOfAxis, 0.0);
        for (size_t index = 0; index < numberOfAxis; ++index) {
            char path[64];
            const char * context = "Config";
            sprintf(path, xmlQueryOffset.c_str(), static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousOffsets[index]);
            sprintf(path, xmlQueryScale.c_str(), static_cast<int>(index + 1));
            xmlConfig.GetXMLValue(context, path, previousScales[index]);
        }
        // compute new offsets
        vctDoubleVec newOffsets(numberOfAxis);
        newOffsets.Assign(averageValidSamples);
        newOffsets.Divide(-1000.0); // convert back to Amps and negate
        newOffsets.ElementwiseMultiply(previousScales);
        newOffsets.Add(previousOffsets);

        // ask one last confirmation from user
        std::cout << "Status: current offsets in XML configuration file: " << previousOffsets << std::endl
                  << "Status: new current offsets:                       " << newOffsets << std::endl
                  << std::endl
                  << "Do you want to save these values? [S/s]" << std::endl;
        key = cmnGetChar();
        if ((key == 's') || (key == 'S')) {
            vctIntVec newOffsetsInt(newOffsets);
            for (size_t index = 0; index < numberOfAxis; ++index) {
                char path[64];
                const char * context = "Config";
                sprintf(path, xmlQueryOffset.c_str(), static_cast<int>(index + 1));
                xmlConfig.SetXMLValue(context, path, newOffsetsInt[index]);
            }
            std::string newConfigFile = configFile + "-new";
            xmlConfig.SaveAs(newConfigFile);
            std::cout << "Status: new config file is \"" << newConfigFile << "\"" << std::endl;
        } else {
            std::cout << "Status: user didn't want to save new offsets." << std::endl;
        }
    } else {
        std::cout << "Status: no data saved in config file." << std::endl;
    }

    delete port;
    return 0;
}
