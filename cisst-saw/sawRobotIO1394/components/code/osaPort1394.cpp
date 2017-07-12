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

#include "FirewirePort.h"
#include "AmpIO.h"

#include <sawRobotIO1394/osaPort1394.h>
#include <stdexcept>
#include <exception>

using namespace sawRobotIO1394;

osaPort1394::osaPort1394(int portNumber, std::ostream & messageStream)
{
    // Construct handle to firewire port
    mPort = new FirewirePort(portNumber, messageStream);

    // Check number of port users
    if (mPort->NumberOfUsers() > 1) {
        std::ostringstream oss;
        oss << "osaIO1394Port: Found more than one user on firewire port: " << portNumber;
        cmnThrow(osaRuntimeError1394(oss.str()));
    }
}

void osaPort1394::SetProtocol(const ProtocolType & protocol)
{
    switch (protocol) {
    case PROTOCOL_SEQ_RW:
        mPort->SetProtocol(FirewirePort::PROTOCOL_SEQ_RW);
        break;
    case PROTOCOL_SEQ_R_BC_W:
        mPort->SetProtocol(FirewirePort::PROTOCOL_SEQ_R_BC_W);
        break;
    case PROTOCOL_BC_QRW:
        mPort->SetProtocol(FirewirePort::PROTOCOL_BC_QRW);
        break;
    default:
        break;
    }
}

void osaPort1394::Configure(const osaPort1394Configuration & config)
{
    // Add all the robots
    for (std::vector<osaRobot1394Configuration>::const_iterator iter = config.Robots.begin();
         iter != config.Robots.end();
         ++iter) {
        osaRobot1394 * robot = new osaRobot1394(*iter);
        this->AddRobot(robot);
    }

    // Add all the digital inputs
    for (std::vector<osaDigitalInput1394Configuration>::const_iterator it = config.DigitalInputs.begin();
         it != config.DigitalInputs.end();
         ++it) {
        osaDigitalInput1394 * digitalInput = new osaDigitalInput1394(*it);
        this->AddDigitalInput(digitalInput);
    }
}

void osaPort1394::AddRobot(osaRobot1394 * robot)
{
    if (robot == 0) {
        cmnThrow(osaRuntimeError1394("osaPort1394::AddRobot: Robot pointer is null."));
    }

    const osaRobot1394Configuration & config = robot->GetConfiguration();

    // Check to make sure this robot isn't already added
    if (mRobotsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(robot->Name() + ": robot name is not unique."));
    }

    // Construct a vector of boards relevant to this robot
    std::vector<osaActuatorMapping> actuatorBoards(config.NumberOfActuators);
    std::vector<osaBrakeMapping> brakeBoards(config.NumberOfBrakes);
    int currentBrake = 0;

    for (int i = 0; i < config.NumberOfActuators; i++) {

        // Board for the actuator
        int boardId = config.Actuators[i].BoardID;

        // If the board hasn't been created, construct it and add it to the port
        if (mBoards.count(boardId) == 0) {
            mBoards[boardId] = new AmpIO(boardId);
            mPort->AddBoard(mBoards[boardId]);
        }

        // Add the board to the list of boards relevant to this robot
        actuatorBoards[i].Board = mBoards[boardId];
        actuatorBoards[i].Axis = config.Actuators[i].AxisID;

        // Board for the brake if any
        osaAnalogBrake1394Configuration * brake = config.Actuators[i].Brake;
        if (brake) {
            // Board for the brake
            boardId = brake->BoardID;
            
            // If the board hasn't been created, construct it and add it to the port
            if (mBoards.count(boardId) == 0) {
                mBoards[boardId] = new AmpIO(boardId);
                mPort->AddBoard(mBoards[boardId]);
            }
            
            // Add the board to the list of boards relevant to this robot
            brakeBoards[currentBrake].Board = mBoards[boardId];
            brakeBoards[currentBrake].Axis = brake->AxisID;
            currentBrake++;
        }
    }

    // Set the robot boards
    robot->SetBoards(actuatorBoards, brakeBoards);

    // Store the robot by name
    mRobots.push_back(robot);
    mRobotsByName[config.Name] = robot;
}

osaRobot1394 * osaPort1394::Robot(const std::string & name)
{
    robotByName_iterator it = mRobotsByName.find(name);
    return (it == mRobotsByName.end() ? 0 : it->second);
}

const osaRobot1394 * osaPort1394::Robot(const std::string & name) const
{
    robotByName_const_iterator it = mRobotsByName.find(name);
    return (it == mRobotsByName.end() ? 0 : it->second);
}

osaRobot1394 * osaPort1394::Robot(const int i)
{
    return mRobots[i];
}

const osaRobot1394 * osaPort1394::Robot(const int i) const
{
    return mRobots[i];
}

void osaPort1394::AddDigitalInput(osaDigitalInput1394 * digitalInput)
{
    if (digitalInput == 0) {
        cmnThrow(osaRuntimeError1394("osaPort1394::AddDigitalInput: digital input pointer is null."));
    }

    const osaDigitalInput1394Configuration & config = digitalInput->Configuration();

    // Check to make sure this digital input isn't already added
    if (mDigitalInputsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(digitalInput->Name() + ": digital input name is not unique."));
    }

    // Construct a vector of boards relevant to this digital input
    int boardID = config.BoardID;

    // If the board hasn't been created, construct it and add it to the port
    if (mBoards.count(boardID) == 0) {
        mBoards[boardID] = new AmpIO(boardID);
        mPort->AddBoard(mBoards[boardID]);
    }

    // Assign the board to the digital input
    digitalInput->SetBoard(mBoards[boardID]);

    // Store the digital input by name
    mDigitalInputs.push_back(digitalInput);
    mDigitalInputsByName[config.Name] = digitalInput;
}

void osaPort1394::AddDigitalOutput(osaDigitalOutput1394 * digitalOutput)
{
    if (digitalOutput == 0) {
        cmnThrow(osaRuntimeError1394("osaPort1394::AddDigitalOutput: digital output pointer is null."));
    }

    const osaDigitalOutput1394Configuration & config = digitalOutput->Configuration();

    // Check to make sure this digital output isn't already added
    if (mDigitalOutputsByName.count(config.Name) > 0) {
        cmnThrow(osaRuntimeError1394(digitalOutput->Name() + ": digital output name is not unique."));
    }

    // Construct a vector of boards relevant to this digital output
    int boardID = config.BoardID;

    // If the board hasn't been created, construct it and add it to the port
    if (mBoards.count(boardID) == 0) {
        mBoards[boardID] = new AmpIO(boardID);
        mPort->AddBoard(mBoards[boardID]);
    }

    // Assign the board to the digital output
    digitalOutput->SetBoard(mBoards[boardID]);

    // Store the digital output by name
    mDigitalOutputs.push_back(digitalOutput);
    mDigitalOutputsByName[config.Name] = digitalOutput;
}

osaPort1394::~osaPort1394()
{
    // Delete robots before deleting boards
    for (robot_iterator iter = mRobots.begin();
         iter != mRobots.end();
         ++iter) {
        if (*iter != 0) {
            delete *iter;
        }
    }
    mRobots.clear();

    // Delete digital inputs before deleting boards
    for (digital_input_iterator iter = mDigitalInputs.begin();
         iter != mDigitalInputs.end();
         ++iter) {
        if (*iter != 0) {
            delete *iter;
        }
    }
    mDigitalInputs.clear();

    // Delete digital outputs before deleting boards
    for (digital_output_iterator iter = mDigitalOutputs.begin();
         iter != mDigitalOutputs.end();
         ++iter) {
        if (*iter != 0) {
            delete *iter;
        }
    }
    mDigitalInputs.clear();

    // Delete board structures
    for (board_iterator iter = mBoards.begin();
         iter != mBoards.end();
         ++iter) {
        if (iter->second != 0) {
            mPort->RemoveBoard(iter->first);
            delete iter->second;
        }
    }
    mBoards.clear();

    // Delete firewire port
    if (mPort != 0) {
        delete mPort;
    }
}

void osaPort1394::Read(void)
{
    // Read from all boards on the port
    mPort->ReadAllBoards();

    // Poll the state for each robot
    for (robot_iterator robot = mRobots.begin();
         robot != mRobots.end();
         ++robot) {
        // Poll the board validity
        (*robot)->PollValidity();

        // Poll this robot's state
        (*robot)->PollState();

        // Convert bits to usable numbers
        (*robot)->ConvertState();

        // Perform post conversion checks and computations
        (*robot)->CheckState();
    }

    // Poll the state for each digital input
    for (digital_input_iterator iter = mDigitalInputs.begin();
         iter != mDigitalInputs.end();
         ++iter) {
        (*iter)->PollState();
    }
    // Poll the state for each digital output
    for (digital_output_iterator iter = mDigitalOutputs.begin();
         iter != mDigitalOutputs.end();
         ++iter) {
        (*iter)->PollState();
    }
}

void osaPort1394::Write(void)
{
    // Write to all boards
    mPort->WriteAllBoards();
}

int osaPort1394::NumberOfBoards(void) const {
    return mBoards.size();
}

int osaPort1394::NumberOfRobots(void) const {
    return mRobots.size();
}

int osaPort1394::NumberOfDigitalInputs(void) const {
    return mDigitalInputs.size();
}

int osaPort1394::NumberOfDigitalOutputs(void) const {
    return mDigitalOutputs.size();
}

void osaPort1394::GetRobotNames(std::vector<std::string> & names) const
{
    names.clear();
    for (robot_const_iterator iter = mRobots.begin();
         iter != mRobots.end();
         ++iter) {
        names.push_back((*iter)->Name());
    }
}

void osaPort1394::GetDigitalInputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (digital_input_const_iterator iter = mDigitalInputs.begin();
         iter != mDigitalInputs.end();
         ++iter) {
        names.push_back((*iter)->Name());
    }
}

void osaPort1394::GetDigitalOutputNames(std::vector<std::string> & names) const
{
    names.clear();
    for (digital_output_const_iterator iter = mDigitalOutputs.begin();
         iter != mDigitalOutputs.end();
         ++iter) {
        names.push_back((*iter)->Name());
    }
}
