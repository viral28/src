/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ankur Kapoor, Peter Kazanzides
  Created on: 2004-04-30

  (C) Copyright 2004-2012 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaThreadBuddy.h>
#include <cisstOSAbstraction/osaSleep.h>

CMN_IMPLEMENT_SERVICES(mtsTaskPeriodicConstructorArg);

void mtsTaskPeriodicConstructorArg::SerializeRaw(std::ostream & outputStream) const
{
    mtsGenericObject::SerializeRaw(outputStream);
    cmnSerializeRaw(outputStream, Name);
    cmnSerializeRaw(outputStream, Period);
    cmnSerializeRaw(outputStream, IsHardRealTime);
    cmnSerializeRaw(outputStream, StateTableSize);
}

void mtsTaskPeriodicConstructorArg::DeSerializeRaw(std::istream & inputStream)
{
    mtsGenericObject::DeSerializeRaw(inputStream);
    cmnDeSerializeRaw(inputStream, Name);
    cmnDeSerializeRaw(inputStream, Period);
    cmnDeSerializeRaw(inputStream, IsHardRealTime);
    cmnDeSerializeRaw(inputStream, StateTableSize);
}

void mtsTaskPeriodicConstructorArg::ToStream(std::ostream & outputStream) const
{
    outputStream << "Name: " << Name
                 << ", Period: " << Period
                 << ", IsHardRealTime: " << IsHardRealTime
                 << ", StateTableSize: " << StateTableSize << std::endl;
}

void mtsTaskPeriodicConstructorArg::ToStreamRaw(std::ostream & outputStream, const char delimiter,
                                                bool headerOnly, const std::string & headerPrefix) const
{
    mtsGenericObject::ToStreamRaw(outputStream, delimiter, headerOnly, headerPrefix);
    if (headerOnly) {
        outputStream << headerPrefix << "-name" << delimiter
                     << headerPrefix << "-period" << delimiter
                     << headerPrefix << "-isHardRealTime" << delimiter
                     << headerPrefix << "-stateTableSize";
    } else {
        outputStream << this->Name << delimiter
                     << this->Period << delimiter
                     << this->IsHardRealTime << delimiter
                     << this->StateTableSize;
    }
}

bool mtsTaskPeriodicConstructorArg::FromStreamRaw(std::istream & inputStream, const char delimiter)
{
    mtsGenericObject::FromStreamRaw(inputStream, delimiter);
    if (inputStream.fail())
        return false;
    inputStream >> Name >> Period;
    if (inputStream.fail())
        return false;
    if (inputStream.eof()) {
        IsHardRealTime = false;
        StateTableSize = STATE_TABLE_DEFAULT_SIZE;
        return (typeid(*this) == typeid(mtsTaskPeriodicConstructorArg));
    }
    inputStream >> IsHardRealTime;
    if (inputStream.fail())
        return false;
    if (inputStream.eof()) {
        StateTableSize = STATE_TABLE_DEFAULT_SIZE;
        return (typeid(*this) == typeid(mtsTaskPeriodicConstructorArg));
    }
    inputStream >> StateTableSize;
    if (inputStream.fail())
        return false;
    return (typeid(*this) == typeid(mtsTaskPeriodicConstructorArg));
}

/********************* Methods that call user methods *****************/

void * mtsTaskPeriodic::RunInternal(void *data)
{
    if (ExecIn && ExecIn->GetConnectedInterface()) {
        CMN_LOG_CLASS_RUN_ERROR << "RunInternal for " << this->GetName() 
                                << " called, even though task receives thread from "
                                << ExecIn->GetConnectedInterface()->GetComponent()->GetName() << std::endl;
        return 0;
    }

    if (this->State == mtsComponentState::INITIALIZING) {
        SaveThreadStartData(data);
        this->StartupInternal();
        if (CaptureThread) {
            return 0;
        }
    }

    while ((this->State == mtsComponentState::ACTIVE) || (this->State == mtsComponentState::READY)) {
        if (this->State == mtsComponentState::ACTIVE) {
            DoRunInternal();
            if (StateTable.GetToc() - StateTable.GetTic() > Period) {
                OverranPeriod = true;
            }
        }
        // Wait for remaining period also handles thread suspension
        ThreadBuddy.WaitForRemainingPeriod();
    }

    CMN_LOG_CLASS_RUN_WARNING << "End of task " << Name << std::endl;
    CleanupInternal();
    return this->ReturnValue;
}

void mtsTaskPeriodic::StartupInternal(void) {
    CMN_LOG_CLASS_INIT_VERBOSE << "Starting StartupInternal (periodic) for " << Name << std::endl;
    // user defined initialization, find commands from associated resource interfaces
    ThreadBuddy.Create(GetName().c_str(), AbsoluteTimePeriod); // convert to nano seconds

    // Call base class StartupInternal, which also calls user-supplied Startup.
    // If all goes well, this changes the state to READY.
    BaseType::StartupInternal();

    // allow no more stack allocation. allowing this will
    // break realtime performance.
    // lock all pages too
    //ThreadBuddy.LockStack();
    if (IsHardRealTime) {
        // Might as well wait for loop to start before going hard-real-time
        while ((this->State == mtsComponentState::READY) && !CaptureThread) {
            ThreadBuddy.WaitForRemainingPeriod();
        }
        ThreadBuddy.MakeHardRealTime();
    }
}


void mtsTaskPeriodic::CleanupInternal() {

    if (IsHardRealTime) {
        ThreadBuddy.MakeSoftRealTime();
    }
    ThreadBuddy.UnlockStack();

    //If the task was waiting on a queue, i.e. semaphore, mailbox,
    //etc, it is removed from such a queue and messaging tasks
    //pending on its message queue are unblocked with an error return.
    ThreadBuddy.Delete();

    BaseType::CleanupInternal();
}

void mtsTaskPeriodic::StartInternal(void)
{
    ThreadBuddy.Resume();
}

/********************* Task constructor and destructor *****************/

mtsTaskPeriodic::mtsTaskPeriodic(const std::string & name, double periodicityInSeconds,
                                 bool isHardRealTime, unsigned int sizeStateTable,
                                 bool newThread):
    mtsTaskContinuous(name, sizeStateTable, newThread),
    ThreadBuddy(),
    Period(periodicityInSeconds),
    IsHardRealTime(isHardRealTime)
{
    AbsoluteTimePeriod.FromSeconds(periodicityInSeconds);
    CMN_ASSERT(GetPeriodicity() > 0);
}

mtsTaskPeriodic::mtsTaskPeriodic( const std::string & name, 
                                  const osaAbsoluteTime & period,
                                  bool isHardRealTime, 
                                  unsigned int sizeStateTable,
                                  bool newThread ):
    mtsTaskContinuous(name, sizeStateTable, newThread),
    ThreadBuddy(),
    Period(period.ToSeconds()),
    AbsoluteTimePeriod(period),
    IsHardRealTime(isHardRealTime)
{
    CMN_ASSERT(GetPeriodicity() > 0);
}

mtsTaskPeriodic::mtsTaskPeriodic(const mtsTaskPeriodicConstructorArg &arg):
    mtsTaskContinuous(arg.Name, arg.StateTableSize, true),
    ThreadBuddy(),
    Period(arg.Period),
    IsHardRealTime(arg.IsHardRealTime)
{
    AbsoluteTimePeriod.FromSeconds(arg.Period);
    CMN_ASSERT(GetPeriodicity() > 0);
}

mtsTaskPeriodic::~mtsTaskPeriodic() {
    CMN_LOG_CLASS_RUN_DEBUG << "mtsTaskPeriodic destructor: deleting task " << Name << std::endl;
    //If the task was waiting on a queue, i.e. semaphore, mailbox,
    //etc, it is removed from such a queue and messaging tasks
    //pending on its message queue are unblocked with an error return.

    Kill();
    // adeguet1, is this sleep still necessary?
    // Now, wait for 2 periods to see if it was killed
    // osaSleep(2.0 * this->PeriodInSeconds); // all expressed in seconds
}


/********************* Methods to change task state ******************/

void mtsTaskPeriodic::Suspend(void)
{
    if (this->State == mtsComponentState::ACTIVE) {
        BaseType::Suspend();
        ThreadBuddy.Suspend();
        CMN_LOG_CLASS_RUN_DEBUG << "Suspended task " << Name << std::endl;
    }
}


double mtsTaskPeriodic::GetPeriodicity(void) const
{
    return AbsoluteTimePeriod.ToSeconds();
}

/*! Return true if thread is periodic.  Currently, returns true if
  the thread was created with a period > 0. */
bool mtsTaskPeriodic::IsPeriodic(void) const
{
    return Period > 0.0;
}
