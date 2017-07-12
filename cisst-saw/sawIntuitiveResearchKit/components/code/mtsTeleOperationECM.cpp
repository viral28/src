/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-01-21

  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationECM::mtsTeleOperationECM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mMTML(0),
    mMTMR(0),
    mECM(0),
    mTeleopState("DISABLED")
{
    Init();
}

mtsTeleOperationECM::mtsTeleOperationECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mMTML(0),
    mMTMR(0),
    mECM(0),
    mTeleopState("DISABLED")
{
    Init();
}

mtsTeleOperationECM::~mtsTeleOperationECM()
{
    if (mMTML) {
        delete mMTML;
    }
    if (mMTMR) {
        delete mMTMR;
    }
    if (mECM) {
        delete mECM;
    }
}

void mtsTeleOperationECM::Init(void)
{
    if (!mMTML) {
        mMTML = new RobotMTM;
    }
    if (!mMTMR) {
        mMTMR = new RobotMTM;
    }
    if (!mECM) {
        mECM = new RobotECM;
    }
}

void mtsTeleOperationECM::Configure(const std::string & CMN_UNUSED(filename))
{
    // configure state machine
    mTeleopState.AddState("DISABLED");
    mTeleopState.AddState("SETTING_ECM_STATE");
    mTeleopState.AddState("SETTING_MTMS_STATE");
    mTeleopState.AddState("ENABLED");
    mTeleopState.AddAllowedDesiredState("DISABLED");
    mTeleopState.AddAllowedDesiredState("ENABLED");

    // state change, to convert to string events for users (Qt, ROS)
    mTeleopState.SetStateChangedCallback(&mtsTeleOperationECM::StateChanged,
                                         this);

    // run for all states
    mTeleopState.SetRunCallback(&mtsTeleOperationECM::RunAllStates,
                                this);
    // disabled
    mTeleopState.SetTransitionCallback("DISABLED",
                                       &mtsTeleOperationECM::TransitionDisabled,
                                       this);

    // setting ECM state
    mTeleopState.SetEnterCallback("SETTING_ECM_STATE",
                                  &mtsTeleOperationECM::EnterSettingECMState,
                                  this);
    mTeleopState.SetTransitionCallback("SETTING_ECM_STATE",
                                       &mtsTeleOperationECM::TransitionSettingECMState,
                                       this);

    // setting MTMs state
    mTeleopState.SetEnterCallback("SETTING_MTMS_STATE",
                                  &mtsTeleOperationECM::EnterSettingMTMsState,
                                  this);
    mTeleopState.SetTransitionCallback("SETTING_MTMS_STATE",
                                       &mtsTeleOperationECM::TransitionSettingMTMsState,
                                       this);

    // enabled
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsTeleOperationECM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsTeleOperationECM::RunEnabled,
                                this);
    mTeleopState.SetTransitionCallback("ENABLED",
                                       &mtsTeleOperationECM::TransitionEnabled,
                                       this);

    mScale = 0.2;
    mIsClutched = false;

    StateTable.AddData(mMTML->PositionCartesianCurrent, "MTMLCartesianPosition");
    StateTable.AddData(mMTMR->PositionCartesianCurrent, "MTMRCartesianPosition");
    StateTable.AddData(mECM->PositionCartesianCurrent, "ECMCartesianPosition");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(mScale, "Scale");
    mConfigurationStateTable->AddData(mRegistrationRotation, "RegistrationRotation");

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTML");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian",
                                       mMTML->GetPositionCartesian);
        interfaceRequired->AddFunction("GetPositionCartesianDesired",
                                       mMTML->GetPositionCartesianDesired);
        interfaceRequired->AddFunction("GetVelocityCartesian",
                                       mMTML->GetVelocityCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian",
                                       mMTML->SetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotControlState",
                                       mMTML->GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState",
                                       mMTML->SetRobotControlState);
        interfaceRequired->AddFunction("LockOrientation",
                                       mMTML->LockOrientation);
        interfaceRequired->AddFunction("UnlockOrientation",
                                       mMTML->UnlockOrientation);
        interfaceRequired->AddFunction("SetWrenchBody",
                                       mMTML->SetWrenchBody);
        interfaceRequired->AddFunction("SetWrenchBodyOrientationAbsolute",
                                       mMTML->SetWrenchBodyOrientationAbsolute);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MTMLErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("MTMR");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian",
                                       mMTMR->GetPositionCartesian);
        interfaceRequired->AddFunction("GetPositionCartesianDesired",
                                       mMTMR->GetPositionCartesianDesired);
        interfaceRequired->AddFunction("GetVelocityCartesian",
                                       mMTMR->GetVelocityCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian",
                                       mMTMR->SetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotControlState",
                                       mMTMR->GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState",
                                       mMTMR->SetRobotControlState);
        interfaceRequired->AddFunction("LockOrientation",
                                       mMTMR->LockOrientation);
        interfaceRequired->AddFunction("UnlockOrientation",
                                       mMTMR->UnlockOrientation);
        interfaceRequired->AddFunction("SetWrenchBody",
                                       mMTMR->SetWrenchBody);
        interfaceRequired->AddFunction("SetWrenchBodyOrientationAbsolute",
                                       mMTMR->SetWrenchBodyOrientationAbsolute);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MTMRErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("ECM");
    if (interfaceRequired) {
        // ECM, use PID desired position to make sure there is no jump when engaging
        interfaceRequired->AddFunction("GetPositionCartesian",
                                       mECM->GetPositionCartesian);
        interfaceRequired->AddFunction("GetPositionCartesianDesired",
                                       mECM->GetPositionCartesianDesired);
        interfaceRequired->AddFunction("SetPositionCartesian",
                                       mECM->SetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotControlState",
                                       mECM->GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState",
                                       mECM->SetRobotControlState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::ECMErrorEventHandler,
                                                this, "Error");
    }

    // footpedal events
    interfaceRequired = AddInterfaceRequired("Clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::ClutchEventHandler, this, "Button");
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Setting");
    if (interfaceProvided) {
        // commands
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                              "GetPeriodStatistics"); // mtsIntervalStatistics

        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::SetDesiredState, this,
                                           "SetDesiredState", std::string("DISABLED"));
        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::SetScale, this,
                                           "SetScale", 0.5);
        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::SetRegistrationRotation, this,
                                           "SetRegistrationRotation", vctMatRot3());
        interfaceProvided->AddCommandReadState(*mConfigurationStateTable,
                                               mScale,
                                               "GetScale");
        interfaceProvided->AddCommandReadState(*mConfigurationStateTable,
                                               mRegistrationRotation,
                                               "GetRegistrationRotation");
        interfaceProvided->AddCommandReadState(StateTable,
                                               mMTML->PositionCartesianCurrent,
                                               "GetPositionCartesianMTML");
        interfaceProvided->AddCommandReadState(StateTable,
                                               mMTMR->PositionCartesianCurrent,
                                               "GetPositionCartesianMTMR");
        interfaceProvided->AddCommandReadState(StateTable,
                                               mECM->PositionCartesianCurrent,
                                               "GetPositionCartesianECM");
        // events
        interfaceProvided->AddEventWrite(MessageEvents.Status,
                                         "Status", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Warning,
                                         "Warning", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Error,
                                         "Error", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.DesiredState,
                                         "DesiredState", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.CurrentState,
                                         "CurrentState", std::string(""));
        // configuration
        interfaceProvided->AddEventWrite(ConfigurationEvents.Scale,
                                         "Scale", 0.5);
    }
}

void mtsTeleOperationECM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsTeleOperationECM::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // run based on state
    mTeleopState.Run();
}

void mtsTeleOperationECM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationECM::StateChanged(void)
{
    const std::string newState = mTeleopState.CurrentState();
    MessageEvents.CurrentState(newState);
    MessageEvents.Status(this->GetName() + ", current state " + newState);
}

void mtsTeleOperationECM::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    // get master left Cartesian position/velocity
    executionResult = mMTML->GetPositionCartesian(mMTML->PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master left");
        mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTML->GetVelocityCartesian(mMTML->VelocityCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.GetVelocityCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian velocity from master left");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // get master right Cartesian position
    executionResult = mMTMR->GetPositionCartesian(mMTMR->PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master right");
        mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTMR->GetVelocityCartesian(mMTMR->VelocityCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.GetVelocityCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian velocity from master right");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // get slave Cartesian position
    executionResult = mECM->GetPositionCartesian(mECM->PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from slave");
        mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mECM->GetPositionCartesianDesired(mECM->PositionCartesianDesired);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.GetPositionCartesianDesired failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from slave");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == "DISABLED")
        && (mTeleopState.CurrentState() != "DISABLED")) {
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }
}

void mtsTeleOperationECM::TransitionDisabled(void)
{
    if (mTeleopState.DesiredState() == "ENABLED") {
        mTeleopState.SetCurrentState("SETTING_ECM_STATE");
    }
}

void mtsTeleOperationECM::EnterSettingECMState(void)
{
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // request state if needed
    mtsStdString armState;
    mECM->GetRobotControlState(armState);
    if (armState.Data != "DVRK_POSITION_CARTESIAN") {
        mECM->SetRobotControlState(mtsStdString("DVRK_POSITION_CARTESIAN"));
    }
}

void mtsTeleOperationECM::TransitionSettingECMState(void)
{
    // check if anyone wanted to disable anyway
    if (mTeleopState.DesiredState() == "DISABLED") {
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }
    // check state
    mtsStdString armState;
    mECM->GetRobotControlState(armState);
    if (armState.Data == "DVRK_POSITION_CARTESIAN") {
        mTeleopState.SetCurrentState("SETTING_MTMS_STATE");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        MessageEvents.Error(this->GetName() + ": timed out while setting up ECM state");
        mTeleopState.SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationECM::EnterSettingMTMsState(void)
{
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // request state if needed
    mtsStdString armState;
    mMTML->GetRobotControlState(armState);
    if (armState.Data != "DVRK_EFFORT_CARTESIAN") {
        mMTML->SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
    }
    mMTMR->GetRobotControlState(armState);
    if (armState.Data != "DVRK_EFFORT_CARTESIAN") {
        mMTMR->SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
    }
}

void mtsTeleOperationECM::TransitionSettingMTMsState(void)
{
    // check if anyone wanted to disable anyway
    if (mTeleopState.DesiredState() == "DISABLED") {
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }
    // check state
    mtsStdString leftArmState, rightArmState;
    mMTML->GetRobotControlState(leftArmState);
    mMTMR->GetRobotControlState(rightArmState);
    if ((leftArmState.Data == "DVRK_EFFORT_CARTESIAN") &&
        (rightArmState.Data == "DVRK_EFFORT_CARTESIAN")) {
        mTeleopState.SetCurrentState("ENABLED");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        MessageEvents.Error(this->GetName() + ": timed out while setting up MTMs state");
        mTeleopState.SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationECM::EnterEnabled(void)
{
    // set cartesian effort parameters
    mMTML->SetWrenchBodyOrientationAbsolute(true);
    mMTML->LockOrientation(mMTML->PositionCartesianCurrent.Position().Rotation());
    mMTMR->SetWrenchBodyOrientationAbsolute(true);
    mMTMR->LockOrientation(mMTMR->PositionCartesianCurrent.Position().Rotation());

    // initial state for MTM force feedback
    // -1- initial distance between left and right masters
    vct3 vectorLR;
    vectorLR.DifferenceOf(mMTMR->PositionCartesianCurrent.Position().Translation(),
                          mMTML->PositionCartesianCurrent.Position().Translation());
    mInitial.dLR = vectorLR.Norm();
    // -2- mid-point, aka center of image
    mInitial.C.SumOf(mMTMR->PositionCartesianCurrent.Position().Translation(),
                     mMTML->PositionCartesianCurrent.Position().Translation());
    mInitial.C.Multiply(0.5);
    // -3- image up vector
    mInitial.Up.CrossProductOf(vectorLR, mInitial.C);
    mInitial.Up.NormalizedSelf();
    // -4- width of image, depth of arms wrt image plan
    vct3 side;
    side.CrossProductOf(mInitial.C, mInitial.Up);
    side.NormalizedSelf();
    mInitial.w = 0.5 * vctDotProduct(side, vectorLR);
    mInitial.d = 0.5 * vctDotProduct(mInitial.C.Normalized(), vectorLR);
    // -5- compute MTMs frame
    mInitial.Frame.Translation().Assign(mInitial.C);
    mInitial.Frame.Rotation().Row(0).Assign(side);
    mInitial.Frame.Rotation().Row(1).Assign(mInitial.Up);
    mInitial.Frame.Rotation().Row(2).Assign(vctCrossProduct(side, mInitial.Up));

#if 1
    std::cerr << "L: " << mMTML->PositionCartesianCurrent.Position().Translation() << std::endl
              << "R: " << mMTMR->PositionCartesianCurrent.Position().Translation() << std::endl
              << "C:  " << mInitial.C << std::endl
              << "Up: " << mInitial.Up << std::endl
              << "d:  " << mInitial.dLR << std::endl
              << "w:  " << mInitial.w << std::endl
              << "d:  " << mInitial.d << std::endl
              << "Si: " << side << std::endl
              << "F:  " << std::endl << mInitial.Frame << std::endl;
#endif
    mECM->PositionCartesianInitial = mECM->PositionCartesianDesired.Position();
}

void mtsTeleOperationECM::RunEnabled(void)
{
    const vct3 frictionForceCoeff(-10.0, -10.0, -10.0);
    const double distanceForceCoeff = 150.0;

    // -1- vector between left and right masters
    vct3 vectorLR;
    vectorLR.DifferenceOf(mMTMR->PositionCartesianCurrent.Position().Translation(),
                          mMTML->PositionCartesianCurrent.Position().Translation());
    // -2- mid-point, aka center of image
    vct3 c;
    c.SumOf(mMTMR->PositionCartesianCurrent.Position().Translation(),
            mMTML->PositionCartesianCurrent.Position().Translation());
    c.Multiply(0.5);
    vct3 directionC = c.Normalized();
    // -3- image up vector
    vct3 up;
    up.CrossProductOf(vectorLR, c);
    up.NormalizedSelf();
    // -4- Width of image
    vct3 side;
    side.CrossProductOf(c, up);
    side.NormalizedSelf();
    // -5- find desired position for L and R
    vct3 goalL(c);
    goalL.AddProductOf(-mInitial.w, side);
    goalL.AddProductOf(-mInitial.d, directionC);
    vct3 goalR(c);
    goalR.AddProductOf(mInitial.w, side);
    goalR.AddProductOf(mInitial.d, directionC);

    // compute forces on L and R based on error in position
    vct3 forceFriction;
    vct3 force;
    prmForceCartesianSet wrenchR, wrenchL;

    // MTMR
    // apply force
    force.DifferenceOf(goalR,
                       mMTMR->PositionCartesianCurrent.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchR.Force().Ref<3>(0).Assign(force);
    // add friction force
    forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                       mMTMR->VelocityCartesianCurrent.VelocityLinear());
    wrenchR.Force().Ref<3>(0).Add(forceFriction);
    // apply
    mMTMR->SetWrenchBody(wrenchR);

    // MTML
    // apply force
    force.DifferenceOf(goalL,
                       mMTML->PositionCartesianCurrent.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchL.Force().Ref<3>(0).Assign(force);
    // add friction force
    forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                       mMTML->VelocityCartesianCurrent.VelocityLinear());
    wrenchL.Force().Ref<3>(0).Add(forceFriction);
    // apply
    mMTML->SetWrenchBody(wrenchL);


    // compute new MTMs frame
    vctFrm3 frame;
    frame.Translation().Assign(c);
    frame.Rotation().Row(0).Assign(side);
    frame.Rotation().Row(1).Assign(up);
    frame.Rotation().Row(2).Assign(vctCrossProduct(side, up));

    // Transformation since last clutch
    vctFrm3 displacement;
    frame.ApplyTo(mInitial.Frame.Inverse(), displacement);

    // New ECM position
    vctFrm3 goal;
    displacement.ApplyTo(mECM->PositionCartesianInitial, goal);
    mECM->PositionCartesianSet.Goal().Assign(goal);
    mECM->SetPositionCartesian(mECM->PositionCartesianSet);
}

void mtsTeleOperationECM::TransitionEnabled(void)
{
    if (mTeleopState.DesiredState() == "DISABLED") {
        mTeleopState.SetCurrentState("DISABLED");
    }
}

void mtsTeleOperationECM::MTMLErrorEventHandler(const std::string & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    MessageEvents.Error(this->GetName() + ": received from left master [" + message + "]");
}

void mtsTeleOperationECM::MTMRErrorEventHandler(const std::string & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    MessageEvents.Error(this->GetName() + ": received from right master [" + message + "]");
}

void mtsTeleOperationECM::ECMErrorEventHandler(const std::string & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    MessageEvents.Error(this->GetName() + ": received from slave [" + message + "]");
}

void mtsTeleOperationECM::ClutchEventHandler(const prmEventButton & button)
{
    // if the teleoperation is activated
    if (mTeleopState.DesiredState() == "ENABLED") {
         if (button.Type() == prmEventButton::PRESSED) {
            mIsClutched = true;
            MessageEvents.Status(this->GetName() + ": console clutch pressed");

            // set MTMs in effort mode, no force applied but gravity and locked orientation
            prmForceCartesianSet wrench;
            mMTML->SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
            mMTML->SetWrenchBody(wrench);
            mMTML->SetGravityCompensation(true);
            mMTML->LockOrientation(mMTML->PositionCartesianCurrent.Position().Rotation());
            mMTMR->SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
            mMTMR->SetWrenchBody(wrench);
            mMTMR->SetGravityCompensation(true);
            mMTMR->LockOrientation(mMTMR->PositionCartesianCurrent.Position().Rotation());
        } else {
            mIsClutched = false;
            MessageEvents.Status(this->GetName() + ": console clutch released");
            mTeleopState.SetCurrentState("SETTING_MTMS_STATE");
        }
    }
}

void mtsTeleOperationECM::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        MessageEvents.Error(this->GetName() + ": unsupported state " + state);
        return;
    }
    // if state is same as current, return
    if (mTeleopState.CurrentState() == state) {
        return;
    }
    // try to set the desired state
    if (!mTeleopState.SetDesiredState(state)) {
        MessageEvents.Error(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    MessageEvents.DesiredState(state);
    MessageEvents.Status(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationECM::SetScale(const double & scale)
{
    mConfigurationStateTable->Start();
    mScale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.Scale(mScale);
}

void mtsTeleOperationECM::SetRegistrationRotation(const vctMatRot3 & rotation)
{
    mConfigurationStateTable->Start();
    mRegistrationRotation = rotation;
    mConfigurationStateTable->Advance();
}
