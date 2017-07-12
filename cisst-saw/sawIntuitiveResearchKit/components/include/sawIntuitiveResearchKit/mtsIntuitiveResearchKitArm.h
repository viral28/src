/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitArm_h
#define _mtsIntuitiveResearchKitArm_h

#include <cisstNumerical/nmrPInverse.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robLSPB.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitArm: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg);
    virtual inline ~mtsIntuitiveResearchKitArm() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    virtual void SetSimulated(void);

protected:

    /*! Define wrench reference frame */
    typedef enum {WRENCH_UNDEFINED, WRENCH_SPATIAL, WRENCH_BODY} WrenchType;

    /*! Load BaseFrame and DH parameters from JSON */
    void ConfigureDH(const Json::Value & jsonConfig);

    /*! Initialization, including resizing data members and setting up
      cisst/SAW interfaces */
    virtual void Init(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    virtual void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState) = 0;
    virtual void SetRobotControlState(const std::string & state) = 0;
    /*! Convert enum to string using function provided by cisstDataGenerator. */
    void GetRobotControlState(std::string & state) const;
    bool CurrentStateIs(const mtsIntuitiveResearchKitArmTypes::RobotStateType & state);
    bool CurrentStateIs(const mtsIntuitiveResearchKitArmTypes::RobotStateType & state1,
                        const mtsIntuitiveResearchKitArmTypes::RobotStateType & state2);

    /*! Get data from the PID level based on current state. */
    virtual void GetRobotData(void);

    /*! Homing procedure, bias encoders from potentiometers. */
    virtual void RunHomingBiasEncoder(void);

    /*! Homing procedure, power the robot. */
    virtual void RunHomingPower(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    virtual void RunHomingCalibrateArm(void) = 0;

    /*! Cartesian state. */
    virtual void RunPositionJoint(void);
    virtual void RunPositionGoalJoint(void);
    virtual void RunPositionCartesian(void);
    virtual void RunPositionGoalCartesian(void);

    /*! Effort state. */
    virtual void RunEffortCartesian(void);

    /*! Compute forces/position for PID when orientation is locked in
      effort cartesian mode or gravity compensation. */
    virtual void RunEffortOrientationLocked(void);

    /*! Run method called for all states not handled in base class. */
    inline virtual void RunArmSpecific(void) {};

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    virtual void SetPositionJointLocal(const vctDoubleVec & newPosition);

    /*! Methods used for commands */
    virtual void Freeze(void);
    virtual void SetPositionJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionGoalJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetWrenchSpatial(const prmForceCartesianSet & newForce);
    virtual void SetWrenchBody(const prmForceCartesianSet & newForce);
    /*! Apply the wrench relative to the body or to reference frame (i.e. absolute). */
    virtual void SetWrenchBodyOrientationAbsolute(const bool & absolute);
    virtual void SetGravityCompensation(const bool & gravityCompensation);

    /*! Set base coordinate frame, this will be added to the kinematics */
    virtual void SetBaseFrameEventHandler(const prmPositionCartesianGet & newBaseFrame);
    virtual void SetBaseFrame(const prmPositionCartesianSet & newBaseFrame);

    /*! Event handler for PID joint limit. */
    virtual void JointLimitEventHandler(const vctBoolVec & flags);

    /*! Event handler for PID errors. */
    void ErrorEventHandler(const std::string & message);

    /*! Event handler for EncoderBias done. */
    void BiasEncoderEventHandler(const int & nbSamples);

    /*! Configuration methods specific to derived classes. */
    virtual size_t NumberOfAxes(void) const = 0;           // used IO: ECM 4, PSM 7, MTM 8
    virtual size_t NumberOfJoints(void) const = 0;         // used PID: ECM 4, PSM 7, MTM 7
    virtual size_t NumberOfJointsKinematics(void) const = 0; // used for inverse kinematics: ECM 4, PSM 6, MTM 7
    virtual size_t NumberOfBrakes(void) const = 0;         // ECM 3, PSM 0, MTM 0

    virtual bool UsePIDTrackingError(void) const = 0;      // ECM true, PSM false, MTM false
    inline virtual bool UsePotsForSafetyCheck(void) const {
        return true;
    }

    virtual robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                                    const vctFrm4x4 & cartesianGoal) = 0;

    // Interface to PID component
    mtsInterfaceRequired * PIDInterface;
    struct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableJoints;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetPositionJointDesired;
        mtsFunctionRead  GetStateJoint;
        mtsFunctionRead  GetStateJointDesired;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetCheckJointLimit;
        mtsFunctionWrite SetJointLowerLimit;
        mtsFunctionWrite SetJointUpperLimit;
        mtsFunctionRead  GetVelocityJoint;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionWrite SetTorqueJoint;
        mtsFunctionWrite SetTorqueOffset;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionWrite SetTrackingErrorTolerance;
    } PID;

    // Interface to IO component
    mtsInterfaceRequired * IOInterface;
    struct InterfaceRobotTorque {
        mtsFunctionRead  GetSerialNumber;
        mtsFunctionWrite SetCoupling;
        mtsFunctionVoid  EnablePower;
        mtsFunctionVoid  DisablePower;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionRead  GetBrakeAmpStatus;
        mtsFunctionWrite BiasEncoder;
        mtsFunctionWrite ResetSingleEncoder;
        mtsFunctionRead  GetAnalogInputPosSI;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite UsePotsForSafetyCheck;
        mtsFunctionWrite SetPotsToEncodersTolerance;
        mtsFunctionVoid  BrakeRelease;
        mtsFunctionVoid  BrakeEngage;
    } RobotIO;

    // Interface to SUJ component
    mtsInterfaceRequired * SUJInterface;

    // Main provided interface
    mtsInterfaceProvided * RobotInterface;

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite RobotState;
    } MessageEvents;

    // Cache cartesian goal position
    prmPositionCartesianSet CartesianSetParam;
    bool IsGoalSet;

    // internal kinematics
    prmPositionCartesianGet CartesianGetLocalParam;
    vctFrm4x4 CartesianGetLocal;
    prmPositionCartesianGet CartesianGetLocalDesiredParam;
    vctFrm4x4 CartesianGetLocalDesired;

    // with base frame included
    prmPositionCartesianGet CartesianGetParam, CartesianGetPreviousParam;
    vctFrm4x4 CartesianGet;
    prmPositionCartesianGet CartesianGetDesiredParam;
    vctFrm4x4 CartesianGetDesired;

    // joints
    prmPositionJointGet JointGetParam;
    vctDoubleVec JointGet;
    vctDoubleVec JointGetDesired;
    prmPositionJointSet JointSetParam;
    vctDoubleVec JointSet;

    // efforts
    vctDoubleMat JacobianBody, JacobianBodyTranspose, JacobianSpatial;
    vctDoubleVec JointExternalEffort;
    WrenchType mWrenchType;
    prmForceCartesianSet mWrenchSet;
    bool mWrenchBodyOrientationAbsolute;
    prmForceTorqueJointSet TorqueSetParam;
    // to estimate wrench from joint efforts
    nmrPInverseDynamicData mJacobianPInverseData;
    prmForceCartesianGet mWrenchGet;

    // used by MTM only
    bool EffortOrientationLocked;
    vctDoubleVec EffortOrientationJoint;
    vctMatRot3 EffortOrientation;
    // gravity compensation
    bool mGravityCompensation;
    void AddGravityCompensationEfforts(vctDoubleVec & efforts);
    // add custom efforts for derived classes
    inline virtual void AddCustomEfforts(vctDoubleVec & CMN_UNUSED(efforts)) {};

    //! robot current joint velocity
    prmVelocityJointGet JointVelocityGetParam;
    vctDoubleVec JointVelocityGet;

    prmStateJoint StateJointParam, StateJointDesiredParam;

    // Velocities
    vctFrm4x4 CartesianGetPrevious;
    prmVelocityCartesianGet CartesianVelocityGetParam;

    robManipulator Manipulator;
    vctFrm4x4 CartesianPositionFrm;

    // Base frame
    vctFrm4x4 BaseFrame;
    bool BaseFrameValid;

    mtsIntuitiveResearchKitArmTypes::RobotStateType RobotState;

    struct {
        robLSPB LSPB;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Start;
        vctDoubleVec Goal;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        double EndTime; // time should be set to 0.0 if there is no on-going trajectory
        mtsFunctionWrite GoalReachedEvent; // sends true if goal reached, false otherwise
    } JointTrajectory;

    vctDoubleVec PotsToEncodersTolerance;

    // Home Action
    bool HomedOnce;
    bool HomingGoesToZero;
    double HomingTimer;
    bool HomingBiasEncoderRequested;
    bool HomingPowerRequested;
    bool HomingCalibrateArmStarted;

    unsigned int mCounter;

    // Flag to determine if this is connected to actual IO/hardware or simulated
    bool mIsSimulated;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
