/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawOpenDynamicsEngine/osaODEManipulator.h>
#include <sawOpenDynamicsEngine/mtsODEManipulatorTask.h>
#include <cisstCommon/cmnLogger.h>

// main constructor
mtsODEManipulatorTask::mtsODEManipulatorTask( const std::string& name,
					      double period,
					      osaOSGManipulator* manipulator,
					      osaCPUMask cpumask,
					      int priority ) :
  mtsTaskPeriodic( name, period, true ),
  manipulator( manipulator ),
  input( NULL ),
  output( NULL ),
  ctl( NULL ),
  cpumask( cpumask ),
  priority( priority ){

  input = AddInterfaceProvided( "Input" );
  if( input ){
    
    StateTable.AddData( qin, "PositionsInput" );
    input->AddCommandWriteState( StateTable, qin, "SetPositionJoint" );

  }
  else{
    CMN_LOG_RUN_ERROR << "Failed to create interface Input for " << GetName()
		      << std::endl;
  }

  output = AddInterfaceProvided( "Output" );
  if( output ){

    StateTable.AddData( qout, "PositionsOutput" );
    output->AddCommandReadState( StateTable, qout, "GetPositionJoint" );

  }
  else{
    CMN_LOG_RUN_ERROR << "Failed to create interface Output for " << GetName()
		      << std::endl;
  }

}

void mtsODEManipulatorTask::Run(){

  ProcessQueuedCommands();

  if( manipulator.get() != NULL ){

    if( manipulator->SetPositions( qin.Goal() ) != 
	osaOSGManipulator::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to get position for " << GetName() 
			<< std::endl;
    }
    
    if( manipulator->GetPositions( qout.Position()) != 
	osaOSGManipulator::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to get position for " << GetName() 
			<< std::endl;
    }

  }

}

void mtsODEManipulatorTask::Attach( mtsODEManipulatorTask* mtstool ){
  manipulator->Attach( mtstool->manipulator.get() );
}



