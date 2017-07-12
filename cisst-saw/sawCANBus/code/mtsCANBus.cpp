/*

  Author(s): Simon Leonard
  Created on: Dec 02 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <sawCANBus/mtsCANBus.h>
#include <cisstCommon/cmnLogger.h>

CMN_IMPLEMENT_SERVICES( mtsCANBus );

mtsCANBus::mtsCANBus( const std::string& componentname, osaCANBus* can ) :
  mtsComponent( componentname ),
  can( can ),
  io( NULL ),
  ctl( NULL ){

  // Ensure the CAN is valid
  if( can == NULL )
    { CMN_LOG_CLASS_RUN_ERROR << "No CAN device!" << std::endl; }

  // Create the IO interface and add read/write commands
  io = AddInterfaceProvided( "IO" );
  if( io ){
    io->AddCommandRead ( &mtsCANBus::mtsRead,  this, "Read" );
    io->AddCommandWrite( &mtsCANBus::mtsWrite, this, "Write" );
  }
  else{
    CMN_LOG_CLASS_RUN_ERROR << "Failed to create the interface IO" << std::endl;
  }

  // Create the CTL itnerface
  ctl = AddInterfaceProvided( "CTL" );
  if( ctl ){
    ctl->AddCommandVoid( &mtsCANBus::mtsOpen,  this, "Open" );
    ctl->AddCommandVoid( &mtsCANBus::mtsClose, this, "Close" );
  }
  else{
    CMN_LOG_CLASS_RUN_ERROR << "Failed to create the interface CTL"<< std::endl;
  }

}

// Write a CAN frame to the device
void mtsCANBus::mtsOpen()
{ if( can != NULL ) { can->Open(); } }

// Write a CAN frame to the device
void mtsCANBus::mtsClose()
{ if( can != NULL ) { can->Close(); } }

// Read a CAN frame from the device
void mtsCANBus::mtsRead( mtsCANBusFrame &frame ) const {
  if( can != NULL ){
    osaCANBusFrame f;
    can->Recv( f );
    frame = mtsCANBusFrame( f );
  }
}

// Write a CAN frame to the device
void mtsCANBus::mtsWrite( const mtsCANBusFrame &frame ) {
  if( can != NULL )
    { can->Send( (osaCANBusFrame)frame ); }
}

