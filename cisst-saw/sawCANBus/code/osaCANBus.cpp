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

//#include <sawCANBus/osaCANBus.h>
#include <sawCANBus/osaCANBus.h>
#include <cisstCommon/cmnLogger.h>

// default constructor of a can device
osaCANBus::osaCANBus( osaCANBus::Rate rate,
		    osaCANBus::Loopback loopback ) : 
  loopback( loopback ){ 

  // Ensure that the rate is ok
  if( (rate == osaCANBus::RATE_150) ||
      (rate == osaCANBus::RATE_300) ||
      (rate == osaCANBus::RATE_1000) ){
    this->rate = rate;
  }
  else{
    CMN_LOG_RUN_WARNING << CMN_LOG_DETAILS
			<< ": Illegal CAN rate " << rate
			<< std::endl;
  }
}

osaCANBus::~osaCANBus(){}
