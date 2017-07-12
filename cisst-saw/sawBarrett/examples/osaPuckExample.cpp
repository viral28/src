#include <cisstOSAbstraction/osaSleep.h>

#include <sawBarrett/osaPuck.h>
#include <sawCANBus/osaSocketCAN.h>

int main( int argc, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 3 ){
    std::cerr << "Usage: " << argv[0] << " can[?] PID" << std::endl;
    return -1;
  }

  osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );
  if( can.Open() != osaCANBus::ESUCCESS ){
    std::cerr << argv[0] << ": Failed to open device " << argv[1] << std::endl;
    return -1;
  }

  std::istringstream iss( argv[2] );
  int pid;
  iss >> pid;

  osaPuck puck( (osaPuck::ID)pid, &can );

  // Reset the firmware
  if( puck.Reset() != osaPuck::ESUCCESS ){
    std::cerr << ": Failed to reset the puck." << std::endl;
    return -1;
  }
  osaSleep( 1. );

  // Ready the puck
  if( puck.Ready() != osaPuck::ESUCCESS ){
    std::cerr << ": Failed to ready the puck." << std::endl;
    return -1;
  }
  osaSleep( 1.0 );

  // configure the puck
  if( puck.InitializeMotor() != osaPuck::ESUCCESS ){
    std::cerr << argv[0] << ": Failed to initialize puck" << std::endl;
  }

  if( can.Close() != osaCANBus::ESUCCESS ){
    std::cerr << argv[0] << ": Failed to close device " << argv[1] << std::endl;
    return -1;
  }

  return 0;
}
