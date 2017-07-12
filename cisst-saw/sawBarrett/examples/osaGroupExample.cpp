#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawBarrett/osaGroup.h>
#include <sawCANBus/osaSocketCAN.h>

int main( int argc, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 2 ){
    std::cout << "Usage: " << argv[0] << " can[0-1]" << std::endl;
    return -1;
  }

  osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );

  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  osaGroup broadcast( osaGroup::BROADCAST, &can );

  osaGroup uppertorques ( osaGroup::UPPERARM, &can );
  osaGroup lowertorques ( osaGroup::FOREARM,  &can );

  osaGroup upperpositions ( osaGroup::UPPERARM_POSITION, &can );
  osaGroup lowerpositions ( osaGroup::FOREARM_POSITION,  &can );


  if( broadcast.Reset() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to reset the puck" << std::endl;
    return -1;
  }
  osaSleep( 3.0 );
    
  if( broadcast.Ready() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to ready the puck" << std::endl;
    return -1;
  }
  osaSleep( 1.0 );

  if( upperpositions.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize upper positions" << std::endl;
    return -1;
  }

  if( uppertorques.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize upper torques" << std::endl;
    return -1;
  }

  if( lowerpositions.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize lower positions" << std::endl;
    return -1;
  }

  if( lowertorques.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize lower torques" << std::endl;
    return -1;
  }

  double t1 = osaGetTime();
  size_t cnt=0;

  for( size_t i=0; i<10000; i++ ){

    vctDynamicVector<double> q;
    upperpositions.GetPositions( q );
    lowerpositions.GetPositions( q );

    vctFixedSizeVector<double,4> t( 0.0 );
    uppertorques.SetTorques( t );
    lowertorques.SetTorques( t );

    cnt++;
    if( cnt == 1000 ){
      double t2 = osaGetTime();
      std::cout << 1000.0 / (t2 - t1) << std::endl;
      t1 = t2;
      cnt = 0;
    }

  }

  if( can.Close() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to close " << argv[1] << std::endl;
    return -1;
  }

  return 0;
}
