#include <cisstCommon/cmnGetChar.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawBarrett/mtsWAM.h>
#include <sawCANBus/osaSocketCAN.h>

class WAMprobe : public mtsTaskPeriodic {

private:
  
  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetTorques;

public:

  WAMprobe() : mtsTaskPeriodic( "WAMprobe", 0.002, true ){

    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    input->AddFunction( "GetPositionJoint", GetPositions );
    output->AddFunction( "SetTorqueJoint", SetTorques );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
    prmPositionJointGet q;
    GetPositions( q );

    std::cout << q.Position() << std::endl;
    
    prmForceTorqueJointSet t;
    t.SetSize( 7 );
    t.ForceTorque().SetAll( 0.0 );
    SetTorques( t );

  }
  
  void Cleanup(){}

};

int main( int argc, char** argv ){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

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

  mtsWAM WAM( "WAM", &can, osaWAM::WAM_7DOF, OSA_CPU4, 80 );
  WAM.Configure();
  WAM.SetPositions( vctDynamicVector<double>(7, 
					     0.0, -cmnPI_2, 0.0, cmnPI, 
					     0.0, 0.0, 0.0 ) );
  taskManager->AddComponent( &WAM );

  WAMprobe probe;
  taskManager->AddComponent( &probe );

  taskManager->Connect( probe.GetName(), "Input",  WAM.GetName(),   "Output" );
  taskManager->Connect( probe.GetName(), "Output", WAM.GetName(),   "Input" );


  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  std::cout << "ENTER to exit" << std::endl;
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  if( can.Close() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  return 0;
}
