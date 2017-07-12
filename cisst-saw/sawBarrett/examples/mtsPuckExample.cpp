#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstOSAbstraction/osaSleep.h>

#include <cisstCommon/cmnGetChar.h>

#include <sawBarrett/mtsPuck.h>
#include <sawCANBus/osaSocketCAN.h>

// A client CAN task that read/write from/to a mtsCAN component
class Puckclient : public mtsTaskPeriodic{

private:

  mtsFunctionRead  read;    // read mts function
  mtsFunctionWrite write;   // write mts function

  mtsFunctionVoid  ready;   // ready the puck
  mtsFunctionVoid  reset;   // reset the puck
  mtsFunctionVoid  init;    // initialize the puck

public:

  // Constructor
  Puckclient() : 
    mtsTaskPeriodic( "client", 1.0, true ){   // task name

    // Required interface with Read/Write function
    mtsInterfaceRequired* CTL = AddInterfaceRequired( "CTL" );
    if( CTL != NULL ){
      CTL->AddFunction( "Reset",      reset );
      CTL->AddFunction( "Ready",      ready );
      CTL->AddFunction( "Initialize", init  );
    }

  }

  ~Puckclient(){}

  void Configure(){}

  // Open the CAN device
  void Startup(){
    reset();
    osaSleep(1);
    ready();
    osaSleep(1);
    init();
  }

  void Run(){ ProcessQueuedCommands(); }

  // Close the CAN device
  void Cleanup(){}

};

int main( int argc, char** argv ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 3 ){
    std::cerr << "Usage: " << argv[0] << " can[?] ID" << std::endl;
    return -1;
  }

  // get local component manager                                                
  mtsManagerLocal *taskManager;
  taskManager = mtsManagerLocal::GetInstance();

  osaSocketCAN can( argv[1], osaCANBus::RATE_1000 );
  if( can.Open() != osaCANBus::ESUCCESS ){
    std::cerr << argv[0] << ": Failed to open device " << argv[1] << std::endl;
    return -1;
  }

  std::istringstream iss( argv[2] );
  int id;
  iss >> id;

  mtsPuck puck( std::string("puck")+argv[2], (osaPuck::ID)id, &can );
  taskManager->AddComponent( &puck );

  Puckclient client;
  client.Configure();
  taskManager->AddComponent( &client );
  
  // Connect the interfaces
  taskManager->Connect( client.GetName(), "CTL", puck.GetName(), "CTL" );

  taskManager->CreateAll();
  taskManager->WaitForStateAll(mtsComponentState::READY);

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );
  
  // Wait to exit
  std::cout << "ENTER to exit." << std::endl;
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  if( can.Close() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to close " << argv[1] << std::endl;
    return -1;
  }

  return 0;
}


