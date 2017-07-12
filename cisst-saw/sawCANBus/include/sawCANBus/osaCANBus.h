

#ifndef _osaCANBus_h
#define _osaCANBus_h

#include <sawCANBus/osaCANBusFrame.h>
#include <sawCANBus/sawCANBusExport.h>

//! Generic CAN bus
/**
   The only thing this class does is to define the interface that must be
   implemented by a CAN bus. Any implementation of a CAN device depends on an 
   actual device driver.
*/
class CISST_EXPORT osaCANBus {
  
 public:
  
  //! The different CAN rates 
  enum Rate { RATE_150 =150000, 
	      RATE_300 =300000, 
	      RATE_1000=1000000 };
  
  //! The different CAN rates 
  enum Errno { ESUCCESS, 
	       EFAILURE };
  
  enum Flags{ MSG_NOFLAG   = 0x00,
              MSG_CONFIRM  = 0x01,   // ask for a confirmation 
	      MSG_DONTWAIT = 0x02 }; // enables non-blocking operation

  enum Loopback{ LOOPBACK_ON, LOOPBACK_OFF };

 public:
  
  // CAN Filter
  /**
     CAN filters are used by the kernel to filter frames that are not aimed for
     a specific node id.
   */
  class Filter {

  public:
    
    osaCANBusFrame::Mask mask;
    osaCANBusFrame::ID id;
    
    Filter( osaCANBusFrame::Mask mask, 
	    osaCANBusFrame::ID id ) : mask( mask ), id( id ) {}
    
  };

 protected:
  
  //! The rate of the device
  osaCANBus::Rate rate;
  osaCANBus::Loopback loopback;

 public:
  
  //! Default constructor
  /**
     Initialize a CAN device with the given rate
     \param rate The CAN rate
  */
  osaCANBus( osaCANBus::Rate rate, 
	    osaCANBus::Loopback loopback = osaCANBus::LOOPBACK_OFF );
  
  //! Default destructor
  virtual ~osaCANBus();
  
  //! Open the CAN device
  virtual osaCANBus::Errno Open() = 0;

  //! Close the CAN device
  virtual osaCANBus::Errno Close() = 0;

  //! Send a CAN frame on the bus
  /**
     Call this method to send a CAN frame on the bus. The method can be blocking
     \param frame[in] The CAN frame to send on the bus
     \param block Block the device until the operation is completed
  */
  virtual osaCANBus::Errno Send( const osaCANBusFrame& frame, 
				osaCANBus::Flags flags=osaCANBus::MSG_NOFLAG )=0;

  //! Receive a CAN frame
  /**
     Call this method to receive a CAN frame. The method can be blocking.
     \param frame[out] The CAN frame received from the bus
     \param block Block the device until a CAN frame is received
  */
  virtual osaCANBus::Errno Recv( osaCANBusFrame& frame, 
				osaCANBus::Flags flags=osaCANBus::MSG_NOFLAG )=0;
  

  //! Add a CAN filter to the device
  /**
     Call this method to add a CAN filter to the device. This is OS and 
     dependant as it typically interacts with the driver to screen CAN frames.
  */
  virtual osaCANBus::Errno AddFilter( const osaCANBus::Filter& filter ) = 0;

};


#endif // _osaCANBus_h
