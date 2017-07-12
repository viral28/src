/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaODEWorld_h
#define _osaODEWorld_h

#include <ode/ode.h>

#include <cisstVector/vctFixedSizeVector.h>
#include <cisstOSAbstraction/osaMutex.h>

#include <sawOpenSceneGraph/osaOSGWorld.h>

#include <sawOpenDynamicsEngine/osaODEBody.h>
#include <sawOpenDynamicsEngine/sawOpenDynamicsEngineExport.h>

struct CISST_EXPORT osaODEContact{
  
  osaODEBody* body1;
  osaODEBody* body2;
  vctFixedSizeVector<double,3> position;
  vctFixedSizeVector<double,3> normal;
  double depth;
  
  osaODEContact();

  osaODEContact( osaODEBody* b1, 
		   osaODEBody* b2,
		   const vctFixedSizeVector<double,3>& pos,
		   const vctFixedSizeVector<double,3>& n,
		   double d );
  /*
  friend std::ostream& operator<<( std::ostream& os, const osaODEContact& c ){
    os << "Body1:    " << c.body1->GetName() << std::endl
       << "Body2:    " << c.body2->GetName() << std::endl
       << "Position: " << c.position << std::endl
       << "Normal:   " << c.normal << std::endl
       << "Depth:    " << c.depth;
    return os;
  }
  */  
};

class CISST_EXPORT osaODEWorld : public osaOSGWorld {

 private:
  
  //! The time step of the engine
  double timestep;
  
  //! Contacts
  std::list<osaODEContact> contacts;

  //! The ODE world ID
  /**
     In ODE, a world contain all the bodies and the spaces containing the 
     geometries
  */
  dWorldID worldid;
  
  //! The ODE space ID
  /**
     In ODE, a space contains all the geometries (shape) of the bodies. Spaces
     are used to process collisions between geometries.
  */
  dSpaceID spaceid;

  //! The floor geom
  dGeomID floor;
  
  //! The ODE contact group
  /**
     Contact group is used to add contacts between geometries at each iteration.
     This contact group is the one used by the world
  */
  dJointGroupID contactsgid;

  //! Return the contacts group ID
  /**
     This is used internally for processing collisions. 
     \return The contacts group ID.
  */
  dJointGroupID GetGroupID() const { return contactsgid; }

  //! The maximum number of contacts
  /**
     This value determines the manimum number of contacts that can happen 
     between two bodies. Using several contact point slows down the simulation 
     while too few gives bad results (100 contacts points is actually quite 
     large)
  */
  static const size_t NUM_CONTACTS = 50;

 protected:
  
  //! Default gravity
  static const vctFixedSizeVector<double,3> GRAVITY;

  double contacterp;
  double contactbounce;
  double mu;
  double surfacelayer;

 public:

  //! Create a new world
  /**
     Create a new ODE world. This initializes the ODE engine and create a new
     world and a new top level space. It also sets simulation parameters such as
     error reduction parameter (ERP) and constraint force mixing (CFM)
     \param gravity A gravity vector. The defalt value is 
                    \$\begin{bmatrix} 0 & 0 & -9.81 \end{bmatrix}\$.
  */
  osaODEWorld( double period,
		 const vctFixedSizeVector<double,3>& gravity = GRAVITY );
  
  //! Destroy the world!
  ~osaODEWorld();

  //! Return the world ID
  dWorldID GetWorldID() const { return worldid; }

  //! Return the space ID
  dSpaceID GetSpaceID() const { return spaceid; }

  double GetTimeStep() const { return timestep; }

  void SetGravity( const vctFixedSizeVector<double,3>& g = GRAVITY )
  { dWorldSetGravity( GetWorldID(), g[0], g[1], g[2] ); }

  void SetERP( double erp )
  { dWorldSetERP( GetWorldID(), erp ); }

  void SetCFM( double cfm )
  { dWorldSetCFM( GetWorldID(), cfm ); }

  void SetContactMaxCorrectingVel( double cmcv )
  { dWorldSetContactMaxCorrectingVel( GetWorldID(), cmcv ); }

  //! Process collisions between two geometries
  /**
     This is the main call to process collisions between geometries. This must
     be public since it is called from a C function.
     \param o1 The geometry of the first object
     \param o2 The geometry of the second object
  */
  void Collision( dGeomID o1, dGeomID o2 );
  
  //! Take a simulation step
  void Step();
  
  //! Get a list of contacts
  std::list< osaODEContact > GetContacts();

  void SetContactERP( double erp ){ contacterp = erp; }
  void SetContactBouncing( double cb ){ contactbounce = cb; }
  void SetContactFriction( double mu ){ this->mu = mu; }
  void SetContactSurfaceLayer( double sl )
  { dWorldSetContactSurfaceLayer( GetWorldID(), sl ); }


};

#endif

