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


#include <sawOpenDynamicsEngine/osaODEWorld.h>
#include <cisstCommon/cmnLogger.h>

osaODEContact::osaODEContact() :
  body1( NULL ),
  body2( NULL ),
  position( vctFixedSizeVector<double,3>( 0.0 ) ),
  normal( vctFixedSizeVector<double,3>( 0.0 ) ),
  depth( 0.0 ) {}

osaODEContact::osaODEContact( osaODEBody* b1, 
				  osaODEBody* b2,
				  const vctFixedSizeVector<double,3>& pos,
				  const vctFixedSizeVector<double,3>& n,
				  double d ) :
  body1( b1 ),
  body2( b2 ),
  position( pos ),
  normal( n ),
  depth( d ) {}


const vctFixedSizeVector<double,3> osaODEWorld::GRAVITY = vctFixedSizeVector<double,3>(0.0, 0.0, -9.81);

osaODEWorld::osaODEWorld( double period,
			      const vctFixedSizeVector<double,3>& g ) : 

  timestep( period ),
  contacterp( 0.8 ),
  contactbounce( 0.1 ),
  mu( 0.2 ){

  dInitODE2(0);                           // initialize the engine

  worldid = dWorldCreate();               // create new world for bodies
  spaceid = dSimpleSpaceCreate(0);        // create a new space for geometries
  contactsgid = dJointGroupCreate(0);     // create a new contact group

  //floor = dCreatePlane( GetSpaceID(), 0.0, 0.0, 1.0, 0.0 );
  
  // Set the gravity
  SetGravity( g );

  // error reduction parameter
  SetERP( 0.8 );

  // constraint force mixing
  SetCFM( 0.0000001 );
  
  // set the surface layer depth
  SetContactSurfaceLayer( 0.0001 );

  SetContactMaxCorrectingVel( 0.001 );

}

// destroy the world
osaODEWorld::~osaODEWorld(){ 
  dSpaceDestroy( GetSpaceID() );
  dWorldDestroy( GetWorldID() ); 
  dCloseODE();
}

void osaODEWorld::Collision( dGeomID o1, dGeomID o2 ){

  dContact dContacts[osaODEWorld::NUM_CONTACTS];

  int mode = dContactSoftERP | dContactSoftCFM;

  if( 0.0 < contactbounce ) 
    mode |= dContactBounce;

  for(size_t i=0; i<osaODEWorld::NUM_CONTACTS; i++){

    dContacts[i].surface.mode = mode;
    dContacts[i].surface.mu = mu;
    dContacts[i].surface.mu2 = mu;
    dContacts[i].surface.bounce = contactbounce;
    dContacts[i].surface.soft_cfm = 0.0001;
    dContacts[i].surface.soft_erp = contacterp;

  }
 
  int N = dCollide( o1, o2,
		    osaODEWorld::NUM_CONTACTS,
		    &(dContacts[0].geom),
		    sizeof(dContact) );

  for(int i=0; i<N; i++){

    dJointAttach( dJointCreateContact( GetWorldID(), 
				       GetGroupID(), 
				       &dContacts[i] ),
		  dGeomGetBody(o1), 
		  dGeomGetBody(o2) );

    if( o1 != floor && o2 != floor ){
      osaODEBody* body1 = (osaODEBody*)dGeomGetData( o1 );
      osaODEBody* body2 = (osaODEBody*)dGeomGetData( o2 );

      dContactGeom geom = dContacts[i].geom;

      osaODEContact contact(body1, 
			      body2,
			      vctFixedSizeVector<double,3>(geom.pos[0],
							   geom.pos[1],
							   geom.pos[2] ),
			      vctFixedSizeVector<double,3>(geom.normal[0],
							   geom.normal[1],
							   geom.normal[2]),
			      geom.depth);
      contacts.push_back( contact );
      
    }

  }

}

static void space_collision(void *argv, dGeomID o1, dGeomID o2){
  osaODEWorld* world = (osaODEWorld*)argv;

  // if either geoms is a space
  if(dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
    // colliding a space with something
    dSpaceCollide2(o1, o2, argv, &space_collision);
    
    // collide all geoms internal to the space(s)
    if(dGeomIsSpace (o1)) dSpaceCollide((dSpaceID)o1, argv, &space_collision);
    if(dGeomIsSpace (o2)) dSpaceCollide((dSpaceID)o2, argv, &space_collision);
  }

  else {

    // colliding two non-space geoms, so generate contacts
    dBodyID b1 = dGeomGetBody( o1 );
    dBodyID b2 = dGeomGetBody( o2 );
    
    // dAreConnected doesn't like if both bodies are "zero" (that is if both
    // geoms are non-placeable
    //world->Collision( o1, o2 ); 
    if( b1 != 0 && b2 != 0 ) {
      if( dAreConnected( b1, b2 ) == 0 )
	{ world->Collision( o1, o2 ); }
    }
    else{ world->Collision( o1, o2 ); }
  }


}

void osaODEWorld::Step() {

  contacts.clear();

  dSpaceCollide( GetSpaceID(), (void*)this, space_collision);
  dWorldStep( GetWorldID(), GetTimeStep() );

  dJointGroupEmpty( GetGroupID() );

}

std::list<osaODEContact> osaODEWorld::GetContacts(){
  return contacts;
  

  /*
  std::list< osaODEContact > matches;

  std::list< osaODEContact >::const_iterator contact;
  for( contact=ContactsList.begin(); contact!=ContactsList.end(); contact++ ){
    
    if( contact->body1->GetName() == name || contact->body2->GetName() == name )
      { matches.push_back( *contact ); }

  }

  return matches;
  */
}


