/*

  Author(s): Simon Leonard
  Created on: November 11 2009

  (C) Copyright 2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaODEBody_h
#define _osaODEBody_h

#include <ode/ode.h>

#include <osg/Geode>

#include <cisstVector/vctFixedSizeVector.h>
#include <cisstVector/vctMatrixRotation3.h>

#include <sawOpenSceneGraph/osaOSGBody.h>
#include <sawOpenDynamicsEngine/sawOpenDynamicsEngineExport.h>

// Forward declaration
class osaODEWorld;

class CISST_EXPORT osaODEBody : public osaOSGBody {
  
 public:

  // The state of a body
  struct State {
    vctMatrixRotation3<double>   R;
    vctFixedSizeVector<double,3> t;
    vctFixedSizeVector<double,3> v;
    vctFixedSizeVector<double,3> w;
  };

 private:
  
  //! The ODE state of the body
  osaODEBody::State ODEstate;
  double scale;
  //! The World ID
  osaODEWorld* odeworld;

  //! The ODE body ID
  dBodyID bodyid;

  //! The ODE mass
  dMass* mass;

  //! The ODE mesh ID
  /**
     This is a structure used by ODE to represent a triangular mesh
  */
  dTriMeshDataID meshid;

  //! The ODE geom ID
  /**
     This is a structure used by ODE to process collisions
  */
  dGeomID geomid;

  //! Data used by ODE Tri Mesh
  dVector3* Vertices;
  int       VertexCount;
  vctDynamicMatrix<double> vctVertices;
  dTriIndex* Indices;
  int        IndexCount;
  double simplify_ratio;

  //! The ODE space ID
  /**
    This is a structure used by ODE to represent collision spaces
  */
  dSpaceID space;

  // Transformation from the center of mass to the intertial frame (base)
  vctFrame4x4<double> Rtcomb;

  void BuildODETriMesh( const vctFixedSizeVector<double,3>& com );

  // This is called from each OSG update traversal
  virtual void UpdateTransform();

  vctMatrixRotation3<double> GetOrientation() const;

  vctFixedSizeVector<double,3> GetPosition() const;

  void Initialize( const vctFrame4x4<double>& Rtwb );
		   
  void Initialize( const vctFrame4x4<double>& Rtwb,
		   double m,
		   const vctFixedSizeVector<double,3>& com,
		   const vctFixedSizeMatrix<double,3,3>& moit );
		   
  
 public:

  //! Main constructor
  /**
     This create an ODE body that can be inserted in an ODE world. This body
     does not have mass such that it cannot be moved (static object).
     \param model The file name of the body's geometry
     \param odeworld The world the body belongs to
     \param Rtwb The position/orientation of the body
     \param osgworld An osg group to use. If NULL the root of odeworld is used
  */
  osaODEBody( const std::string& model,
	      osaODEWorld* odeworld,
	      const vctFrame4x4<double>& Rtwb,
	      double scale = 1.0,
	      double alpha = 1.0,
	      const std::string& options = std::string(""),
	      osaOSGWorld* osgworld = NULL,
	      double simplify_ratio = 0.0 );


  //! Constructor
  /**
     This create an ODE body that can be inserted in an ODE world. This body
     does not have mass such that it cannot be moved (static object). The space
     ID is defined to the world's space.
     \param model The file name of the body's geometry
     \param odeWorld The world the body belongs to
     \param Rtwb The position/orientation of the body
     \param osgworld An osg group to use. If NULL the root of odeworld is used
  */
  osaODEBody( const std::string& model,
	      osaODEWorld* odeworld,
	      const vctFrm3& Rtwb,
	      double scale = 1.0,
	      double alpha = 1.0,
	      const std::string& options = std::string(""),
	      osaOSGWorld* osgworld = NULL,
	      double simplify_ratio = 0.0 );
			

  //! Main constructor
  /**
     This create an ODE body that can be inserted in an ODE world. This body 
     has a mass such that it can be moved.
     \param model The file name of the body's geometry
     \param odeWorld The world the body belongs to
     \param Rtwb The position/orientation of the body
     \param m The mass of the body
     \param com The center of mass of the body
     \param moit The moment of inertia tensor
     \param osgworld An osg group to use. If NULL the root of odeworld is used
  */
  osaODEBody(	const std::string& model,
		osaODEWorld* odeWorld,
		const vctFrame4x4<double>& Rtwb,
		double m,
		const vctFixedSizeVector<double,3>& com,
		const vctFixedSizeMatrix<double,3,3>& moit,
		double scale = 1.0,
		double alpha = 1.0,
		const std::string& options = std::string(""),
		osaOSGWorld* osgworld = NULL,
		double simplify_ratio = 0.0 );

  //! Main constructor
  /**
     This create an ODE body that can be inserted in an ODE world. This body 
     has a mass such that it can be moved.
     \param model The file name of the body's geometry
     \param odeWorld The world the body belongs to
     \param Rtwb The position/orientation of the body
     \param m The mass of the body
     \param com The center of mass of the body
     \param moit The moment of inertia tensor
     \param osgworld An osg group to use. If NULL the root of odeworld is used
  */
  osaODEBody(	const std::string& model,
		osaODEWorld* odeworld, 
		const vctFrm3& Rt,
		double m,
		const vctFixedSizeVector<double,3>& tbcom,
		const vctFixedSizeMatrix<double,3,3>& moit,
		double scale = 1.0,
		double alpha = 1.0,
		const std::string& options = std::string(""),
		osaOSGWorld* osgworld = NULL,
		double simplify_ratio = 0.0 );

  //! Main constructor
  /**
     This create an ODE body that can be inserted in an ODE world. This body 
     has a mass such that it can be moved. The center of mass is set to zero
     and the tensor to identity and space ID set to the world's space.
     \param model The file name of the body's geometry
     \param odeWorld The world the body belongs to
     \param Rtwb The position/orientation of the body
     \param m The mass of the body
     \param osgworld An osg group to use. If NULL the root of odeworld is used
  */
  osaODEBody(	const std::string& model,
		osaODEWorld* odeworld, 
		const vctFrame4x4<double>& Rt,
		double m,
		double scale = 1.0,
		double alpha = 1.0,
		const std::string& options = std::string(""),
		osaOSGWorld* osgworld = NULL,
		double simplify_ratio = 0.0 );
  
  //! Main constructor
  /**
     This create an ODE body that can be inserted in an ODE world. This body 
     has a mass such that it can be moved. The center of mass is set to zero
     and the tensor to identity and space ID set to the world's space.
     \param model The file name of the body's geometry
     \param odeWorld The world the body belongs to
     \param Rtwb The position/orientation of the body
     \param m The mass of the body
     \param osgworld An osg group to use. If NULL the root of odeworld is used
  */
  osaODEBody(	const std::string& model,
		osaODEWorld* odeworld, 
		const vctFrm3& Rt,
		double m,
		double scale = 1.0,
		double alpha = 1.0,
		const std::string& options = std::string(""),
		osaOSGWorld* osgworld = NULL,
		double simplify_ratio = 0.0 );
  

  //! Default destructor
  ~osaODEBody();

  //! Enable the body in a simulation
  void Enable();

  //! Disable the body in a simulation
  void Disable();

  //! Query the ID of the body
  dBodyID GetBodyID() const { return bodyid; }

  //! Query the ID of the body
  dGeomID GetGeomID() const { return geomid; }

  const std::string& GetUserData();

  void SetTransform( const vctFrame4x4<double>& Rt );
  vctFrm3 GetTransform() const;

  //vctDynamicMatrix<double> GetVertices() const;
  
};

#endif
