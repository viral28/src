/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2009-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnPortability.h>
#include <cisstOSAbstraction/osaSleep.h>

#if (CISST_OS == CISST_DARWIN)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <sawGLUTSimulator/osaGLUT.h>
#include "osaGeometry.h"
#include "osaMeshTriangular.h"

#include <iostream>

osaGLUT* osaGLUT::glut = NULL;

// keyboard callback function
void KeyboardFunc(unsigned char c, int x, int y)
{ osaGLUT::glut->Keyboard( (int)c, x, y ); }

// special key callback function
void SpecialFunc(int c, int x, int y)
{ osaGLUT::glut->Keyboard( c, x, y ); }

// display callback function
void DisplayFunc()
{ osaGLUT::glut->Draw(); }

void IdleFunc(){
  osaGLUT::glut->Refresh(); 
  osaSleep( 0.03 );
}

// process the keyboard input
void osaGLUT::Keyboard(int k, int, int){

  //ESC key (27) quits
  if(k == 27){ exit(0); }

  //left arrow
  if(k == GLUT_KEY_LEFT){
    azimuth -= 0.015;
    if(azimuth <0) {azimuth += 2*cmnPI;}
  }

  //up arrow
  if(k == GLUT_KEY_UP) {if(elevation <= cmnPI/2)  {elevation +=0.005;}}

  //right arrow
  if(k == GLUT_KEY_RIGHT){
    azimuth += 0.015;
    if(azimuth > 2*cmnPI) {azimuth -= 2*cmnPI;}
  }

  //down arrow
  if(k == GLUT_KEY_DOWN) {if(elevation >= -cmnPI/2) {elevation -=0.005;}}

  if(k == 'z') { distance *= 0.980; }
  if(k == 'x') { distance *= 1.02; }

  glutPostRedisplay();
}

void osaGLUT::Refresh()
{ glutPostRedisplay(); }

// compute the camera position
vctFixedSizeVector<double,3> osaGLUT::CameraPosition() const {
  return vctFixedSizeVector<double,3>( distance*cos(azimuth)*cos(elevation),
				       distance*sin(azimuth)*cos(elevation),
				       distance*sin(elevation) );
}

// draw the floor
void osaGLUT::DrawGrid(double width, int subdivision){
  glLineWidth(1);
  glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_LINES);

  for(int i = 0; i<=subdivision; i++){
    float stepsize = width/((float) (subdivision));

    glVertex3f(-width/2 + i*stepsize,width/2,0);
    glVertex3f(-width/2 + i*stepsize,-width/2,0);

    glVertex3f(width/2,-width/2 + i*stepsize,0);
    glVertex3f(-width/2,-width/2 + i*stepsize,0);

  }

  glEnd();
}

// draw the XZY axis
void osaGLUT::DrawXYZ(){
  glLineWidth(2);
  glBegin(GL_LINES);
  glColor3f(1.0,0.0,0.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(1.0,0.0,0.0);
  glColor3f(0.0,1.0,0.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(0.0,1.0,0.0);
  glColor3f(0.0,0.0,1.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(0.0,0.0,1.0);
  glEnd();
  glLineWidth(1);
}

// draw everything
void osaGLUT::Draw(){

  //clear screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //Set up Projection matrix
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();

  GLdouble aspect = ( (GLdouble) width / (GLdouble) height );
  gluPerspective( 60, aspect, 0.001, 10);

  //Set up Modelview matrix
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  
  //set camera location and bearing
  vctFixedSizeVector<double,3> xyz = CameraPosition();

  gluLookAt(xyz[0], xyz[1], xyz[2], 0, 0, 0, 0, 0, 1);
  
  DrawGrid(10,10);
  DrawXYZ();

  for( size_t i=0; i<geoms.size(); i++ )
    { geoms[i]->Draw(); }

  glutSwapBuffers();

}

void osaGLUT::Register( const osaGeometry* geom ){  
  if( glut != NULL && geom != NULL )
    glut->geoms.push_back( geom );
}

void osaGLUT::StartMainLoop(void)
{
  glutMainLoop(); // blocking call
}

osaGeometry* osaGLUT::LoadOBJ( const std::string& filename ){
  osaMeshTriangular* trimesh = new osaMeshTriangular();
  trimesh->LoadOBJ( filename );
  Register(trimesh);
  return trimesh;
}

osaGLUT::osaGLUT( int argc, char** argv){
  
  osaGLUT::glut = this;

  this->x = 0;
  this->y = 0;
  this->width = 640;
  this->height = 480;
  //this->sleepPeriod = -1.0;

  glutInit(&argc, argv);

  //RGB color, z-buffer, double drawing buffer
  glutInitDisplayMode (GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowPosition(x, y);
  glutInitWindowSize(width, height);
  glutCreateWindow(argv[0]);

  //Set glut callback functions
  glutDisplayFunc( DisplayFunc );
  glutIdleFunc( IdleFunc );
  glutKeyboardFunc( KeyboardFunc );
  glutSpecialFunc( SpecialFunc );

  // (REFERENCE: SGI 'light.c' EXAMPLE)
  GLfloat mat_ambient[]    = { 1.0, 1.0, 1.0, 1.0 };  // RGBA
  GLfloat mat_diffuse[]    = { 1.0, 1.0, 1.0, 1.0 };  // RGBA
  GLfloat mat_specular[]   = { 1.0, 1.0, 1.0, 1.0 };  // RGBA
  glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
  glMaterialf(GL_FRONT,  GL_SHININESS, 20.0);
  
  GLfloat lightAmbient[]   = { 0.1, 0.1, 0.1, 1.0 };
  GLfloat lightDiffuse[]   = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lightSpecular[]  = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lightZero[]      = { 0.1, 0.1, 0.1, 0.5 };
  GLfloat lightPosition0[] = {  30.0,  30.0,  30.0, 1.0 };
  GLfloat lightPosition1[] = { -30.0, -30.0, -30.0, 1.0 };

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_SMOOTH);
  glViewport(0, 0, width, height);

  // initialise lighting state set light properties
  glLightModelfv( GL_LIGHT_MODEL_AMBIENT, lightZero);   // global ambient light
  glLightfv( GL_LIGHT0, GL_AMBIENT,  lightAmbient );
  glLightfv( GL_LIGHT0, GL_DIFFUSE,  lightDiffuse );
  glLightfv( GL_LIGHT0, GL_SPECULAR, lightSpecular );
  glLightfv( GL_LIGHT0, GL_POSITION, lightPosition0 );
    
  glLightfv( GL_LIGHT1, GL_AMBIENT,  lightAmbient);
  glLightfv( GL_LIGHT1, GL_DIFFUSE,  lightDiffuse);
  glLightfv( GL_LIGHT1, GL_SPECULAR, lightSpecular);
  glLightfv( GL_LIGHT1, GL_POSITION, lightPosition1 );
  
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_NORMALIZE);
  //glEnable(GL_LINE_SMOOTH);
  //glEnable(GL_BLEND);

  azimuth = 0;
  elevation = 3.14/4.0;
  distance = 1.5;
  perspective = 60;

}
