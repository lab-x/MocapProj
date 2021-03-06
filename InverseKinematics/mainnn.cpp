#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include "point.h"
#include "fabrik.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"    // for glm::mat4_cast // casting quaternion->mat4
#include "glm/gtx/quaternion.hpp"  // for glm::rotation
#include "glm/gtx/string_cast.hpp" // for glm::to_string

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265
float posX = 2.0f;
float posY = 2.0f;
float posZ = 0.0f;
float fov = -40.0f;
float rotx = 168;
float roty = 180;

int jointNum = 0;
float tolerance = 0.01;
float moveStep = 0.2;
float epsilon = moveStep * 10;
float goal[3];
float mouse[3];
std::vector<float> endPosition[3];
using namespace std;

//****************************************************
// Some Classes
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};


//****************************************************
// Global Variables
//****************************************************
Viewport    viewport;
Fabrik fabrik(tolerance, epsilon);

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport(0,0,viewport.w,viewport.h);// sets the rectangle that will be the window
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();                // loading the identity matrix for the screen

  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system


  // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  // glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center

  glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch

  //------------------------------------------------------------
}

//****************************************************
// helper methods
//****************************************************
void generateLinks() {
  Point p0(0, 0, 10);
  Point p1(0, 5, 10);
  Point p2(5, 10, 10);
  Point p3(7, 8, 10);
  Point p4(7, 6, 10);

  fabrik.setGoal(p4);
  goal[0] = 7;
  goal[1] = 6;
  goal[2] = 10;
  fabrik.setJoints(p0,p1,p2,p3,p4);

  jointNum = 5;
}

//Code adapted from http://stackoverflow.com/questions/21654501/opengl-glut-draw-pyramid-skeleton-bones
void drawLink(Point point1, Point point2) {
  Eigen::Vector3d _startBoneVec3(point1.getX(), point1.getY(), point1.getZ());
  Eigen::Vector3d _endddBoneVec3(point2.getX(), point2.getY(), point2.getZ());
  // for JOINTS
  //
  glPushMatrix();
      glTranslated( _startBoneVec3(0),_startBoneVec3(1),_startBoneVec3(2) );
      glutWireSphere(0.2,30,30);
  glPopMatrix();
  //
  // for BONES
  //
  glPushMatrix();
      float boneLength = ( _endddBoneVec3  - _startBoneVec3 ).norm();
      glTranslated(        _startBoneVec3(0),_startBoneVec3(1),_startBoneVec3(2) );
      glm::vec3 _glmStart( _startBoneVec3(0),_startBoneVec3(1),_startBoneVec3(2) );
      glm::vec3 _glmEnddd( _endddBoneVec3(0),_endddBoneVec3(1),_endddBoneVec3(2) );
      glm::vec3 _glmDirrr   = glm::normalize( _glmEnddd - _glmStart ); // super important to normalize!!!
      glm::quat _glmRotQuat = glm::rotation( glm::vec3(0,0,1),_glmDirrr); // calculates rotation quaternion between 2 normalized vectors
    //glm::mat4 _glmRotMat = glm::mat4_cast(_glmRotQuat); // quaternion -> mat4
      glm::mat4 _glmRotMat = glm::toMat4   (_glmRotQuat); // quaternion -> mat4
    //std::cout <<  glm::to_string(_glmDirrr) << std::endl;
      glMultMatrixf(glm::value_ptr(_glmRotMat));
      glutWireCone(0.5,boneLength,4,20); // cone with 4 slices = pyramid-like
  glPopMatrix();
}

void printCamera() {
  std::cout << "Pos X, Y, Z:" << posX << " " << posY << " " << posZ << "\n";
  std::cout << "fov, rotx, roty:" << fov << " " << rotx << " " << roty << "\n\n";
}

//****************************************************
// sets the window up
//****************************************************
void initScene(int argc, char *argv[]) {
  glClearColor(0.2f, 0.2f, 0.2f, 0.0f); // Clear to black, fully transparent

  myReshape(viewport.w,viewport.h);

  generateLinks();
}


//***************************************************
// function that does the actual drawing
//***************************************************
void myDisplay() {

  //glEnable(GL_CULL_FACE);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );


  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fov, 1.0, 0.1, 800.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0,0,fov);
  glTranslatef(posX,posY,1);
  glRotatef(rotx,1,0,0);
  glRotatef(roty,0,1,0);  
  glColor3f(0.9f, 0.9f, 0.9f);

  // Enable lighting

  GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mat_shininess[] = { 50.0 };
  GLfloat lightpos0[] = {10, 10, 10, 0.};
  glClearColor (0.0, 0.0, 0.0, 0.0);
  glShadeModel (GL_SMOOTH);

  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glLightfv(GL_LIGHT0, GL_POSITION, lightpos0);

  GLfloat lmodel_ambient[] = { 0.9, 0.9, 0.9, 1.0 };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glPointSize(5.0f);

  glBegin(GL_LINES);
  for(int i=-10;i<=10;i++) {
    if (i==-10) { glColor3f(.6,.3,.3); } else { glColor3f(.25,.25,.25); };
    glVertex3f(i,0,-20);
    glVertex3f(i,0,20);
    // if (i==-10) { glColor3f(.3,.3,.6); } else { glColor3f(.25,.25,.25); };
    // glVertex3f(-10,0,i);
    // glVertex3f(10,0,i);
  };
  glEnd();

  glBegin(GL_POINTS);
  glColor3f(.9,.1,.1);
  glVertex3f(goal[0], goal[1], goal[2]);
  glEnd();

  fabrik.setGoal(goal[0], goal[1], goal[2]);
  fabrik.compute();

  Point* joints = fabrik.getJoints();
  for (int i = 0; i < jointNum-1; i++) {
    drawLink(joints[i], joints[i+1]);
  }

  glFlush();
  glutSwapBuffers();
}
//****************************************************
// called by glut when there are no messages to handle
//****************************************************
void myFrameMove() {
  //nothing here for now
#ifdef _WIN32
  Sleep(10);                                   //give ~10ms back to OS (so as not to waste the CPU)
#endif
  glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}

//Maps mouse movement to goal position changes.
//Moving the mouse on the x and y axis changes the goal accordingly.
//If the Shift button is held, then moving the mouse in the y-axis
//will instead move the goal in the z-axis
void updateGoal(int x, int y, bool hold, bool shift) {
    // GLint viewport2[4];
    // GLdouble modelview[16];
    // GLdouble projection[16];
    // GLfloat winX, winY, winZ;
    // GLdouble posX, posY, posZ;
 
    // glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    // glGetDoublev( GL_PROJECTION_MATRIX, projection );
    // glGetIntegerv( GL_VIEWPORT, viewport2 );
 
    // winX = (float)x;
    // winY = (float)viewport2[3] - (float)y;
    // glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
 
    // gluUnProject( winX, winY, winZ, modelview, projection, viewport2, &posX, &posY, &posZ);
 
    // goal[0] = posX;
    // goal[1] = posY;

    // std::cout << posX << " " << posY << " " << posZ << "\n";

    float scale = 0.03; 
    if (hold) {
      if (shift) {
        goal[2] += (y - mouse[1])*scale;
      } else {
        goal[1] += -(y - mouse[1])*scale;
      }
      goal[0] += (x - mouse[0])*scale;
    }
    mouse[0] = x;
    mouse[1] = y;
}

void mouseClicks(int button, int state, int x, int y) {
  int mod = glutGetModifiers();
  if (mod == GLUT_ACTIVE_SHIFT) {
    updateGoal(x, y, false, true);
  } else {
    updateGoal(x, y, false, false);
  }
}

void myPressedMove(int x,int y) {
  int mod = glutGetModifiers();
  if (mod == GLUT_ACTIVE_SHIFT) {
    updateGoal(x, y, true, true);
  } else {
    updateGoal(x, y, true, false);
  }
}


void processKeys(unsigned char key, int x, int y) {
  switch(key) {
    case 43: // +
      fov += 0.5f; break;
    case 45: // -
      fov -= 0.5f; break;
    case 61: // =
      fov += 0.5f; break;
    case 119: // w
      posY +=  0.5f; break;
    case 97: //a
      posX -= 0.5f; break;
    case 115: //s
      posY -= 0.5f; break;
    case 100: //d
      posX += 0.5f; break;
  }
}

void processSpecialKeys(int key, int x, int y) {
  int mod = glutGetModifiers();
  switch(key) {
    case GLUT_KEY_UP :
      if (mod == GLUT_ACTIVE_SHIFT) {
        goal[2] -= moveStep;
      } else if (mod == GLUT_ACTIVE_CTRL) {
        rotx += 1.0f;
      } else {
        goal[1] += moveStep;
      }
      break;
    case GLUT_KEY_DOWN :
      if (mod == GLUT_ACTIVE_SHIFT) {
        goal[2] += moveStep;
      } else if (mod == GLUT_ACTIVE_CTRL) {
        rotx -= 1.0f;
      } else {
        goal[1] -= moveStep;
      }
      break;
    case GLUT_KEY_LEFT :
      if (mod == GLUT_ACTIVE_CTRL) {
        roty -= 1.0f;
      } else {
        goal[0] -= moveStep;
      }
      break;
    case GLUT_KEY_RIGHT :
      if (mod == GLUT_ACTIVE_CTRL) {
        roty += 1.0f;
      } else {
        goal[0] += moveStep;
      }
      break;
  }
}

//****************************************************
// main
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initalize theviewport size
  viewport.w = 800;
  viewport.h = 800;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Inverse Kinematics");

  initScene(argc, argv);                                 // quick function to set up scene


  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized
  glutIdleFunc(myFrameMove);                   // function to run when not handling any other task
  glutKeyboardFunc(processKeys);
  glutSpecialFunc(processSpecialKeys);
  glutMouseFunc(mouseClicks);
  glutMotionFunc(myPressedMove);        
  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else

  return 0;
}








