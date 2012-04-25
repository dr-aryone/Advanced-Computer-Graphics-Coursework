#include "glCanvas.h"
#include "argparser.h"
#include "boundingbox.h"
#include "camera.h"
#include "radiosity.h"
#include "raytracer.h"
#include "photon_mapping.h"
#include "mesh.h"
#include "raytree.h"
#include "utils.h"

// ========================================================
// static variables of GLCanvas class

ArgParser* GLCanvas::args = NULL;
Mesh* GLCanvas::mesh = NULL;
Radiosity* GLCanvas::radiosity = NULL;
RayTracer* GLCanvas::raytracer = NULL;
PhotonMapping* GLCanvas::photon_mapping = NULL;

// State of the mouse cursor
int GLCanvas::mouseButton = 0;
int GLCanvas::mouseX = 0;
int GLCanvas::mouseY = 0;
bool GLCanvas::controlPressed = false;
bool GLCanvas::shiftPressed = false;
bool GLCanvas::altPressed = false;

// params for the raytracing animation
int GLCanvas::raytracing_x;
int GLCanvas::raytracing_y;
int GLCanvas::raytracing_skip;

// ========================================================
// Initialize all appropriate OpenGL variables, set
// callback functions, and start the main event loop.
// This function will not return but can be terminated
// by calling 'exit(0)'
// ========================================================

void GLCanvas::initialize(ArgParser *_args, Mesh *_mesh, 
			  RayTracer *_raytracer, Radiosity *_radiosity, PhotonMapping *_photon_mapping) {

  args = _args;
  mesh = _mesh;
  raytracer = _raytracer;
  radiosity = _radiosity;
  photon_mapping = _photon_mapping;

  // setup glut stuff
  glutInitWindowSize(args->width, args->height);
  glutInitWindowPosition(100,100);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
  glutCreateWindow("OpenGL Viewer");
  HandleGLError("in glcanvas initialize");

#ifdef _WIN32
  GLenum err = glewInit();
  if (err != GLEW_OK) {
      fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      exit(1);
  }
#endif
  // basic rendering 
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_SMOOTH);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
  glCullFace(GL_BACK);
  glDisable(GL_CULL_FACE);

  // Initialize callback functions
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  HandleGLError("finished glcanvas initialize");

  RayTree::initializeVBOs();
  if (radiosity) radiosity->initializeVBOs();
  if (photon_mapping) photon_mapping->initializeVBOs();

  RayTree::setupVBOs();
  if (radiosity) radiosity->setupVBOs();
  if (photon_mapping) photon_mapping->setupVBOs();

  HandleGLError("finished glcanvas initialize");

  // Enter the main rendering loop
  glutMainLoop();
}


// ========================================================

void GLCanvas::InitLight() {
  // Set the last component of the position to 0 to indicate
  // a directional light source

  GLfloat position[4] = { 30,30,100, 1};
  GLfloat diffuse[4] = { 0.75,0.75,0.75,1};
  GLfloat specular[4] = { 0,0,0,1};
  GLfloat ambient[4] = { 0.2, 0.2, 0.2, 1.0 };

  GLfloat zero[4] = {0,0,0,0};
  glLightfv(GL_LIGHT1, GL_POSITION, position);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
  glLightfv(GL_LIGHT1, GL_AMBIENT, zero);
  glEnable(GL_LIGHT1);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_COLOR_MATERIAL);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

  GLfloat spec_mat[4] = {1,1,1,1};
  float glexponent = 30;
  glMaterialfv(GL_FRONT, GL_SHININESS, &glexponent);
  glMaterialfv(GL_FRONT, GL_SPECULAR, spec_mat);

  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  float back_color[] = { 0.2,0.8,0.8,1};
  glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, back_color);
  glEnable(GL_LIGHT1);
}


void GLCanvas::display(void) {
  glDrawBuffer(GL_BACK);

  Vec3f bg = mesh->background_color;
  // Clear the display buffer, set it to the background color
  glClearColor(bg.r(),bg.g(),bg.b(),0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Set the camera parameters
  mesh->camera->glInit(args->width, args->height);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mesh->camera->glPlaceCamera();
  InitLight(); // light will be a headlamp!

  if (args->intersect_backfacing)
    glDisable(GL_CULL_FACE);
  else
    glEnable(GL_CULL_FACE);

  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  
  //  glCallList(display_list_index);
  HandleGLError(); 

  radiosity->drawVBOs();
  photon_mapping->drawVBOs();
  RayTree::drawVBOs();
   
  // Swap the back buffer with the front buffer to display
  // the scene
  glutSwapBuffers();
}

// ========================================================
// Callback function for window resize
// ========================================================

void GLCanvas::reshape(int w, int h) {
  args->width = w;
  args->height = h;

  // Set the OpenGL viewport to fill the entire window
  glViewport(0, 0, (GLsizei)args->width, (GLsizei)args->height);

  // Set the camera parameters to reflect the changes
  mesh->camera->glInit(args->width, args->height);
}

// ========================================================
// Callback function for mouse click or release
// ========================================================

void GLCanvas::mouse(int button, int /*state*/, int x, int y) {
  args->raytracing_animation = false;
  // Save the current state of the mouse.  This will be
  // used by the 'motion' function
  mouseButton = button;
  mouseX = x;
  mouseY = y;

  shiftPressed = (glutGetModifiers() & GLUT_ACTIVE_SHIFT) != 0;
  controlPressed = (glutGetModifiers() & GLUT_ACTIVE_CTRL) != 0;
  altPressed = (glutGetModifiers() & GLUT_ACTIVE_ALT) != 0;
}

// ========================================================
// Callback function for mouse drag
// ========================================================

void GLCanvas::motion(int x, int y) {
  // Left button = rotation
  // (rotate camera around the up and horizontal vectors)
  if (mouseButton == GLUT_LEFT_BUTTON) {
    if (controlPressed && altPressed) {
      mesh->camera->truckCamera((mouseX-x)*0.5, (y-mouseY)*0.5);
      mouseX = x;
      mouseY = y;
    } else {    
      mesh->camera->rotateCamera(0.005*(mouseX-x), 0.005*(mouseY-y));
      mouseX = x;
      mouseY = y;
    }
  }
  // Middle button = translation
  // (move camera perpendicular to the direction vector)
  else if (mouseButton == GLUT_MIDDLE_BUTTON) {
    mesh->camera->truckCamera((mouseX-x)*0.5, (y-mouseY)*0.5);
    mouseX = x;
    mouseY = y;
  }
  // Right button = dolly or zoom
  // (move camera along the direction vector)
  else if (mouseButton == GLUT_RIGHT_BUTTON) {
    if (controlPressed) {
      mesh->camera->zoomCamera(mouseY-y);
    } else {
      mesh->camera->dollyCamera(mouseY-y);
    }
    mouseX = x;
    mouseY = y;
  }

  // Redraw the scene with the new camera parameters
  glutPostRedisplay();
}

// ========================================================
// Callback function for keyboard events
// ========================================================

void GLCanvas::keyboard(unsigned char key, int x, int y) {
  args->raytracing_animation = false;
  switch (key) {
    // RAYTRACING STUFF
  case 'r':  case 'R':
    // animate raytracing of the scene
    args->gather_indirect=false;
    args->raytracing_animation = !args->raytracing_animation;
    if (args->raytracing_animation) {
      raytracing_skip = my_max(args->width,args->height) / 10;
      if (raytracing_skip % 2 == 0) raytracing_skip++;
      assert (raytracing_skip >= 1);
      raytracing_x = raytracing_skip/2;
      raytracing_y = raytracing_skip/2;
      display(); // clear out any old rendering
      printf ("raytracing animation started, press 'R' to stop\n");
    } else
      printf ("raytracing animation stopped, press 'R' to start\n");    
    break;
  case 't':  case 'T': {
    // visualize the ray tree for the pixel at the current mouse position
    int i = x;
    int j = glutGet(GLUT_WINDOW_HEIGHT)-y;
    RayTree::Activate();
    raytracing_skip = 1;
    TraceRay(i,j);
    RayTree::Deactivate();
    // redraw
  RayTree::setupVBOs();
  radiosity->setupVBOs();
  photon_mapping->setupVBOs();


  glutPostRedisplay();
    break; }

  case 'l':  case 'L': { 
    // toggle photon rendering
    args->render_photons = !args->render_photons;
    glutPostRedisplay();
    break; }
  case 'k':  case 'K': { 
    // toggle photon rendering
    args->render_kdtree = !args->render_kdtree;
    glutPostRedisplay();
    break; }
  case 'p':  case 'P': { 
    // toggle photon rendering
    photon_mapping->TracePhotons();
    photon_mapping->setupVBOs();
    glutPostRedisplay();
    break; }
  case 'g':  case 'G': { 
    args->gather_indirect = true;
    args->raytracing_animation = !args->raytracing_animation;
    if (args->raytracing_animation) {
      raytracing_skip = my_max(args->width,args->height) / 10;
      if (raytracing_skip % 2 == 0) raytracing_skip++;
      assert (raytracing_skip >= 1);
      raytracing_x = raytracing_skip/2;
      raytracing_y = raytracing_skip/2;
      display(); // clear out any old rendering
      printf ("photon mapping animation started, press 'G' to stop\n");
    } else
      printf ("photon mapping animation stopped, press 'G' to start\n");    
    break; 
  }
    
    // RADIOSITY STUFF
  case ' ': 
    // a single step of radiosity
    radiosity->Iterate();
    radiosity->setupVBOs();
    glutPostRedisplay();
    break;
  case 'a': case 'A':
    // animate radiosity solution
    args->radiosity_animation = !args->radiosity_animation;
    if (args->radiosity_animation) 
      printf ("radiosity animation started, press 'A' to stop\n");
    else
      printf ("radiosity animation stopped, press 'A' to start\n");
    break;
  case 's': case 'S':
    // subdivide the mesh for radiosity
    radiosity->Cleanup();
    radiosity->getMesh()->Subdivision();
    radiosity->Reset();
    radiosity->setupVBOs();
    glutPostRedisplay();
    break;
  case 'c': case 'C':
    // clear the radiosity solution
    radiosity->Reset();
    radiosity->setupVBOs();
    glutPostRedisplay();
    break;

    // VISUALIZATIONS
  case 'w':  case 'W':
    // render wireframe mode
    args->wireframe = !args->wireframe;
    glutPostRedisplay();
    break;
  case 'v': case 'V':
    // toggle the different visualization modes
    args->render_mode = RENDER_MODE((args->render_mode+1)%NUM_RENDER_MODES);
    switch (args->render_mode) {
    case RENDER_MATERIALS: std::cout << "RENDER_MATERIALS\n"; fflush(stdout); break;
    case RENDER_LIGHTS: std::cout << "RENDER_LIGHTS\n"; fflush(stdout); break;
    case RENDER_UNDISTRIBUTED: std::cout << "RENDER_UNDISTRIBUTED\n"; fflush(stdout); break;
    case RENDER_ABSORBED: std::cout << "RENDER_ABSORBED\n"; fflush(stdout); break;
    case RENDER_RADIANCE: std::cout << "RENDER_RADIANCE\n"; fflush(stdout); break;
    case RENDER_FORM_FACTORS: std::cout << "RENDER_FORM_FACTORS\n"; fflush(stdout); break;
    default: assert(0); }
    radiosity->setupVBOs();
    glutPostRedisplay();
    break;
  case 'i':  case 'I':
    // interpolate patch illumination values
    args->interpolate = !args->interpolate;
    radiosity->setupVBOs();
    glutPostRedisplay();
    break;
  case 'b':  case 'B':
    // interpolate patch illumination values
    args->intersect_backfacing = !args->intersect_backfacing;
    glutPostRedisplay();
    break;

  case 'q':  case 'Q':
    // quit
    delete GLCanvas::photon_mapping;
    delete GLCanvas::raytracer;
    delete GLCanvas::radiosity;
    delete GLCanvas::mesh;
    exit(0);
    break;
  default:
    printf("UNKNOWN KEYBOARD INPUT  '%c'\n", key);
  }
}

MTRand generator;
// trace a ray through pixel (i,j) of the image an return the color
Vec3f GLCanvas::TraceRay(double i, double j) {
  // compute and set the pixel color
  int max_d = my_max(args->width,args->height);
  Vec3f color = Vec3f(0,0,0);
  int sampleAA = args->num_antialias_samples;

  while(sampleAA>0){


  // ==================================
  // ASSIGNMENT: IMPLEMENT ANTIALIASING
  // ==================================
  // Here's what we do with a single sample per pixel:
  // construct & trace a ray through the center of the pixle
    double a = GLOBAL_mtrand.rand();
    double b = GLOBAL_mtrand.rand();
    double x = (i+a-args->width/2.0)/double(max_d)+0.5;
    double y = (j+b-args->height/2.0)/double(max_d)+0.5;
    Ray r = mesh->camera->generateRay(x,y);
    Hit hit = Hit();
    int num_timesteps = mesh->numTimesteps();
    //std::cout << "num_timesteps: " << num_timesteps << std::endl;
    Vec3f tmpcolor = Vec3f(0,0,0);
    
    for (int t = 0; t<num_timesteps-1; t++) {
      //std::cout << "t:" << t << std::endl;
      tmpcolor += raytracer->TraceRay(r,hit,args->num_bounces,t)/args->num_antialias_samples;
    }
    color += tmpcolor/(double)num_timesteps;
  // add that ray for visualizatio;
    RayTree::AddMainSegment(r,0,hit.getT());
    sampleAA--;
  }

  // return the color
  return color;
}

// Scan through the image from the lower left corner across each row
// and then up to the top right.  Initially the image is sampled very
// coarsely.  Increment the static variables that track the progress
// through the scans
int GLCanvas::DrawPixel() {
  if (raytracing_x > args->width) {
    raytracing_x = raytracing_skip/2;
    raytracing_y += raytracing_skip;
  }
  if (raytracing_y > args->height) {
    if (raytracing_skip == 1) return 0;
    raytracing_skip = raytracing_skip / 2;
    if (raytracing_skip % 2 == 0) raytracing_skip++;
    assert (raytracing_skip >= 1);
    raytracing_x = raytracing_skip/2;
    raytracing_y = raytracing_skip/2;
    glEnd();
    glPointSize(raytracing_skip);
    glBegin(GL_POINTS);
  }

  // compute the color and position of intersection
  Vec3f color= TraceRay(raytracing_x, raytracing_y);
  double r = linear_to_srgb(color.x());
  double g = linear_to_srgb(color.y());
  double b = linear_to_srgb(color.z());
  glColor3f(r,g,b);
  //  glColor3f(1,0,0);
  double x = 2 * (raytracing_x/double(args->width)) - 1;
  double y = 2 * (raytracing_y/double(args->height)) - 1;
  glVertex3f(x,y,-1);
  raytracing_x += raytracing_skip;
  return 1;
}


void GLCanvas::idle() {
  if (args->radiosity_animation) {
    double undistributed = radiosity->Iterate();
    if (undistributed < 0.001) {
      args->radiosity_animation = false;
      std::cout << "undistributed < 0.001, animation stopped\n"; fflush(stdout);
    }
    radiosity->setupVBOs();
    glutPostRedisplay();
  }
  if (args->raytracing_animation) {
    // draw 100 pixels and then refresh the screen and handle any user input
    glDisable(GL_LIGHTING);
    glDrawBuffer(GL_FRONT);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glPointSize(raytracing_skip);
    glBegin(GL_POINTS);
    for (int i = 0; i < 100; i++) {
      if (!DrawPixel()) {
	args->raytracing_animation = false;
	break;
      }
    }
    glEnd();
    glFlush();
  }
}

// ========================================================
// ========================================================

int HandleGLError(const std::string &message) {
  GLenum error;
  int i = 0;
  while ((error = glGetError()) != GL_NO_ERROR) {
    if (message != "") {
      std::cout << "[" << message << "] ";
    }
    std::cout << "GL ERROR(" << i << ") " << gluErrorString(error) << std::endl;
    i++;
  }
  if (i == 0) return 1;
  return 0;
}

// ========================================================
// ========================================================
