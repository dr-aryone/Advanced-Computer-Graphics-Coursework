#include "glCanvas.h"

#include <time.h>

#include "MersenneTwister.h"
#include "argparser.h"
#include "mesh.h"
#include "radiosity.h"
#include "photon_mapping.h"
#include "raytracer.h"
#include "utils.h"

MTRand GLOBAL_mtrand;

// =========================================
// =========================================

int main(int argc, char *argv[]) {
  
  // deterministic (repeatable) randomness
  GLOBAL_mtrand = MTRand(37);
  // "real" randomness
  //GLOBAL_mtrand = MTRand((unsigned)time(0));
  //
  
  ArgParser *args = new ArgParser(argc, argv);
  glutInit(&argc, argv);

  Mesh *mesh = new Mesh();
  mesh->Load(args->input_file,args);
  RayTracer *raytracer = new RayTracer(mesh,args);
  Radiosity *radiosity = new Radiosity(mesh,args);
  PhotonMapping *photon_mapping = new PhotonMapping(mesh,args);
  raytracer->setRadiosity(radiosity);
  raytracer->setPhotonMapping(photon_mapping);
  radiosity->setRayTracer(raytracer);
  radiosity->setPhotonMapping(photon_mapping);
  photon_mapping->setRayTracer(raytracer);
  photon_mapping->setRadiosity(radiosity);

  GLCanvas::initialize(args,mesh,raytracer,radiosity,photon_mapping); 

  // well it never returns from the GLCanvas loop...
  delete args;
  return 0;
}

// =========================================
// =========================================
