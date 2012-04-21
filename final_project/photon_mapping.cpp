#include "glCanvas.h"

#include <iostream>
#include <algorithm>

#include "argparser.h"
#include "photon_mapping.h"
#include "mesh.h"
#include "matrix.h"
#include "face.h"
#include "primitive.h"
#include "kdtree.h"
#include "utils.h"
#include "raytracer.h"

// ==========
// DESTRUCTOR
PhotonMapping::~PhotonMapping() {
  // cleanup all the photons
  delete kdtree;
}

// ========================================================================
// Recursively trace a single photon

void PhotonMapping::TracePhoton(const Vec3f &position, const Vec3f &direction, 
				const Vec3f &energy, int iter) {
  
  if(iter>args->num_bounces){
    return;
  }

  Hit h = Hit();
  Ray R = Ray(position, direction);
  bool intersect = raytracer->CastRay(R, h, false);
  if(!intersect){
    return;
  }
  Material *m = h.getMaterial();
  Vec3f normal = h.getNormal();
  Vec3f point = R.pointAtParameter(h.getT());
  Vec3f opDirec = direction;
  opDirec.Negate();
  opDirec.Normalize();
  Vec3f diffuse = m->getDiffuseColor(), reflec = m->getReflectiveColor();
  double diffuseAnswer = diffuse.x()+diffuse.y()+diffuse.z();
  double reflecAnswer = reflec.x()+reflec.y()+reflec.z();
  double total = reflecAnswer+diffuseAnswer;
  diffuseAnswer /= total;
  reflecAnswer /= total;
  double seed = GLOBAL_mtrand.rand();
  if(seed <= diffuseAnswer && seed >= 0){
    Vec3f newEnergy = energy * diffuse;
    Vec3f newPosition = point;
    Vec3f newDirection = Vec3f(GLOBAL_mtrand.rand(),GLOBAL_mtrand.rand(),GLOBAL_mtrand.rand());
    newDirection.Normalize();
    Photon answer = Photon(point,opDirec,newEnergy,iter+1);
    kdtree->AddPhoton(answer);
    TracePhoton(newPosition, newDirection, newEnergy, iter+1);
  }
  else if(seed>diffuseAnswer && seed <= 1){
    Vec3f newEnergy = energy * reflec;
    Vec3f newPosition = point;
    Vec3f newDirection = direction - 2 * direction.Dot3(normal) * normal;
    Photon answer = Photon(point,opDirec,newEnergy,iter+1);
    kdtree->AddPhoton(answer);
    TracePhoton(newPosition, newDirection, newEnergy, iter+1);
  }
  // ==============================================
  // ASSIGNMENT: IMPLEMENT RECURSIVE PHOTON TRACING
  // ==============================================

  // Trace the photon through the scene.  At each diffuse or
  // reflective bounce, store the photon in the kd tree.

  // One optimization is to *not* store the first bounce, since that
  // direct light can be efficiently computed using classic ray
  // tracing.



}


// ========================================================================
// Trace the specified number of photons through the scene

void PhotonMapping::TracePhotons() {
  std::cout << "trace photons" << std::endl;

  // first, throw away any existing photons
  delete kdtree;

  // consruct a kdtree to store the photons
  BoundingBox *bb = mesh->getBoundingBox();
  Vec3f min = bb->getMin();
  Vec3f max = bb->getMax();
  Vec3f diff = max-min;
  min -= 0.001*diff;
  max += 0.001*diff;
  kdtree = new KDTree(BoundingBox(min,max));

  // photons emanate from the light sources
  const std::vector<Face*>& lights = mesh->getLights();

  // compute the total area of the lights
  double total_lights_area = 0;
  for (unsigned int i = 0; i < lights.size(); i++) {
    total_lights_area += lights[i]->getArea();
  }

  // shoot a constant number of photons per unit area of light source
  // (alternatively, this could be based on the total energy of each light)
  for (unsigned int i = 0; i < lights.size(); i++) {  
    double my_area = lights[i]->getArea();
    int num = args->num_photons_to_shoot * my_area / total_lights_area;
    // the initial energy for this photon
    Vec3f energy = my_area/double(num) * lights[i]->getMaterial()->getEmittedColor();
    Vec3f normal = lights[i]->computeNormal();
    for (int j = 0; j < num; j++) {
      Vec3f start = lights[i]->RandomPoint();
      // the initial direction for this photon (for diffuse light sources)
      Vec3f direction = RandomDiffuseDirection(normal);
      TracePhoton(start,direction,energy,0);
    }
  }
}


// ======================================================================
// During ray tracing, when a diffuse (or partially diffuse) object is
// hit, gather the nearby photons to approximate indirect illumination

Vec3f PhotonMapping::GatherIndirect(const Vec3f &point, const Vec3f &normal, const Vec3f &direction_from) const {


  if (kdtree == NULL) { 
    std::cout << "WARNING: Photons have not been traced throughout the scene." << std::endl;
    return Vec3f(0,0,0); 
  }
  int count = args->num_photons_to_collect;
  count = 500;
  //std::cout<<count<<std::endl;
  Vec3f xVec = Vec3f(0.25,0.25,0.25);
  BoundingBox newbb = BoundingBox(point-xVec, point+xVec);
  std::vector<Photon> photonlist;
  bool captured = false;
  while(!captured){
    kdtree->CollectPhotonsInBox(newbb,photonlist);
    if(photonlist.size()>(unsigned int)count){
      captured = true;
    }
    else{
      xVec *= 2;
      newbb = BoundingBox(point-xVec, point+xVec);
      photonlist.clear();
    }
  }
  std::vector<Photon> sortedPhotons;
  double smallest = 100;
  double lowerbound = -1;
  int index = -1;
  for(int i = 0; i<count; i++){
    smallest = 100;
    for (unsigned int j = 0; j<photonlist.size(); j++){
      Vec3f connector = photonlist[j].getPosition()-point;
      double dist = connector.Length();
      if(smallest>dist && lowerbound < dist){
	smallest = dist;
	index = j;
      }
    }
    //std::cout<<index<<" "<<smallest<<" "<<lowerbound<<std::endl;
    assert (index>=0);
    assert((unsigned int)index<photonlist.size());
    assert(smallest>0);

    lowerbound = smallest;
    sortedPhotons.push_back(photonlist[index]);
  }
  
  Vec3f totalEnergy = Vec3f(0,0,0);
  for (unsigned int i=0; i < sortedPhotons.size(); i++){
    totalEnergy += sortedPhotons[i].getEnergy();
  }
  /*xVec = point-xVec;
  double radius = xVec.Length();
  double surfaceArea = 4 * M_PI * (radius * radius);
  */
  //totalEnergy *= sortedPhotons.size();
  
  //  std::cout<<totalEnergy<<std::endl;
  return totalEnergy;
  // ================================================================
  // ASSIGNMENT: GATHER THE INDIRECT ILLUMINATION FROM THE PHOTON MAP
  // ================================================================


  // collect the closest args->num_photons_to_collect photons
  // determine the radius that was necessary to collect that many photons
  // average the energy of those photons over that radius


  // return the color
  //return Vec3f(0,0,0);
}



// ======================================================================
// PHOTON VISUALIZATION FOR DEBUGGING
// ======================================================================


void PhotonMapping::initializeVBOs() {
  glGenBuffers(1, &photon_verts_VBO);
  glGenBuffers(1, &photon_direction_indices_VBO);
  glGenBuffers(1, &kdtree_verts_VBO);
  glGenBuffers(1, &kdtree_edge_indices_VBO);
}

void PhotonMapping::setupVBOs() {
  photon_verts.clear();
  photon_direction_indices.clear();
  kdtree_verts.clear();
  kdtree_edge_indices.clear();

  // initialize the data
  int dir_count = 0;
  int edge_count = 0;
  BoundingBox *bb = mesh->getBoundingBox();
  double max_dim = bb->maxDim();

  if (kdtree == NULL) return;
  std::vector<const KDTree*> todo;  
  todo.push_back(kdtree);
  while (!todo.empty()) {
    const KDTree *node = todo.back();
    todo.pop_back(); 
    if (node->isLeaf()) {
      const std::vector<Photon> &photons = node->getPhotons();
      int num_photons = photons.size();
      for (int i = 0; i < num_photons; i++) {
	const Photon &p = photons[i];
	Vec3f energy = p.getEnergy()*args->num_photons_to_shoot;
	const Vec3f &position = p.getPosition();
	Vec3f other = position + p.getDirectionFrom()*0.02*max_dim;
	photon_verts.push_back(VBOPosColor(position,energy));
	photon_verts.push_back(VBOPosColor(other,energy));
	photon_direction_indices.push_back(VBOIndexedEdge(dir_count,dir_count+1)); dir_count+=2;
      }

      // initialize kdtree vbo
      const Vec3f& min = node->getMin();
      const Vec3f& max = node->getMax();
      kdtree_verts.push_back(VBOPos(Vec3f(min.x(),min.y(),min.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(min.x(),min.y(),max.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(min.x(),max.y(),min.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(min.x(),max.y(),max.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(max.x(),min.y(),min.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(max.x(),min.y(),max.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(max.x(),max.y(),min.z())));
      kdtree_verts.push_back(VBOPos(Vec3f(max.x(),max.y(),max.z())));

      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count  ,edge_count+1)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+1,edge_count+3)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+3,edge_count+2)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+2,edge_count  )); 

      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+4,edge_count+5)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+5,edge_count+7)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+7,edge_count+6)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+6,edge_count+4)); 

      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count  ,edge_count+4)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+1,edge_count+5)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+2,edge_count+6)); 
      kdtree_edge_indices.push_back(VBOIndexedEdge(edge_count+3,edge_count+7)); 


      edge_count += 8;

    } else {
      todo.push_back(node->getChild1());
      todo.push_back(node->getChild2());
    } 
  }
  assert (2*photon_direction_indices.size() == photon_verts.size());
  int num_directions = photon_direction_indices.size();
  int num_edges = kdtree_edge_indices.size();

  // cleanup old buffer data (if any)
  cleanupVBOs();

  // copy the data to each VBO
  if (num_directions > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,photon_verts_VBO); 
    glBufferData(GL_ARRAY_BUFFER,
		 sizeof(VBOPosColor) * num_directions * 2,
		 &photon_verts[0],
		 GL_STATIC_DRAW); 
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,photon_direction_indices_VBO); 
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		 sizeof(VBOIndexedEdge) * num_directions,
		 &photon_direction_indices[0], GL_STATIC_DRAW);
  } 

  if (num_edges > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,kdtree_verts_VBO); 
    glBufferData(GL_ARRAY_BUFFER,
		 sizeof(VBOPos) * num_edges * 2,
		 &kdtree_verts[0],
		 GL_STATIC_DRAW); 
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,kdtree_edge_indices_VBO); 
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		 sizeof(VBOIndexedEdge) * num_edges,
		 &kdtree_edge_indices[0], GL_STATIC_DRAW);
  } 
}

void PhotonMapping::drawVBOs() {

  glDisable(GL_LIGHTING);
  if (args->render_photons) {
    int num_directions = photon_direction_indices.size();
    if (num_directions > 0) {
      // render directions
      glLineWidth(1);
      glBindBuffer(GL_ARRAY_BUFFER, photon_verts_VBO);
      glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, sizeof(VBOPosColor), BUFFER_OFFSET(0));
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_FLOAT, sizeof(VBOPosColor), BUFFER_OFFSET(12));
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, photon_direction_indices_VBO);
      glDrawElements(GL_LINES, num_directions*2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_VERTEX_ARRAY);
      // render hit points
      glPointSize(3);
      glBindBuffer(GL_ARRAY_BUFFER, photon_verts_VBO);
      glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, sizeof(VBOPosColor)*2, BUFFER_OFFSET(0));
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_FLOAT, sizeof(VBOPosColor)*2, BUFFER_OFFSET(12));
      glDrawArrays(GL_POINTS, 0, num_directions);
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_VERTEX_ARRAY);
    }
  }
  if (args->render_kdtree) {
    int num_edges = kdtree_edge_indices.size();
    if (num_edges > 0) {
      glDisable(GL_LIGHTING);
      // render edges
      glLineWidth(1);
      glColor3f(0,1,1);
      glBindBuffer(GL_ARRAY_BUFFER, kdtree_verts_VBO);
      glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, sizeof(VBOPos), BUFFER_OFFSET(0));
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, kdtree_edge_indices_VBO);
      glDrawElements(GL_LINES, num_edges*2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
      glDisableClientState(GL_VERTEX_ARRAY);
    }
  }
}

void PhotonMapping::cleanupVBOs() {
  glDeleteBuffers(1, &photon_verts_VBO);
  glDeleteBuffers(1, &photon_direction_indices_VBO);
  glDeleteBuffers(1, &kdtree_verts_VBO);
  glDeleteBuffers(1, &kdtree_edge_indices_VBO);
}

