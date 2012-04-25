#include "utils.h"
#include "material.h"
#include "argparser.h"
#include "sphere.h"
#include "vertex.h"
#include "mesh.h"
#include "ray.h"
#include "hit.h"
#include <iostream>

// ====================================================================
// ====================================================================

double lesserPositiveOf(double a, double b) {
  return (a>0 && a<b) ? a : (b>0 ? b : -1);
}

double lesserPositiveOfWithin(double a, double b) {
  return (a>0 && (a<b || b<0)) ? a : (b>0 ? b : -1);
}

bool Sphere::intersect(const Ray &r, Hit &h) const {
  // Test whether a ray intersects this sphere and if so where.
  
  // plug the explicit ray equation into the implict sphere equation and solve
  
  // return true if the sphere was intersected, and update
  // the hit data structure to contain the value of t for the ray at
  // the intersection point, the material, and the normal
  //return false;*/
  
  
  Vec3f R0 = r.getOrigin(),
	Rd = r.getDirection(),
	O = center;
  Rd.Normalize();
  
  double t,
	 a = 1,
	 b = 2*Rd.Dot3(R0 - O),
	 c = (R0 - O).Dot3(R0 - O) - radius*radius,
	 
	 discriminant = sqrt(b*b - 4*a*c);
  
  if (discriminant!=discriminant) return false;
  
  else if (discriminant == 0) {
    t = (-1)*b/(2*a);
    if (t <= 0) return false;
  }
  else {
    double tplus  = ((-1)*b + discriminant)/(2*a),
	   tminus = ((-1)*b - discriminant)/(2*a);
    t = liesWithin(r.getOrigin()) ? lesserPositiveOfWithin(tplus, tminus) : lesserPositiveOf(tplus, tminus);
    if (t == -1) return false;
  }  
  
  Vec3f newpoint = (R0-O) + Rd*t;
  
  Vec3f norm = newpoint; // - center;
  
  norm.Normalize();
  
  h.set(t, material, norm);
  return true;
} 

// ====================================================================
// ====================================================================

bool Sphere::liesWithin(Vec3f point) const {
  // Efectively, is the distance between the center and the point smaller than the radius?
  // NOTE: Squared values are used because squareroots are highly inefficient.
  Vec3f relative = point - center;
  double distanceSquared = relative[0]*relative[0] + relative[1]*relative[1] + relative[2]*relative[2];
  return distanceSquared < radius*radius;
}

// helper function to place a grid of points on the sphere
Vec3f ComputeSpherePoint(double s, double t, const Vec3f center, double radius) {
  double angle = 2*M_PI*s;
  double y = -cos(M_PI*t);
  double factor = sqrt(1-y*y);
  double x = factor*cos(angle);
  double z = factor*-sin(angle);
  Vec3f answer = Vec3f(x,y,z);
  answer *= radius;
  answer += center;
  return answer;
}

void Sphere::addRasterizedFaces(Mesh *m, ArgParser *args) {
  
  // and convert it into quad patches for radiosity
  int h = args->sphere_horiz;
  int v = args->sphere_vert;
  assert (h % 2 == 0);
  int i,j;
  int va,vb,vc,vd;
  Vertex *a,*b,*c,*d;
  int offset = m->numVertices(); //vertices.size();

  // place vertices
  m->addVertex(center+radius*Vec3f(0,-1,0));  // bottom
  for (j = 1; j < v; j++) {  // middle
    for (i = 0; i < h; i++) {
      double s = i / double(h);
      double t = j / double(v);
      m->addVertex(ComputeSpherePoint(s,t,center,radius));
    }
  }
  m->addVertex(center+radius*Vec3f(0,1,0));  // top

  // the middle patches
  for (j = 1; j < v-1; j++) {
    for (i = 0; i < h; i++) {
      va = 1 +  i      + h*(j-1);
      vb = 1 + (i+1)%h + h*(j-1);
      vc = 1 +  i      + h*(j);
      vd = 1 + (i+1)%h + h*(j);
      a = m->getVertex(offset + va);
      b = m->getVertex(offset + vb);
      c = m->getVertex(offset + vc);
      d = m->getVertex(offset + vd);
      m->addRasterizedPrimitiveFace(a,b,d,c,material);
    }
  }

  for (i = 0; i < h; i+=2) {
    // the bottom patches
    va = 0;
    vb = 1 +  i;
    vc = 1 + (i+1)%h;
    vd = 1 + (i+2)%h;
    a = m->getVertex(offset + va);
    b = m->getVertex(offset + vb);
    c = m->getVertex(offset + vc);
    d = m->getVertex(offset + vd);
    m->addRasterizedPrimitiveFace(d,c,b,a,material);
    // the top patches
    va = 1 + h*(v-1);
    vb = 1 +  i      + h*(v-2);
    vc = 1 + (i+1)%h + h*(v-2);
    vd = 1 + (i+2)%h + h*(v-2);
    a = m->getVertex(offset + va);
    b = m->getVertex(offset + vb);
    c = m->getVertex(offset + vc);
    d = m->getVertex(offset + vd);
    m->addRasterizedPrimitiveFace(b,c,d,a,material);
  }
}
