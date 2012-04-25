#include "raytracer.h"
#include "material.h"
#include "vectors.h"
#include "argparser.h"
#include "raytree.h"
#include "utils.h"
#include "mesh.h"
#include "face.h"
#include "primitive.h"
#include "photon_mapping.h"
#include "MersenneTwister.h"
#include "sphere.h"
#include <cmath>
#include <iostream>
#include <climits>

#define PI 3.14159265358979323
// Two Helpers for waaaay down below when we ray trace transparent surfaces.
double getAngleBetween(Vec3f theta1, Vec3f theta2) { return acos(theta1.Dot3(theta2)/(theta1.Length()*theta2.Length())) * 180/PI; }
double getRefractionAngle(double incidentAngle, double i_incidence, double i_refraction) {  return asin(sin(incidentAngle * PI/180) * (i_incidence/i_refraction)) * 180/PI;  }


double epsilon = pow(2,-24);
Vec3f epsilonVec = Vec3f(epsilon, epsilon, epsilon);

// ===========================================================================
// casts a single ray through the scene geometry and finds the closest hit
bool RayTracer::CastRay(const Ray &ray, Hit &h, bool use_rasterized_patches, int timestep) const {
  bool answer = false;
  h.set(INT_MAX, NULL, Vec3f(0,0,0));
  // intersect each of the quads
  Hit htmp;
  for (int i = 0; i < mesh->numOriginalQuads(); i++) {
    Face *f = mesh->getOriginalQuad(i);
    htmp = h;
    if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
    if (h.getT() > htmp.getT()) h = htmp;
  }
  
  // intersect each of the primitives (either the patches, or the original primitives)
  if (use_rasterized_patches) {
    for (int i = 0; i < mesh->numRasterizedPrimitiveFaces(); i++) {
      Face *f = mesh->getRasterizedPrimitiveFace(i);
      htmp = h;
      if (f->intersect(ray,h,args->intersect_backfacing)) answer = true;
      if (h.getT() > htmp.getT()) h = htmp;
    }
  } else {
    int num_primitives = mesh->numPrimitives();
    for (int j = 0; j<num_primitives; j++) {
      htmp = h;
      if (mesh->getPrimitive(timestep, j)->intersect(Ray(ray.getOrigin()-epsilonVec,ray.getDirection()),h))
	answer = true;
      if (h.getT() > htmp.getT()) h = htmp;
    }
  }
  return answer;
}

bool RayTracer::CastRayWithBackfacing(const Ray &ray, Hit &h, bool use_rasterized_patches) const {
  
  bool ib_old = args->intersect_backfacing;
  args->intersect_backfacing = 1;
  bool castResult = CastRay(ray, h, use_rasterized_patches);
  args->intersect_backfacing = ib_old;
  
  return castResult;
}

// ===========================================================================
// does the recursive (shadow rays & recursive rays) work
Vec3f RayTracer::TraceRay(Ray &ray, Hit &hit, int bounce_count, double refractive_index, int timestep) const {
  
  // First cast a ray and see if we hit anything.
  hit = Hit();
  bool intersect = CastRay(ray,hit,false);
    
  // if there is no intersection, simply return the background color
  if (intersect == false) {
    return Vec3f(srgb_to_linear(mesh->background_color.r()),
		 srgb_to_linear(mesh->background_color.g()),
		 srgb_to_linear(mesh->background_color.b()));
    
  }
  
  // otherwise decide what to do based on the material
  Material *m = hit.getMaterial();
  assert (m != NULL);
  
  // rays coming from the light source are set to white, don't bother to ray trace further.
  if (m->getEmittedColor().Length() > 0.001) {
    return Vec3f(1,1,1);
  } 
  
  
  Vec3f normal = hit.getNormal();
  Vec3f point = ray.pointAtParameter(hit.getT());
  Vec3f answer;
  
  // ----------------------------------------------
  //  start with the indirect light (ambient light)
  Vec3f diffuse_color = m->getDiffuseColor(hit.get_s(),hit.get_t());
  if (args->gather_indirect) {
    // photon mapping for more accurate indirect light
    answer = diffuse_color * (photon_mapping->GatherIndirect(point, normal, ray.getDirection()) + args->ambient_light);
  } else {
    // the usual ray tracing hack for indirect light
    answer = diffuse_color * args->ambient_light;
  }      
  
  // ----------------------------------------------
  // add contributions from each light that is not in shadow
  
  
  int num_lights = mesh->getLights().size();
  for (int i = 0; i < num_lights; i++) {
    
    Face *f = mesh->getLights()[i];
    Vec3f lightColor = f->getMaterial()->getEmittedColor() * f->getArea();
    Vec3f myLightColor;
    int samplecount = args->num_shadow_samples;
    
    if(samplecount==1){
      Vec3f lightCentroid = f->computeCentroid();
      Vec3f dirToLightCentroid = lightCentroid-point;
      dirToLightCentroid.Normalize();
    

      double distToLightCentroid = (lightCentroid-point).Length();
      myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
      //Vec3f epVec=dirToLightCentroid*epsilon;
      bool shadowInt=false;	
      Hit shadowHit;
      Ray shadowRay = Ray(point-epsilonVec,dirToLightCentroid);
      int num_primitives = mesh->numPrimitives();
      for (int i = 0; i<num_primitives; i++) {
	if (mesh->getPrimitive(i)->intersect(shadowRay,shadowHit)) shadowInt = true;
      }
      RayTree::AddShadowSegment(shadowRay,0,shadowHit.getT());


      if(!shadowInt){
	answer += (m->Shade(ray,hit,dirToLightCentroid,myLightColor,args)/args->num_shadow_samples);
      }
    }
    else{

      while(samplecount>0){
	Vec3f lightCentroid = f->RandomPoint();
	Vec3f dirToLightCentroid = lightCentroid-point;
	dirToLightCentroid.Normalize();
	double distToLightCentroid = (lightCentroid-point).Length();
	myLightColor = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
	//Vec3f epVec=dirToLightCentroid*epsilon;
	bool shadowInt=false;
	Hit shadowHit;
	Ray shadowRay = Ray(point,dirToLightCentroid);
	int num_primitives = mesh->numPrimitives();
	for (int i = 0; i<num_primitives; i++) {
	  if (mesh->getPrimitive(i)->intersect(shadowRay,shadowHit)) shadowInt = true;
	}
    // ===========================================
    // ASSIGNMENT:  ADD SHADOW & SOFT SHADOW LOGIC
    // ===========================================
	RayTree::AddShadowSegment(shadowRay,0,shadowHit.getT());
  
  
    // add the lighting contribution from this particular light at this point
    // (fix this to check for blockers between the light & this surface)
	if(!shadowInt){
	  answer += (m->Shade(ray,hit,dirToLightCentroid,myLightColor,args)/args->num_shadow_samples);
	}
	samplecount--;
      }
    }
  }
      
  // ----------------------------------------------
  // add contribution from reflection, if the surface is shiny
  Vec3f reflectiveColor = m->getReflectiveColor();
  
  
  if(bounce_count > 0 && !(m->getTransparency())){
    Vec3f V = ray.getDirection();
    Vec3f R = V - 2 * V.Dot3(normal) * normal;
    Ray tempray=Ray(point+R*epsilon,R);
    Hit newhit;
    answer+=TraceRay(tempray,newhit,bounce_count-1)*reflectiveColor;
    RayTree::AddReflectedSegment(tempray,epsilon,newhit.getT());
  } else if(bounce_count > 0 && m->getTransparency()){
    Vec3f V = ray.getDirection();
    Vec3f R = V - 2 * V.Dot3(normal) * normal;
    Ray tempray=Ray(point+R*epsilon,R);
    Hit newhit;
    answer+=TraceRay(tempray,newhit,bounce_count-1)*reflectiveColor;
    RayTree::AddReflectedSegment(tempray,epsilon,newhit.getT());
  }
  
  if(m->getTransparency()){
    // Calculate angles and such
    
    double nr = 1.0/m->getRefractiveIndex();
    
    Vec3f I = -ray.getDirection(),
	  N = normal,
	  T = (nr * (N.Dot3(I)) - sqrt( 1 - pow(nr, 2) * (1 - pow(N.Dot3(I), 2)) ) ) * N - nr * I;
    
    T.Normalize();
    
    Hit insideHit;
    
    Vec3f tinystep = ray.pointAtParameter(hit.getT() + epsilon*3);
    
    Ray refractionRay = Ray(tinystep, T); //firstRefractionVector);
    CastRayWithBackfacing(refractionRay, insideHit, false);
    RayTree::AddTransmittedSegment(refractionRay,epsilon,insideHit.getT());
    
    
    
    N = -insideHit.getNormal();
    I = -T;
    nr = m->getRefractiveIndex()/1.0;
    Vec3f T2 = (nr * (N.Dot3(I)) - sqrt( 1 - pow(nr, 2) * (1 - pow(N.Dot3(I), 2)) ) ) * N - nr * I;
    T2.Normalize();
    
    Vec3f escapePoint = refractionRay.pointAtParameter(insideHit.getT() + epsilon*3);
    Ray escapeRay = Ray(escapePoint, T2);
    answer += TraceRay(escapeRay, insideHit, bounce_count) * m->getRefractiveColor();
    RayTree::AddReflectedSegment(escapeRay, epsilon, insideHit.getT());
  }
  
  return answer; 
}
