#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "glCanvas.h"
#include <cassert>
#include <string>
#include "vectors.h"
#include "image.h"

class ArgParser;
class Ray;
class Hit;

// ====================================================================
// ====================================================================
// A simple Phong-like material 

class Material {

public:

  Material(const std::string &texture_file, const Vec3f &d_color,
	   const Vec3f &r_color, const Vec3f &e_color, double roughness_,
	   bool refractive, double refractive_index, const Vec3f &refractive_color) {
    textureFile = texture_file;
    if (textureFile != "") {
      image = new Image(textureFile);
      ComputeAverageTextureColor();
    } else {
      diffuseColor = d_color;
      image = NULL;
    }
    reflectiveColor = r_color;
    emittedColor = e_color;
    roughness = roughness_;
    // need to initialize texture_id after glut has started
    texture_id = 0;
    transparent = refractive;
    refractiveIndex = refractive_index;
    refractiveColor = refractive_color;
  }
  
  ~Material();

  // ACCESSORS
  const Vec3f& getDiffuseColor() const { return diffuseColor; }
  const Vec3f getDiffuseColor(double s, double t) const;
  const Vec3f& getReflectiveColor() const { return reflectiveColor; }
  const Vec3f& getEmittedColor() const { return emittedColor; }
  const Vec3f& getRefractiveColor() const { return refractiveColor; }
  bool getTransparency() const { return transparent; }
  double getRefractiveIndex() const { return refractiveIndex; }
  double getRoughness() const { return roughness; }
  bool hasTextureMap() const { return (textureFile != ""); } 
  GLuint getTextureID();

  // SHADE
  // compute the contribution to local illumination at this point for
  // a particular light source
  Vec3f Shade
  (const Ray &ray, const Hit &hit, const Vec3f &dirToLight, 
   const Vec3f &lightColor, ArgParser *args) const;
  
protected:

  Material() { exit(0); }
  Material(const Material&) { exit(0); }
  const Material& operator=(const Material&) { exit(0); }

  void ComputeAverageTextureColor();

  // REPRESENTATION
  Vec3f diffuseColor;
  Vec3f reflectiveColor;
  Vec3f emittedColor;
  Vec3f refractiveColor;
  double roughness;
  bool transparent;
  double refractiveIndex;
  
  std::string textureFile;
  GLuint texture_id;
  Image *image;
};

// ====================================================================
// ====================================================================

#endif

