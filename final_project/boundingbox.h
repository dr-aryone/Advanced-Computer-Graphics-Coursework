#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

#include "glCanvas.h"

#include <cassert>
#include <algorithm>
#include "vectors.h"
#include "utils.h"

// ====================================================================
// ====================================================================

class BoundingBox {

public:

  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  BoundingBox() { Set(Vec3f(0,0,0),Vec3f(0,0,0)); }
  BoundingBox(const Vec3f &v) { Set(v,v); }
  BoundingBox(const Vec3f &_minimum, const Vec3f &_maximum) { Set(_minimum,_maximum); }
  ~BoundingBox() { }

  // =========
  // ACCESSORS
  void Get(Vec3f &_minimum, Vec3f &_maximum) const {
    _minimum = minimum;
    _maximum = maximum; }
  const Vec3f& getMin() const { return minimum; }
  const Vec3f& getMax() const { return maximum; }
  void getCenter(Vec3f &c) const {
    c = maximum; 
    c -= minimum;
    c *= 0.5f;
    c += minimum;
  }
  double maxDim() const {
    double x = maximum.x() - minimum.x();
    double y = maximum.y() - minimum.y();
    double z = maximum.z() - minimum.z();
    return my_max(x,my_max(y,z));
  }

  // =========
  // MODIFIERS
  void Set(const BoundingBox &bb) {
      Set(bb.minimum,bb.maximum); 
  }
  void Set(const Vec3f &_minimum, const Vec3f &_maximum) {
    assert (minimum.x() <= maximum.x() &&
	    minimum.y() <= maximum.y() &&
	    minimum.z() <= maximum.z());
    minimum = _minimum;
    maximum = _maximum; }
  void Extend(const Vec3f &v) {
    minimum = Vec3f(my_min(minimum.x(),v.x()),
		    my_min(minimum.y(),v.y()),
		    my_min(minimum.z(),v.z()));
    maximum = Vec3f(my_max(maximum.x(),v.x()),
		    my_max(maximum.y(),v.y()),
		    my_max(maximum.z(),v.z())); 
  }  
  void Extend(const BoundingBox &bb) {
    Extend(bb.minimum);
    Extend(bb.maximum); 
  }

  // =========
  // DEBUGGING 
  void Print(const char *s="") const {
    printf ("BOUNDING BOX %s: %f %f %f  -> %f %f %f\n", s,
            minimum.x(),minimum.y(),minimum.z(),
            maximum.x(),maximum.y(),maximum.z()); }

  void initializeVBOs();
  void setupVBOs();
  void drawVBOs();
  void cleanupVBOs();

private:

  // ==============
  // REPRESENTATION
  Vec3f minimum;
  Vec3f maximum;
  
  GLuint bb_verts_VBO;
  GLuint bb_edge_indices_VBO;
};

// ====================================================================
// ====================================================================

#endif

