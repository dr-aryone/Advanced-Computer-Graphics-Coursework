#ifndef __VBO_STRUCTS_H__
#define __VBO_STRUCTS_H__

// helper for VBOs
#define BUFFER_OFFSET(i) ((char *)NULL + (i))

// ======================================================================
// helper structures for VBOs, for rendering (note, the data stored in
// each of these is application specific, adjust as needed!)

struct VBOPos {
  VBOPos() {}
  VBOPos(const Vec3f &p) {
      x = p.x(); y = p.y(); z = p.z();
  }
  float x, y, z;    // position
};

struct VBOPosColor {
  VBOPosColor() {}
  VBOPosColor(const Vec3f &p,const Vec3f &c) {
    x = p.x(); y = p.y(); z = p.z();
    r = c.r(); g = c.g(); b = c.b();
  }
  float x, y, z;    // position
  float r, g, b;    // color
};

struct VBOPosColor4 {
  VBOPosColor4() {}
  VBOPosColor4(const Vec3f &p,const Vec4f &c) {
    x = p.x(); y = p.y(); z = p.z();
    r = c.r(); g = c.g(); b = c.b(); a = c.a();
  }
  float x, y, z;    // position
  float r, g, b, a;    // color
};

struct VBOPosNormal {
  VBOPosNormal() {}
  VBOPosNormal(const Vec3f &p, const Vec3f &n) {
    x = p.x(); y = p.y(); z = p.z();
    nx = n.x(); ny = n.y(); nz = n.z();
  }
  float x, y, z;    // position
  float nx, ny, nz; // normal
};

struct VBOPosNormalColor {
  VBOPosNormalColor() {}
  VBOPosNormalColor(const Vec3f &p, const Vec3f &n, const Vec3f &c) {
    x = p.x(); y = p.y(); z = p.z();
    nx = n.x(); ny = n.y(); nz = n.z();
    r = c.r(); g = c.g(); b = c.b();
  }
  float x, y, z;    // position
  float nx, ny, nz; // normal
  float r, g, b;    // color
};

struct VBOPosNormalColorTexture {
  VBOPosNormalColorTexture() {}
  VBOPosNormalColorTexture(const Vec3f &p, const Vec3f &n, const Vec3f &c, float s_, float t_) {
    x = p.x(); y = p.y(); z = p.z();
    nx = n.x(); ny = n.y(); nz = n.z();
    r = c.x(); g = c.y(); b = c.z();
    s = s_;
    t = t_;
  }
  float x, y, z;    // position
  float nx, ny, nz; // normal
  float r, g, b;    // color
  float s,t;
};

// ======================================================================

struct VBOIndexedEdge {
  VBOIndexedEdge() {}
  VBOIndexedEdge(unsigned int a, unsigned int b) {
    verts[0] = a;
    verts[1] = b;
  }
  unsigned int verts[2];
};

struct VBOIndexedTri {
  VBOIndexedTri() {}
  VBOIndexedTri(unsigned int a, unsigned int b, unsigned int c) {
    verts[0] = a;
    verts[1] = b;
    verts[2] = c;
  }
  unsigned int verts[3];
};

struct VBOIndexedQuad {
  VBOIndexedQuad() {}
  VBOIndexedQuad(unsigned int a, unsigned int b, unsigned int c, unsigned int d) {
    verts[0] = a;
    verts[1] = b;
    verts[2] = c;
    verts[3] = d;
  }
  unsigned int verts[4];
};

// ======================================================================

#endif
