#include "raytree.h"

// ====================================================================
// Initialize the static variables
int RayTree::activated = 0;  
std::vector<Segment> RayTree::main_segments;
std::vector<Segment> RayTree::shadow_segments;
std::vector<Segment> RayTree::reflected_segments;
std::vector<Segment> RayTree::transmitted_segments;

GLuint RayTree::raytree_verts_VBO;
GLuint RayTree::raytree_edge_indices_VBO;
std::vector<VBOPosColor4> RayTree::raytree_verts; 
std::vector<VBOIndexedEdge> RayTree::raytree_edge_indices;

// ====================================================================

void RayTree::initializeVBOs() {
  glGenBuffers(1, &raytree_verts_VBO);
  glGenBuffers(1, &raytree_edge_indices_VBO);
}

void RayTree::setupVBOs() {
  raytree_verts.clear();
  raytree_edge_indices.clear();

  Vec4f main_color(0.7,0.7,0.7,0.7);
  Vec4f shadow_color(0.1,0.9,0.1,0.7);
  Vec4f reflected_color(0.9,0.1,0.1,0.7);
  Vec4f transmitted_color(0.1,0.1,0.9,0.7);

  // initialize the data
  unsigned int i;
  int count = 0;
  for (i = 0; i < main_segments.size(); i++) {
    raytree_verts.push_back(VBOPosColor4(main_segments[i].getStart(),main_color));
    raytree_verts.push_back(VBOPosColor4(main_segments[i].getEnd(),main_color));
    raytree_edge_indices.push_back(VBOIndexedEdge(count,count+1)); count+=2;
  }
  for (i = 0; i < shadow_segments.size(); i++) {
    raytree_verts.push_back(VBOPosColor4(shadow_segments[i].getStart(),shadow_color));
    raytree_verts.push_back(VBOPosColor4(shadow_segments[i].getEnd(),shadow_color));
    raytree_edge_indices.push_back(VBOIndexedEdge(count,count+1)); count+=2;
  }
  for (i = 0; i < reflected_segments.size(); i++) {
    raytree_verts.push_back(VBOPosColor4(reflected_segments[i].getStart(),reflected_color));
    raytree_verts.push_back(VBOPosColor4(reflected_segments[i].getEnd(),reflected_color));
    raytree_edge_indices.push_back(VBOIndexedEdge(count,count+1)); count+=2;
  }
  for (i = 0; i < transmitted_segments.size(); i++) {
    raytree_verts.push_back(VBOPosColor4(transmitted_segments[i].getStart(),transmitted_color));
    raytree_verts.push_back(VBOPosColor4(transmitted_segments[i].getEnd(),transmitted_color));
    raytree_edge_indices.push_back(VBOIndexedEdge(count,count+1)); count+=2;
  }

  assert (2*raytree_edge_indices.size() == raytree_verts.size());
  int num_edges = raytree_edge_indices.size();

  // cleanup old buffer data (if any)
  cleanupVBOs();

  // copy the data to each VBO
  if (num_edges > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,raytree_verts_VBO); 
    glBufferData(GL_ARRAY_BUFFER,
		 sizeof(VBOPosColor4) * num_edges * 2,
		 &raytree_verts[0],
		 GL_STATIC_DRAW); 
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,raytree_edge_indices_VBO); 
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		 sizeof(VBOIndexedEdge) * num_edges,
		 &raytree_edge_indices[0], GL_STATIC_DRAW);
  } 
}

void RayTree::drawVBOs() {
  int num_edges = raytree_edge_indices.size();
  if (num_edges == 0) return;

  // this allows you to see rays passing through objects
  // turn off the depth test and blend with the current pixel color
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

  glDisable(GL_LIGHTING);
  glLineWidth(2);
  glBindBuffer(GL_ARRAY_BUFFER, raytree_verts_VBO);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, sizeof(VBOPosColor4), BUFFER_OFFSET(0));
  glEnableClientState(GL_COLOR_ARRAY);
  glColorPointer(4, GL_FLOAT, sizeof(VBOPosColor4), BUFFER_OFFSET(12));
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, raytree_edge_indices_VBO);
  glDrawElements(GL_LINES, num_edges*2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glEnable(GL_DEPTH_TEST);
}

void RayTree::cleanupVBOs() {
  glDeleteBuffers(1, &raytree_verts_VBO);
  glDeleteBuffers(1, &raytree_edge_indices_VBO);
}
