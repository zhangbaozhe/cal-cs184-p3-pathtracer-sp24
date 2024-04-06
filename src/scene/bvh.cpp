#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  // BBox bbox;

  // for (auto p = start; p != end; p++) {
  //   BBox bb = (*p)->get_bbox();
  //   bbox.expand(bb);
  // }

  // BVHNode *node = new BVHNode(bbox);
  // node->start = start;
  // node->end = end;

  // return node;

  if (end - start == 1) {
    BBox bbox = (*start)->get_bbox();
    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;
    return node;
  } 

  BBox bbox;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);

  Vector3D extent = bbox.extent;
  // find the axis with the largest extent
  int axis = extent.x > extent.y ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2);

  auto comparator = [axis](Primitive *a, Primitive *b) {
    return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
  };

  std::sort(start, end, comparator);

  size_t size = end - start;
  if (size <= max_leaf_size) {
    node->start = start;
    node->end = end;
    return node;
  } 

  auto mid = start + size / 2;
  node->l = construct_bvh(start, mid, max_leaf_size);
  node->r = construct_bvh(mid, end, max_leaf_size);
  return node;



}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.



  // for (auto p : primitives) {
  //   total_isects++;
  //   if (p->has_intersection(ray))
  //     return true;
  // }
  // return false;

  // if (ray.o.x >= node->bb.min.x && ray.o.x <= node->bb.max.x &&
  //     ray.o.y >= node->bb.min.y && ray.o.y <= node->bb.max.y &&
  //     ray.o.z >= node->bb.min.z && ray.o.z <= node->bb.max.z) {
  //   return true;
  // }
  

  double t0 = 0.0;
  double t1 = 0.0;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true;
      }
    }
    return false;
  }

  if (has_intersection(ray, node->l)) {
    return true;
  }

  if (has_intersection(ray, node->r)) {
    return true;
  }

  return false;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.



  // bool hit = false;
  // for (auto p : primitives) {
  //   total_isects++;
  //   hit = p->intersect(ray, i) || hit;
  // }
  // return hit;

  // if (ray.o.x >= node->bb.min.x && ray.o.x <= node->bb.max.x &&
  //     ray.o.y >= node->bb.min.y && ray.o.y <= node->bb.max.y &&
  //     ray.o.z >= node->bb.min.z && ray.o.z <= node->bb.max.z) {
  //   for (auto p = node->start; p != node->end; p++) {
  //     (*p)->intersect(ray, i);
  //   }
  //   return true;
  // }


  double t0 = 0.0;
  double t1 = 0.0;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit;
  }

  bool hit = false;
  if (intersect(ray, i, node->l)) {
    hit = true;
  }
  
  if (intersect(ray, i, node->r)) {
    hit = true;
  }

  return hit;

}

} // namespace SceneObjects
} // namespace CGL
