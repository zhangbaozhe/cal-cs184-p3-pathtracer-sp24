#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  return intersect(r, nullptr);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  // ref: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  constexpr double epsilon = std::numeric_limits<double>::epsilon();

  Vector3D e1 = p2 - p1;
  Vector3D e2 = p3 - p1;

  Vector3D s1 = cross(r.d, e2);
  double divisor = dot(s1, e1);

  // Parallel to the plane or not
  if (divisor > -epsilon && divisor < epsilon)
    return false;

  double invDivisor = 1.0 / divisor;

  Vector3D s = r.o - p1;
  double u = dot(s, s1) * invDivisor;

  if (u < 0 || u > 1)
    return false;

  Vector3D s2 = cross(s, e1);
  double v = dot(r.d, s2) * invDivisor;

  if (v < 0 || u + v > 1)
    return false;

  double t = dot(e2, s2) * invDivisor;

  if (t < r.min_t || t > r.max_t)
    return false;

  if (isect == nullptr) {
    return true;
  }
  
  r.max_t = t;
  isect->t = t;
  isect->n = (1 - u - v) * n1 + u * n2 + v * n3;
  isect->primitive = this;
  isect->bsdf = get_bsdf();
  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
