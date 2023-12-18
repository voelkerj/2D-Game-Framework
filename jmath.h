#ifndef JMATH_H
#define JMATH_H

#include "entity.h"
#include "geometry.h"

namespace jmath {
Point furthest_point(std::vector<Point> vertices_WCS_entity, Vector2& d) {
  Point point;

  float highest_dot = -std::numeric_limits<float>::max();

  for (int i = 0; i < vertices_WCS_entity.size(); i++) {
    Vector2 entity_vertex(vertices_WCS_entity[i]);
    float dot = d.dot(entity_vertex);

    if (dot > highest_dot) {
      highest_dot = dot;

      point.x = entity_vertex[0];
      point.y = entity_vertex[1];
    }
  }
  return point;
}

Vector2 support(std::vector<Point> vertices_wcs_a, std::vector<Point> vertices_wcs_b, Vector2& d) {
  Vector2 d_inv = d * -1;

  Point pt_two = furthest_point(vertices_wcs_b, d_inv);

  return furthest_point(vertices_wcs_a, d) - pt_two;
}

Vector2 triple_product(Vector2& a, Vector2& b, Vector2& c) {
  // I do not understand how this is a "triple product" I've read multiple
  // versions of this function that seem to use completely different math,
  // and I don't think any are actually "triple product"...
  Vector2 result;

  float ac = a.dot(c);
  float bc = b.dot(c);

  result.value[0] = b.value[0] * ac - a.value[0] * bc;
  result.value[1] = b.value[1] * ac - a.value[1] * bc;

  return result;
}

std::vector<Point> clip(Point v1, Point v2, Vector2 normal, float o) {
  std::vector<Point> clipped;

  float dist1 = normal * v1 - o;
  float dist2 = normal * v2 - o;

  if (dist1 < 0 & dist2 < 0) {
    std::cout << dist1 << ", " << dist2 << "\n";
  }

  if (dist1 >= 0)
    clipped.push_back(v1);
  if (dist2 >= 0)
    clipped.push_back(v2);

  if (dist1 * dist2 < 0) {
    Vector2 e = v2 - v1;

    float u = dist1 / (dist1 - dist2);
    e = e * u;
    e = e + v1;

    Point pt;
    pt.x = e[0];
    pt.y = e[1];

    clipped.push_back(pt);
  }
  return clipped;
}
}  // namespace jmath

#endif