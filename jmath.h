#ifndef JMATH_H
#define JMATH_H

#include "entity.h"
#include "geometry.h"

namespace jmath {
Point furthest_point(Entity* entity, Vector2& d) {
  Point point;

  float highest_dot = -std::numeric_limits<float>::max();

  for (int i = 0; i < entity->vertices_WCS.size(); i++) {
    Vector2 entity_vertex(entity->vertices_WCS[i]);
    float dot = d.dot(entity_vertex);

    if (dot > highest_dot) {
      highest_dot = dot;

      point.x = entity_vertex[0];
      point.y = entity_vertex[1];
    }
  }
  return point;
}

Vector2 support(Entity* A, Entity* B, Vector2& d) {
  Vector2 d_inv = d * -1;

  Point pt_two = furthest_point(B, d_inv);

  return furthest_point(A, d) - pt_two;
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
}  // namespace jmath

#endif