#ifndef COLLISION_H
#define COLLISION_H

#include "entity.h"
#include "geometry.h"
#include "jmath.h"

#include <vector>
#include <memory>

class Collision {
 public:
  Entity* A_;
  Entity* B_;

  bool debug_{false};

  Point simplex_[3]; // The last simplex for the collision between these two entities
  std::vector<float>depth_; // Depth of the collision for each point in the manifold
  Vector2 normal_;  // Collision normal
  std::vector<Point> manifold_;
  bool resolved_;
  std::vector<std::pair<Point, Point>> lines_to_draw_;

  Collision(Entity* a, Entity* b, Vector2 simplex[3], bool debug_);
  Collision() {std::cout << "Creating collision via default constructor: " << this << "\n";};
  ~Collision();

  // Delete copy and copy assignment
  Collision(const Collision&) = delete;
  Collision& operator=(const Collision&) = delete;

  void EPA();
  Edge FindClosestEdge(std::vector<Point>& polygon);
  void FindCollisionManifold();
  Edge FindRelevantEdge(Entity* entity, bool positive_direction);
  void update(Vector2 simplex[3]);
};

Collision::~Collision() {
  std::cout << this << ": Collision destructor called...\n";
}

Collision::Collision(Entity* a, Entity* b, Vector2 simplex[3], bool debug) {
  std::cout << "Creating collision: " << this << "\n";
  A_ = a;
  B_ = b;

  debug = debug_;

  resolved_ = false;

  update(simplex);
}

void Collision::update(Vector2 simplex[3]) {
  // Reset values
  depth_.clear();
  normal_.reset();
  manifold_.clear();
  lines_to_draw_.clear();

  // Convert simplex to vector of Points (aka a polygon)
  Point pt;

  for (int idx = 0; idx < 3; idx++) {
    pt.x = simplex[idx][0];
    pt.y = simplex[idx][1];

    simplex_[idx] = pt;
  }

  // Resolve Collision
  EPA();
  FindCollisionManifold();
  // resolved_ = true;
}

void Collision::EPA() {
  // Adapted from: https://dyn4j.org/2010/05/epa-expanding-polytope-algorithm/

  // Create polygon and populate it with simplex initially
  std::vector<Point> polygon;
  polygon.push_back(simplex_[0]);
  polygon.push_back(simplex_[1]);
  polygon.push_back(simplex_[2]);

  int iter{0};
  while (true) {
    // Find edge closest to origin
    Edge e = FindClosestEdge(polygon);

    // Get support vector along edge normal
    Vector2 p = jmath::support(A_, B_, e.normal);

    // Dot product support vector with normal
    float d = p.dot(e.normal);

    // If that dot product minus length of edge is less than tolerance
    if (abs(d - e.distance) < .00001 || iter == 99) {
      // penetration vector is the edge normal
      // penetration depth is d
      normal_ = e.normal;
      float collision_depth = d;

      return;
    } else {
      // Convert Vector2 to point
      Point pt;
      pt.x = p[0];
      pt.y = p[1];

      // Add support point to polygon
      polygon.insert(polygon.begin() + e.index, pt);

      iter++;
    }
  }
}

// Closest edge to origin
Edge Collision::FindClosestEdge(std::vector<Point>& polygon) {
  Edge edge;

  edge.distance = std::numeric_limits<float>::max();

  // For each point of simplex
  for (int idx = 0; idx < polygon.size(); idx++) {
    // Get the next point's index to form an edge
    int idx_next_point;

    if (idx == polygon.size() - 1)
      idx_next_point = 0;
    else
      idx_next_point = idx + 1;

    // Create vector of edge by subtracting start point from end point
    Vector2 edge_vect(polygon[idx_next_point] - polygon[idx]);

    // Get vector from the origin to current point
    Vector2 oa = polygon[idx];

    // triple_product(e, oa, e) to get vector from edge to origin
    Vector2 n = jmath::triple_product(edge_vect, oa, edge_vect);

    // Convert that vector to unit vector
    n.convert_to_unit();

    // Get distance from origin to edge
    float distance = n.dot(oa);

    // Check if this distance is closer than previous
    if (distance < edge.distance) {
      edge.distance = distance;
      edge.normal = n;
      edge.index = idx_next_point;
    }
  }
  return edge;
}

void Collision::FindCollisionManifold() {
  // Adapted from: https://dyn4j.org/2011/11/contact-points-using-clipping/

  // Find relevant edges
  Edge e1 = FindRelevantEdge(A_, true);
  Vector2 e1_vect(e1.v1, e1.v2);
  Edge e2 = FindRelevantEdge(B_, false);
  Vector2 e2_vect(e2.v1, e2.v2);

  // Determine reference and incident edge
  Edge ref_edge, inc_edge;
  Vector2 ref_vect, inc_vect;
  bool flip{false};

  if (abs(e1_vect.dot(normal_)) <= abs(e2_vect.dot(normal_))) {
    ref_edge = e1;
    ref_vect = e1_vect;
    inc_edge = e2;
    inc_vect = e2_vect;
  } else {
    ref_edge = e2;
    ref_vect = e2_vect;
    inc_edge = e1;
    inc_vect = e1_vect;

    flip = true;
  }

  ref_vect.convert_to_unit();

  // Clip 1
  float o1 = ref_vect * ref_edge.v1;
  manifold_ = jmath::clip(inc_edge.v1, inc_edge.v2, ref_vect, o1);

  if (manifold_.size() < 2)
    return;

  // Clip 2
  float o2 = ref_vect * ref_edge.v2;
  // ref_vect = ref_vect * -1;
  manifold_ = jmath::clip(manifold_[0], manifold_[1], (ref_vect * -1), o2 * -1);
  // manifold_ = jmath::clip(manifold_[0], manifold_[1], ref_vect, o2 * -1);

  if (manifold_.size() < 2)
    return;

  // Clip 3
  // Vector2 ref_normal(abs(ref_vect[1]), abs(ref_vect[0]));
  Vector2 ref_normal = ref_vect.cross(1);

  // if (flip) {
  //   ref_normal = ref_normal * -1;
  // }

  Vector2 ref_edge_max(ref_edge.max);

  float max = ref_normal.dot(ref_edge_max);

  Vector2 pt_0(manifold_[0]);
  Vector2 pt_1(manifold_[1]);

  if (ref_normal.dot(pt_0) - max < 0.0)
    manifold_.erase(manifold_.begin());
  if (ref_normal.dot(pt_1) - max < 0.0)
    manifold_.erase(manifold_.begin()+1);

  // Setup draw lines if in debug mode
  if (debug_) {
    lines_to_draw_.push_back(std::make_pair(ref_edge.v1, ref_edge.v2));
    lines_to_draw_.push_back(std::make_pair(inc_edge.v1, inc_edge.v2));

    // Determine midpoint of ref_edge
    Point midpoint;
    midpoint.x = ((ref_edge.v2.x - ref_edge.v1.x) / 2) + ref_edge.v1.x;
    midpoint.y = ((ref_edge.v2.y - ref_edge.v1.y) / 2) + ref_edge.v1.y;

    // Offset the ref_normal to be drawn starting at ref_edge midpoint
    Point normal_end(midpoint.x + ref_normal.value[0], midpoint.y + ref_normal.value[1]);
    lines_to_draw_.push_back(std::make_pair(midpoint, normal_end));
  }
}

Edge Collision::FindRelevantEdge(Entity* entity, bool positive_direction) {
  Vector2 normal;
  if (positive_direction)
    normal = normal_;
  else
    normal = normal_ * -1;

  // Find farthest vertex along separation normal
  float highest_dot = -std::numeric_limits<float>::max();
  int vertex_idx{0};

  for (int idx = 0; idx < entity->vertices_WCS.size(); idx++) {
    Vector2 vertex(entity->vertices_WCS[idx]);
    float projection = normal.dot(vertex);

    if (projection > highest_dot) {
      highest_dot = projection;
      vertex_idx = idx;
    }
  }

  Vector2 vertex_vect(entity->vertices_WCS[vertex_idx]);

  // Get the adjacent edges
  int next_idx;
  int prev_idx;

  if (vertex_idx == entity->vertices_WCS.size() - 1)
    next_idx = 0;
  else
    next_idx = vertex_idx + 1;

  if (vertex_idx == 0)
    prev_idx = entity->vertices_WCS.size() - 1;
  else
    prev_idx = vertex_idx - 1;

  Vector2 v0(entity->vertices_WCS[prev_idx]);
  Vector2 v1(entity->vertices_WCS[next_idx]);

  Vector2 left = vertex_vect - v1;
  left.convert_to_unit();

  Vector2 right = vertex_vect - v0;
  right.convert_to_unit();

  // Find the edge containing that vertex that is most perpendicular to collision normal
  // Most perpendicular edge will have a dot product closer to zero
  Point pt1, pt2;
  if (right.dot(normal) <
      left.dot(
          normal)) {  // Maybe this needs to be abs() of each dot product...TBD!
    pt1 = entity->vertices_WCS[prev_idx];
    pt2 = entity->vertices_WCS[vertex_idx];
  } else {
    pt1 = entity->vertices_WCS[vertex_idx];
    pt2 = entity->vertices_WCS[next_idx];
  }

  Edge edge;
  edge.v1 = pt1;
  edge.v2 = pt2;
  edge.max = entity->vertices_WCS[vertex_idx];
  return edge;
}

class CollisionProcessor {
 public:
  std::vector<std::unique_ptr<Collision>> active_collisions_;
  Vector2 simplex_[3];
  bool debug{false};

  void evaluate_collisions(std::vector<Entity*> entities);
  void find_or_create_collision(Entity* A, Entity* B);
  void prune_resolved_collisions();

  // Broadphase
  std::vector<std::pair<Entity*, Entity*>> brute_force(
      std::vector<Entity*> entities);
  bool AABB_overlap(Entity* A, Entity* B);

  // Narrowphase
  bool GJK(Entity* A, Entity* B);

  // void reset_collisions(std::vector<Entity*> entities);
};

void CollisionProcessor::evaluate_collisions(std::vector<Entity*> entities) {

  for (int i = 0; i < entities.size(); i++) {
    entities[i]->calculate_vertices();
    entities[i]->calculate_vertices_in_WCS();
  }

  std::vector<std::pair<Entity*, Entity*>> pairs = brute_force(entities);

  // For each pair
  for (int idx = 0; idx < pairs.size(); idx++) {
    if (GJK(pairs[idx].first, pairs[idx].second)) {
      find_or_create_collision(pairs[idx].first, pairs[idx].second);
    }
  }
}

void CollisionProcessor::find_or_create_collision(Entity* A, Entity* B) {
  // if the active collisions vector is not empty
  if (!active_collisions_.empty()) {

    // Loop through each existing collision
    for (auto&& collision : active_collisions_) {

      // if the two entites are not already involved in a collision
      if (!((A == collision->A_ || A == collision->B_) &&
            (B == collision->A_ || B == collision->B_))) {
              
        // Collision collision(A, B, simplex_, debug);
        std::unique_ptr<Collision> new_collision = std::make_unique<Collision>(A, B, simplex_, debug);

        active_collisions_.push_back(std::move(new_collision));

      } else { // We have already noted this collision, just update it with a new simplex
        collision->update(simplex_);
        collision->debug_ = debug; // Update debug state
      }
    }
  } else {  // active collisions vector is empty, create a new collision by default
    // Collision collision(A, B, simplex_, debug);
    std::unique_ptr<Collision> new_collision = std::make_unique<Collision>(A, B, simplex_, debug);

    active_collisions_.push_back(std::move(new_collision));
  }
}

void CollisionProcessor::prune_resolved_collisions() {
  int idx = 0;

  for (auto&& collision : active_collisions_) {
    if (collision->resolved_) {
      active_collisions_.erase(active_collisions_.begin() + idx);
      idx++;
    }
  }
}

std::vector<std::pair<Entity*, Entity*>> CollisionProcessor::brute_force(
    std::vector<Entity*> entities) {
  std::vector<std::pair<Entity*, Entity*>> pairs;

  for (int i = 0; i < entities.size(); i++) {
    for (int j = i + 1; j < entities.size(); j++) {
      if (AABB_overlap(entities[i], entities[j])) {
        std::pair<Entity*, Entity*> pair(entities[i], entities[j]);
        pairs.push_back(pair);
      }
    }
  }
  return pairs;
}

bool CollisionProcessor::AABB_overlap(Entity* A, Entity* B) {
  // Used to quickly eliminate shapes that are guaranteed to not overlap
  float d1x = B->get_min_x_WCS() - A->get_max_x_WCS();
  float d1y = B->get_min_y_WCS() - A->get_max_y_WCS();
  float d2x = A->get_min_x_WCS() - B->get_max_x_WCS();
  float d2y = A->get_min_y_WCS() - B->get_max_y_WCS();

  if (d1x > 0.0f || d1y > 0.0f)
    return false;
  if (d2x > 0.0f || d2y > 0.0f)
    return false;

  return true;
}

bool CollisionProcessor::GJK(Entity* A, Entity* B) {
  // Adapted from: https://github.com/kroitor/gjk.c/blob/master/gjk.c

  int index = 0;

  Vector2 d(1, 0);

  Vector2 a = jmath::support(A, B, d);
  simplex_[index] = a;

  if (a.dot(d) <= 0)
    return false;  // No collision

  d = a * -1;

  while (true) {
    Vector2 a = jmath::support(A, B, d);
    index++;
    simplex_[index] = a;

    if (a.dot(d) <= 0)  // check if point B is beyond the origin with CO * OB
      return false;     // No collision

    Vector2 ao = a * -1;

    if (index < 2)  // Line segment
    {
      Vector2 b = simplex_[0];
      Vector2 ab = b - a;
      d = jmath::triple_product(ab, ao, ab);
      if (d.norm() == 0) {
        d.value[0] = ab[1];
        d.value[1] = -ab[0];
      }
      continue;
    }

    Vector2 b = simplex_[1];
    Vector2 c = simplex_[0];
    Vector2 ab = b - a;
    Vector2 ac = c - a;

    Vector2 ac_perpendicular = jmath::triple_product(ab, ac, ac);

    if (ac_perpendicular.dot(ao) >= 0) {
      d = ac_perpendicular;
    } else {
      Vector2 ab_perpendicular = jmath::triple_product(ac, ab, ab);

      if (ab_perpendicular.dot(ao) < 0)
        return true;  // Collision

      simplex_[0] = simplex_[1];

      d = ab_perpendicular;
    }
    simplex_[1] = simplex_[2];
    --index;
  }
  return false;
}
#endif