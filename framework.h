#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <cinttypes>
#include <iostream>  //DEBUG
#include <map>
#include <string>
#include <vector>

#include "SDL.h"
#include "SDL_image.h"
#include "geometry.h"
#include "physics.h"

struct Camera {
  // Position of Camera's center point
  //   -centerpoint is also the center of the window
  float pos_x;
  float pos_y;

  // Width of the Camera's projected FOV
  float FOV_width;
  float FOV_height;
};

class Animation {
 public:
  int frame_idx{0};
  int update_interval;  // milliseconds
  Uint32 prev_update_ticks{0};
  std::vector<SDL_Rect> frame_rects;

  Animation(){};
  ~Animation(){};

  void add_frame(int x, int y, int width, int height);
  SDL_Rect get_frame(Uint32 current_ticks);
  void reset();
};

void Animation::add_frame(int x, int y, int width, int height) {
  SDL_Rect rect;
  rect.x = x;
  rect.y = y;
  rect.w = width;
  rect.h = height;

  frame_rects.push_back(rect);
}

SDL_Rect Animation::get_frame(Uint32 current_ticks) {

  Uint32 elapsed_ticks = current_ticks - this->prev_update_ticks;

  // if we are past the update interval, get next frame
  if (elapsed_ticks >= update_interval) {
    this->prev_update_ticks = current_ticks;
    frame_idx++;
  }

  // if we have advanced past the last frame, get first frame
  if (frame_idx > frame_rects.size() - 1)
    reset();

  return frame_rects[frame_idx];
}

void Animation::reset() {
  frame_idx = 0;
}

class Entity {
 public:
  float size_x;
  float size_y;
  float mass{0};
  State state;
  std::map<std::string, Animation> animations;
  std::string current_animation;
  SDL_Texture* sprite_sheet;
  MomentOfInertia MoI;
  Shape shape;
  std::vector<Point> vertices;
  std::vector<Point> vertices_WCS;
  bool collision{false};
  // Vector2 collision_vector;
  // float collision_depth;
  std::string name;

  Entity(){};
  ~Entity(){};

  void load_sprite_sheet(std::string sprite_sheet_path, SDL_Renderer* renderer);
  void update_state(std::vector<Force> forces, std::vector<Moment> moments,
                    Uint32 elapsed_time);

  void calculate_MOI();
  void calculate_vertices();
  void calculate_vertices_in_WCS();

  float get_min_x_WCS();
  float get_max_x_WCS();
  float get_min_y_WCS();
  float get_max_y_WCS();
};

void Entity::load_sprite_sheet(std::string sprite_sheet_path,
                               SDL_Renderer* renderer) {
  sprite_sheet = IMG_LoadTexture(renderer, sprite_sheet_path.c_str());

  if (!sprite_sheet) {
    std::cout << "Failed to load file " << sprite_sheet_path << "\n";
    exit(1);
  }
}

void Entity::update_state(std::vector<Force> forces,
                          std::vector<Moment> moments, Uint32 elapsed_time) {

  float elapsed_time_float = elapsed_time / 1000.0;  // Convert ms to s

  Force resultant_force;
  Moment resultant_moment;

  // TRANSLATION
  // Force
  if (forces.size() > 0) {
    for (Force& force : forces) {
      resultant_force.fx += force.fx;
      resultant_force.fy += force.fy;
    }

    // Acceleration
    state.acc_x += resultant_force.fx / mass;
    state.acc_y += resultant_force.fy / mass;
  }

  // Velocity
  state.vel_x += state.acc_x * elapsed_time_float;
  state.vel_y += state.acc_y * elapsed_time_float;

  // Position
  state.pos_x += state.vel_x * elapsed_time_float;
  state.pos_y += state.vel_y * elapsed_time_float;

  // ROTATION
  // Moment
  if (moments.size() > 0) {
    for (Moment& moment : moments) {
      resultant_moment.value += moment.value;
    }

    // Angular Velocity
    state.angular_velocity +=
        (resultant_moment.value / MoI.value) * (M_PI / 180);
  }

  // Angle
  state.angle += state.angular_velocity * elapsed_time_float;
}

void Entity::calculate_MOI() {
  if (shape == circle)
    MoI.calculate_circle(mass, size_x / 2.0);
  else if (shape == box)
    MoI.calculate_box(mass, size_x, size_y);
}

void Entity::calculate_vertices() {
  vertices.clear();
  if (shape == circle) {
    for (int deg = 360; deg > 0; deg -= 5) {
      Point vertex;

      // Body coordinate system
      vertex.x = (size_x / 2) * cos(deg * (M_PI / 180));
      vertex.y = (size_y / 2) * sin(deg * (M_PI / 180));

      vertices.push_back(vertex);
    }
  } else if (shape == box) {

    // Body coordinate system
    Point vertex;
    vertex.x = -size_x / 2;
    vertex.y = -size_y / 2;
    Point vertex2;
    vertex2.x = size_x / 2;
    vertex2.y = -size_y / 2;
    Point vertex3;
    vertex3.x = size_x / 2;
    vertex3.y = size_y / 2;
    Point vertex4;
    vertex4.x = -size_x / 2;
    vertex4.y = size_y / 2;

    vertices.push_back(vertex);
    vertices.push_back(vertex2);
    vertices.push_back(vertex3);
    vertices.push_back(vertex4);
  }
}

void Entity::calculate_vertices_in_WCS() {
  vertices_WCS.clear();
  if (shape == circle) {
    // Convert to world coordinate system (ignoring rotation because circle)
    for (Point vertex : vertices) {
      vertex.x += state.pos_x;
      vertex.y += state.pos_y;
      vertices_WCS.emplace_back(vertex);
    }
  } else if (shape == box) {
    Point new_vertex;
    for (Point vertex : vertices) {
      new_vertex.x = (vertex.x * cos(state.angle * (M_PI / 180)) -
                      vertex.y * sin(state.angle * (M_PI / 180))) +
                     state.pos_x;
      new_vertex.y = (vertex.x * sin(state.angle * (M_PI / 180)) +
                      vertex.y * cos(state.angle * (M_PI / 180))) +
                     state.pos_y;
      vertices_WCS.emplace_back(new_vertex);
    }
  }
}

float Entity::get_min_x_WCS() {
  float min_x = std::numeric_limits<float>::max();
  for (Point vertex : vertices_WCS) {
    if (vertex.x < min_x)
      min_x = vertex.x;
  }
  return min_x;
}

float Entity::get_max_x_WCS() {
  float max_x = -std::numeric_limits<float>::max();
  for (Point vertex : vertices_WCS) {
    if (vertex.x > max_x)
      max_x = vertex.x;
  }
  return max_x;
}

float Entity::get_min_y_WCS() {
  float min_y = std::numeric_limits<float>::max();
  for (Point vertex : vertices_WCS) {
    if (vertex.y < min_y)
      min_y = vertex.y;
  }
  return min_y;
}

float Entity::get_max_y_WCS() {
  float max_y = -std::numeric_limits<float>::max();
  for (Point vertex : vertices_WCS) {
    if (vertex.y > max_y)
      max_y = vertex.y;
  }
  return max_y;
}

class Graphics {
 public:
  std::vector<Entity*> entity_queue;
  std::vector<std::pair<Point*, Point*>> line_queue;
  std::vector<Vector2*> vector_queue;
  std::vector<Point*> point_queue;
  SDL_Window* window;
  int window_width;
  int window_height;
  SDL_Renderer* renderer;
  Camera current_camera;
  float scale_x;
  float scale_y;
  bool debug{false};

  Graphics();
  ~Graphics();

  void add_to_queue(Entity& entity);
  void draw_queue(Uint32 current_ticks);
  void clear_queue();

  void add_to_queue(Point& a, Point& b);
  void add_to_queue(Point& a);

  void clear_screen();

  private:
  void draw_entity(Uint32 current_ticks, Entity* entity);
  void draw_line(Point* a, Point* b);
  void draw_point(Point* a);
  bool point_within_camera_view(float x, float y);
};

Graphics::Graphics() {
  window = SDL_CreateWindow("Framework", SDL_WINDOWPOS_UNDEFINED,
                            SDL_WINDOWPOS_UNDEFINED, 1, 1,
                            SDL_WINDOW_FULLSCREEN_DESKTOP);
  SDL_GetWindowSize(window, &window_width, &window_height);

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF); 

  current_camera.pos_x = 0;
  current_camera.pos_y = 0;
  current_camera.FOV_width = 19.2 * 2;
  current_camera.FOV_height = 10.8 * 2;

  scale_x = window_width / current_camera.FOV_width;
  scale_y = window_height / current_camera.FOV_height;
}

Graphics::~Graphics() {
  SDL_DestroyWindow(window);
}

void Graphics::add_to_queue(Entity& entity) {
  entity_queue.push_back(&entity);
}

void Graphics::add_to_queue(Point& a, Point& b) {
  std::pair<Point*, Point*> line(&a,&b);
  line_queue.push_back(line);
}

void Graphics::add_to_queue(Point& a) {
  point_queue.push_back(&a);
}

bool Graphics::point_within_camera_view(float x, float y) {

  if (x >= current_camera.pos_x &
      x <= current_camera.pos_x + current_camera.FOV_width &
      y >= current_camera.pos_y &
      y <= current_camera.pos_y + current_camera.FOV_height)
    return true;

  return false;
}

void Graphics::draw_queue(Uint32 current_ticks) {
  // Draw Entities
  for (auto& entity : entity_queue) {
    draw_entity(current_ticks, entity);
  }

  // Draw Points
  for (auto& point : point_queue) {
    draw_point(point);
  }

  // Draw Lines
  for (auto& line : line_queue) {
    draw_line(line.first, line.second);
  }

  SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);
  SDL_RenderPresent(renderer);
}

void Graphics::draw_entity(Uint32 current_ticks, Entity* entity) {
  
    // If entity is within the camera bounds
    if (point_within_camera_view(
            entity->state.pos_x - entity->size_x / 2,
            entity->state.pos_y - entity->size_y / 2) ||  // Check Bottom-Left
        point_within_camera_view(
            entity->state.pos_x - entity->size_x / 2,
            entity->state.pos_y + entity->size_y / 2) ||  // Check Top-Left
        point_within_camera_view(
            entity->state.pos_x + entity->size_x / 2,
            entity->state.pos_y - entity->size_y / 2) ||  // Check Bottom-Right
        point_within_camera_view(
            entity->state.pos_x + entity->size_x / 2,
            entity->state.pos_y + entity->size_y / 2))  // Check Top-Right
    {

      // Draw Entity
      Animation& animation = entity->animations[entity->current_animation];

      SDL_Rect clipping_rect = animation.get_frame(current_ticks);

      SDL_Rect destination_rect;
      destination_rect.x =
          (entity->state.pos_x - entity->size_x / 2 - current_camera.pos_x) *
          scale_x;
      destination_rect.y =
          window_height -
          (entity->state.pos_y - entity->size_y / 2 - current_camera.pos_y) *
              scale_y -
          (entity->size_y * scale_y);
      destination_rect.w = entity->size_x * scale_x;
      destination_rect.h = entity->size_y * scale_y;

      SDL_RenderCopyEx(renderer, entity->sprite_sheet, &clipping_rect,
                       &destination_rect, -entity->state.angle, NULL,
                       SDL_FLIP_NONE);

      // Draw Vertices
      if (debug) {
        // Color entities red when they collide
        if (entity->collision) {
          SDL_SetTextureColorMod(entity->sprite_sheet, 255, 0, 0);
        } else {
          SDL_SetTextureColorMod(entity->sprite_sheet, 255, 255, 255);
        }

        // Display entity vertices
        for (Point point : entity->vertices_WCS) {
          SDL_SetRenderDrawColor(renderer, 0x48, 0xff, 0x82, 0xFF);

          float pt_x = (point.x - current_camera.pos_x) * scale_x;
          float pt_y =
              window_height -
              (point.y - entity->size_y - current_camera.pos_y) * scale_y -
              (entity->size_y * scale_y);

          SDL_RenderDrawPoint(renderer, pt_x, pt_y);
        }

        // Display origin
        SDL_SetRenderDrawColor(renderer, 0xbf, 0x1b, 0x38, 0xFF);

        float pt_x = (0 - current_camera.pos_x) * scale_x;
        float pt_y = window_height -
                     (0 - entity->size_y - current_camera.pos_y) * scale_y -
                     (entity->size_y * scale_y);
        SDL_RenderDrawPoint(renderer, pt_x, pt_y);

        SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);
      }
    }
}

void Graphics::draw_line(Point* a, Point* b) {
  float a_x = (a->x - current_camera.pos_x) * scale_x;
  float a_y = window_height - (a->y - current_camera.pos_y) * scale_y;

  float b_x = (b->x - current_camera.pos_x) * scale_x;
  float b_y = window_height - (b->y - current_camera.pos_y) * scale_y;

  SDL_SetRenderDrawColor(renderer, 0xbf, 0x1b, 0x38, 0xFF);

  SDL_RenderDrawLine(renderer, a_x, a_y, b_x, b_y);
}

void Graphics::draw_point(Point* a) {
  SDL_SetRenderDrawColor(renderer, 0xff, 0xe9, 0x1a, 0xFF);

  float a_x = (a->x - current_camera.pos_x) * scale_x;
  float a_y = window_height - (a->y - current_camera.pos_y) * scale_y;

  SDL_RenderDrawPoint(renderer, a_x, a_y); // Actual Point

  SDL_RenderDrawPoint(renderer, a_x + 1, a_y);
  SDL_RenderDrawPoint(renderer, a_x - 1, a_y);
  SDL_RenderDrawPoint(renderer, a_x, a_y + 1);
  SDL_RenderDrawPoint(renderer, a_x, a_y - 1);
  SDL_RenderDrawPoint(renderer, a_x + .7071, a_y + .7071);
  SDL_RenderDrawPoint(renderer, a_x + .7071, a_y - .7071);
  SDL_RenderDrawPoint(renderer, a_x - .7071, a_y - .7071);
  SDL_RenderDrawPoint(renderer, a_x - .7071, a_y + .7071);
}

void Graphics::clear_queue() {
  entity_queue.clear();
  point_queue.clear();
  line_queue.clear();
  vector_queue.clear();
}

void Graphics::clear_screen() {
  SDL_RenderClear(renderer);
}

class FrameManager {
 public:
  Uint32 frame_start_ticks;
  Uint32 frame_end_ticks;
  int fps;

  FrameManager(){};
  ~FrameManager(){};

  void start_frame();
  void end_frame();
};

void FrameManager::start_frame() {
  frame_start_ticks = SDL_GetTicks();
}

void FrameManager::end_frame() {
  frame_end_ticks = SDL_GetTicks();
  if ((frame_end_ticks - frame_start_ticks) < (1000 / fps)) {
    SDL_Delay((1000 / fps) - (frame_end_ticks - frame_start_ticks));
  }
}

class InputHandler {
 public:
  InputHandler(){};
  ~InputHandler(){};

  void start_frame();

  void handle_keyboard_event(SDL_Event event);

  bool was_pressed(const SDL_Scancode& key);
  bool was_released(const SDL_Scancode& key);
  bool is_held(const SDL_Scancode& key);

 private:
  std::map<SDL_Scancode, bool> pressed_keys;
  std::map<SDL_Scancode, bool> released_keys;
  std::map<SDL_Scancode, bool> held_keys;
};

void InputHandler::start_frame() {
  pressed_keys.clear();
  released_keys.clear();
}

void InputHandler::handle_keyboard_event(SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    pressed_keys[event.key.keysym.scancode] = true;
    held_keys[event.key.keysym.scancode] = true;
  }
  if (event.type == SDL_KEYUP) {
    held_keys[event.key.keysym.scancode] = false;
    released_keys[event.key.keysym.scancode] = true;
  }
}

bool InputHandler::was_pressed(const SDL_Scancode& key) {
  return pressed_keys[key];
}

bool InputHandler::was_released(const SDL_Scancode& key) {
  return released_keys[key];
}

bool InputHandler::is_held(const SDL_Scancode& key) {
  return held_keys[key];
}

class CollisionProcessor {
 public:
  Vector2 simplex[3];
  Vector2 collision_normal;
  float collision_depth;
  std::vector<Point> _manifold;
  bool debug{false};

  bool AABB_overlap(Entity* A, Entity* B);
  std::vector<std::pair<Entity*, Entity*>> brute_force(
      std::vector<Entity*> entities);
  Point furthest_point(Entity* entity, Vector2& d);
  Vector2 support(Entity* A, Entity* B, Vector2& d);
  bool GJK(Entity* A, Entity* B);
  bool same_direction(Vector2& d, const Vector2& ao);
  Vector2 triple_product(Vector2& a, Vector2& b, Vector2& c);
  void EPA(Entity* A, Entity* B);
  Edge FindClosestEdge(std::vector<Point>& polygon);
  std::vector<Point> clip(Point v1, Point v2, Vector2 normal, float o);

  std::vector<Point> FindCollisionManifold(Entity* A, Entity* B);
  Edge FindRelevantEdge(Entity* entity, bool positive_direction);

  void evaluate_collisions(std::vector<Entity*> entities);
  void reset_collisions(std::vector<Entity*> entities);
};

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

Point CollisionProcessor::furthest_point(Entity* entity, Vector2& d) {
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

Vector2 CollisionProcessor::support(Entity* A, Entity* B, Vector2& d) {
  Vector2 d_inv = d * -1;

  Point pt_two = furthest_point(B, d_inv);

  return furthest_point(A, d) - pt_two;
}

Vector2 CollisionProcessor::triple_product(Vector2& a, Vector2& b, Vector2& c) {
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

bool CollisionProcessor::GJK(Entity* A, Entity* B) {
  // Adapted from: https://github.com/kroitor/gjk.c/blob/master/gjk.c

  int index = 0;

  Vector2 d(1, 0);

  Vector2 a = support(A, B, d);
  simplex[index] = a;

  if (a.dot(d) <= 0)
    return false;  // No collision

  d = a * -1;

  while (true) {
    Vector2 a = support(A, B, d);
    index++;
    simplex[index] = a;

    if (a.dot(d) <= 0)  // check if point B is beyond the origin with CO * OB
      return false;     // No collision

    Vector2 ao = a * -1;

    if (index < 2)  // Line segment
    {
      Vector2 b = simplex[0];
      Vector2 ab = b - a;
      d = triple_product(ab, ao, ab);
      if (d.norm() == 0) {
        d.value[0] = ab[1];
        d.value[1] = -ab[0];
      }
      continue;
    }

    Vector2 b = simplex[1];
    Vector2 c = simplex[0];
    Vector2 ab = b - a;
    Vector2 ac = c - a;

    Vector2 ac_perpendicular = triple_product(ab, ac, ac);

    if (ac_perpendicular.dot(ao) >= 0) {
      d = ac_perpendicular;
    } else {
      Vector2 ab_perpendicular = triple_product(ac, ab, ab);

      if (ab_perpendicular.dot(ao) < 0)
        return true;  // Collision

      simplex[0] = simplex[1];

      d = ab_perpendicular;
    }
    simplex[1] = simplex[2];
    --index;
  }
  return false;
}

void CollisionProcessor::EPA(Entity* A, Entity* B) {
  // Adapted from: https://dyn4j.org/2010/05/epa-expanding-polytope-algorithm/

  // Convert simplex to vector of Points (aka a polygon)
  std::vector<Point> polygon;
  Point pt;

  for (int idx = 0; idx < 3; idx++) {
    pt.x = simplex[idx][0];
    pt.y = simplex[idx][1];

    polygon.push_back(pt);
  }

  int iter{0};
  while (true) {
    // Find edge closest to origin
    Edge e = FindClosestEdge(polygon);

    // Get support vector along edge normal
    Vector2 p = support(A, B, e.normal);

    // Dot product support vector with normal
    float d = p.dot(e.normal);

    // If that dot product minus length of edge is less than tolerance
    if (abs(d - e.distance) < .00001 || iter == 99) {
      // penetration vector is the edge normal
      // penetration depth is d
      collision_normal = e.normal;
      collision_depth = d;

      return;
    } else {
      // Convert Vector2 to point
      pt.x = p[0];
      pt.y = p[1];

      // Add support point to polygon
      polygon.insert(polygon.begin() + e.index, pt);

      iter++;
    }
  }
}

Edge CollisionProcessor::FindClosestEdge(std::vector<Point>& polygon) {
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
    Vector2 n = triple_product(edge_vect, oa, edge_vect);

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

bool CollisionProcessor::same_direction(Vector2& d, const Vector2& ao) {
  return d.dot(ao) > 0;
}

void CollisionProcessor::reset_collisions(std::vector<Entity*> entities) {
  _manifold.clear();
  for (int idx = 0; idx < entities.size(); idx++) {
    entities[idx]->collision = false;
  }
}

Edge CollisionProcessor::FindRelevantEdge(Entity* entity, bool positive_direction) {
  Vector2 normal;
  if (positive_direction)
    normal = collision_normal;
  else
    normal = collision_normal * -1;

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
  if (right.dot(normal) < left.dot(normal)) { // Maybe this needs to be abs() of each dot product...TBD!
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

std::vector<Point> CollisionProcessor::FindCollisionManifold(Entity* A, Entity* B) {
  // Adapted from: https://dyn4j.org/2011/11/contact-points-using-clipping/
  
  // Find relevant edges
  Edge e1 = FindRelevantEdge(A, true);
  Vector2 e1_vect(e1.v1, e1.v2);
  Edge e2 = FindRelevantEdge(B, false);
  Vector2 e2_vect(e2.v1, e2.v2);

  // Determine reference and incident edge
  Edge ref_edge, inc_edge;
  Vector2 ref_vect, inc_vect;
  bool flip{false};

  if (abs(e1_vect.dot(collision_normal)) <= abs(e2_vect.dot(collision_normal))) {
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
  std::vector<Point> clipped_points = clip(inc_edge.v1, inc_edge.v2, ref_vect, o1);

  if (clipped_points.size() == 0)
    return clipped_points;

  // Clip 2
  float o2 = ref_vect * ref_edge.v2;
  clipped_points = clip(clipped_points[0], clipped_points[1], (ref_vect * -1), o2 * -1);
  
  if (clipped_points.size() == 0)
    return clipped_points;

  // Clip 3
  Vector2 ref_normal(abs(ref_vect[1]), abs(ref_vect[0])); // = ref_vect.cross(-1);

  if (flip)
    ref_normal = ref_normal * -1;

  Vector2 ref_edge_max(ref_edge.max);

  float max = ref_normal.dot(ref_edge_max);

  Vector2 pt_0(clipped_points[0]);
  Vector2 pt_1(clipped_points[1]);

  // if (ref_normal.dot(pt_0) - max < 0)
  //   clipped_points.erase(clipped_points.begin());
  // if (ref_normal.dot(pt_1) - max < 0)
  //   clipped_points.erase(clipped_points.begin()+1);

  return clipped_points;
}

std::vector<Point> CollisionProcessor::clip(Point v1, Point v2, Vector2 normal, float o) {
  std::vector<Point> clipped_points;

  float dist1 = normal * v1 - o;
  float dist2 = normal * v2 - o;

  if (dist1 < 0 & dist2 < 0) {
    std::cout << dist1 << ", " << dist2 << "\n";
  }

  if (dist1 >= 0)
    clipped_points.push_back(v1);
  if (dist2 >= 0)
    clipped_points.push_back(v2);

  if (dist1 * dist2 < 0) {
    Vector2 e = v2 - v1;

    float u = dist1 / (dist1 - dist2);
    e = e * u;
    e = e + v1;

    Point pt;
    pt.x = e[0];
    pt.y = e[1];

    clipped_points.push_back(pt);
  }
  return clipped_points;
}

void CollisionProcessor::evaluate_collisions(std::vector<Entity*> entities) {

  for (int i = 0; i < entities.size(); i++) {
    entities[i]->calculate_vertices();
    entities[i]->calculate_vertices_in_WCS();
  }

  std::vector<std::pair<Entity*, Entity*>> pairs = brute_force(entities);

  // For each pair
  for (int idx = 0; idx < pairs.size(); idx++) {
    if (GJK(pairs[idx].first, pairs[idx].second)) {
      pairs[idx].first->collision = true;
      pairs[idx].second->collision = true;

      EPA(pairs[idx].first, pairs[idx].second);
      std::vector<Point> manifold = FindCollisionManifold(pairs[idx].first, pairs[idx].second);
      _manifold = manifold;

      if (debug) {
        std::cout << "New Manifold\n";
        for (auto pt : manifold) {
          std::cout << pt.x << ", " << pt.y << "\n";
        }
      }
    }
  }
}
#endif