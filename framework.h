#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <cinttypes>
#include <iostream>  //DEBUG
#include <map>
#include <string>
#include <vector>

#include "SDL.h"
#include "SDL_image.h"
#include "physics.h"

// STRUCTS
struct State {
  float pos_x{0};
  float pos_y{0};
  float vel_x{0};
  float vel_y{0};
  float acc_x{0};
  float acc_y{0};
  float angular_velocity{0};  // Degrees per second
  float angle{0};             // Degrees
};

struct Camera {
  // Position of Camera's center point
  //   -centerpoint is also the center of the window
  float pos_x;
  float pos_y;

  // Width of the Camera's projected FOV
  float FOV_width;
  float FOV_height;
};

class Point {
 public:
  float x;
  float y;

  Point operator+(Point other);
  Point operator-(Point other);
};

Point Point::operator+(Point other) {
  Point p;
  p.x = x + other.x;
  p.y = y + other.y;
  return p;
}

Point Point::operator-(Point other) {
  Point p;
  p.x = x - other.x;
  p.y = y - other.y;
  return p;
}

enum Shape { circle, box };

// CLASSES
class Vector2 {
 public:
  float value[2];

  Vector2();
  Vector2(float a, float b);
  Vector2(Point p);  // Convert a point to a vector
  ~Vector2(){};

  float operator[](int idx);
  Vector2 operator+(Vector2 other);
  Vector2 operator-(Vector2 other);
  Vector2 operator*(int number);
  Vector2 operator*(float number);
  Vector2 operator=(Point p);

  float norm();
  float dot(Vector2 other);
  float cross(Vector2 other);

  void print();
};

Vector2::Vector2() {
  value[0] = 0;
  value[1] = 0;
}

Vector2::Vector2(float a, float b) {
  value[0] = a;
  value[1] = b;
}

Vector2::Vector2(Point p) {
  value[0] = p.x;
  value[1] = p.y;
}

float Vector2::operator[](int idx) {
  return value[idx];
}

Vector2 Vector2::operator+(Vector2 other) {
  Vector2 result(value[0] + other[0], value[1] + other[1]);
  return result;
}

Vector2 Vector2::operator-(Vector2 other) {
  Vector2 result(value[0] - other[0], value[1] - other[1]);
  return result;
}

Vector2 Vector2::operator*(int number) {
  Vector2 result(value[0] * number, value[1] * number);
  return result;
}

Vector2 Vector2::operator*(float number) {
  Vector2 result(value[0] * number, value[1] * number);
  return result;
}

Vector2 Vector2::operator=(Point p) {
  Vector2 result(p);
  return result;
}

float Vector2::norm() {
  return sqrt(pow(value[0], 2) + pow(value[1], 2));
}

float Vector2::dot(Vector2 other) {
  return value[0] * other[0] + value[1] * other[1];
}

float Vector2::cross(Vector2 other) {
  return value[0] * other[1] - value[1] * other[0];
}

void Vector2::print() {
  std::cout << value[0] << ", " << value[1] << "\n";
}

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
    for (int deg = 0; deg < 360; deg++) {
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
    vertex2.x = -size_x / 2;
    vertex2.y = size_y / 2;
    Point vertex3;
    vertex3.x = size_x / 2;
    vertex3.y = -size_y / 2;
    Point vertex4;
    vertex4.x = size_x / 2;
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
      new_vertex.x = (vertex.x * cos(state.angle * (M_PI / 180)) - vertex.y * sin(state.angle * (M_PI / 180))) + state.pos_x;
      new_vertex.y = (vertex.x * sin(state.angle * (M_PI / 180)) + vertex.y * cos(state.angle * (M_PI / 180))) + state.pos_y;
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
  std::vector<Entity*> queue;
  SDL_Window* window;
  SDL_Renderer* renderer;
  bool debug{false};

  Graphics();
  ~Graphics();

  void add_to_queue(Entity& entity);
  void draw_queue(Camera camera, Uint32 current_ticks);
  void clear_queue();

  void clear_screen();

  bool point_within_camera_view(float x, float y, Camera camera);
};

Graphics::Graphics() {
  window = SDL_CreateWindow("Framework", SDL_WINDOWPOS_UNDEFINED,
                            SDL_WINDOWPOS_UNDEFINED, 1, 1,
                            SDL_WINDOW_FULLSCREEN_DESKTOP);
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);
}

Graphics::~Graphics() {
  SDL_DestroyWindow(window);
}

void Graphics::add_to_queue(Entity& entity) {
  queue.push_back(&entity);
}

bool Graphics::point_within_camera_view(float x, float y, Camera camera) {

  if (x >= camera.pos_x & x <= camera.pos_x + camera.FOV_width &
      y >= camera.pos_y & y <= camera.pos_y + camera.FOV_height)
    return true;

  return false;
}

void Graphics::draw_queue(Camera camera, Uint32 current_ticks) {
  for (auto& entity : queue) {
    // If entity is within the camera bounds
    if (point_within_camera_view(entity->state.pos_x - entity->size_x / 2,
                                 entity->state.pos_y - entity->size_y / 2,
                                 camera) ||  // Check Bottom-Left
        point_within_camera_view(entity->state.pos_x - entity->size_x / 2,
                                 entity->state.pos_y + entity->size_y / 2,
                                 camera) ||  // Check Top-Left
        point_within_camera_view(entity->state.pos_x + entity->size_x / 2,
                                 entity->state.pos_y - entity->size_y / 2,
                                 camera) ||  // Check Bottom-Right
        point_within_camera_view(entity->state.pos_x + entity->size_x / 2,
                                 entity->state.pos_y + entity->size_y / 2,
                                 camera))  // Check Top-Right
    {

      // Draw Entity
      Animation& animation = entity->animations[entity->current_animation];

      SDL_Rect clipping_rect = animation.get_frame(current_ticks);

      int window_width;
      int window_height;
      SDL_GetWindowSize(window, &window_width, &window_height);

      float scale_x = window_width / camera.FOV_width;
      float scale_y = window_height / camera.FOV_height;

      SDL_Rect destination_rect;
      destination_rect.x =
          (entity->state.pos_x - entity->size_x / 2 - camera.pos_x) * scale_x;
      destination_rect.y =
          window_height -
          (entity->state.pos_y - entity->size_y / 2 - camera.pos_y) * scale_y -
          (entity->size_y * scale_y);
      destination_rect.w = entity->size_x * scale_x;
      destination_rect.h = entity->size_y * scale_y;

      if (entity->collision) {
        SDL_SetTextureColorMod(entity->sprite_sheet, 255, 0, 0);
      } else {
        SDL_SetTextureColorMod(entity->sprite_sheet, 255, 255, 255);
      }

      SDL_RenderCopyEx(renderer, entity->sprite_sheet, &clipping_rect,
                       &destination_rect, -entity->state.angle, NULL,
                       SDL_FLIP_NONE);

      // Draw Vertices
      if (debug) {
        entity->calculate_vertices_in_WCS();
        for (Point point : entity->vertices_WCS) {
          SDL_SetRenderDrawColor(renderer, 0x48, 0xff, 0x82, 0xFF);

          float pt_x = (point.x - camera.pos_x) * scale_x;
          float pt_y = window_height - (point.y - entity->size_y - camera.pos_y) * scale_y - (entity->size_y * scale_y);

          SDL_RenderDrawPoint(renderer, pt_x, pt_y);
        }
        SDL_SetRenderDrawColor(renderer, 0xbf, 0x1b, 0x38, 0xFF);

        float pt_x = (0 - camera.pos_x) * scale_x;
        float pt_y = window_height - (0 - entity->size_y - camera.pos_y) * scale_y - (entity->size_y * scale_y);
        SDL_RenderDrawPoint(renderer, pt_x, pt_y);

        SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);
      }
    }
  }
  SDL_RenderPresent(renderer);
}

void Graphics::clear_queue() {
  queue.clear();
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
  bool debug{false};
  bool AABB_overlap(Entity* A, Entity* B);
  std::vector<std::pair<Entity*, Entity*>> brute_force(
      std::vector<Entity*> entities);
  Point furthest_point(Entity* entity, Vector2& d);
  Vector2 support(Entity* A, Entity* B, Vector2& d);
  bool GJK(Entity* A, Entity* B);
  bool same_direction(Vector2& d, const Vector2& ao);
  Vector2 triple_product(Vector2& a, Vector2& b, Vector2& c);

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

  if (debug)
    std::cout << A->name << " and " << B->name << " potentially colliding!\n";

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

  A->calculate_vertices_in_WCS();
  B->calculate_vertices_in_WCS();
  int index = 0;
  Vector2 simplex[3];

  Vector2 d(1, 0);

  Vector2 a = support(A, B, d);
  simplex[index] = a;

  if (a.dot(d) <= 0)
    return false;  // No collision

  d = a * -1;

  // int iter{0};
  while (true) {
    // std::cout << iter << "\n";
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
    // iter++;
  }
  return false;
}

bool CollisionProcessor::same_direction(Vector2& d, const Vector2& ao) {
  return d.dot(ao) > 0;
}

void CollisionProcessor::evaluate_collisions(std::vector<Entity*> entities) {
  std::vector<std::pair<Entity*, Entity*>> pairs = brute_force(entities);

  // For each pair
  for (int idx = 0; idx < pairs.size(); idx++) {
    if (GJK(pairs[idx].first, pairs[idx].second)) {
      pairs[idx].first->collision = true;
      pairs[idx].second->collision = true;
    }
  }
}

void CollisionProcessor::reset_collisions(std::vector<Entity*> entities) {
  for (int idx = 0; idx < entities.size(); idx++) {
    entities[idx]->collision = false;
  }
}
#endif