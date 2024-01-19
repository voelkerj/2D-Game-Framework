#ifndef ENTITY_H
#define ENTITY_H

#include "SDL.h"
#include "SDL_image.h"
#include "physics.h"
#include "animation.h"
#include "geometry.h"

#include <string>
#include <vector>
#include <map>
#include <iostream>

class Entity {
 public:
  float size_x;
  float size_y;
  float mass{0};
  float invMass{0};
  float friction{0.25};
  State state;
  std::map<std::string, Animation> animations;
  std::string current_animation;
  SDL_Texture* sprite_sheet;
  MomentOfInertia MoI;
  Shape shape;
  std::vector<Point> vertices;
  std::vector<Point> vertices_WCS;
  std::string name;

  Entity(std::string name_in, Shape shape_in, State state_in, float size_x_in, float size_y_in, std::string sprite_path, SDL_Renderer* renderer);
  Entity(Shape shape_in, State state_in, float size_x_in, float size_y_in, std::string sprite_path, SDL_Renderer* renderer);

  Entity(const Entity &entity);
  Entity& operator=(Entity entity);

  void load_sprite_sheet(std::string sprite_sheet_path, SDL_Renderer* renderer);
  void update_state(std::vector<Force> forces, std::vector<Moment> moments,
                    Uint32 elapsed_time);
  void print_state();

  void calculate_MOI();
  void calculate_vertices();
  void calculate_vertices_in_WCS();

  float get_min_x_WCS();
  float get_max_x_WCS();
  float get_min_y_WCS();
  float get_max_y_WCS();
};

Entity& Entity::operator=(Entity entity) {
  size_x = entity.size_x;
  size_y = entity.size_y;
  mass = entity.mass;
  invMass = entity.invMass;
  friction = entity.friction;
  state = entity.state;
  animations = entity.animations;
  current_animation = entity.current_animation;
  sprite_sheet = entity.sprite_sheet;
  MoI = entity.MoI;
  shape = entity.shape;
  vertices = entity.vertices;
  vertices_WCS = entity.vertices_WCS;
  name = entity.name;
  // std::cout << "Copy Assignment " << &entity << " (" << entity.name << ") " << "to " << this << " (" << name << ")" << "\n";

  return *this;
}

Entity::Entity(const Entity &entity) {
  size_x = entity.size_x;
  size_y = entity.size_y;
  mass = entity.mass;
  invMass = entity.invMass;
  friction = entity.friction;
  state = entity.state;
  animations = entity.animations;
  current_animation = entity.current_animation;
  sprite_sheet = entity.sprite_sheet;
  MoI = entity.MoI;
  shape = entity.shape;
  vertices = entity.vertices;
  vertices_WCS = entity.vertices_WCS;
  name = entity.name;

  // std::cout << "Copying " << &entity << " (" << entity.name << ") " << "to " << this << " (" << name << ")" << "\n";
}

Entity::Entity(std::string name_in, Shape shape_in, State state_in, float size_x_in, float size_y_in, std::string sprite_path, SDL_Renderer* renderer) {
  name = name_in;
  shape = shape_in;
  state = state_in;
  size_x = size_x_in;
  size_y = size_y_in;
  load_sprite_sheet(sprite_path, renderer);
}

Entity::Entity(Shape shape_in, State state_in, float size_x_in, float size_y_in, std::string sprite_path, SDL_Renderer* renderer) {
  name = std::to_string((unsigned long long) (void**)this); // witchcraft from the internet
  shape = shape_in;
  state = state_in;
  size_x = size_x_in;
  size_y = size_y_in;
  load_sprite_sheet(sprite_path, renderer);
}

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

void Entity::print_state() {
  std::cout << "=== " << name << " State Information ===\n";
  std::cout << "Center: " << state.pos_x << ", " << state.pos_y << "\n";
  if (shape == box) {
    calculate_vertices();
    calculate_vertices_in_WCS();
    std::cout << "Vertices: ";
    for (auto vertex : vertices_WCS) {
      std::cout << "(" << vertex.x << ", " << vertex.y << "), ";
    }
    std::cout << "\n";
  }
  std::cout << "Angle: " << state.angle << "\n";
  std::cout << "Velocity: " << state.vel_x << ", " << state.vel_y << "\n";
  std::cout << "Acceleration: " << state.acc_x << ", " << state.acc_y << "\n";
  std::cout << "Angular Velocity: " << state.angular_velocity << "\n";
  std::cout << "========================\n";
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

struct EntityPair {
  Entity* A;
  Entity* B;
};

#endif