#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <cinttypes>
#include <iostream>  //DEBUG
#include <map>
#include <string>
#include <vector>

#include "SDL.h"
#include "SDL_image.h"
#include "math.h"

// STRUCTS
struct State {
  int pos_x;
  int pos_y;
  float vel_x;
  float vel_y;
  float acc_x;
  float acc_y;
};

struct World {
  int size_x;
  int size_y;
};

struct Camera {
  // Position of Camera's center point
  //   -centerpoint is also the center of the window
  int pos_x;
  int pos_y;

  // Width of the Camera's projected FOV
  float FOV_width;
  float FOV_height;
};

// CLASSES
class Animation {
 public:
  int frame_idx;
  int update_interval;
  Uint32 prev_update_ticks;
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
  Uint32 elapsed_ticks = current_ticks - prev_update_ticks;

  // if we are past the update interval, get next frame
  if (elapsed_ticks >= update_interval) {
    frame_idx++;
    prev_update_ticks = current_ticks;
  }

  // if we have advanced past the last frame, get first frame
  if (frame_idx > frame_rects.size() - 1)
    reset();

  SDL_Rect current_rect = frame_rects[frame_idx];

  return current_rect;
}

void Animation::reset() {
  frame_idx = 0;
}

class Force {
 public:
  float fx;
  float fy;

  Force();
  Force(float fx_in, float fy_in);
  ~Force(){};

  float magnitude();
  float direction();
};

Force::Force() {
  fx = 0;
  fy = 0;
}

Force::Force(float fx_in, float fy_in) {
  fx = fx_in;
  fy = fy_in;
}

float Force::magnitude() {
  return sqrt(pow(fx, 2) + pow(fy, 2));
}

float Force::direction() {
  return atan2(fy, fx);
}

class Entity {
 public:
  int size_x;
  int size_y;
  float mass;
  State state;
  std::map<std::string, Animation> animations;
  std::string current_animation;
  bool collision;
  SDL_Texture* sprite_sheet;

  Entity(){};
  ~Entity(){};

  void load_sprite_sheet(std::string sprite_sheet_path, SDL_Renderer* renderer);
  void update_state(std::vector<Force> forces, Uint32 elapsed_time);
};

void Entity::load_sprite_sheet(std::string sprite_sheet_path,
                               SDL_Renderer* renderer) {
  sprite_sheet = IMG_LoadTexture(renderer, sprite_sheet_path.c_str());
}

void Entity::update_state(std::vector<Force> forces, Uint32 elapsed_time) {
  std::vector<Force>::iterator force_ptr;

  Force resultant;

  // Force
  for (force_ptr = forces.begin(); force_ptr < forces.end(); force_ptr++) {
    resultant.fx += force_ptr->fx;
    resultant.fy += force_ptr->fy;
  }

  // Acceleration
  state.acc_x = resultant.fx / mass;
  state.acc_y = resultant.fy / mass;

  // Velocity
  state.vel_x = state.acc_x * elapsed_time;
  state.vel_y = state.acc_y * elapsed_time;

  // Position
  state.pos_x = state.vel_x * elapsed_time;
  state.pos_y = state.vel_y * elapsed_time;
}

class Graphics {
 public:
  std::vector<Entity> queue;
  SDL_Window* window;
  SDL_Renderer* renderer;

  Graphics();
  ~Graphics();

  void add_to_queue(Entity entity);
  void draw_queue(Camera camera, Uint32 current_ticks);
  void clear_queue();

  void clear_screen();
};

Graphics::Graphics() {
  window = SDL_CreateWindow("Framework",
                            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1,
                            1, SDL_WINDOW_FULLSCREEN_DESKTOP);
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  SDL_SetRenderDrawColor(renderer, 0x29, 0x32, 0x41, 0xFF);
}

Graphics::~Graphics() {
  SDL_DestroyWindow(window);
}

void Graphics::add_to_queue(Entity entity) {
  queue.push_back(entity);
}

void Graphics::draw_queue(Camera camera, Uint32 current_ticks) {
  std::vector<Entity>::iterator entity_ptr;

  for (entity_ptr = queue.begin(); entity_ptr < queue.end(); entity_ptr++) {
    // If entity is within the camera bounds
    if (entity_ptr->state.pos_x > camera.pos_x &
        entity_ptr->state
            .pos_x<(camera.pos_x + camera.FOV_width) & entity_ptr->state.pos_y>
                camera.pos_y &
        entity_ptr->state.pos_y > (camera.pos_y + camera.FOV_height)) {

      // Draw Entity
      SDL_Rect clipping_rect =
          entity_ptr->animations[entity_ptr->current_animation].get_frame(
              current_ticks);

      int window_width;
      int window_height;
      SDL_GetWindowSize(window, &window_width, &window_height);

      float scale_x = window_width / camera.FOV_width;
      // std::cout << window_width << ", " << camera.FOV_width << "\n";
      float scale_y = window_height / camera.FOV_height;

      SDL_Rect destination_rect;
      // destination_rect.x = entity_ptr->state.pos_x * scale_x;
      // destination_rect.y = window_height - (entity_ptr->state.pos_y * scale_y);
      // destination_rect.w = entity_ptr->size_x * scale_x;
      // destination_rect.h = entity_ptr->size_y * scale_y;

      destination_rect.x = 50;
      destination_rect.y = 50;
      destination_rect.w = 64;
      destination_rect.h = 16;

      // std::cout << scale_x << ", " << scale_y << "\n";
      // std::cout << destination_rect.x << ", " << destination_rect.y << ", " << destination_rect.w << ", " << destination_rect.h << "\n";
      // std::cout << clipping_rect.x << ", " << clipping_rect.y << ", " << clipping_rect.w << ", " << clipping_rect.h << "\n";

      SDL_RenderCopy(renderer, entity_ptr->sprite_sheet, &clipping_rect,
                     &destination_rect);
    }
  }
  // std::cout << "drawing\n";
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

#endif