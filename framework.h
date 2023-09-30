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
  float pos_x;
  float pos_y;
  float vel_x;
  float vel_y;
  float acc_x;
  float acc_y;
  float angle;
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

// CLASSES
class Animation {
 public:
  int frame_idx{0};
  int update_interval; // milliseconds
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

  if (!sprite_sheet) {
    std::cout << "Failed to load file " << sprite_sheet_path << "\n";
    exit(1);
  }
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
  std::vector<Entity*> queue;
  SDL_Window* window;
  SDL_Renderer* renderer;

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
  SDL_SetRenderDrawColor(renderer, 0xbf, 0x1d, 0x36, 0xFF);
}

Graphics::~Graphics() {
  SDL_DestroyWindow(window);
}

void Graphics::add_to_queue(Entity& entity)
{
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
    if (point_within_camera_view(entity->state.pos_x,
                                 entity->state.pos_y,
                                 camera) ||  // Check Bottom-Left
        point_within_camera_view(entity->state.pos_x,
                                 entity->state.pos_y + entity->size_y,
                                 camera) ||  // Check Top-Left
        point_within_camera_view(entity->state.pos_x + entity->size_x,
                                 entity->state.pos_y,
                                 camera) ||  // Check Bottom-Right
        point_within_camera_view(entity->state.pos_x + entity->size_x,
                                 entity->state.pos_y + entity->size_y,
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
      destination_rect.x = (entity->state.pos_x - camera.pos_x) * scale_x;
      destination_rect.y =
          window_height -
          (entity->state.pos_y - camera.pos_y) *
              scale_y - (entity->size_y * scale_y);
      destination_rect.w = entity->size_x * scale_x;
      destination_rect.h = entity->size_y * scale_y;

      SDL_RenderCopy(renderer, entity->sprite_sheet, &clipping_rect,
                     &destination_rect);
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

#endif