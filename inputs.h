#ifndef INPUTS_H
#define INPUTS_H

#include <map>

#include "SDL.h"

class InputHandler {
 public:
  InputHandler(){};
  ~InputHandler(){};

  void start_frame();

  void handle_keyboard_event(SDL_Event event);
  void handle_mouse_event(SDL_Event event);

  bool was_pressed(const SDL_Scancode& key);
  bool was_released(const SDL_Scancode& key);
  bool is_held(const SDL_Scancode& key);

  bool left_click{false};
  bool right_click{false};
  bool middle_click{false};
  Point mouse_coords; // Screen coordinate system

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

void InputHandler::handle_mouse_event(SDL_Event event) {
  //Update Mouse position
  int x;
  int y;

  SDL_GetMouseState(&x, &y);
  mouse_coords.x = x;
  mouse_coords.y = y;

  if (event.type == SDL_MOUSEBUTTONDOWN) {
    if (event.button.button == SDL_BUTTON_LEFT)
      left_click = true;
    if (event.button.button == SDL_BUTTON_MIDDLE)
      middle_click = true;
    if (event.button.button == SDL_BUTTON_RIGHT)
      right_click = true;
  }
  if (event.type == SDL_MOUSEBUTTONUP) {
    if (event.button.button == SDL_BUTTON_LEFT)
      left_click = false;
    if (event.button.button == SDL_BUTTON_MIDDLE)
      middle_click = false;
    if (event.button.button == SDL_BUTTON_RIGHT)
      right_click = false;
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