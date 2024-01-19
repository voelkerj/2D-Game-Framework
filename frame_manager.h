#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include "SDL.h"

class FrameManager {
 public:
  Uint32 frame_start_ticks;
  Uint32 frame_end_ticks;
  int fps{60};

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

#endif