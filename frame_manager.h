#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include "SDL.h"

class FrameManager {
 public:
  Uint32 frame_start_ticks;
  Uint32 frame_end_ticks;
  float elapsed_time{0}; // Elapsed time of previous frame [s]
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

  elapsed_time = (frame_end_ticks - frame_start_ticks) / 1000;

  if (elapsed_time < (1.0 / fps)) {
    SDL_Delay((1.0 / fps) - elapsed_time);
    elapsed_time = 1.0 / fps;
  }
}

#endif