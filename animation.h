#ifndef ANIMATION_H
#define ANIMATION_H

#include "SDL.h"
#include <vector>

class Animation {
 public:
  int frame_idx{0};
  int update_interval;  // milliseconds
  Uint32 prev_update_ticks{0};
  std::vector<SDL_Rect> frame_rects;

  Animation(){};
  Animation(const Animation& animation);
  ~Animation(){};

  void add_frame(int x, int y, int width, int height);
  SDL_Rect get_frame(Uint32 current_ticks);
  void reset();
};

Animation::Animation(const Animation& animation) {
  this->frame_idx = animation.frame_idx;
  this->update_interval = animation.update_interval;
  this->prev_update_ticks = animation.prev_update_ticks;
  this->frame_rects = animation.frame_rects;
}

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

#endif