#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <cinttypes>
#include <vector>

#include "math.h"

#include "SDL.h"

// STRUCTS
struct State
{
    int pos_x;
    int pos_y;
    float vel_x;
    float vel_y;
    float acc_x;
    float acc_y;
};

// CLASSES
class Animation
{
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

void Animation::add_frame(int x, int y, int width, int height)
{
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = width;
    rect.h = height;

    frame_rects.push_back(rect);
};

SDL_Rect Animation::get_frame(Uint32 current_ticks)
{
    Uint32 elapsed_ticks = current_ticks - prev_update_ticks;

    if (elapsed_ticks >= update_interval) // if we are past the update interval
    {
        frame_idx++;
        prev_update_ticks = current_ticks;
    }

    if (frame_idx > frame_rects.size() - 1)
        reset();

    SDL_Rect current_rect = frame_rects[frame_idx];

    return current_rect;
};

void Animation::reset()
{
    frame_idx = 0;
};

class Force
{
    public:
    float fx;
    float fy;

    float magnitude();
    float direction();
};

float Force::magnitude() {
    return sqrt(pow(fx,2) + pow(fy,2));
};

float Force::direction() {
    return atan2(fy, fx);
}

#endif