#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "SDL.h"
#include "entity.h"

#include <vector>

struct Camera {
  // Position of Camera's center point
  //   -centerpoint is also the center of the window
  float pos_x;
  float pos_y;

  // Width of the Camera's projected FOV
  float FOV_width;
  float FOV_height;
};

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

#endif